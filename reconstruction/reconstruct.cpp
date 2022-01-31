#include "reconstruct.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;

void rotateAndShiftPoints(vector<Point3f> & pointsInput,
                    Mat & R,
                    Mat & t,
                    vector<Point3f> & pointsOutput) {
    Mat Rf, tf;
    R.convertTo(Rf, CV_32F);
    t.convertTo(tf, CV_32F);
    for (auto &point : pointsInput) {
        Mat pointMat = (Mat_<float>(3, 1) << point.x, point.y, point.z);
        Mat pointMatResult = Rf * pointMat + tf;
        Point3d pointResult(pointMatResult);
        pointsOutput.push_back(pointResult);
    }

}

float Point3fNorm(Point3f & point){
    return sqrt(point.x * point.x + 
        point.y * point.y + 
        point.z * point.z);
}

float findScale(vector<Point3f> & points1, vector<Point3f> & points2) {
    float scaleSum = 0.0f;
    for (size_t i = 0; i < points1.size(); i++) {
        float norm1 = Point3fNorm(points1[i]);
        float norm2 = Point3fNorm(points2[i]);
        float scaleCurrent = norm1 / norm2;
        scaleSum += scaleCurrent;
    }
    float scale = scaleSum / points1.size();
    return scale;
}

void scalePoints(vector<Point3f> & points, float scale) {
    for (size_t i = 0; i < points.size(); i++) {
        points[i].x = scale * points[i].x;
        points[i].y = scale * points[i].y;
        points[i].z = scale * points[i].z;
    }
}

void wrtitePly(string plyPath,
 vector<Point3f> & pointsCloudTotal,
 vector<Vec3b> & pointsCloudTotalColors) {
    ofstream plyFile;
    plyFile.open (plyPath);
    plyFile << "ply" << endl;
    plyFile << "format ascii 1.0" << endl;
    plyFile << "element vertex " << pointsCloudTotal.size() << endl;
    plyFile << "property float x" << endl;
    plyFile << "property float y" << endl;
    plyFile << "property float z" << endl;
    plyFile << "property uchar diffuse_blue" << endl;
    plyFile << "property uchar diffuse_green" << endl;
    plyFile << "property uchar diffuse_red" << endl;
    plyFile << "end_header" << endl;
    for (size_t i = 0; i < pointsCloudTotal.size(); i++) {
        Point3f point = pointsCloudTotal[i];
        Vec3b color = pointsCloudTotalColors[i];
        plyFile << point.x << " " <<
         point.y << " " << 
         point.z << " " <<
        (int)color[0] << " " <<
         (int)color[1] << " " << 
         (int)color[2] << " " << endl;
    }
    plyFile.close();
}

void reconstruct(string imagesDirPath){

    string OUTPUT_PLY_PATH = "pointCloud.ply";

    cout << "reconstruct ..." << endl;

    vector<String> images;
    glob(imagesDirPath, images);
  
    int nfeatures = 0;
    int nOctaveLayers = 4;
    double contrastThreshold = 0.035;
    double edgeThreshold = 20;
    double sigma = 2.0;

    Ptr<Feature2D> detector = cv::SiftFeatureDetector::create(nfeatures,
                                                            nOctaveLayers,
                                                            contrastThreshold,
                                                            edgeThreshold,
                                                            sigma);


    //https://stackoverflow.com/questions/44081455/c-siftfeaturedetector-is-not-loading
    //https://docs.opencv.org/4.4.0/d5/d6f/tutorial_feature_flann_matcher.html
    //https://www.analyticsvidhya.com/blog/2019/10/detailed-guide-powerful-sift-technique-image-matching-python/
    Ptr<DescriptorMatcher> matcher = 
        DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    const float ratioThresh = 0.7f;
   
    Mat imageOld;
    Mat grayOld;
    Mat descriptorsOld;
    vector<KeyPoint> keypointsOld;

    Mat image;
    Mat gray;
    Mat descriptors;
    vector<KeyPoint> keypoints;

    vector<vector<DMatch> > knnMatches;
    vector<DMatch> goodMatches;
    vector<DMatch> goodMatchesInliers;
    vector<DMatch> goodMatches2;
    vector<DMatch> goodMatches2Old;

    Mat R, t;
    Mat R_old, t_old;

    vector<Point3f> pointsCloudTotal;
    vector<Vec3b> pointsCloudTotalColors;

    vector<Point3f> triangulatedPoints3d;
    vector<Point3f> triangulatedPoints3dOld;

    Mat CAMERA_MATRIX = (Mat_<float>(3,3) << 1487.886270357746, 0, 547.1524898799552,
                                0, 1488.787677381604, 979.9460018614599,
                                0, 0, 1);
    // to get CAMERA_MATRIX: calibrate camera with:
    // sfm --calibrate path_to_images

    double PROBABILITY = 0.999;
    double THRESHOLD = 1.0;

    double DISTANCE_THRESHOLD = 100.0;

    imageOld = imread(images[0]);
    cvtColor(imageOld, grayOld, COLOR_BGR2GRAY);
    detector->detectAndCompute(grayOld,
                            noArray(),
                            keypointsOld, 
                            descriptorsOld);

    Mat RTotal = Mat::eye(3, 3, CV_64F);
    Mat tTotal = Mat::zeros(3, 1, CV_64F);

    int nZeroIntersect = 0;
    for(int imageIndex = 1; imageIndex < images.size(); imageIndex++) {

        cout << imageIndex << endl;
        cout << images[imageIndex] << endl;
        
        image = imread(images[imageIndex]);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        detector->detectAndCompute(gray, noArray(), keypoints, descriptors);

        
        
        matcher->knnMatch(descriptorsOld, descriptors, knnMatches, 2 );

        //-- Filter matches using the Lowe's ratio test
        vector<Point2f> points1;
        vector<Point2f> points2;
        for (size_t i = 0; i < knnMatches.size(); i++)
        {   
            float distanceLimit = ratioThresh * knnMatches[i][1].distance;
            if (knnMatches[i][0].distance < distanceLimit)
            {      
                goodMatches.push_back(knnMatches[i][0]);
                int inx1 = knnMatches[i][0].queryIdx;
                points1.push_back(keypointsOld[inx1].pt);
                int inx2 = knnMatches[i][0].trainIdx;
                points2.push_back(keypoints[inx2].pt);
            }
        }

        vector<unsigned char> inliersMask(points1.size());
        Mat essentialMatrix = findEssentialMat(points1,
                                            points2,
                                            CAMERA_MATRIX,
                                            RANSAC,
                                            PROBABILITY,
                                            THRESHOLD,
                                            inliersMask);
        
        vector<Point2f> points1Inliers;
        vector<Point2f> points2Inliers;
        for (size_t i = 0; i < points1.size(); i++) {
            if (inliersMask[i]) {
                goodMatchesInliers.push_back(goodMatches[i]);
                points1Inliers.push_back(points1[i]);
                points2Inliers.push_back(points2[i]);
            }
        }

        Mat triangulatedPoints, mask2;
        recoverPose(essentialMatrix,
            points1Inliers,
            points2Inliers,
            CAMERA_MATRIX,
            R,
            t,
            DISTANCE_THRESHOLD,
            mask2, 
            triangulatedPoints);

        int nPoints2 = countNonZero(mask2);
        int index2 = 0;
        Mat triangulatedPoints2 = Mat::zeros(Size(nPoints2, 4), CV_32FC1);
        vector<Point2f> points12;
        vector<Point2f> points22;
        vector<Vec3b> triangulatedPoints3dColors;
        for (size_t i = 0; i < goodMatchesInliers.size(); i++) {
            if (mask2.at<unsigned char>(i)) { 
                DMatch match = goodMatchesInliers[i];
                goodMatches2.push_back(match);
                triangulatedPoints.col(i)
                                .copyTo(triangulatedPoints2.col(index2));
                points12.push_back(points1Inliers[i]);
                points22.push_back(points2Inliers[i]);
                int inx_old = match.queryIdx;
                Point2f point = keypointsOld[inx_old].pt;
                Vec3b color = imageOld.at<Vec3b>(point);
                triangulatedPoints3dColors.push_back(color);
                index2++;
            }
        }

        convertPointsFromHomogeneous(triangulatedPoints2.t(),
                                    triangulatedPoints3d);

        int nCrossClouds = 0;
        if (imageIndex == 1) {
            pointsCloudTotal.insert(pointsCloudTotal.end(),
                                triangulatedPoints3d.begin(),
                                triangulatedPoints3d.end());
        } else {
            // find cross of two sequental point clouds:
            vector<Point3f> triangulatedPoints3dIntersection;
            vector<Point3f> triangulatedPoints3dOldIntersection;
            for (size_t i = 0; i < goodMatches2.size(); i++) {
                for (size_t j = 0; j < goodMatches2Old.size(); j++) {
                    int indexFromOld = goodMatches2Old[j].trainIdx;
                    int indexFromCurrent = goodMatches2[i].queryIdx;
                    if ( indexFromOld == indexFromCurrent) {
                        triangulatedPoints3dIntersection
                                .push_back(triangulatedPoints3d[i]);
                        triangulatedPoints3dOldIntersection
                                .push_back(triangulatedPoints3dOld[j]);
                        nCrossClouds++;
                    }
                }
            }
            if (nCrossClouds == 0) {
                nZeroIntersect++;
            }

            vector<Point3f> triangulatedPoints3dOldIntersectionRotated;
            rotateAndShiftPoints(triangulatedPoints3dOldIntersection,
                    R_old,
                    t_old,
                    triangulatedPoints3dOldIntersectionRotated);
            float scale = findScale(triangulatedPoints3dOldIntersectionRotated,
                                    triangulatedPoints3dIntersection);
            t *= scale;
            scalePoints(triangulatedPoints3d, scale);

            RTotal = R_old * RTotal;
            tTotal = (R_old * tTotal) + t_old;
            vector<Point3f> triangulatedPoints3dRotatedToBase;
            Mat RTotalInv = RTotal.t();
            Mat tTotalInv = -(RTotalInv * tTotal);
            rotateAndShiftPoints(triangulatedPoints3d,
                    RTotalInv,
                    tTotalInv,
                    triangulatedPoints3dRotatedToBase);
            pointsCloudTotal.insert(pointsCloudTotal.end(),
                                    triangulatedPoints3dRotatedToBase.begin(),
                                    triangulatedPoints3dRotatedToBase.end() );

        }
        pointsCloudTotalColors.insert(pointsCloudTotalColors.end(),
                                    triangulatedPoints3dColors.begin(),
                                    triangulatedPoints3dColors.end());
        cout << "nCrossClouds : " << nCrossClouds << endl;
        
        keypointsOld = keypoints;
        keypoints.clear();

        descriptorsOld = descriptors.clone();
        descriptors.release();
        
        imageOld = image.clone();
        image.release();

        R_old = R.clone();
        R.release();
        t_old = t.clone();
        t.release();

        triangulatedPoints3dOld = triangulatedPoints3d;
        triangulatedPoints3d.clear();
        
        knnMatches.clear();

        goodMatches.clear();

        goodMatchesInliers.clear();

        goodMatches2Old = goodMatches2;
        goodMatches2.clear();


        cout << endl;
    }
    imageOld.release();
    grayOld.release();
    image.release();
    gray.release();
    descriptorsOld.release();
    descriptors.release();

    cout << endl;
    cout << "nZeroIntersect : " << nZeroIntersect << endl;
    cout << "pointsCloudTotal.size(): " << pointsCloudTotal.size() << endl;
    
    cout << endl;
    cout << "reconstruction done" << endl;

    wrtitePly(OUTPUT_PLY_PATH, pointsCloudTotal, pointsCloudTotalColors);
}

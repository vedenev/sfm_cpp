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
    const float ratio_thresh = 0.7f;
   
    Mat image_old;
    Mat gray_old;
    Mat descriptors_old;
    vector<KeyPoint> keypoints_old;

    Mat image;
    Mat gray;
    Mat descriptors;
    vector<KeyPoint> keypoints;

    Mat img_matches;
    vector<vector<DMatch> > knn_matches;
    vector<DMatch> good_matches;
    vector<DMatch> good_matches_Inliers;
    vector<DMatch> good_matches_2;
    vector<DMatch> good_matches_2_old;

    Mat R, t;
    Mat R_old, t_old;

    vector<Point3f> pointsCloudTotal;
    vector<Vec3b> pointsCloudTotalColors;

    vector<Point3f> triangulatedPoints3d;
    vector<Point3f> triangulatedPoints3d_old;

    Mat CAMERA_MATRIX = (Mat_<float>(3,3) << 1487.886270357746, 0, 547.1524898799552,
                                0, 1488.787677381604, 979.9460018614599,
                                0, 0, 1);
    // to get CAMERA_MATRIX: calibrate camera with:
    // sfm --calibrate path_to_images

    double PROBABILITY = 0.999;
    double THRESHOLD = 1.0;

    double DISTANCE_THRESHOLD = 100.0;

    image_old = imread(images[0]);
    cvtColor(image_old, gray_old, COLOR_BGR2GRAY);
    detector->detectAndCompute(gray_old,
                            noArray(),
                            keypoints_old, 
                            descriptors_old);

    Mat RTotal = Mat::eye(3, 3, CV_64F);
    Mat tTotal = Mat::zeros(3, 1, CV_64F);

    int nZeroIntersect = 0;
    for(int imageIndex = 1; imageIndex < images.size(); imageIndex++) {

        cout << imageIndex << endl;
        cout << images[imageIndex] << endl;
        
        image = imread(images[imageIndex]);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        detector->detectAndCompute(gray, noArray(), keypoints, descriptors);

        
        
        matcher->knnMatch(descriptors_old, descriptors, knn_matches, 2 );

        //-- Filter matches using the Lowe's ratio test
        
        vector<Point2f> points1;
        vector<Point2f> points2;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {   
            float distanceLimit = ratio_thresh * knn_matches[i][1].distance;
            if (knn_matches[i][0].distance < distanceLimit)
            {      
                good_matches.push_back(knn_matches[i][0]);
                int inx1 = knn_matches[i][0].queryIdx;
                points1.push_back(keypoints_old[inx1].pt);
                int inx2 = knn_matches[i][0].trainIdx;
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
                good_matches_Inliers.push_back(good_matches[i]);
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
        vector<Point2f> points1_2;
        vector<Point2f> points2_2;
        vector<Vec3b> triangulatedPoints3dColors;
        for (size_t i = 0; i < good_matches_Inliers.size(); i++) {
            if (mask2.at<unsigned char>(i)) { 
                DMatch match = good_matches_Inliers[i];
                good_matches_2.push_back(match);
                triangulatedPoints.col(i).copyTo(triangulatedPoints2.col(index2));
                points1_2.push_back(points1Inliers[i]);
                points2_2.push_back(points2Inliers[i]);
                int inx_old = match.queryIdx;
                Point2f point = keypoints_old[inx_old].pt;
                Vec3b color = image_old.at<Vec3b>(point);
                triangulatedPoints3dColors.push_back(color);
                index2++;
            }
        }

        convertPointsFromHomogeneous(triangulatedPoints2.t(), triangulatedPoints3d);

        int nCrossClouds = 0;
        if (imageIndex == 1) {
            pointsCloudTotal.insert(pointsCloudTotal.end(), triangulatedPoints3d.begin(), triangulatedPoints3d.end());
        } else {
            // find cross of two sequental point clouds:
            vector<Point3f> triangulatedPoints3d_intersection;
            vector<Point3f> triangulatedPoints3d_old_intersection;
            for (size_t i = 0; i < good_matches_2.size(); i++) {
                for (size_t j = 0; j < good_matches_2_old.size(); j++) {
                    if (good_matches_2_old[j].trainIdx == good_matches_2[i].queryIdx) {
                        triangulatedPoints3d_intersection.push_back(triangulatedPoints3d[i]);
                        triangulatedPoints3d_old_intersection.push_back(triangulatedPoints3d_old[j]);
                        nCrossClouds++;
                    }
                }
            }
            if (nCrossClouds == 0) {
                nZeroIntersect++;
            }

            vector<Point3f> triangulatedPoints3d_old_intersection_rotated;
            rotateAndShiftPoints(triangulatedPoints3d_old_intersection,
                    R_old,
                    t_old,
                    triangulatedPoints3d_old_intersection_rotated);
            float scale = findScale(triangulatedPoints3d_old_intersection_rotated, triangulatedPoints3d_intersection);
            t *= scale;
            scalePoints(triangulatedPoints3d, scale);

            RTotal = R_old * RTotal;
            tTotal = (R_old * tTotal) + t_old;
            vector<Point3f> triangulatedPoints3d_rotated_to_base;
            Mat RTotalInv = RTotal.t();
            Mat tTotalInv = -(RTotalInv * tTotal);
            rotateAndShiftPoints(triangulatedPoints3d,
                    RTotalInv,
                    tTotalInv,
                    triangulatedPoints3d_rotated_to_base);
            pointsCloudTotal.insert(pointsCloudTotal.end(), triangulatedPoints3d_rotated_to_base.begin(), triangulatedPoints3d_rotated_to_base.end() );

        }
        pointsCloudTotalColors.insert(pointsCloudTotalColors.end(), triangulatedPoints3dColors.begin(), triangulatedPoints3dColors.end());
        cout << "nCrossClouds : " << nCrossClouds << endl;
        
        keypoints_old = keypoints;
        keypoints.clear();

        descriptors_old = descriptors.clone();
        descriptors.release();
        
        image_old = image.clone();
        image.release();

        R_old = R.clone();
        R.release();
        t_old = t.clone();
        t.release();

        triangulatedPoints3d_old = triangulatedPoints3d;
        triangulatedPoints3d.clear();
        
        knn_matches.clear();

        good_matches.clear();

        good_matches_Inliers.clear();

        good_matches_2_old = good_matches_2;
        good_matches_2.clear();


        cout << endl;
    }
    image_old.release();
    gray_old.release();
    image.release();
    gray.release();
    descriptors_old.release();
    descriptors.release();
    img_matches.release();

    cout << endl;
    cout << "nZeroIntersect : " << nZeroIntersect << endl;
    cout << "pointsCloudTotal.size(): " << pointsCloudTotal.size() << endl;

    wrtitePly(OUTPUT_PLY_PATH, pointsCloudTotal, pointsCloudTotalColors);
}

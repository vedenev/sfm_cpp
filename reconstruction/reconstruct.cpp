#include "reconstruct.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
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
        //cout << "type: " << R.type() << " " << pointMat.type() << " " << t.type() << endl;
        Mat pointMatResult = Rf * pointMat + tf;
        //Mat pointMatResult = Rf * (pointMat - tf);
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
    float scaleSum2 = 0.0f;
    for (size_t i = 0; i < points1.size(); i++) {
        float norm1 = Point3fNorm(points1[i]);
        float norm2 = Point3fNorm(points2[i]);
        float scaleCurrent = norm1 / norm2;
        //cout << "scaleCurrent: " << norm1 << " " << norm2 << " " << scaleCurrent << endl;
        scaleSum += scaleCurrent;
        scaleSum2 += scaleCurrent * scaleCurrent;
    }
    float scale = scaleSum / points1.size();
    float std_ = sqrt((scaleSum2 / points1.size())  - scale * scale);
    cout << "scale stat: " << "mean: " << scale << " std: " << std_ << " stdn: " << std_ / scale << endl;
    for (size_t i = 0; i < 3; i++) {
        cout << "a: " << points1[i].x << " " << points1[i].y << " " << points1[i].z << "  " << scale * points2[i].x << " " << scale * points2[i].y << " " << scale * points2[i].z << endl;
    }
    return scale;
}

void reconstruct(string imagesDirPath){
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
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
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
    detector->detectAndCompute(gray_old, noArray(), keypoints_old, descriptors_old);

    int nZeroIntersect = 0;
    for(int imageIndex = 1; imageIndex < images.size(); imageIndex++) {
    //for(int imageIndex = 1; imageIndex < 2; imageIndex++) {
    //for(int imageIndex = 33; imageIndex < 35; imageIndex++) {
    //for(int imageIndex = 24; imageIndex < 27; imageIndex++) {
    //for(int imageIndex = 1; imageIndex < 7; imageIndex++) {

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
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {      
                good_matches.push_back(knn_matches[i][0]);
                int inx1 = knn_matches[i][0].queryIdx;
                points1.push_back(keypoints_old[inx1].pt);
                int inx2 = knn_matches[i][0].trainIdx;
                points2.push_back(keypoints[inx2].pt);
                
            }
        }
        //-- Draw matches
        
        //drawMatches(image_old, keypoints_old, image, keypoints, good_matches, img_matches, Scalar::all(-1),
        //            Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //imwrite("./tmp/" + to_string(imageIndex) + ".png", img_matches);

        //https://python.hotexamples.com/examples/cv2/-/findEssentialMat/python-findessentialmat-function-examples.html

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


        

        cout << "inliersMask : " << countNonZero(inliersMask) << endl;

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
        for (size_t i = 0; i < good_matches_Inliers.size(); i++) {
            if (mask2.at<unsigned char>(i)) { 
                good_matches_2.push_back(good_matches_Inliers[i]);
                triangulatedPoints.col(i).copyTo(triangulatedPoints2.col(index2));
                points1_2.push_back(points1Inliers[i]);
                points2_2.push_back(points2Inliers[i]);
                index2++;
            }
        }

        

        
        //drawMatches(image_old.clone(), keypoints_old, image.clone(), keypoints, good_matches_2, img_matches, Scalar::all(-1),
        //            Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //imwrite("./tmp/" + to_string(imageIndex) + ".png", img_matches);

        //https://github.com/opencv/opencv/blob/8b4fa2605e1155bbef0d906bb1f272ec06d7796e/modules/calib3d/test/test_cameracalibration.cpp#L1513

        
        convertPointsFromHomogeneous(triangulatedPoints2.t(), triangulatedPoints3d);

        vector<Point3f> triangulatedPoints3d_rotated;
        rotateAndShiftPoints(triangulatedPoints3d,
                    R,
                    t,
                    triangulatedPoints3d_rotated);
        //for (size_t i = 0; i < good_matches_2.size(); i++) {
        //    float xz = (points1_2[i].x - CAMERA_MATRIX.at<float>(0, 2)) / CAMERA_MATRIX.at<float>(0, 0);
        //    float yz = (points1_2[i].y - CAMERA_MATRIX.at<float>(1, 2)) / CAMERA_MATRIX.at<float>(1, 1);
        //    float xz_3d = triangulatedPoints3d[i].x / triangulatedPoints3d[i].z;
        //    float yz_3d = triangulatedPoints3d[i].y / triangulatedPoints3d[i].z;
        //    cout << "z: " << xz << " " << yz << "   " << xz_3d << " " << yz_3d << endl;
        //}
        //for (size_t i = 0; i < good_matches_2.size(); i++) {
        //    float xz = (points2_2[i].x - CAMERA_MATRIX.at<float>(0, 2)) / CAMERA_MATRIX.at<float>(0, 0);
        //    float yz = (points2_2[i].y - CAMERA_MATRIX.at<float>(1, 2)) / CAMERA_MATRIX.at<float>(1, 1);
        //    float xz_3d = triangulatedPoints3d_rotated[i].x / triangulatedPoints3d_rotated[i].z;
        //    float yz_3d = triangulatedPoints3d_rotated[i].y / triangulatedPoints3d_rotated[i].z;
        //    cout << "zr: " << xz << " " << yz << "   " << xz_3d << " " << yz_3d << endl;
        //}
        
        cout << "ponits1 : " << points1.size() << endl;
        cout << "good_matches : " << good_matches.size() << endl;
        cout << "good_matches_Inliers : " << good_matches_Inliers.size() << endl;
        cout << "mask2 : " << countNonZero(mask2) << endl;
        cout << "good_matches_2 : " << good_matches_2.size() << endl;
        cout << "points1Inliers: " << points1Inliers.size() << endl;
        cout << "triangulatedPoints: " << triangulatedPoints.rows <<  " " << triangulatedPoints.cols << endl;
        cout << "triangulatedPoints2: " << triangulatedPoints2.size << endl; // 4 x 247
        cout << "triangulatedPoints3d: " << triangulatedPoints3d.size() << endl; //247
        cout << "triangulatedPoints3d[0]: " << triangulatedPoints3d[0] << endl;
        


        int nCrossClouds = 0;
        if (imageIndex == 1) {
            pointsCloudTotal.insert(pointsCloudTotal.end(), triangulatedPoints3d.begin(), triangulatedPoints3d.end() );
        } else {
            // find cross of two sequental point clouds:
            vector<Point3f> triangulatedPoints3d_intersection;
            vector<Point3f> triangulatedPoints3d_old_intersection;
            //for (size_t j = 0; j < good_matches_2_old.size(); j++) {
            //    cout << "gm_old: " << good_matches_2_old[j].trainIdx << endl;
            //}
            //for (size_t i = 0; i < good_matches_2.size(); i++) {
            //    cout << "gm: " << good_matches_2[i].queryIdx << endl;
            //}
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
        }
        cout << "nCrossClouds : " << nCrossClouds << endl;
        

        //cout << "R_old =" << R_old << endl;
        //cout << "R =" << R << endl;


        //if (imageIndex > 1) {
        //    cout << "tp3d_old[0]: " << triangulatedPoints3d_old[0].x << " " << triangulatedPoints3d_old[0].y << " " << triangulatedPoints3d_old[0].z << endl;
        //    cout << "tp3d[0]: " << triangulatedPoints3d[0].x << " " << triangulatedPoints3d[0].y << " " << triangulatedPoints3d[0].z << endl;
        //}

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
}

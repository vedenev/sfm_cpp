#include "reconstruct.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


void reconstruct(string imagesDirPath){
    vector<String> images;
    glob(imagesDirPath, images);
    
    //int nfeatures = 0;
    //int nOctaveLayers = 3;
    //double contrastThreshold = 0.04;
    //double edgeThreshold = 10;
    //double sigma = 1.6;

    int nfeatures = 0;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.004;
    double edgeThreshold = 3;
    double sigma = 1.6;

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

    for(int imageIndex = 1; imageIndex < images.size(); imageIndex++) {
    //for(int imageIndex = 1; imageIndex < 2; imageIndex++) {

        
        
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


        cout << imageIndex << endl;

        cout << "inliersMask : " << countNonZero(inliersMask) << endl;

        Mat R, t, triangulatedPoints, mask2;
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
        //Mat column = Mat::zeros(Size(1, 4), CV_32FC1);
        for (size_t i = 0; i < good_matches_Inliers.size(); i++) {
            if (mask2.at<unsigned char>(i)) { 
                good_matches_2.push_back(good_matches_Inliers[i]);
                triangulatedPoints.col(i).copyTo(triangulatedPoints2.col(index2));
                index2++;
            }
        }

        //https://github.com/opencv/opencv/blob/8b4fa2605e1155bbef0d906bb1f272ec06d7796e/modules/calib3d/test/test_cameracalibration.cpp#L1513

        vector<Point3f> triangulatedPoints3d;
        convertPointsFromHomogeneous(triangulatedPoints2.t(), triangulatedPoints3d);

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
        if (imageIndex > 1) {
            // find cross of two sequental point clouds:
            for (size_t i = 0; i < good_matches_2.size(); i++) {
                for (size_t j = 0; j < good_matches_2_old.size(); j++) {
                    if (good_matches_2_old[j].trainIdx == good_matches_2_old[i].queryIdx) {
                        nCrossClouds++;
                    }
                }
            }
        }
        cout << "nCrossClouds : " << nCrossClouds << endl;
        cout << endl;


        keypoints_old = keypoints;
        keypoints.clear();

        descriptors_old = descriptors;
        image_old = image;
        

        knn_matches.clear();
        good_matches.clear();
        good_matches_Inliers.clear();
        good_matches_2_old = good_matches_2;
        good_matches_2.clear();
 
    }
    image_old.release();
    gray_old.release();
    image.release();
    gray.release();
    descriptors_old.release();
    descriptors.release();
    img_matches.release();

}
#include "reconstruct.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


void reconstruct(string imagesDirPath){
    vector<String> images;
    glob(imagesDirPath, images);
    
    Ptr<Feature2D> detector = cv::SiftFeatureDetector::create();
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

    Mat CAMERA_MATRIX = (Mat_<float>(3,3) << 1487.886270357746, 0, 547.1524898799552,
                                0, 1488.787677381604, 979.9460018614599,
                                0, 0, 1);
    // to get CAMERA_MATRIX: calibrate camera with:
    // sfm --calibrate path_to_images

    double PROBABILITY = 0.999;
    double THRESHOLD = 1.0;
    double DISTANCE_THRESHOLD = 10.0;

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




        Mat R, t, triangulatedPoints;
        recoverPose	(essentialMatrix,
            points1,
            points2,
            CAMERA_MATRIX,
            R,
            t,
            DISTANCE_THRESHOLD,
            inliersMask,
            triangulatedPoints);
        
        cout << imageIndex << endl;
        cout << t << endl;
        cout << R << endl;
        //cout << triangulatedPoints.rows <<  " " << triangulatedPoints.cols << endl;
        cout << format(triangulatedPoints.t(), Formatter::FMT_PYTHON) << endl;
        cout << endl;


        keypoints_old = keypoints;
        keypoints.clear();

        descriptors_old = descriptors;
        image_old = image;
        

        knn_matches.clear();
        good_matches.clear();
 
    }
    image_old.release();
    gray_old.release();
    image.release();
    gray.release();
    descriptors_old.release();
    descriptors.release();
    img_matches.release();

}
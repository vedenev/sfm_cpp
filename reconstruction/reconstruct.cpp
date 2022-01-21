#include "reconstruct.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


void reconstruct(string imagesDirPath){
    vector<String> images;
    glob(imagesDirPath, images);
    Mat image1, gray1;
    Mat image2, gray2;
    image1 = imread(images[26]);
    cvtColor(image1, gray1, COLOR_BGR2GRAY);
    image2 = imread(images[27]);
    cvtColor(image2, gray2, COLOR_BGR2GRAY);

    //https://stackoverflow.com/questions/44081455/c-siftfeaturedetector-is-not-loading

    //int nfeatures = 0;
    //int nOctaveLayers = 3;
    //double contrastThreshold = 0.04;
    //double edgeThreshold = 10;
    //double sigma = 1.6;

    //int nfeatures = 0;
    //int nOctaveLayers = 3;
    //double contrastThreshold = 0.004;
    //double edgeThreshold = 3;
    //double sigma = 0.8;

    //Ptr<Feature2D> detector = cv::SiftFeatureDetector::create(nfeatures,
    //                                                        nOctaveLayers,
    //                                                        contrastThreshold,
    //                                                        edgeThreshold,
    //                                                        sigma);
    Ptr<Feature2D> detector = cv::SiftFeatureDetector::create();
    vector<KeyPoint> keypoints1;
    Mat descriptors1;
    //detector->detect(gray1, keypoints1);
    detector->detectAndCompute(gray1, noArray(), keypoints1, descriptors1);

    vector<KeyPoint> keypoints2;
    Mat descriptors2;
    detector->detectAndCompute(gray2, noArray(), keypoints2, descriptors2);

    //cout << keypoints1.size() << endl;
    //drawKeypoints(gray1, keypoints1, image1);
    //imshow("1", image1);
    //waitKey();
    
    
    //https://docs.opencv.org/4.4.0/d5/d6f/tutorial_feature_flann_matcher.html
    //https://www.analyticsvidhya.com/blog/2019/10/detailed-guide-powerful-sift-technique-image-matching-python/
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector<vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );

     //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Draw matches
    Mat img_matches;
    drawMatches(image1, keypoints1, image2, keypoints2, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    imshow("Good Matches", img_matches );
    waitKey();
    

}
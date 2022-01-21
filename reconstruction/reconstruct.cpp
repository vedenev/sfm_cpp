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
    
    

    for(int imageIndex = 0; imageIndex < (images.size() - 1); imageIndex++) {

        Mat image1, gray1;
        Mat image2, gray2;
        vector<KeyPoint> keypoints1;
        Mat descriptors1;
        vector<KeyPoint> keypoints2;
        Mat descriptors2;
        Mat img_matches;
        vector<vector<DMatch> > knn_matches;
        std::vector<DMatch> good_matches;

        image1 = imread(images[imageIndex]);
        cvtColor(image1, gray1, COLOR_BGR2GRAY);
        image2 = imread(images[imageIndex + 1]);
        cvtColor(image2, gray2, COLOR_BGR2GRAY);

        

        
        
        
        detector->detectAndCompute(gray1, noArray(), keypoints1, descriptors1);

        
        detector->detectAndCompute(gray2, noArray(), keypoints2, descriptors2);

        
        
        matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );

        //-- Filter matches using the Lowe's ratio test
        
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        //-- Draw matches
        
        drawMatches(image1, keypoints1, image2, keypoints2, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        imwrite("./tmp/" + to_string(imageIndex) + ".png", img_matches);
 
    }

}
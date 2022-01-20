#include "calibrate.hpp"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
using namespace cv;
using namespace std;


// https://learnopencv.com/camera-calibration-using-opencv/

int CHECKERBOARD[2]{6,9};

void calibrate(string imagesDirPath) {
    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpoints;
    vector<Point3f> objp;
    int width = 0;
    int height = 0;
    

    setChessboardCoordinates(objp);
    processImages(imagesDirPath, objpoints, imgpoints, objp, width, height);
    
    Mat cameraMatrix,distCoeffs,R,T;
    auto imageSize = Size(height, width);

    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    int calibrationOtions = CALIB_FIX_K1 + CALIB_FIX_K2 + CALIB_FIX_K3 +
                                CALIB_ZERO_TANGENT_DIST;
    double residual = calibrateCamera(objpoints,
                                    imgpoints,
                                    imageSize,
                                    cameraMatrix,
                                    distCoeffs,
                                    R,
                                    T,
                                    calibrationOtions);

    std::cout << "residual : " << residual << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    //std::cout << "Rotation vector : " << R << std::endl;
    //std::cout << "Translation vector : " << T << std::endl;

}

void setChessboardCoordinates(vector<Point3f> & objp) {
    // Defining the world coordinates for 3D points
    for(int i = 0; i<CHECKERBOARD[1]; i++)
    {
        for(int j = 0; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }


}

void processImages(string imagesDirPath,
                vector<vector<Point3f>> & objpoints,
                vector<vector<Point2f>> & imgpoints,
                vector<Point3f> & objp,
                int & width,
                int & height) {
    
    vector<String> images;
    glob(imagesDirPath, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;

    auto options = CALIB_CB_ADAPTIVE_THRESH |
                    CALIB_CB_FAST_CHECK |
                    CALIB_CB_NORMALIZE_IMAGE;
    auto size = cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]);
    cv::TermCriteria criteria(cv::TermCriteria::Type::EPS |
                    cv::TermCriteria::Type::MAX_ITER,
                        30,
                        0.001);

    // Looping over all the images in the directory
    for(int i = 0; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image
        // then success = true  
        success = cv::findChessboardCorners(gray, size, corner_pts, options);

        /* 
            * If desired number of corner are detected,
            * we refine the pixel coordinates and display 
            * them on the images of checker board
        */
        if(success) {
            
            
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, 
                            corner_pts,
                            cv::Size(11,11),
                            cv::Size(-1,-1),criteria);
            
            
            
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
    }

    width = gray.cols;
    height = gray.rows;

}
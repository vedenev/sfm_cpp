#ifndef DELETEDIRECTORYCONTENTS_HPP_INCLUDED
#define DELETEDIRECTORYCONTENTS_HPP_INCLUDED

#include <opencv2/opencv.hpp>
#include <string>
using namespace cv;
using namespace std;

void calibrate(string imagesDirPath);
void setChessboardCoordinates(vector<Point3f> & objp);
void processImages(string imagesDirPath,
                vector<vector<Point3f>> & objpoints,
                vector<vector<Point2f>> & imgpoints,
                vector<Point3f> & objp,
                int & width,
                int & height);

#endif
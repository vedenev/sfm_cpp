#ifndef RECONSTRUCT_HPP_INCLUDED
#define RECONSTRUCT_HPP_INCLUDED

#include <string>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

void rotateAndShiftPoints(vector<Point3f> & pointsInput,
                    Mat & R,
                    Mat & t,
                    vector<Point3f> & pointsOutput);

float Point3fNorm(Point3f & point);
float findScale(vector<Point3f> & points1, vector<Point3f> & points2);
void scalePoints(vector<Point3f> & points, float scale);
void wrtitePly(string plyPath,
 vector<Point3f> & pointsCloudTotal,
 vector<Vec3b> & pointsCloudTotalColors);

void reconstruct(string imagesDirPath);

#endif
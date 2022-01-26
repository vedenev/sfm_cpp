#include <iostream>
#include <opencv2/opencv.hpp>
#include "utils/video2frames.hpp"
#include "calibration/calibrate.hpp"
#include "reconstruction/reconstruct.hpp"

using namespace std;
using namespace cv;

// how to configure jsons:
//https://github.com/Cuda-Chen/opencv-config-with-vscode

void printUsage() {
	cout << "   usage:" << endl;
	cout <<  endl;
	cout << "sfm --video2frames path_to_video" << endl;
	cout << "   converts video to frames in png format" << endl; 
	cout << "   png will be stored in to a folder with name of the video file" << endl;
	cout <<  endl;
	cout << "sfm --calibrate path_to_images" << endl;
	cout << "   calibrate camera" << endl;
	cout << "   path_to_images is directory with images" << endl;
	cout << "   images are 6x9 chessboard images" << endl;
	cout << "   images are png" << endl;
	cout << "   then it prints calibration parameters" << endl;
	cout <<  endl;
	cout << "sfm --reconstruct path_to_images" << endl;
	cout << "   reconstruct 3d scene" << endl;
	cout << "   path_to_images is directory with images" << endl;
}

int main(int argc, char *argv[]){

	if (argc != 3) {
		printUsage();
		return 0;
	}

	string flag(argv[1]);

	if (flag == "--video2frames") {
		video2frames(argv[2]);
		return 0;
	}

	if (flag == "--calibrate") {
		calibrate(argv[2]);
		return 0;
	}

	if (flag == "--reconstruct") {
		reconstruct(argv[2]);
		return 0;
	}

	printUsage();

	return 0;
}
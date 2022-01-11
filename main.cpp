#include<iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// how to configure jsons:
//https://github.com/Cuda-Chen/opencv-config-with-vscode

int main(){
    Mat image;
	VideoCapture capture;
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);
	capture.open(0);

	while(true) {
		capture >> image;
		imshow("test2", image);

		int c = waitKey(10);
		if (c == 27) break;
	}

	return 0;
}
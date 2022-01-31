#include "video2frames.hpp"
#include "deleteDirectoryContents.hpp"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iomanip>
using namespace cv;
namespace fs = std::filesystem;
using namespace std;


void video2frames(string videoPath) {
    VideoCapture reader(videoPath);

    if(!reader.isOpened()){
        cout << "Error opening video file: " << videoPath << endl;
    }

    auto videoPathFs = fs::path(videoPath);
    auto outputDirecory = videoPathFs.parent_path() / videoPathFs.stem();
    if(fs::is_directory(outputDirecory)) {
        deleteDirectoryContents(outputDirecory);
    } else {
        fs::create_directory(outputDirecory);
    }

    Mat frame;
    int frameIndex = 0;
    cout << "convert " << videoPathFs.filename() << " to frames..." << endl;
    while(true){
        reader >> frame;
        
        if (frame.empty())
        break;

        stringstream fileNameTmpBase;
        fileNameTmpBase << setw(4) << setfill('0') << frameIndex << ".png";
        auto fileNameTmp = outputDirecory / fileNameTmpBase.str();
        imwrite(fileNameTmp, frame);

        frameIndex++;
    }
    cout << "convertion done" << endl;
 
  reader.release();


}
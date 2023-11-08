#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <cmath>
#include <vector>

//import classes
#include "../visualizer/Visualizer.h"
#include "../blobDetector/BlobDetector.h"
#include "Scripts.h"

using namespace cv;

void Scripts::cameraRunner(int cameraNumber, BlobDetector& blobDetector){

    std::vector<Blob> blobVector;
    VideoCapture cap(cameraNumber);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return;
    }

    int frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = 30;  // Frames per second
    std::string outputFilename = "../videos/output_video.mp4";

    // Define the codec and create a VideoWriter object
    cv::VideoWriter videoWriter(outputFilename, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(frameWidth, frameHeight));

    if (!videoWriter.isOpened()) {
        std::cerr << "Error: Could not create the VideoWriter object." << std::endl;
        return;
    }

    Mat frame, dst;
    int counter = 0;
    while(true){
        cap >> frame;
        if(frame.empty()){
            std::cout << "End of video." << std::endl;
            break;
        }
        if(counter == 0){
            // Calibrate
            blobDetector.calibrate(frame);
            counter++;
        }

        // Detect
        blobVector = blobDetector.detect(frame,dst);
        imshow("Frame", frame);

        //Writing frames to a video, use visualizer class to save individual frames
        videoWriter.write(frame);


        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
    videoWriter.release();  // Release the VideoWriter
    cap.release();  // Release the VideoCapture
}


void Scripts::videoRunner(const std::string&fileName, BlobDetector& blobDetector){
    //File name of file in videos folder
    //Child blob detector class/object for various blob detection types

    std::string videoPath = "../videos/" + fileName;
    VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the video file." << std::endl;
        return;
    }

    int frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frameHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = 30;  // Frames per second
    std::string outputFilename = "../videos/output_video.mp4";


    // Define the codec and create a VideoWriter object
    cv::VideoWriter videoWriter(outputFilename, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(frameWidth, frameHeight));

    if (!videoWriter.isOpened()) {
        std::cerr << "Error: Could not create the VideoWriter object." << std::endl;
        return;
    }

    Mat frame, dst;
    int counter = 0;
    while(true){
        cap >> frame;
        if(frame.empty()){
            std::cout << "End of video." << std::endl;
            break;
        }
        if(counter == 0){
            // Calibrate function
            blobDetector.calibrate(frame);
            counter++;
        }

        // Detect function
        blobDetector.detect(frame,dst);
        imshow("Frame", frame);

        //Writing frames to a video, use visualizer class to save individual frames
        videoWriter.write(frame);


        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
    videoWriter.release();  // Release the VideoWriter
    cap.release();  // Release the VideoCapture
}

void Scripts::imageRunner(const std::string&fileName, BlobDetector& blobDetector){
    //File name of file in images folder
    //Child blob detector class/object for various blob detection types

    std::string imagePath = "../images/" + fileName;

    cv::Mat frame = cv::imread(imagePath);
    cv::Mat dst;

    //Calibrate function    
    blobDetector.calibrate(frame);

    // Detect function
    blobDetector.detect(frame,dst);
    
    imshow("Detected Frame", frame);

    Visualizer::saveFrame(frame, "../images/output_image.jpeg");

    char key = (char) waitKey(30);
}

#include "uas_benchmarking/UASBenchmarking.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <cmath>
#include <iostream>
#include "uas_computer_vision/BlobDetector.h"
#include "uas_computer_vision/BasicBlobDetector.h"
#include <filesystem>

void UASBenchmarking::run(const std::string& datasetPath, const std::string& datasetLabelsPath, BlobDetector& blobDetector){
    std::filesystem::path path(datasetPath);
    std::vector<std::string> imagePaths;
    double totalError = 0;

    // Gather all the JPG file paths
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG" || entry.path().extension() == ".png") {
            imagePaths.push_back(entry.path().string());
        }
    }

    // Sort the file paths
    std::sort(imagePaths.begin(), imagePaths.end());

    // Process the images in alphabetical order
    for (const auto& imagePath : imagePaths) {
        cv::Mat frame = cv::imread(imagePath);
        cv::Mat dst;
        blobDetector.detect(frame, dst);
        
        // Load corresponding label image
        std::filesystem::path labelPath = std::filesystem::path(datasetLabelsPath) / std::filesystem::path(imagePath).filename();
        labelPath.replace_extension(".png");

        // Check if both images are of the same size and type
        cv::Mat labelImage = cv::imread(labelPath.string(), cv::IMREAD_GRAYSCALE);  // Load label as grayscale

        if(dst.empty() || labelImage.empty()) {
            std::cerr << "Error loading images." << std::endl;
            return;  // or handle error appropriately
        }

        // Ensure both images are of the same type
        dst.convertTo(dst, CV_8U);
        labelImage.convertTo(labelImage, CV_8U);

        if(dst.size() != labelImage.size() || dst.type() != labelImage.type()) {
            std::cerr << "Image sizes or types mismatch." << std::endl;
            return;
        }

        // Subtract the images
        cv::Mat diffImage;
        cv::absdiff(dst, labelImage, diffImage);

        // Count non-zero pixels
        int differingPixels = cv::countNonZero(diffImage);
        std::cout<<"Image Error: "<<differingPixels<<std::endl;
        totalError += differingPixels;
    }

    std::cout << "Total error: " << totalError << std::endl;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting Computer Vision Benchmarking Node..." << std::endl;
	std::string datasetPath = "/home/nic/dev/ATTACKS-SW/Datasets/dataSet1/originalImages";
    std::string datasetLabelsPath = "/home/nic/dev/ATTACKS-SW/Datasets/dataSet1/binaryImages";
    BasicBlobDetector basicBlobDetector;
    UASBenchmarking benchmarking;
    benchmarking.run(datasetPath, datasetLabelsPath, basicBlobDetector);
    std::cout << "done" << std::endl;
	return 0;
}
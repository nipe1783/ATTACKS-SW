#include "Benchmarking.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>
#include "../blobDetector/BlobDetector.h"
#include "../blobDetector/BasicBlobDetector.h"
#include "../blobDetector/VaryingLightBlobDetector.h"
#include "../visualizer/Visualizer.h"
#include <filesystem>
#include <chrono>

void Benchmarking::runBasic(const std::string& datasetPath, const std::string& datasetLabelsPath, BasicBlobDetector& blobDetector){
    std::filesystem::path path(datasetPath);
    std::vector<std::string> imagePaths;
    double totalError = 0;
    double totalTime = 0;

    // Gather all the PNG file paths
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".PNG") {
            imagePaths.push_back(entry.path().string());
        }
    }
    // Sort the file paths
    std::sort(imagePaths.begin(), imagePaths.end());

    // Calibrate
    cv::Mat frame = cv::imread(imagePaths[0]);
    blobDetector.calibrate(frame);

    std::ofstream data;
    data.open("../benchmarking/data/dataBasic.csv");
    data << "Execution Time (ms),Pixel Error \n";

    // Process the images in alphabetical order
    for (const auto& imagePath : imagePaths) {
        cv::Mat frame = cv::imread(imagePath);
        cv::Mat dst;

        auto start = std::chrono::high_resolution_clock::now();
        blobDetector.detect(frame, dst);
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time = stop - start;

        std::cout << "Execution time: " << time.count() << " ms" << std::endl;
        totalTime += time.count();

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
        Visualizer::twoFrame(dst, labelImage);
        cv::waitKey(0);
        int differingPixels = cv::countNonZero(diffImage);
        std::cout<<"Image Error: "<<differingPixels<<std::endl;
        totalError += differingPixels;

        data << time.count() << "," << differingPixels << "\n";
    }

    data.close();

    std::cout << "Total error: " << totalError << std::endl;
    std::cout << "Total execution time: " << totalTime << " ms" << std::endl;
}

void Benchmarking::runDoG(const std::string& datasetPath, const std::string& datasetLabelsPath, DoGBlobDetector& blobDetector){
    std::filesystem::path path(datasetPath);
    std::vector<std::string> imagePaths;
    double totalError = 0;
    double totalTime = 0;

    // Gather all the PNG file paths
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".PNG") {
            imagePaths.push_back(entry.path().string());
        }
    }
    // Sort the file paths
    std::sort(imagePaths.begin(), imagePaths.end());

    // Calibrate
    cv::Mat frame = cv::imread(imagePaths[0]);
    blobDetector.calibrate(frame);

    std::ofstream data;
    data.open("../benchmarking/data/dataDoG.csv");
    data << "Execution Time (ms),Pixel Error \n";

    // Process the images in alphabetical order
    for (const auto& imagePath : imagePaths) {
        cv::Mat frame = cv::imread(imagePath);
        cv::Mat dst;

        auto start = std::chrono::high_resolution_clock::now();
        blobDetector.detect(frame, dst);
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time = stop - start;

        std::cout << "Execution time: " << time.count() << " ms" << std::endl;
        totalTime += time.count();

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
        Visualizer::twoFrame(dst, labelImage);
        cv::waitKey(0);
        int differingPixels = cv::countNonZero(diffImage);
        std::cout<<"Image Error: "<<differingPixels<<std::endl;
        totalError += differingPixels;

        data << time.count() << "," << differingPixels << "\n";
    }

    data.close();

    std::cout << "Total error: " << totalError << std::endl;
    std::cout << "Total execution time: " << totalTime << " ms" << std::endl;
}

void Benchmarking::runVarying(const std::string& datasetPath, const std::string& datasetLabelsPath, VaryingLightBlobDetector& blobDetector){
    std::filesystem::path path(datasetPath);
    std::vector<std::string> imagePaths;
    double totalError = 0;
    double totalTime = 0;

    // Gather all the PNG file paths
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".PNG") {
            imagePaths.push_back(entry.path().string());
        }
    }
    // Sort the file paths
    std::sort(imagePaths.begin(), imagePaths.end());

    // Calibrate
    cv::Mat frame = cv::imread(imagePaths[0]);
    blobDetector.calibrate(frame);

    std::ofstream data;
    data.open("../benchmarking/data/dataVarying.csv");
    data << "Execution Time (ms),Pixel Error \n";
    
    // Process the images in alphabetical order
    for (const auto& imagePath : imagePaths) {
        cv::Mat frame = cv::imread(imagePath);
        cv::Mat dst;

        auto start = std::chrono::high_resolution_clock::now();
        blobDetector.detect(frame, dst);
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time = stop - start;

        std::cout << "Execution time: " << time.count() << " ms" << std::endl;
        totalTime += time.count();
        
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
        Visualizer::twoFrame(dst, labelImage);
        cv::waitKey(0);
        int differingPixels = cv::countNonZero(diffImage);
        std::cout<<"Image Error: "<<differingPixels<<std::endl;
        totalError += differingPixels;

        data << time.count() << "," << differingPixels << "\n";
    }

    data.close();

    std::cout << "Total error: " << totalError << std::endl;
    std::cout << "Total execution time: " << totalTime << " ms" << std::endl;
}

Benchmarking::Benchmarking()
{
}

Benchmarking::~Benchmarking()
{
}

// double Benchmarking::run(std::string datasetPath, std::string datasetLabelsPath, BlobDetector blobDetector, double gamma, int sigma1, int sigma2, double alpha, double tau, double areaThreshold, cv::Scalar lowerBound, cv::Scalar upperBound){
//     std::filesystem::path path(datasetPath);
//     std::vector<std::string> imagePaths;
//     double totalError = 0;
//     // Gather all the JPG file paths
//     for (const auto & entry : std::filesystem::directory_iterator(path)) {
//         if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG") {
//             imagePaths.push_back(entry.path().string());
//         }
//     }

//     // Sort the file paths
//     std::sort(imagePaths.begin(), imagePaths.end());

//     // Process the images in alphabetical order
//     for (const auto& imagePath : imagePaths) {
//         cv::Mat frame = cv::imread(imagePath);
//         cv::Mat dst;
//         blobDetector.detect1(frame, dst, lowerBound, upperBound, gamma, sigma1, sigma2, alpha, tau, areaThreshold);
        
//         // Load corresponding label image
//         std::filesystem::path labelPath = std::filesystem::path(datasetLabelsPath) / std::filesystem::path(imagePath).filename();
//         labelPath.replace_extension(".png");

//         // Check if both images are of the same size and type
//         cv::Mat labelImage = cv::imread(labelPath.string(), cv::IMREAD_GRAYSCALE);  // Load label as grayscale

//         if(dst.empty() || labelImage.empty()) {
//             std::cerr << "Error loading images." << std::endl;
//             return 0;  // or handle error appropriately
//         }

//         // Ensure both images are of the same type
//         dst.convertTo(dst, CV_8U);
//         labelImage.convertTo(labelImage, CV_8U);

//         if(dst.size() != labelImage.size() || dst.type() != labelImage.type()) {
//             std::cerr << "Image sizes or types mismatch." << std::endl;
//             return 0;  // or handle error appropriately
//         }


//         // Compare the two images
//         // Visualizer::saveTwoFrame(dst, labelImage, "../output/total/" + std::filesystem::path(imagePath).filename().string() + "-gamma:" + std::to_string(gamma) + "-sigma1:" + std::to_string(sigma1) + "-sigma2:" + std::to_string(sigma2) + ".jpg");
//         // Subtract the images
//         cv::Mat diffImage;
//         cv::absdiff(dst, labelImage, diffImage);

//         // Count non-zero pixels
//         int differingPixels = cv::countNonZero(diffImage);
//         totalError += differingPixels;
//     }

//     // std::cout << "Total error: " << totalError << std::endl;
//     return totalError;
// }

// void Benchmarking::runSimple(std::string datasetPath, BlobDetection blobDetector){
//     std::filesystem::path path(datasetPath);
//     std::string imagePath;
//     for (const auto & entry : std::filesystem::directory_iterator(path)) {
//         if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG") {

//             imagePath = entry.path().string();
//             std::cout << "Processing image: " << imagePath << std::endl;

//             cv::Mat frame = cv::imread(imagePath);
//             cv::Mat dst;

//             blobDetector.simpleDetect(frame, dst);
//         }
//     }
// }
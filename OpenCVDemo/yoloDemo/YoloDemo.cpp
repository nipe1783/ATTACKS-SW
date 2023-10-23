#include <opencv2/opencv.hpp>
#include <vector>

std::tuple<std::vector<int>, std::vector<double>, std::vector<cv::Rect>> unwrap_detection(const cv::Mat& input_image, const cv::Mat& output_data) {
    std::vector<int> class_ids;
    std::vector<double> confidences;
    std::vector<cv::Rect> boxes;

    int rows = output_data.rows;

    int image_width = input_image.cols;
    int image_height = input_image.rows;

    double x_factor = static_cast<double>(image_width) / 640.0;
    double y_factor = static_cast<double>(image_height) / 640.0;

    for (int r = 0; r < rows; ++r) {
        float confidence = output_data.at<float>(r, 4);
        if (confidence >= 0.4) {
            cv::Point class_number;
            double class_score;
            cv::minMaxLoc(output_data.row(r).colRange(5, output_data.cols), nullptr, &class_score, nullptr, &class_number);

            int class_id = class_number.x;
            if (output_data.at<float>(r, 5 + class_id) > 0.25) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id);

                float x = output_data.at<float>(r, 0);
                float y = output_data.at<float>(r, 1);
                float w = output_data.at<float>(r, 2);
                float h = output_data.at<float>(r, 3);

                int left = static_cast<int>((x - 0.5 * w) * x_factor);
                int top = static_cast<int>((y - 0.5 * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    return std::make_tuple(class_ids, confidences, boxes);
}

void detectImage(const std::string& imagePath) {
    std::vector<int> class_ids;
    std::vector<double> confidences;
    std::vector<cv::Rect> boxes;
    std::tuple<std::vector<int>, std::vector<double>, std::vector<cv::Rect>> result;

    // Load the model
    auto net = cv::dnn::readNet("/Users/nicolasperrault/Desktop/ATTACKS-SW/OpenCVDemo/yoloDemo/yolov5s.onnx");
    
    // Read the image
    cv::Mat img = cv::imread(imagePath);
    if (img.empty()) {
        std::cerr << "Could not read the image: " << imagePath << std::endl;
        return;
    }

    // Preprocess the image
    cv::Mat blob;
    const double scalefactor = 1.0/255.0;
    cv::Size size = cv::Size(640, 640);
    cv::Scalar mean = cv::Scalar(0, 0, 0);
    bool swapRB = true;

    cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB);
    net.setInput(blob);

    // Forward pass
    std::vector<cv::Mat> predictions;
    net.forward(predictions, net.getUnconnectedOutLayersNames());

    // Extract the first prediction
    const cv::Mat &output = predictions[0];
    
    // ... handle the output as necessary ...
    result = unwrap_detection(img, output);
    int x = 3;
}
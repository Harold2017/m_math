//
// Created by Harold on 2021/11/30.
//

// reference: https://github.com/vjgpt/Object-Detection

#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <unordered_map>

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float maskThreshold = 0.3; // Mask threshold
std::vector<std::string> classes;
std::vector<cv::Scalar> colors;

// Draw the predicted bounding box, colorize and show the mask on the image
void drawBox(cv::Mat& frame, int classId, float conf, cv::Rect box, cv::Mat& objectMask)
{
    //Draw a rectangle displaying the bounding box
    cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x+box.width, box.y+box.height), cv::Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    box.y = cv::max(box.y, labelSize.height);
    cv::rectangle(frame, cv::Point(box.x, box.y - round(1.5*labelSize.height)), cv::Point(box.x + round(1.5*labelSize.width), box.y + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(box.x, box.y), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);

    cv::Scalar color = colors[classId%colors.size()];
    // Comment the above line and uncomment the two lines below to generate different instance colors
    //int colorInd = rand() % colors.size();
    //Scalar color = colors[colorInd];

    // Resize the mask, threshold, color and apply it on the image
    cv::resize(objectMask, objectMask, cv::Size(box.width, box.height));
    cv::Mat mask = (objectMask > maskThreshold);
    cv::Mat coloredRoi = (0.3 * color + 0.7 * frame(box));
    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    // Draw the contours on the image
    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    mask.convertTo(mask, CV_8U);
    cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(coloredRoi, contours, -1, color, 5, cv::LINE_8, hierarchy, 100);
    coloredRoi.copyTo(frame(box), mask);
}

// For each frame, extract the bounding box and mask for each detected object
void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
{
    cv::Mat outDetections = outs[0];
    cv::Mat outMasks = outs[1];

    // Output size of masks is NxCxHxW where
    // N - number of detected boxes
    // C - number of classes (excluding background)
    // HxW - segmentation shape
    const int numDetections = outDetections.size[2];
    const int numClasses = outMasks.size[1];

    outDetections = outDetections.reshape(1, outDetections.total() / 7);
    for (int i = 0; i < numDetections; ++i)
    {
        float score = outDetections.at<float>(i, 2);
        if (score > confThreshold)
        {
            // Extract the bounding box
            int classId = static_cast<int>(outDetections.at<float>(i, 1));
            int left = static_cast<int>(frame.cols * outDetections.at<float>(i, 3));
            int top = static_cast<int>(frame.rows * outDetections.at<float>(i, 4));
            int right = static_cast<int>(frame.cols * outDetections.at<float>(i, 5));
            int bottom = static_cast<int>(frame.rows * outDetections.at<float>(i, 6));

            left = cv::max(0, cv::min(left, frame.cols - 1));
            top = cv::max(0, cv::min(top, frame.rows - 1));
            right = cv::max(0, cv::min(right, frame.cols - 1));
            bottom = cv::max(0, cv::min(bottom, frame.rows - 1));
            cv::Rect box = cv::Rect(left, top, right - left + 1, bottom - top + 1);

            // Extract the mask for the object
            cv::Mat objectMask(outMasks.size[2], outMasks.size[3],CV_32F, outMasks.ptr<float>(i,classId));

            // Draw bounding box, colorize and show the mask on the image
            drawBox(frame, classId, score, box, objectMask);

        }
    }
}

struct Mask
{
    cv::Scalar color;
    cv::Rect bounding_box;  // <topleft.x, topleft.y, width, height>
};

struct MaskedImg
{
    cv::Mat img;
    std::unordered_map<int, std::vector<Mask>> masks;
};

// Draw the mask on the image
void drawMask(MaskedImg& mi, int classId, cv::Rect const& box, cv::Mat& objectMask)
{
    cv::Scalar color = colors[classId%colors.size()];
    // Resize the mask, threshold, color and apply it on the image
    cv::resize(objectMask, objectMask, cv::Size(box.width, box.height));
    cv::Mat mask = (objectMask > maskThreshold);
    cv::Mat coloredRoi = (color + 0 * mi.img(box));
    coloredRoi.copyTo(mi.img(box), mask);
    mi.masks[classId].push_back(Mask{ color, box });
}

// get masked img
MaskedImg masked_img(cv::Mat const& frame, std::vector<cv::Mat> const& outs)
{
    cv::Mat outDetections = outs[0];
    cv::Mat outMasks = outs[1];

    // Output size of masks is NxCxHxW where
    // N - number of detected boxes
    // C - number of classes (excluding background)
    // HxW - segmentation shape
    const int numDetections = outDetections.size[2];
    const int numClasses = outMasks.size[1];

    MaskedImg mi;
    mi.img = cv::Mat(frame.rows, frame.cols, frame.type(), cv::Scalar(0));

    outDetections = outDetections.reshape(1, outDetections.total() / 7);
    for (int i = 0; i < numDetections; ++i)
    {
        float score = outDetections.at<float>(i, 2);
        if (score > confThreshold)
        {
            // Extract the bounding box
            int classId = static_cast<int>(outDetections.at<float>(i, 1));
            int left = static_cast<int>(frame.cols * outDetections.at<float>(i, 3));
            int top = static_cast<int>(frame.rows * outDetections.at<float>(i, 4));
            int right = static_cast<int>(frame.cols * outDetections.at<float>(i, 5));
            int bottom = static_cast<int>(frame.rows * outDetections.at<float>(i, 6));

            left = cv::max(0, cv::min(left, frame.cols - 1));
            top = cv::max(0, cv::min(top, frame.rows - 1));
            right = cv::max(0, cv::min(right, frame.cols - 1));
            bottom = cv::max(0, cv::min(bottom, frame.rows - 1));
            cv::Rect box = cv::Rect(left, top, right - left + 1, bottom - top + 1);

            // Extract the mask for the object
            cv::Mat objectMask(outMasks.size[2], outMasks.size[3], CV_32F, outMasks.ptr<float>(i,classId));

            // Colorize and draw the mask on the image
            drawMask(mi, classId, box, objectMask);
        }
    }

    return mi;
}

int main(int argc, char* argv[])
{
    // parser
    const cv::String keys =
        "{help h usage ?   |                                                                                                  | print this message           }"
        "{labels_file l    |../mrcnn/mscoco_labels.names                                                                      | class file                   }"
        "{colors c         |../mrcnn/colors.txt                                                                               | colors file                  }"
        "{text_graph t     |../mrcnn/mask_rcnn_inception_v2_coco_2018_01_28/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt      | text graph file              }"
        "{model_weights m  |../mrcnn/mask_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb                         | model weights file           }"
        "{input i          |<none>                                                                                            | input image for segmentation }"
        "{output o         |<none>                                                                                            | output image                 }"
        ;
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        parser.printMessage();
        exit(0);
    }
    else if (!parser.has("input"))
    {
        printf("invalid arguments\n");
        parser.printMessage();
        exit(1);
    }

    // Load names of classes
    std::string classesFile = parser.get<cv::String>("labels_file");
    std::ifstream ifs(classesFile.c_str());
    std::string line;
    while (std::getline(ifs, line)) classes.push_back(line);

    // Load the colors
    std::string colorsFile = parser.get<cv::String>("colors");
    std::ifstream colorFptr(colorsFile.c_str());
    while (getline(colorFptr, line)) {
        char* pEnd;
        double r, g, b;
        r = strtod (line.c_str(), &pEnd);
        g = strtod (pEnd, nullptr);
        b = strtod (pEnd, nullptr);
        colors.push_back(cv::Scalar(r, g, b, 255.0));
    }

    // Give the configuration and weight files for the model
    cv::String textGraph = parser.get<cv::String>("text_graph");
    cv::String modelWeights = parser.get<cv::String>("model_weights");

    // Load the network
    cv::dnn::Net net = cv::dnn::readNetFromTensorflow(modelWeights, textGraph);
    // follow my gist to build OpenCV with cuDNN: https://gist.github.com/Harold2017/e1c2d50146d02fb461a7e016adb20907
    net.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CUDA);

    cv::String input = parser.get<cv::String>("input");
    cv::String output = parser.get<cv::String>("output");
    cv::Mat frame;
    frame = cv::imread(input, cv::IMREAD_COLOR);
    // auto cap = cv::VideoCapture(input);
    // if (!cap.read(frame))
    // {
    //     printf("error to read input image\n");
    //     exit(1);
    // }
    cv::Mat blob;

    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1.0, cv::Size(frame.cols, frame.rows), cv::Scalar(), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output from the output layers
    std::vector<cv::String> outNames(2);
    outNames[0] = "detection_out_final";
    outNames[1] = "detection_masks";
    std::vector<cv::Mat> outs;
    net.forward(outs, outNames);

    // // Extract the bounding box and mask for each of the detected objects
    // postprocess(frame, outs);

    // // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    // std::vector<double> layersTimes;
    // double freq = cv::getTickFrequency() / 1000;
    // double t = net.getPerfProfile(layersTimes) / freq;
    // cv::String label = cv::format("Mask-RCNN : Inference time for a frame : %.2f ms", t);
    // putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    // // Write the frame with the detection boxes
    // cv::Mat detectedFrame;
    // frame.convertTo(detectedFrame, CV_8U);
    // cv::imwrite(output, detectedFrame);

    // cv::String kWinName = "Mask-RCNN Object detection and Segmentation in OpenCV";
    // cv::namedWindow(kWinName, cv::WINDOW_NORMAL);
    // cv::imshow(kWinName, frame);
    // cv::waitKey();

    auto mi = masked_img(frame, outs);
    cv::imshow("masked img", mi.img);
    cv::waitKey();

    cv::Mat rgba;
    cv::cvtColor(mi.img, rgba, cv::COLOR_RGB2RGBA);
    // find all black pixel and set alpha value to zero:
    for (int y = 0; y < rgba.rows; ++y)
    for (int x = 0; x < rgba.cols; ++x)
    {
        cv::Vec4b & pixel = rgba.at<cv::Vec4b>(y, x);
        // if pixel is white
        if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)
        {
            // set alpha to zero:
            pixel[3] = 0;
        }
    }
    cv::String outpng = output.substr(0, output.find_last_of('.'))+".png";
    cv::imwrite(outpng, rgba);

    cv::String meta = output.substr(0, output.find_last_of('.'))+".yaml";
    cv::FileStorage fs(meta, cv::FileStorage::WRITE);
    fs << "masked image file" << output;
    fs << "masks classes" << int(mi.masks.size());
    for (auto const& kv : mi.masks)
    {
        fs << "class id " + std::to_string(kv.first) << "{";
        for (auto const& m : kv.second)
            fs << "color" << m.color << "bounding_box" << m.bounding_box;
        fs << "}";
    }
    fs.release();

    return 0;
}

//
// Created by Harold on 2021/4/23.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "m_curvefit_opencv.h"
#include "utils.h"
#include "stopwatch.h"

using namespace std;

void help(char** argv);

void demo(char** argv);

// according to https://stackoverflow.com/questions/11722569/opencv-line-fitting-algorithm
// cv::DIST_L2 is a standard unweighted least squares fit, others are RANSAC

int main(int argc, char** argv) {
    // demo(argv);

    cv::RNG rng(time(NULL));
    int i, count = rng.uniform(0, 100) + 3;
    double a = (double)rng.uniform(0., 200.);
    double b = (double)rng.uniform(0., 40.);
    double angle = (double)rng.uniform(0., CV_PI);
    double cos_a = cos(angle), sin_a = sin(angle);
    cv::Point pt1, pt2;
    vector<cv::Point> points(count);
    cv::Vec4f line;
    float d, t;
    b = MIN(a * 0.3f, b);

    // generate some points that are close to the line
    for (i = 0; i < count; i++) {
        float x = (float)rng.uniform(-1., 1.) * a;
        float y = (float)rng.uniform(-1., 1.) * b;
        points[i].x = cvRound(x * cos_a - y * sin_a + 250);
        points[i].y = cvRound(x * sin_a + y * cos_a + 250);
    }

    {
        TIME_BLOCK("CV");
        cv::fitLine(points, line, cv::DIST_L2, 1, 0.001, 0.001);
    }
    std::cout << "CV - slope: " << line[1] / line[0] << ", intercept: " << line[3] - (line[1] / line[0]) * line[2] << std::endl;

    auto cf = M_MATH::CurveFitCV();
    std::vector<double> x, y;
    for (auto const& pt : points) {
        x.push_back(pt.x);
        y.push_back(pt.y);
    }
    {
        TIME_BLOCK("HM");
        cf.polyfit(x.data(), y.data(), x.size(), 1);
    }
    std::cout << "HM - slope: " << cf.get_slope() << ", intercept: " << cf.get_intercept() << std::endl;

    return 0;
}

void help(char** argv) {
    cout << "\nTwo dimensional line fitting"
         << "\nCall"
         << "\n" << argv[0] << "\n"
         << "\n 'q', 'Q' or ESC to quit"
         << "\n" << endl;
}

void demo(char** argv) {
    cv::Mat img(500, 500, CV_8UC3);
    cv::RNG rng(-1);
    help(argv);
    for (;;) {
        char key;
        int i, count = rng.uniform(0, 100) + 3, outliers = count / 5;
        float a = (float)rng.uniform(0., 200.);
        float b = (float)rng.uniform(0., 40.);
        float angle = (float)rng.uniform(0., CV_PI);
        float cos_a = cos(angle), sin_a = sin(angle);
        cv::Point pt1, pt2;
        vector<cv::Point> points(count);
        cv::Vec4f line1, line2, line3, line4, line5, line6, line7;
        float d, t;
        b = MIN(a * 0.3f, b);

        // generate some points that are close to the line
        for (i = 0; i < count - outliers; i++) {
            float x = (float)rng.uniform(-1., 1.) * a;
            float y = (float)rng.uniform(-1., 1.) * b;
            points[i].x = cvRound(x * cos_a - y * sin_a + img.cols / 2);
            points[i].y = cvRound(x * sin_a + y * cos_a + img.rows / 2);
        }

        // generate outlier points
        for (; i < count; i++) {
            points[i].x = rng.uniform(0, img.cols);
            points[i].y = rng.uniform(0, img.rows);
        }

        // DIST_C not supported

        // find the optimal line based on |x1-x2| + |y1-y2|
        cv::fitLine(points, line1, cv::DIST_L1, 1, 0.001, 0.001);
        // find the optimal line based on euclidean distance
        cv::fitLine(points, line2, cv::DIST_L2, 1, 0.001, 0.001);
        // find the optimal line based on 2(sqrt(1+x*x/2) - 1))
        cv::fitLine(points, line4, cv::DIST_L12, 1, 0.001, 0.001);
        // find the optimal line based on c^2(|x|/c-log(1+|x|/c)), c = 1.3998
        cv::fitLine(points, line5, cv::DIST_FAIR, 1, 0.001, 0.001);
        // find the optimal line based on c^2/2(1-exp(-(x/c)^2)), c = 2.9846
        cv::fitLine(points, line6, cv::DIST_WELSCH, 1, 0.001, 0.001);
        // find the optimal line based on |x|<c ? x^2/2 : c(|x|-c/2), c=1.345
        cv::fitLine(points, line7, cv::DIST_HUBER, 1, 0.001, 0.001);

        // draw the points
        img = cv::Scalar::all(0);
        for (i = 0; i < count; i++)
            cv::circle(img, points[i], 2,
                i < count - outliers ? cv::Scalar(0, 0, 255)
                : cv::Scalar(0, 255, 255),
                cv::FILLED, 8, 0);

        // ... and the long enough line to cross the whole image
        d = sqrt((double)line1[0] * line1[0] + (double)line1[1] * line1[1]);
        line1[0] /= d;
        line1[1] /= d;
        t = (float)(img.cols + img.rows);
        pt1.x = cvRound(line1[2] - line1[0] * t);
        pt1.y = cvRound(line1[3] - line1[1] * t);
        pt2.x = cvRound(line1[2] + line1[0] * t);
        pt2.y = cvRound(line1[3] + line1[1] * t);
        auto color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img, //target image
            "1", //text
            cv::Point((pt1 + pt2) / 2), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color, //font color
            2);

        d = sqrt((double)line2[0] * line2[0] + (double)line2[1] * line2[1]);
        line2[0] /= d;
        line2[1] /= d;
        pt1.x = cvRound(line2[2] - line2[0] * t);
        pt1.y = cvRound(line2[3] - line2[1] * t);
        pt2.x = cvRound(line2[2] + line2[0] * t);
        pt2.y = cvRound(line2[3] + line2[1] * t);
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img,
            "2",
            cv::Point((pt1 + pt2) / 2),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);

        d = sqrt((double)line4[0] * line4[0] + (double)line4[1] * line4[1]);
        line4[0] /= d;
        line4[1] /= d;
        pt1.x = cvRound(line4[2] - line4[0] * t);
        pt1.y = cvRound(line4[3] - line4[1] * t);
        pt2.x = cvRound(line4[2] + line4[0] * t);
        pt2.y = cvRound(line4[3] + line4[1] * t);
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img,
            "4",
            cv::Point((pt1 + pt2) / 2),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);

        d = sqrt((double)line5[0] * line5[0] + (double)line5[1] * line5[1]);
        line5[0] /= d;
        line5[1] /= d;
        pt1.x = cvRound(line5[2] - line5[0] * t);
        pt1.y = cvRound(line5[3] - line5[1] * t);
        pt2.x = cvRound(line5[2] + line5[0] * t);
        pt2.y = cvRound(line5[3] + line5[1] * t);
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img,
            "5",
            cv::Point((pt1 + pt2) / 2),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);

        d = sqrt((double)line6[0] * line6[0] + (double)line6[1] * line6[1]);
        line6[0] /= d;
        line6[1] /= d;
        pt1.x = cvRound(line6[2] - line6[0] * t);
        pt1.y = cvRound(line6[3] - line6[1] * t);
        pt2.x = cvRound(line6[2] + line6[0] * t);
        pt2.y = cvRound(line6[3] + line6[1] * t);
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img,
            "6",
            cv::Point((pt1 + pt2) / 2),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);

        d = sqrt((double)line7[0] * line7[0] + (double)line7[1] * line7[1]);
        line7[0] /= d;
        line7[1] /= d;
        pt1.x = cvRound(line7[2] - line7[0] * t);
        pt1.y = cvRound(line7[3] - line7[1] * t);
        pt2.x = cvRound(line7[2] + line7[0] * t);
        pt2.y = cvRound(line7[3] + line7[1] * t);
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(img, pt1, pt2, color, 2, 8, 0);
        cv::putText(img,
            "7",
            cv::Point((pt1 + pt2) / 2),
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color,
            2);

        cv::imshow("Fit Lines", img);
        key = (char)cv::waitKey(0);
        if (key == 27 || key == 'q' || key == 'Q') // 'ESC'
            break;
    }
}

//
// Created by Harold on 2021/6/30.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "m_opencv_utils.h"

using namespace std;
using namespace cv;

Vec3b RandomColor(int value)  //generate random color
{
	value=value % 255;  //0-255
	RNG rng;
	int aa = rng.uniform(0, value);
	int bb = rng.uniform(0, value);
	int cc = rng.uniform(0, value);
	return Vec3b(aa, bb, cc);
}

int main() {
    auto img = imread(samples::findFile( "test.jpg" ), IMREAD_GRAYSCALE);
    if (img.empty()){
        printf("Error opening image\n");
        return EXIT_FAILURE;
    }
    imshow("img", img);

    Mat I;
    //GaussianBlur(I,I,Size(5,5),2);  // gaussian blur
    auto thrld = threshold(img, I, 0, 255, THRESH_BINARY | THRESH_OTSU);  // use OTSU to find optimal threshold
    Canny(I, I, thrld * 0.5, thrld, 3, true);  // canny edge dectection
    imshow("Edges", I); 


    Mat marks(I.size(), CV_32S);  // markers for watershed algo
	marks = Scalar::all(0);


    // 1. find contours
	vector<vector<Point>> contours;  
	vector<Vec4i> hierarchy;  
	findContours(I, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE); 
    printf("number of contours: %zu\n", contours.size());
	Mat imageContours = Mat::zeros(I.size(), CV_8UC1);  // contours image	
	int index = 0;
	int compCount = 0;
	for( ; index >= 0; index = hierarchy[index][0], compCount++ ) 
	{
        // directly use contours to label markers, indexing different region contours, seed number == contour number
        // if not go to 2, it will consists lots small areas
		//drawContours(marks, contours, index, Scalar::all(compCount+1), 1, 8, hierarchy);
		drawContours(imageContours, contours, index, Scalar(255), 1, 8, hierarchy);  
	}
    imshow("Contours", imageContours);

    /* get contour regions
    std::vector<cv::Mat> subregions; subregions.reserve(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        Mat mask = Mat::zeros(I.size(), CV_8UC1);
        drawContours(mask, contours, i, Scalar(255), FILLED);
        Mat contourRegion;
        Mat imageROI;
        I.copyTo(imageROI, mask);
        contourRegion = imageROI(boundingRect(contours[i]));
        subregions.push_back(contourRegion);
    }
    */

    // 2. use erode and dilate to remove small parts
    morphologyEx(imageContours, imageContours, MORPH_OPEN, getStructuringElement(MorphShapes::MORPH_RECT, Size(3, 3)), Point(-1, -1), 0);
    Mat board, tmp;
    dilate(imageContours, board, Mat(), Point(-1, -1), 2);
    erode(board, tmp, Mat(), Point(-1, -1), 1);
    board -= tmp;
    imshow("board", board);
    Mat dt;
    distanceTransform(img, dt, DIST_L2, 3, CV_32F);
    dt = M_MATH::To8U(dt);
    imshow("dt_o", dt);
    double dt_max;
    minMaxLoc(dt, nullptr, &dt_max, nullptr, nullptr, noArray());
    threshold(dt, dt, 0.5 * dt_max, 255, THRESH_BINARY);
    imshow("dt", dt);
    Mat stats, centroids;
    auto num_of_labels = connectedComponentsWithStats(dt, marks, stats, centroids, 4, CV_32S);
    printf("number of labels: %d\n", num_of_labels);
    marks *= 255 / (num_of_labels + 1);
    marks.setTo(255, board == 255);


	// display marker before watershed
	Mat marksShows;
	convertScaleAbs(marks, marksShows);
	imshow("marks", marksShows);
    Mat rgb;
    cvtColor(I, rgb, COLOR_GRAY2BGR);
	watershed(rgb, marks);
    //marks.setTo(0, marks == -1);

	// display marker after watershed
	Mat afterWatershed;
	convertScaleAbs(marks, afterWatershed);
	imshow("After Watershed", afterWatershed);

	// fill color in every region
	Mat PerspectiveImage = Mat::zeros(I.size(), CV_8UC3);
	for (int i = 0; i < marks.rows; i++) {
		for (int j = 0; j < marks.cols; j++) {
			int index = marks.at<int>(i, j);
			if (marks.at<int>(i, j) == -1) {
				PerspectiveImage.at<Vec3b>(i, j) = Vec3b(255,255,255);
			}			 
			else {
				PerspectiveImage.at<Vec3b>(i, j) = RandomColor(index);
			}
		}
	}
	imshow("After ColorFill", PerspectiveImage);

    // merge segmented results with original image for display
	Mat wshed;
	addWeighted(rgb, 0.4, PerspectiveImage, 0.6, 0, wshed);
	imshow("AddWeighted Image", wshed);

	waitKey();
}
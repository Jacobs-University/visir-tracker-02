/*
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"//for the goodFeaturesToTrack() function
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/tracking.hpp"
*/

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>//for the goodFeaturesToTrack() function
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>


#include "types.h"
#include <iostream>

RNG rng(12345);
int maxCorners = 200;
Mat img, img_gray;

void goodFeaturesToTrack(int, void*)
{
    /// Parameters for Shi-Tomasi algorithm
    maxCorners = MAX(maxCorners, 1);
    std::vector<Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3, gradientSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    /// Copy the source image
    Mat copy = img.clone();

    /// Apply corner detection
    goodFeaturesToTrack(img_gray,
        corners,
        maxCorners,
        qualityLevel,
        minDistance,
        Mat(),
        blockSize,
        gradientSize,
        useHarrisDetector,
        k);


    /// Draw corners detected
    std::cout << "Number of corners detected: " << corners.size() << std::endl;
    int radius = 4;
    for (size_t i = 0; i < corners.size(); i++)
    {
        circle(copy, corners[i], radius, Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), FILLED);
    }

    imshow("Camera", copy);
}

int main() {

    // Create some random colors
    std::vector<Scalar> colors;
    RNG rng;
    for (int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r, g, b));
    }

	VideoCapture camera;
	if (!camera.open(0)) {
		printf("Can't find a camera\n");
		return 1;
	};

    Mat old_frame, old_gray;
    std::vector<Point2f> p0, p1;
    // Take first frame and find corners in it
    camera>> old_frame;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(old_gray, p0, 100, 0.01, 7, Mat(), 7, false, 0.04);
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());

	// Main loop
	for(;;) {
		camera >> img;
        cvtColor(img, img_gray, COLOR_BGR2GRAY);

        // calculate optical flow
        std::vector<uchar> status;
        std::vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        cv::calcOpticalFlowPyrLK(old_gray, img_gray, p0, p1, status, err, Size(15, 15), 3, criteria);

        std::vector<Point2f> good_new;
        for (uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask, p1[i], p0[i], colors[i], 2);
                circle(img, p1[i], 5, colors[i], -1);
            }
        }

        Mat frame;
        add(img, mask, frame);
        imshow("Camera", frame);

		//imshow("Camera", img);
        goodFeaturesToTrack(0, 0);
		int key = waitKey(5);
		if (key == 27 || key == 'q') break;

        // Now update the previous frame and previous points
        old_gray = img_gray.clone();
        p0 = good_new;
	}
	camera.release();
	return 0;
}

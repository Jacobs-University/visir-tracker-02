#include "types.h"
using namespace std;

// Parameters for Shi-Tomasi Corner Detector
int maxCorners = 128;
double qualityLevel = 0.01;
double minDistance = 10;

int main() {
	VideoCapture camera;
	if (!camera.open(0)) {
		printf("Can't find a camera\n");
		return 1;
	}

	// Setup various variables for Lukas-Kanade method
	Mat old_img, old_grey;
	vector<Point2f> p0, p1;
	vector<float> err;
	vector<uchar> status;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

	// Getting initial old image and its features
	camera >> old_img; camera >> old_img; camera >> old_img;
	cvtColor(old_img, old_grey, COLOR_BGR2GRAY);
	goodFeaturesToTrack(old_grey, p0, maxCorners, qualityLevel, minDistance);
	Mat mask = Mat::zeros(old_img.size(), old_img.type());

	// Main loop
	Mat img, grey;
	int frameCounter = 0;
	for(;;) {
		camera >> img;
		if (img.empty())
			break;
		cvtColor(img, grey, COLOR_BGR2GRAY);

		// Getting good features to track every 300th frame and visualising
		if (frameCounter % 300 == 0)
			goodFeaturesToTrack(grey, p0, maxCorners, qualityLevel, minDistance);
		for (size_t i = 0; i < p0.size(); i++)
			circle(img, p0[i], 5, Scalar(255,255,255), FILLED);
		frameCounter++;

		// Calculating optical flow and visualising
		vector<Point2f> p2;
		calcOpticalFlowPyrLK(old_grey, grey, p0, p1, status, err, Size(15,15), 3, criteria);
		for (uint i = 0; i < p0.size(); i++) {
			if (status[i] == 1) {
				p2.push_back(p1[i]);
				line(mask, p1[i], p0[i], Scalar(255,0,0), 2);
			}
		}

		img += mask;
		imshow("Camera", img);
		int key = waitKey(5);
		if (key == 27 || key == 'q') break;
		p0 = p2;
		old_grey = grey.clone();
		if (frameCounter % 300 == 0)
			mask = Mat::zeros(img.size(), img.type());
	}
	camera.release();
	return 0;
}

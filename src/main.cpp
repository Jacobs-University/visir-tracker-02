#include "types.h"

int main() {
	VideoCapture camera;
	if (!camera.open(0)) {
		printf("Can't find a camera\n");
		return 1;
	};
	
	// Main loop
	Mat img, gray, prevGray;
	std::vector<Point2f> p0;
	std::vector<Point2f> p1;
	int frame = 0;
	
	camera >> img;
	cvtColor(img, prevGray, COLOR_BGR2GRAY);
	Mat mask = Mat::zeros(img.size(), img.type());
	goodFeaturesToTrack(gray, p0, 100, 0.01, 10);


	for(;;) {
		camera >> img;
		cvtColor(img, gray, COLOR_BGR2GRAY);
		
		if (frame % 300 == 0) {
			goodFeaturesToTrack(gray, p0, 100, 0.01, 10);
			//update initial tracking points every 300 frames
		}
		for (uint i = 0; i < p0.size(); i++) {
			circle(img, p0[i], 3, Scalar(0, 255, 0), -1, 8);
		}

		std::vector<uchar> status;
		std::vector<float> err;
		TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

		calcOpticalFlowPyrLK(prevGray, gray, p0, p1, status, err, Size(15, 15), 3, criteria);
		std::vector<Point2f> good_new;
		for (uint i = 0; i < p0.size(); i++) {
			if (status[i] == 1) {
				good_new.push_back(p1[i]);
				line(mask, p1[i], p0[i], Scalar(255, 0, 0), 2);
			}
		}

		img += mask;
		imshow("Camera", img);
		int key = waitKey(5);
		if (key == 27 || key == 'q') break;
		frame++;
		prevGray = gray.clone();
		p0 = good_new;
	}
	camera.release();
	return 0;
}

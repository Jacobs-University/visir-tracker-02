#include <opencv2/videoio.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

int main()
{

	// open camera stream
	VideoCapture camera;
	if (!camera.open(0))
	{
		printf("Can't find a camera\n");
		return 1;
	};

	// create matrices and points
	Mat old_frame, old_gray;
	vector<Point2f> p0, p1;

	// skip first few frames, produces better results
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;
	camera >> old_frame;

	// convolute and track features
	cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
	goodFeaturesToTrack(old_gray, p0, 200, 0.01, 10);

	// initialize variables
	Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
	vector<float> err;
	vector<uchar> status;
	int frame_no = 0;

	// increment frame number every frame
	for (;; frame_no++)
	{

		Mat frame, frame_gray;
		vector<Point2f> good_new;
		TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
		Mat img;

		// capture img
		camera >> frame;
		if (frame.empty())
			break;

		// convolute
		cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

		// only execute every 300th frame
		if (frame_no % 300 == 0)
		{
			// reset frames and recalculate corners/features
			frame_no = 0;
			goodFeaturesToTrack(frame_gray, p0, 200, 0.01, 10);
		}

		// calculate optical flow for this frame
		calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);

		// update the new positions for the pts and draw features+lines
		for (uint i = 0; i < p0.size(); i++)
		{
			if (status[i] == 1)
			{
				good_new.push_back(p1[i]);
				line(mask, p1[i], p0[i], Scalar(0, 255, 0), 2);
				circle(frame, p1[i], 5, Scalar(255, 0, 0), -1);
			}
		}

		// add frame and mask to final img
		add(frame, mask, img);
		imshow("Frame", img);

		// listen for quit command 
		int keyboard = waitKey(30);
		if (keyboard == 'q' || keyboard == 27)
			break;

		// update old_gray and p0
		old_gray = frame_gray.clone();
		p0 = good_new;

		// reset the mask every 300 frames
		if (frame_no % 300 == 0)
		{
			mask = Mat::zeros(img.size(), img.type());
		}
	}

	camera.release();
	return 0;

}

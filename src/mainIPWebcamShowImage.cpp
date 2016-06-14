#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
	VideoCapture cap; //

	cap.open("http://10.16.167.74:8080/video?x.mjpeg");
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	namedWindow("MyNegativeVideo", CV_WINDOW_AUTOSIZE);

	while (1)
	{
		Mat frame;
		Mat contours;

		bool bSuccess = cap.read(frame); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		flip(frame, frame, 1);
		imshow("MyVideo", frame); //show the frame in "MyVideo" window

		Canny(frame, contours, 500, 1000, 5, true);
		imshow("MyNegativeVideo", contours);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}
#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
/**
* Helper function to find a cosine of angle between vectors
* from pt0->pt1 and pt0->pt2
*/

using namespace std;
using namespace cv;

double maxValue = 100;
int maxValueSlider = 0;
int maxValueSliderMax = 255;
int adaptiveMethod = ADAPTIVE_THRESH_MEAN_C;
int thresholdType = 0;
int blockSize = 15;
double C = 10.0;
const string windowName = "Control";

/**
* @function on_trackbar
* @brief Callback for trackbar
*/
void on_trackbar(int, void*)
{
	/// Initialize values
	maxValue = 0;

	maxValue = (double)maxValueSlider;
}

/**
* @function on_trackbar
* @brief Callback for trackbar
*/
void on_blockSize(int, void*)
{
	/// Initialize values
	blockSize += 2;

}

int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from web cam

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	createTrackbar("MaxValue", windowName, &maxValueSlider, maxValueSliderMax, on_trackbar);
	createTrackbar("BlockSize", windowName, &maxValueSlider, maxValueSliderMax, on_blockSize);
	createTrackbar("ThresholdType", windowName, &thresholdType, 1);
	//createTrackbar("C constant", windowName, &maxValueSlider, maxValueSliderMax, on_blockSize);

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	while (true)
	{
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, CV_BGR2GRAY); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;
		adaptiveThreshold(imgHSV, imgThresholded, maxValue, adaptiveMethod, thresholdType, blockSize, C);

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;

}


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

RNG rng(12345);
Rect potentialTargetCircle; //For landing fields!
Rect potentialTargetHex;
int bigCircleMin = 25;
int bigCircleMax = 60;
int hexMin = 200;
int hexMax = 10000;

// the function draws all the squares in the image
static void drawSquares(Mat& image, const vector<vector<Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];

		int n = (int)squares[i].size();
		//dont detect the border
		if (p->x > 3 && p->y > 3)
			polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
	}

	imshow("squares", image);
}

/**
* Helper function to find a cosine of angle between vectors
* from pt0->pt1 and pt0->pt2
*/
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
* Helper function to display text in the center of a contour
*/
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

int main()
{
	//cv::Mat src = cv::imread("polygon.png");
	cv::Mat src;
	cv::Mat gray;
	cv::Mat bw;
	cv::Mat dst;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Point> approx;

	VideoCapture capture(0);
	int q;

	while (cvWaitKey(30) != 'q')
	{
		capture >> src;
		if (true)
		{
			// Convert to grayscale
			cv::cvtColor(src, gray, CV_BGR2GRAY);

			// These are the best settings for the ardrone floor cam so far ...
			/**/
			//blur(gray, bw, Size(3, 3));
			medianBlur(gray, gray, 9);
			int lowThreshold = 10;
			int ratio = 3;
			adaptiveThreshold(gray, gray, 255, 0, 0, 255, 10.0);
			cv::Canny(gray, bw, lowThreshold, ratio*lowThreshold, 3);
			cv::imshow("bw", bw);

			//threshold(gray, bw, 183, 255, 0);
			imshow("bw", bw);
			//cv::bitwise_not(bw, bw);

			// Find contours
			cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			src.copyTo(dst);

			for (int i = 0; i < contours.size(); i++)
			{

				// Approximate contour with accuracy proportional
				// to the contour perimeter
				cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

				// Skip small or non-convex objects
				if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
					continue;

				if (approx.size() == 3)
				{
					setLabel(dst, "TRI", contours[i]);    // Triangles
				}
				else if (approx.size() >= 4 && approx.size() <= 6)
				{
					// Number of vertices of polygonal curve
					int vtc = approx.size();

					// Get the cosines of all corners
					std::vector<double> cos;
					for (int j = 2; j < vtc + 1; j++)
						cos.push_back(angle(approx[j%vtc], approx[j - 2], approx[j - 1]));

					// Sort ascending the cosine values
					std::sort(cos.begin(), cos.end());

					// Get the lowest and the highest cosine
					double mincos = cos.front();
					double maxcos = cos.back();

					// Use the degrees obtained above and the number of vertices
					// to determine the shape of the contour
					if (vtc == 4) {
						setLabel(dst, "RECT", contours[i]);
						//Lav en hex rectttt
						cv::Rect ar = cv::boundingRect(contours[i]);
						float aspectRatio = (float)ar.width / (float)ar.height;
						//cout << ar.width << " " << ar.height << endl;
						//cout << " aspect ratio " << aspectRatio << endl;
					}
					/*
					else if (vtc == 5) {
					setLabel(dst, "PENTA", contours[i]);
					Rect r = cv::boundingRect(contours[i]);
					if (r.area)
					}
					*/
					else if (vtc == 6 || vtc == 5) {
						Rect r = cv::boundingRect(contours[i]);
						//cout << r.area() << " hexa area" << r.x << " r.x " << r.y << " r.y " << endl;
						if (r.area() > 500) {
							setLabel(dst, to_string(r.area()), contours[i]);

							//cout << "hex added" << endl;
							potentialTargetHex = boundingRect(contours[i]);
						}
					}
				}
				else
				{
					// Detect and label circles
					double area = cv::contourArea(contours[i]);
					cv::Rect r = cv::boundingRect(contours[i]);
					//cout << r.width << " r.width" << endl;
					int radius = r.width / 2;

					if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
						std::abs(1 - (area / (CV_PI * (radius*radius)))) <= 0.2) {
						setLabel(dst, "CIR", contours[i]);
						//cout << radius << " radius " << r.x << " r.x " << r.y << " r.y " << endl;
						if (radius > bigCircleMin && radius < bigCircleMax) {
							//cout << "circle added to potential target" << endl;
							potentialTargetCircle = boundingRect(contours[i]);
						}
					}
				}
			}
			Point circleMiddle = Point(potentialTargetCircle.x + potentialTargetCircle.width / 2, potentialTargetCircle.y
				+ potentialTargetCircle.height / 2);
			Point hexMiddle = Point(potentialTargetHex.x + potentialTargetHex.width / 2, potentialTargetHex.y + potentialTargetHex.height / 2);
			circle(src, Point(potentialTargetCircle.x + potentialTargetCircle.width/2, potentialTargetCircle.y 
				+ potentialTargetCircle.height/2), 5, Scalar(255, 0, 0));
			circle(src, Point(potentialTargetHex.x + potentialTargetHex.width/2, potentialTargetHex.y + potentialTargetHex.height/2), 5, Scalar(0, 0, 255));

			if (potentialTargetCircle.x != 0 && potentialTargetHex.x != 0) {
				float ratioTarget = (float) circleMiddle.x / (float)hexMiddle.x;
				cout << " mellem cirkel og hex ratio is " << ratioTarget << endl;
				if (ratioTarget > 0.9 && ratioTarget < 1.10) {
					cout << "Target found!" << endl;
				}
				potentialTargetCircle.x = 0;
				potentialTargetHex.x = 0;
			}
			drawSquares(src, contours);

			cv::imshow("src", src);
			cv::imshow("dst", dst);

		}
		else
		{
			break;
		}
	}
	//While end
	return 0;
}


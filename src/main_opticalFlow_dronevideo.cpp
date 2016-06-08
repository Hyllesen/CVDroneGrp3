#include "ardrone/ardrone.h"
#include <Windows.h>

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

boolean opticalFlow(cv::Mat image1, cv::Mat image2);

int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

	// Array for holding the time an image was taken
	
	//milliseconds prevTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	//milliseconds currTime;
	//time_t prevTime = time(0);
	//time_t currTime = time(0);
	cv::Mat prevImage;
	int prevTime = GetTickCount();
	int currTime;
    while (1) {
        // Key input
        int key = cv::waitKey(10);
		
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

		currTime = GetTickCount();

		//std::cout << "Time elapsed since last image = " << (currTime - prevTime) << " milliseconds." << std::endl;

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx*0.4, vy*0.4, vz*0.4, vr*0.4);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);
		
        // Display the image
        //cv::imshow("Camera", image);
		
		if (!prevImage.empty()) {
			opticalFlow(image, prevImage);
			prevImage = image;
		}
		else {
			prevImage = image;
			std::cout << "prevImage.empty() is TRUE!!" << std::endl;
		}

		prevTime = currTime;
    }
	
    // See you
    ardrone.close();

    return 0;
}

boolean opticalFlow(cv::Mat image1, cv::Mat image2) {
	//Point array
	cv::Point points[50][2];

	// Convert the camera images to grayscale
	cv::Mat prev_gray, new_gray;
	cv::cvtColor(image1, new_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(image2, prev_gray, cv::COLOR_BGR2GRAY);

	// Detect corners
	int max_corners = 50;
	std::vector<cv::Point2f> prev_corners;
	std::vector<cv::Point2f> new_corners;

	cv::goodFeaturesToTrack(prev_gray, prev_corners, max_corners, 0.1, 5.0);
	cv::goodFeaturesToTrack(new_gray, new_corners, max_corners, 0.1, 5.0);

	// Calclate optical flow
	std::vector<unsigned char> status;
	std::vector<float> errors;
	cv::calcOpticalFlowPyrLK(prev_gray, new_gray, prev_corners, new_corners, status, errors);

	//Load return statements with -1
	for (int i = 0; i < 50; i++) {
		points[i][0].x = -1;
		points[i][0].y = -1;
		points[i][1].x = -1;
		points[i][1].y = -1;
	}


	// Get optical flows
	for (size_t i = 0; i < status.size(); i++) {
		cv::Point p0(ceil(prev_corners[i].x), ceil(prev_corners[i].y));
		cv::Point p1(ceil(new_corners[i].x), ceil(new_corners[i].y));

		points[i][0].x = p0.x;
		points[i][0].y = p0.y;
		points[i][1].x = p1.x;
		points[i][1].y = p1.y;
	}

	double d = 0;
	double x;
	double y;
	int l = 0;
	//Draw lines
	for (size_t i = 0; i < status.size(); i++) {
		cv::Point p0(ceil(prev_corners[i].x), ceil(prev_corners[i].y));
		cv::Point p1(ceil(new_corners[i].x), ceil(new_corners[i].y));

		x = (p1.x - p0.x)*(p1.x - p0.x);
		y = (p1.y - p0.y)*(p1.y - p0.y);
		d = sqrt(x + y);


		if (d > 10.0) {
			//Line too long
			cv::line(image1, p0, p1, cv::Scalar(0, 0, 255), 2);
			l++;
		}
		else {

			cv::line(image1, p0, p1, cv::Scalar(0, 255, 0), 2);
		}
	}
	//Show picture
	cv::namedWindow("Drone vision", CV_WINDOW_NORMAL);
	cv::imshow("Drone vision", image1);
	//cv::putText(image1, "Alarm: ", cv::Point(0, 0), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255.0, 0.0, 0.0));
	//cv::waitKey(0);


	//calculate return
	double percentage = (l*1.0) / (status.size()*1.0) * 100;

	if (percentage >= 30) {
		printf("Alarm: %f%%\n", percentage);
		return true;
	}

	else {
		return false;
	}
}
#include "ardrone/ardrone.h"
#include <iostream>
#include <fstream>

using namespace std;

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
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

	double x_tot = 0.0;
	boolean backwards = false;

    // Main loop
	while (1) {
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		cv::Mat image = ardrone.getImage();

		// Take off / Landing 
		if (key == ' ') {
			if (ardrone.onGround()) ardrone.takeoff();
			else                    ardrone.landing();
		}

		// Move
		double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
		if (key == 'i' || key == CV_VK_UP)    x = 1.0;
		if (key == 'k' || key == CV_VK_DOWN)  x = -1.0;
		if (key == 'u' || key == CV_VK_LEFT)  r = 1.0;
		if (key == 'o' || key == CV_VK_RIGHT) r = -1.0;
		if (key == 'j') y = 1.0;
		if (key == 'l') y = -1.0;
		if (key == 'q') z = 1.0;
		if (key == 'a') z = -1.0;
		ardrone.move3D(x, y, z, r);

		x_tot += x;
		std::cout << "x total = " << x_tot << std::endl;

		if (x_tot >= 20) backwards = true;
		//if (x_tot <= 0) break;
	
		//if (x_tot < 20 && !backwards) {
		//	ardrone.move3D(1.0, 0.0, 0.0, 0.0);
		//}

		if (x_tot > 0 && backwards) {
			ardrone.move3D(-1.0, 0.0, 0.0, 0.0);
			x_tot -= 1.0;
		}
        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode%4);

        // Display the image from camera
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}
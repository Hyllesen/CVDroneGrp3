#include "ardrone/ardrone.h"

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

	//ardrone.setFlatTrim();
	//ardrone.setCalibration(0);

	boolean turn = false;
	double currentYaw, targetYaw;
	const float SPIN_TOLERANCE = 10.0;
    
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

		//double vx, vy, vz;
		//ardrone.getVelocity(&vx, &vy, &vz);
		//std::cout << "Velocity = " << vx << ", " << vy << ", " << vz << " [m/s]." << std::endl;

		/*
		if (ardrone.onGround() == 0) {
			double lolYaw = ardrone.getYaw() * RAD_TO_DEG;
			if (lolYaw < 0) lolYaw += 360.0;
			std::cout << "lolYaw = " << lolYaw << " degrees." << std::endl;
		} 
		*/

		/*
		Vent 5-10 sekunder før spin ved takeoff - resetter til 245 grader
		*/
		if (key == 's' && turn == false) { 
			turn = true;
			targetYaw = ardrone.getYaw() * RAD_TO_DEG;
			if (targetYaw < 0) targetYaw += 360.0;
			targetYaw += 114.0;
			if (targetYaw > 360) targetYaw -= 360;
			std::cout << "Target Yaw set to " << targetYaw << " degrees." << std::endl;
		}
		
		if (turn) {
			ardrone.move3D(0.0, 0.0, 0.0, 1.0);
			currentYaw = ardrone.getYaw() * RAD_TO_DEG;
			if (currentYaw < 0) currentYaw += 360.0;
			std::cout << "Current Yaw = " << currentYaw << " degrees." << std::endl;
			//std::cout << "targetYaw = " << targetYaw << std::endl;
			//std::cout << "Spin tolerance = " << SPIN_TOLERANCE << std::endl;
			//std::cout << "currentYaw > targetYaw - SPIN_TOLERANCE = " << (currentYaw > targetYaw - SPIN_TOLERANCE) << std::endl;
			//std::cout << "currentYaw < targetYaw + SPIN_TOLERANCE = " << (currentYaw < targetYaw + SPIN_TOLERANCE) << std::endl;
			if ((currentYaw > targetYaw - SPIN_TOLERANCE) && (currentYaw < targetYaw + SPIN_TOLERANCE)) {
				std::cout << "REACHED TARGET WITHIN TOLERANCE!!" << std::endl;
				turn = false;
			}
		}
		// yaw = ardrone.getYaw() * RAD_TO_DEG;
		//if (yaw < 0) yaw += 360;
		//std::cout << "Yaw = " << yaw << " degrees." << std::endl;

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
#include "ardrone/ardrone.h"
#include "zbar.h"

using namespace std;
using namespace cv;
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

//Thresholding stuff
/// Global variables

int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src, src_gray, dst;
char* window_name = "Threshold Demo";

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

cv::Mat image;
zbar::ImageScanner scanner;
int counter = 0;
/// Function headers
void Threshold_Demo(int, void*);

//Thresholding stuff slut
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

	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

	//Thresholding stuff start
	/// Create a window to display results
	String window_name = "Thresholding";
	namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	/// Create Trackbar to choose type of Threshold
	createTrackbar(trackbar_type,
		window_name, &threshold_type,
		max_type, Threshold_Demo);

	createTrackbar(trackbar_value,
		window_name, &threshold_value,
		max_value, Threshold_Demo);

	/// Call the function to initialize
	//Threshold_Demo(0, 0);
	
	
	// Main loop
	while (1) {
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		//cout << "Altitude: " << ardrone.getAltitude() << endl;

		// Get an image
		image = ardrone.getImage();
		//cout << image.channels() << " channels " << endl;
		cvtColor(image, image, CV_RGB2GRAY);
		//src_gray = image;
		//cout << image.channels() << " channels efter RGB2GRAY" << endl;
		//cv::Mat imgout;
		//cout << image.cols << "image ardrone cols" << endl;
		//cvtColor(image, imgout, CV_GRAY2RGB);
		//cvtColor(image, imgout);


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

        // Change camera
        static int mode = 0;
		if (key == 'c') ardrone.setCamera(++mode % 4);
		threshold(image, image, threshold_value, max_BINARY_value, threshold_type);
		
		int width = image.cols;
		int height = image.rows;
		uchar *raw = (uchar *)image.data;

		// wrap image data  
		zbar::Image imageQR(width, height, "Y800", raw, width * height);
		// scan the image for barcodes  
		int n = scanner.scan(imageQR);

		// extract results  
		for (zbar::Image::SymbolIterator symbol = imageQR.symbol_begin();
			symbol != imageQR.symbol_end();
			++symbol) {
			//cout << "Inde i for loop" << endl;
			vector<Point> vp;
			// do something useful with results  
			cout << "Symbol: \"" << symbol->get_data() << '"' << " " << counter << endl;
			int n = symbol->get_location_size();
			for (int i = 0; i<n; i++) {
				vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
			}
			RotatedRect r = minAreaRect(vp);
			Point2f pts[4];
			r.points(pts);
			//for (int i = 0; i<4; i++) {
			//	line(image, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0), 3);
			//}
			//cout << "Angle: " << r.angle << endl;
		}

		// clean up  
		imageQR.set_data(NULL, 0);
		
		//if(imgout)
		//imshow("imgout.jpg", imgout);
		//cout << imgout.cols << " cols" << imgout.rows << " rows" << endl;
		
		//waitKey(); Med dette fjernet kan vi bruge Esc

        // Display the image from camera
        //cv::imshow("QR scanner", image);
		imshow(window_name, image);
		counter++;
    }

    // See you
    ardrone.close();

    return 0;
}

/**
* @function Threshold_Demo
*/
void Threshold_Demo(int, void*)
{
	/* 0: Binary
	1: Binary Inverted
	2: Threshold Truncated
	3: Threshold to Zero
	4: Threshold to Zero Inverted
	*/

	//threshold(image, image, threshold_value, max_BINARY_value, threshold_type);
	//imshow(window_name, image);

	
}

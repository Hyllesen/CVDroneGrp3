#include "ardrone/ardrone.h"
#include "zBar/zbar.h"

using namespace std;
using namespace cv;



//-----------------------------CLASS DECLARATIONS---------------------------
//

class MapQR {
public:
	int x = 0, y = 0, z = 0;
	string name = "DEFAULT";

	MapQR(int _x, int _y, string _name) {
		x = _x;
		y = _y;
		name = _name;
	}
};

class Drone
{
public:
	int x = 0, y = 0, z = 0;
	int startX = 0, startY = 0;
	double angle = 340.0;
	int circleRadius = 5;
	int angleLineLength = 25;

	void setAngle(double _angle) {
		angle = _angle;
		// If angle gets above 360, subtract 360 to make the angle relative to 0
		if (angle >= 360.0) angle -= 360.0;
	}

	void addAngle(double _angle) {
		angle += _angle;
		// If angle gets above 360, subtract 360 to make the angle relative to 0
		if (angle >= 360.0) angle -= 360.0;
	}

	void setX(int _x) {
		x = _x;
	}

	void setY(int _y) {
		y = _y;
	}

	void setZ(int _z) {
		z = _z;
	}

	void addX(int _x) {
		x += _x;
	}

	void addY(int _y) {
		y += _y;
	}

	void addZ(int _z) {
		z += _z;
	}
};

//-----------------------------CONSTANTS------------------------------------
const double MM_PER_PIXEL_X = 16.791277258566978193146417445483;
const double MM_PER_PIXEL_Y = 18.37786259541984732824427480916;
Drone map_drone;
//Adaptive Threshold settings
double maxValue = 255;
int maxValueSlider = 0;
int maxValueSliderMax = 255;
int adaptiveMethod = ADAPTIVE_THRESH_MEAN_C;
int thresholdType = 0;
int blockSize = 15;
double C = 10.0;
const string windowName = "Control";

//-----------------------------METHOD DECLARATIONS---------------------------

// --------------------------------------------------------------------------
// qrSetup(Scanned symbol)
// Description  : Denne metode ops�tter klasser til qr punkt analyse
// Return value : void
// --------------------------------------------------------------------------
void qrSetup();
// -------------------------------------------------------
// --------------------------------------------------------------------------
// getDistance(Scanned symbol)
// Description  : Denne metode returnerer afstanden til QR koden givet i mm
// Return value : double (mm)
// --------------------------------------------------------------------------
double getDistance(zbar::Image::SymbolIterator symbol);
// --------------------------------------------------------------------------
// getAngle(Scanned symbol)
// Description  : Denne metode returnerer vinklen til QR koden, givet i grader
// Return value : double (�)
// --------------------------------------------------------------------------
double getAngle(zbar::Image::SymbolIterator symbol);
// --------------------------------------------------------------------------
// getXCordinate(Vinkel til QR kode, Afstand til QR kode)
// Description  : Denne metode returnerer kameraets relative x-kordinat til QR koden, givet i mm
// Return value : double (pixel)
// --------------------------------------------------------------------------
double getXCordinate(double Ci, double c);
// --------------------------------------------------------------------------
// getYCordinate(Vinkel til QR kode, Afstand til QR kode)
// Description  : Denne metode returnerer kameraets relative y-kordinat til QR koden, givet i mm
// Return value : double (pixel)
// --------------------------------------------------------------------------
double getYCordinate(double Ci, double c);

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

/* Yaw variables */
boolean turn = false;
double currentYaw, targetYaw;
const float SPIN_TOLERANCE = 10.0;

int main(int argc, char *argv[])
{
	//Constants


	std::string groupN = "W01";
	std::string groupE = "W02";
	std::string groupS = "W03";
	std::string groupV = "W00";

	//Map setup
	// Set up QR codes positions
	MapQR W0000(94, 471, "W00.00");
	MapQR W0001(94, 387, "W00.01");
	MapQR W0002(94, 314, "W00.02");
	MapQR W0003(94, 217, "W00.03");
	MapQR W0004(94, 137, "W00.04");

	MapQR W0100(174, 68, "W01.00");
	MapQR W0101(284, 68, "W01.01");
	MapQR W0102(377, 68, "W01.02");
	MapQR W0103(519, 68, "W01.03");
	MapQR W0104(643, 68, "W01.04");

	MapQR W0200(739, 111, "W02.00");
	MapQR W0201(781, 205, "W02.01");
	MapQR W0202(735, 289, "W02.02");
	MapQR W0203(735, 409, "W02.03");
	MapQR W0204(735, 515, "W02.04");

	MapQR W0300(670, 593, "W03.00");
	MapQR W0301(523, 593, "W03.01");
	MapQR W0302(403, 593, "W03.02");
	MapQR W0303(295, 593, "W03.03");
	MapQR W0304(137, 593, "W03.04");

	// Add QR's to map
	map<string, MapQR> mapQRs;
	map<string, MapQR>::iterator it;

	mapQRs.insert(pair<string, MapQR>("W00.00", W0000));
	mapQRs.insert(pair<string, MapQR>("W00.01", W0001));
	mapQRs.insert(pair<string, MapQR>("W00.02", W0002));
	mapQRs.insert(pair<string, MapQR>("W00.03", W0003));
	mapQRs.insert(pair<string, MapQR>("W00.04", W0004));

	mapQRs.insert(pair<string, MapQR>("W01.00", W0100));
	mapQRs.insert(pair<string, MapQR>("W01.01", W0101));
	mapQRs.insert(pair<string, MapQR>("W01.02", W0102));
	mapQRs.insert(pair<string, MapQR>("W01.03", W0103));
	mapQRs.insert(pair<string, MapQR>("W01.04", W0104));

	mapQRs.insert(pair<string, MapQR>("W02.00", W0200));
	mapQRs.insert(pair<string, MapQR>("W02.01", W0201));
	mapQRs.insert(pair<string, MapQR>("W02.02", W0202));
	mapQRs.insert(pair<string, MapQR>("W02.03", W0203));
	mapQRs.insert(pair<string, MapQR>("W02.04", W0204));

	mapQRs.insert(pair<string, MapQR>("W03.00", W0300));
	mapQRs.insert(pair<string, MapQR>("W03.01", W0301));
	mapQRs.insert(pair<string, MapQR>("W03.02", W0302));
	mapQRs.insert(pair<string, MapQR>("W03.03", W0303));
	mapQRs.insert(pair<string, MapQR>("W03.04", W0304));

	it = mapQRs.find("W03.00");



	// AR.Drone class	
	ARDrone ardrone;


	cv::Mat map = cv::imread("plantegning_3.png");

	// Initialize
	if (!ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}

	// Battery
	std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

	// Main loop
	while (1) {
		cv::Mat map_copy = map.clone();
		// Key input
		int key = cv::waitKey(33);
		if (key == 0x1b) break;

		// Get an image
		//cv::Mat image = ardrone.getImage();
		//cout << image.channels() << " channels " << endl;
		cv::Mat displayImage = ardrone.getImage();
		cvtColor(displayImage, displayImage, CV_RGB2GRAY);
		adaptiveThreshold(displayImage, displayImage, maxValue, adaptiveMethod, thresholdType, blockSize, C);
		//cout << image.channels() << " channels efter RGB2GRAY" << endl;
		//cv::Mat imgout;
		//cout << image.cols << "image ardrone cols" << endl;
		//cvtColor(image, imgout, CV_GRAY2RGB);
		//cvtColor(image, imgout);

		// Take off / Landing 
		if (key == ' ') {
			if (ardrone.onGround()) {
				//ardrone.takeoff();
				Sleep(10000); //Vent 10 sekunder ved takeoff - yaw bliver resettet
				//Drej 360 grader og detect QR koder
				turn = true;
			}
			else {
				ardrone.landing();
			}
		}

		if (turn) {	//If we have just started the drone - now it's time to turn and scan for QR codes!
			//ardrone.move3D(0.0, 0.0, 0.0, 0.3);
			currentYaw = ardrone.getYaw() * RAD_TO_DEG;
			if (currentYaw < 0) currentYaw += 360.0;
			std::cout << "Current Yaw = " << currentYaw << " degrees." << std::endl;
			if ((currentYaw > targetYaw - SPIN_TOLERANCE) && (currentYaw < targetYaw + SPIN_TOLERANCE)) {
				std::cout << "REACHED TARGET WITHIN TOLERANCE!!" << std::endl;
				turn = false;
			}
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

		//cv::cvtColor(image, image, CV_GRAY2RGB);
		//cv::cvtColor(image, image, CV_RGB2GRAY);
		int width = displayImage.cols;
		int height = displayImage.rows;
		uchar *raw = (uchar *)displayImage.data;
		// wrap image data  
		zbar::Image imageQR(width, height, "Y800", raw, width * height);

		// scan the image for barcodes  
		int n = scanner.scan(imageQR);



		for (zbar::Image::SymbolIterator symbol = imageQR.symbol_begin(); symbol != imageQR.symbol_end(); ++symbol) {
			//QR Analyse!
			//l�st string
			std::string QRString = symbol->get_data();

			//kordinater respektivt til qr koden i pixels

			it = mapQRs.find(symbol->get_data());

			if (QRString.find(groupN) != std::string::npos) {
				cout << "QR belongs to north" << "\n";
				cout << "Drone is located at x+: " << it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol)) << " eller x-: " << it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol)) << " og y:" << it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol)) << "\n";

				cv::circle(map_copy, cv::Point(it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol)), it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
				cv::circle(map_copy, cv::Point(it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol)), it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);

			}

			if (QRString.find(groupE) != std::string::npos) {
				cout << "QR belongs to east" << "\n";
				cout << "Drone is located at x: " << it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol)) << " og y+: " << it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol)) << " eller y-:" << it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol)) << "\n";

				cv::circle(map_copy, cv::Point(it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol)), it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
				cv::circle(map_copy, cv::Point(it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol)), it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
			}

			if (QRString.find(groupS) != std::string::npos) {
				cout << "QR belongs to south" << "\n";
				cout << "Drone is located at x+: " << it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol)) << " eller x-: " << it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol)) << " og y:" << it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol)) << "\n";

				cv::circle(map_copy, cv::Point(it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol)), it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
				cv::circle(map_copy, cv::Point(it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol)), it->second.x - getXCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
			}

			if (QRString.find(groupV) != std::string::npos) {
				cout << "QR belongs to west" << "\n";
				cout << "Drone is located at x: " << it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol)) << " og y+: " << it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol)) << " eller y-:" << it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol)) << "\n";

				cv::circle(map_copy, cv::Point(it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol)), it->second.y + getYCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
				cv::circle(map_copy, cv::Point(it->second.x + getXCordinate(getAngle(symbol), getDistance(symbol)), it->second.y - getYCordinate(getAngle(symbol), getDistance(symbol))), map_drone.circleRadius, CV_RGB(0, 0, 255), 2);
			}

			//Tegn dronens position p� kortet


			//dronens relative position udregnes

		}
		cv::imshow("map", map_copy);
		cv::imshow("Camera", displayImage);
	}
	ardrone.close();

	return 0;
}

//Metode

double getDistance(zbar::Image::SymbolIterator symbol) {
	vector<Point> vp;

	int n = symbol->get_location_size();

	for (int i = 0; i < n; i++) {
		vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
	}
	RotatedRect r = minAreaRect(vp);
	Point2f pts[4];
	r.points(pts);
	//pixels i h�jde
	int np = 0;
	//pixels i bredde
	int lp = 0;

	//Tegning og udregning
	for (int i = 0; i < 4; i++) {

		//Find h�jden p� siden af qr koden
		int pixels = sqrt((pts[i] - pts[(i + 1) % 4]).y * (pts[i] - pts[(i + 1) % 4]).y);

		if (pixels > 20) {
			np = pixels;
		}
		//Find bredden i bunden af qr koden
		pixels = sqrt((pts[i] - pts[(i + 1) % 4]).x * (pts[i] - pts[(i + 1) % 4]).x);
		if (pixels > 20) {
			lp = pixels;
		}
	}

	//Afstanden til siden fundet i mm
	return (215 * 553) / np;
}

double getAngle(zbar::Image::SymbolIterator symbol) {
	vector<Point> vp;

	int n = symbol->get_location_size();

	for (int i = 0; i < n; i++) {
		vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
	}

	RotatedRect r = minAreaRect(vp);
	Point2f pts[4];
	r.points(pts);
	//pixels i h�jde
	int np = 0;
	//pixels i bredde
	int lp = 0;

	//Tegning og udregning
	for (int i = 0; i < 4; i++) {

		//Find h�jden p� siden af qr koden
		int pixels = sqrt((pts[i] - pts[(i + 1) % 4]).y * (pts[i] - pts[(i + 1) % 4]).y);

		if (pixels > 20) {
			np = pixels;
		}

		//Find bredden i bunden af qr koden
		pixels = sqrt((pts[i] - pts[(i + 1) % 4]).x * (pts[i] - pts[(i + 1) % 4]).x);
		if (pixels > 20) {
			lp = pixels;
		}
	}

	//Afstanden til siden fundet i mm
	double nd = (215 * 553) / np;

	//Observeret st�rrelse
	double S = (215.0 / np) * lp;
	if (S > 215) {
		S = 214.9;
	}

	//Udregning af vinklen
	double radianA = 2.0 * atan(150.0 / nd);
	double A = radianA * (180 / 3.14159);
	double radianB = asin(S / 215);

	double B = radianB * (180 / 3.14159);

	double C = 180 - A - B;

	if (S > 214.8) {
		C = 90;
	}

	return C;
}


double getXCordinate(double Ci, double c) {

	return  (sin((180 - Ci) * 3.1415 / 180) * c) / MM_PER_PIXEL_X;
}

double getYCordinate(double Ci, double c) {

	return (sin((90 - (180 - Ci)) * 3.1415 / 180) * c) / MM_PER_PIXEL_Y;
}
#include "ardrone/ardrone.h"
#include "zbar.h" 

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

using namespace cv;
using namespace std;
using namespace zbar;


int main(int argc, char *argv[])
{
	VideoCapture capture(0);
	Mat img;
	Mat imgout;
	ImageScanner scanner;
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);



	while (cvWaitKey(30) != 'q') {
		capture >> img;
		cvtColor(img, imgout, CV_BGR2GRAY);
		int width = img.cols;
		int height = img.rows;
		uchar *raw = (uchar *)img.data;
		// wrap image data  
		Image image(width, height, "Y800", raw, width * height);
		// scan the image for barcodes  
		int n = scanner.scan(image);
		// extract results  
		for (Image::SymbolIterator symbol = image.symbol_begin();
			symbol != image.symbol_end();
			++symbol) {
			vector<Point> vp;
			// do something useful with results  
			cout << "decoded " << symbol->get_type_name()
				<< " symbol \"" << symbol->get_data() << '"' << " " << endl;
			int n = symbol->get_location_size();
			for (int i = 0; i<n; i++) {
				vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
			}
			RotatedRect r = minAreaRect(vp);
			Point2f pts[4];
			r.points(pts);
			for (int i = 0; i<4; i++) {
				line(imgout, pts[i], pts[(i + 1) % 4], Scalar(255, 0, 0), 3);
			}
			cout << "Angle: " << r.angle << endl;
		}
		imshow("Window", imgout);

	}

	return 0;
}
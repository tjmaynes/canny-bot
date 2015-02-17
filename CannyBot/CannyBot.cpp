// File: CannyBot.cpp
// Authors: Tommy Lin, TJ Maynes
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <math.h>
//#include <alcommon/alproxy.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Global variables
#define PI 3.14159265
#define ELBOW_OFFSET_Y 15
#define UPPER_ARM_LENGTH 105
#define SHOULDER_OFFSET_Y 98
#define SHOULDER_OFFSET_Z 100
#define LOWER_ARM_LENGTH 55.95

const int rows = 4;
const int columns = 4;

// function headers
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
double** transformationMatrix(std::string name, double** matrix, int rows, int columns, const int a, const double alpha, const int distance, double theta);
double** multiplyMatrices(double** RShoulderPitch, double** RShoulderRoll, double** RElbowYaw, double** RElbowRoll, double** RWristRoll);
void prettyPrint(std::string name, double** matrix);
std::string roboShapeVision();

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


// helper functions
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void prettyPrint(std::string name, double** matrix){
	std::cout << "\nThis is matrix = " << name << ".\n" << std::endl;
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < columns; j++) {
			if (j == 3){
				std::cout << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(3) << matrix[i][j] << "\n";
			}
			else {
				std::cout << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(3) << matrix[i][j] << " ";
			}
		}
	}
}

// transformation matrix
double** transformationMatrix(std::string name_of_matrix, double** matrix, int rows, int columns, const int a, const double alpha, const int distance, double theta) {
	// perform transformation matrix stuff here
	// theta is the only one changing. which is why a, alpha, and distance is of type const.
	double temp = 0.00;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			if (i == 0 && j == 0){
				temp = std::cos(theta*PI / 180);
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 0 && j == 1){
				temp = (-(std::sin(theta*PI / 180)) * std::cos(alpha*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 0 && j == 2){
				temp = (std::sin(theta*PI / 180) * std::sin(alpha*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 0 && j == 3){
				temp = (a * std::cos(theta*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 1 && j == 0){
				temp = (std::sin(theta*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 1 && j == 1){
				temp = (std::cos(theta*PI / 180) * std::cos(alpha*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 1 && j == 2){
				temp = (-(std::cos(theta*PI / 180)) * std::sin(alpha*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 1 && j == 3){
				temp = (a * std::sin(theta*PI / 180));
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 2 && j == 0){
				temp = 0;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 2 && j == 1){
				temp = std::sin(alpha*PI / 180);
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 2 && j == 2){
				temp = std::cos(alpha*PI / 180);
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 2 && j == 3){
				temp = distance;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 3 && j == 0){
				temp = 0;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 3 && j == 1){
				temp = 0;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 3 && j == 2){
				temp = 0;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
			if (i == 3 && j == 3){
				temp = 1;
				if (temp == -0.0)
					temp = 0.0;
				matrix[i][j] = temp;
			}
		}
	}

	// print resulting matrix
	prettyPrint(name_of_matrix, matrix);

	return matrix;
}
// multiply matrices
double** multiplyMatrices(double** RShoulderPitch, double** RShoulderRoll, double** RElbowYaw, double** RElbowRoll, double** RWristRoll) {
	double temp = 0.0;
	// create matrix
	double** product1 = new double*[columns];
	for (int i = 0; i < rows; i++){
		product1[i] = new double[rows];
	}
	double** product2 = new double*[columns];
	for (int i = 0; i < rows; i++){
		product2[i] = new double[rows];
	}
	double** product3 = new double*[columns];
	for (int i = 0; i < rows; i++){
		product3[i] = new double[rows];
	}

	double** product4 = new double*[columns];
	for (int i = 0; i < rows; i++){
		product4[i] = new double[rows];
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			product1[i][j] = 0;
			for (int inner = 0; inner < 4; inner++)
			{
				product1[i][j] += RShoulderPitch[i][inner] * RShoulderRoll[inner][j];
			}
		}
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			product2[i][j] = 0;
			for (int inner = 0; inner < 4; inner++)
			{
				product2[i][j] += product1[i][inner] * RElbowRoll[inner][j];
			}
		}
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			product3[i][j] = 0;
			for (int inner = 0; inner < 4; inner++)
			{
				product3[i][j] += product2[i][inner] * RElbowYaw[inner][j];
			}
		}
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			product4[i][j] = 0;
			for (int inner = 0; inner < 4; inner++)
			{
				product4[i][j] += product3[i][j] * RWristRoll[inner][j];
			}
		}
	}
	return product4;
}


// captures object very quickly...
// need to make sure camera is already looking at workspace!
//
std::string roboShapeVision() {
	std::cout << "Setting up NAO Robot camera!\n" << std::endl;
	std::string shape = "";

	bool breakout = false;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> approx;
	cv::Mat image, grayImage, blur, canny, dst;
	int lowThreshold;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;
	cv::VideoCapture camera;
	int camOpen = camera.open(CV_CAP_ANY);

	if (!camera.isOpened()) {
		std::cout << "No Camera" << std::endl;
	}
	cv::namedWindow("CameraCapture", CV_WINDOW_AUTOSIZE);

	// wait 3 seconds for robo camera to be ready
	cv::waitKey(3000);

	// breakout of loop when shape is found
	/*while (cvWaitKey(30) != 'q') {*/
	while (!breakout) {
		camera >> image;
		if (true){
			// perform canny edge detection algorithm
			cv::cvtColor(image, grayImage, CV_RGB2GRAY);

			// higher threshold equals less camera noise
			cv::GaussianBlur(image, blur, cv::Size(9, 9), 1 * ratio, kernel_size);
			cv::Canny(blur, canny, 0, 30, 3);

			// show canny window called robovision
			cv::imshow("RoboVision", canny);

			// Find contours
			cv::findContours(canny.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

			image.copyTo(dst);

				for (int i = 0; i < contours.size(); i++)
				{
					// Approximate contour with accuracy proportional
					// to the contour perimeter
					cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

					// Skip small or non-convex objects

					if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx)){
						continue;
					}

					if (approx.size() == 3)
					{
						setLabel(dst, "TRI", contours[i]);
						shape = "Triangle"; // Triangles
						//breakout = true;
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
						if (vtc == 4){
							setLabel(dst, "RECT", contours[i]);
							shape = "Rectangle";
							//breakout = true;
						}
						else if (vtc == 5){
							setLabel(dst, "PEN", contours[i]);
							shape = "Pentagon";
							//breakout = true;
						}
						else if (vtc == 6){
							setLabel(dst, "HEX", contours[i]);
							shape = "Hexagon";
							//breakout = true;
						}
					}
					else
					{
						// Detect and label circles
						double area = cv::contourArea(contours[i]);
						cv::Rect r = cv::boundingRect(contours[i]);
						int radius = r.width / 2;

						if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
							std::abs(1 - (area / (CV_PI * (radius*radius)))) <= 0.2)
						{
							setLabel(dst, "CIR", contours[i]);
							shape = "Circle";
							//breakout = true;
						}
					}
					if (breakout){
						break;
					}
				}
				cv::imshow("dst", dst);
		}
		else {
			break;
		}
	}
	cvDestroyAllWindows();
	return shape;
}

int main() {
	std::cout << "Welcome to the CannyBot program!\n" << std::endl;
	std::string shape = "";

	// return string value containing shape found on workspace.
	shape = roboShapeVision();

	std::cout << "The shape found on the workspace was a " << shape << ".\n" << std::endl;

	// initialize 4x4 matrices
	double** RShoulderPitch = new double*[columns];
	for (int i = 0; i < rows; i++){
		RShoulderPitch[i] = new double[rows];
	}
	double** RShoulderRoll = new double*[columns];
	for (int i = 0; i < rows; i++){
		RShoulderRoll[i] = new double[rows];
	}
	double** RElbowYaw = new double*[columns];
	for (int i = 0; i < rows; i++){
		RElbowYaw[i] = new double[rows];
	}
	double** RElbowRoll = new double*[columns];
	for (int i = 0; i < rows; i++){
		RElbowRoll[i] = new double[rows];
	}
	double** RWristRoll = new double*[columns];
	for (int i = 0; i < rows; i++){
		RWristRoll[i] = new double[rows];
	}
	double** base_to_start = new double*[columns];
	for (int i = 0; i < rows; i++){
		base_to_start[i] = new double[rows];
	}

	// read input file of theta values
	double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, theta4 = 0.0, theta5 = 0.0;
	std::string filename = "input.txt";
	std::ifstream fs;
	fs.open(filename.c_str());
	std::cout << "Processing input file..." << std::endl;

	// read theta values from input file
	fs >> theta1;
	fs >> theta2;
	fs >> theta3;
	fs >> theta4;
	fs >> theta5;

	// stop reading from input file
	std::cout << "Finished processing file." << std::endl;
	fs.close();

	std::cout << "\nThetas are " << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << ", " << theta5 << std::endl;

	// create matrices
	// pass theta value per row to specified matrix for Right Arm of NAO Robot
	RShoulderPitch = transformationMatrix("RShoulderPitch", RShoulderPitch, rows, columns, 0, -(PI / 2.0), 0, theta1);
	RShoulderRoll = transformationMatrix("RShoulderRoll", RShoulderRoll, rows, columns, 0, (PI / 2.0), 0, theta2 + (PI / 2.0));
	RElbowYaw = transformationMatrix("RElbowYaw", RElbowYaw, rows, columns, -ELBOW_OFFSET_Y, (PI / 2.0), UPPER_ARM_LENGTH, theta3);
	RElbowRoll = transformationMatrix("RElbowRoll", RElbowRoll, rows, columns, 0, -(PI / 2.0), 0, theta4);
	RWristRoll = transformationMatrix("RWristYRoll", RWristRoll, rows, columns, LOWER_ARM_LENGTH, (PI / 2.0), 0, theta5);

	// mulitple matrices together for matrix
	base_to_start = multiplyMatrices(RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristRoll);

	// print resulting base_to_start matrix
	prettyPrint("base_to_start", base_to_start);

	// pass shape value to somewhere


	// program exit
	std::cout << "\nend of program" << std::endl;
	return 0;
}

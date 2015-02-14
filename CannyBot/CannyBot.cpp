// File: CannyBot.cpp
// Authors: Tommy Lin, TJ Maynes
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define PI 3.14159265
#define ELBOW_OFFSET_Y 15
#define UPPER_ARM_LENGTH 105
#define SHOULDER_OFFSET_Y 98
#define SHOULDER_OFFSET_Z 100

// function headers
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
double** transformationMatrix(std::string name, double** matrix, int rows, int columns, const int a, const double alpha, const int distance, double theta);
double** multiplyMatrices(double** RShoulderPitch, double** RShoulderRoll, double** RElbowYaw, double** RElbowRoll);
void prettyPrint(std::string name, double** matrix);
std::string roboShapeVision();

const int rows = 4;
const int columns = 4;

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

double** transformationMatrix(std::string name_of_matrix, double** matrix, int rows, int columns, const int a, const double alpha, const int distance, double theta) {
	// perform transformation matrix stuff here
	// theta is the only one changing. which is why a, alpha, and distance is of type const.
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			if (i == 0 && j == 0)
				matrix[i][j] = std::cos(theta);

			if (i == 0 && j == 1)
				matrix[i][j] = (-std::sin(theta) * std::cos(alpha));

			if (i == 0 && j == 2)
				matrix[i][j] = (std::sin(theta) * std::sin(alpha));

			if (i == 0 && j == 3)
				matrix[i][j] = (a * std::cos(theta));

			if (i == 1 && j == 0)
				matrix[i][j] = (std::sin(theta));

			if (i == 1 && j == 1)
				matrix[i][j] = (std::cos(theta) * std::cos(alpha));

			if (i == 1 && j == 2)
				matrix[i][j] = (-std::cos(theta) * std::sin(alpha));

			if (i == 1 && j == 3)
				matrix[i][j] = (a * std::sin(theta));

			if (i == 2 && j == 0)
				matrix[i][j] = 0;

			if (i == 2 && j == 1)
				matrix[i][j] = std::sin(alpha);

			if (i == 2 && j == 2)
				matrix[i][j] = std::cos(alpha);

			if (i == 2 && j == 3)
				matrix[i][j] = distance;

			if (i == 3 && j == 0)
				matrix[i][j] = 0;

			if (i == 3 && j == 1)
				matrix[i][j] = 0;

			if (i == 3 && j == 2)
				matrix[i][j] = 0;

			if (i == 3 && j == 3)
				matrix[i][j] = 1;
		}
	}

	// print resulting matrix
	prettyPrint(name_of_matrix, matrix);

	return matrix;
}

double** multiplyMatrices(double** RShoulderPitch, double** RShoulderRoll, double** RElbowYaw, double** RElbowRoll) {
	double**product = new double*[columns];
	for (int i = 0; i < rows; i++){
		product[i] = new double[rows];
	}
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			for (int inner = 0; inner < 4; inner++)
			{
				product[i][j] = RShoulderPitch[i][inner] * RShoulderRoll[inner][j];
			}
		}
	}
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			for (int inner = 0; inner < 4; inner++)
			{
				product[i][j] *= RElbowYaw[inner][j];
			}
		}
	}
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			for (int inner = 0; inner < 4; inner++)
			{
				product[i][j] *= RElbowRoll[inner][j];
			}
		}
	}
	return product;
}

void prettyPrint(std::string name, double** matrix){
	std::cout << "\nThis is matrix = " << name << ".\n" << std::endl;
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < columns; j++){
			if (j == 3){
				std::cout << std::setw(6) << matrix[i][j] << "\n";
			} else {
				std::cout << std::setw(6) << matrix[i][j] << " ";
			}
		}
	}
	std::cout << " " << std::endl;
}

/*
std::string roboShapeVision() {
std::cout << "Setting up NAO Robot camera!\n" << std::endl;
std::string shape = "";

cv::VideoCapture camera;
int camOpen = camera.open(CV_CAP_ANY);

if (!camera.isOpened()) {
std::cout << "No Camera" << std::endl;
}
cv::namedWindow("CameraCapture", CV_WINDOW_AUTOSIZE);

std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Point> approx;
cv::Mat image, grayImage, blur, canny;

cv::waitKey(3000);
while (true) {
camera >> image;
cv::imshow("Streaming", image);

//cv::imshow("Streaming", image);

// perform canny edge detection algorithm
if (image.empty())
break;
else if (image.channels() > 1)
cv::cvtColor(image, grayImage, CV_GRAY2RGB);
else
grayImage = image;
cv::GaussianBlur(image, blur, cv::Size(7, 7), 1.5, 1.5);
cv::Canny(blur, canny, 0, 30, 3);

// show canny window called robovision
cv::imshow("RoboVision", canny);


// break out of contour drawing
// Find contours
cv::findContours(canny.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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
shape = "Triangle"; // Triangles
break;
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
shape = "Rectangle";
break;
}
else if (vtc == 5){
shape = "Pentagon";
break;
}
else if (vtc == 6){
shape = "Hexagon";
break;
}
}
else
{
// Detect and label circles
double area = cv::contourArea(contours[i]);
cv::Rect r = cv::boundingRect(contours[i]);
int radius = r.width / 2;

if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
std::abs(1 - (area / (CV_PI * (radius*radius)))) <= 0.2){
shape = "Circle";
break;
}
}
}
if (cv::waitKey(30) >= 0) {
break;
}
}
return shape;
}
*/
int main() {
	std::cout << "Welcome to the CannyBot program!" << std::endl;
	std::string shape = "", matrix1 = "RShoulderPitch", matrix2 = "RShoulderRoll", matrix3 = "RElbowYaw", matrix4="RElbowRoll", result_matrix = "base_to_start";

	// return string value containing shape found on workspace.
	//shape = roboShapeVision();

	std::cout << "The shape found on the workspace was a " << shape << "." << std::endl;

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
	double** base_to_start = new double*[columns];
	for (int i = 0; i < rows; i++){
		base_to_start[i] = new double[rows];
	}

	// read input file of theta values
	double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, theta4 = 0.0;
	std::string filename = "input.txt";
	std::ifstream fs;
	fs.open(filename.c_str());
	std::cout << "Processing input file..." << std::endl;

	// read theta values from input file
	fs >> theta1;
	fs >> theta2;
	fs >> theta3;
	fs >> theta4;

	// stop reading from input file
	std::cout << "Finished processing file." << std::endl;
	fs.close();

	// create matrices
	// pass theta value per row to specified matrix for Right Arm of NAO Robot
	RShoulderPitch = transformationMatrix(matrix1, RShoulderPitch, rows, columns, 0, -(PI / 2.0), 0, theta1);
	RShoulderRoll = transformationMatrix(matrix2, RShoulderRoll, rows, columns, 0, (PI / 2.0), 0, theta2 + (PI / 2.0));
	RElbowYaw = transformationMatrix(matrix3, RElbowYaw, rows, columns, -ELBOW_OFFSET_Y, (PI / 2.0), UPPER_ARM_LENGTH, theta3);
	RElbowRoll = transformationMatrix(matrix4, RElbowRoll, rows, columns, 0, -(PI / 2.0), 0, theta4);

	// mulitple matrices together for matrix
	base_to_start = multiplyMatrices(RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll);

	// print resulting base_to_start matrix
	prettyPrint(result_matrix, base_to_start);

	// program exit
	return 0;
}

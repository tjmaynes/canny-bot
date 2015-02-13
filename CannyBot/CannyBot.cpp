// File: CannyBot.cpp
// Authors: Tommy Lin, TJ Maynes
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include "opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

#define PI 3.14159265
#define ELBOW_OFFSET_Y 15.00
#define UPPER_ARM_LENGTH 105.00
#define SHOULDER_OFFSET_Y 98.00
#define SHOULDER_OFFSET_Z 100.00

float** transformationMatrix(float** matrix, int rows, int columns, const int a, const double alpha, const int distance, double theta){
	// perform transformation matrix stuff here
	// theta is the only one changing. which is why a, alpha, and distance is of type const.
	return matrix;
}

float** translationMatrix(float** matrix){
	return matrix;
}

void setupCamera(){
	cv::VideoCapture cap(0); // open the video camera no. 0
	if (!cap.isOpened())  // if not success, exit program
	{
		std::cout << "Cannot open the video cam" << std::endl;
	}
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	std::cout << "Frame size : " << dWidth << " x " << dHeight << std::endl;
	cv::namedWindow("OpenCVWindow", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	while (1)
	{
		cv::Mat frame;
		bool bSuccess = cap.read(frame); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
			std::cout << "Cannot read a frame from video stream" << std::endl;
			break;
		}
		cv::imshow("OpenCVFrame", frame); //show the frame in "MyVideo" window
		if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			std::cout << "esc key is pressed by user" << std::endl;
			break;
		}
	}
}

int main(){
	int rows = 4;
	int columns = 4;

	// initialize 2D matrices
	float** matrix_one = new float*[columns];
	for (int i = 0; i < rows; i++){
		matrix_one[i] = new float[rows];
	}
	float** matrix_two = new float*[columns];
	for (int i = 0; i < rows; i++){
		matrix_two[i] = new float[rows];
	}
	float** matrix_three = new float*[columns];
	for (int i = 0; i < rows; i++){
		matrix_three[i] = new float[rows];
	}
	float** matrix_four = new float*[columns];
	for (int i = 0; i < rows; i++){
		matrix_four[i] = new float[rows];
	}

	float ** base_to_start = new float*[columns];
	for (int i = 0; i < rows; i++){
		base_to_start[i] = new float[rows];
	}

	// read input file of theta values
	std::string filename = "input.txt";
	std::ifstream fs;
	fs.open(filename.c_str());
	std::cout << "Processing input file...\n" << std::endl;

	// pass theta value per row to specified matrix for Right Arm of NAO Robot
	double theta = 0.0;

	// RShoulderPitch
	fs >> theta;
	matrix_one = transformationMatrix(matrix_one, rows, columns, 0, -(PI/2.0), 0, theta);

	// RShoulderRoll
	fs >> theta;
	matrix_two = transformationMatrix(matrix_two, rows, columns, 0, (PI / 2.0), 0, theta + (PI / 2.0));

	// RElbowYaw
	fs >> theta;
	matrix_three = transformationMatrix(matrix_three, rows, columns, -ELBOW_OFFSET_Y, (PI / 2.0), UPPER_ARM_LENGTH, theta);

	// RElbowRoll
	fs >> theta;
	matrix_four = transformationMatrix(matrix_four, rows, columns, 0, -(PI / 2.0), 0, theta);

	// stop reading from input file
	std::cout << "Finished processing file." << std::endl;
	fs.close();

	// create translation matrices
	matrix_one = translationMatrix(matrix_one);
	matrix_two = translationMatrix(matrix_two);
	matrix_three = translationMatrix(matrix_three);
	matrix_four = translationMatrix(matrix_four);

	// mulitple matrices together for matrix
	//base_to_start = matrix_one*matrix_two*matrix_three*matrix_four;

	// setup camera / do this async??
	//setupCamera();

}
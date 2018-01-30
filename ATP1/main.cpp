//spark
#include <stdlib.h>
#include <iostream>
#include <fstream>   

#include <FlyCapture2.h>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <string.h>
#include <tchar.h>
#include <math.h>
#include "Thorlabs.MotionControl.KCube.Piezo.h"

#include <stdio.h>  
#include <pthread.h>  
#include <assert.h>   

#pragma once

using namespace std;
using namespace FlyCapture2;

cv::Point2f GetSpotCenter(Camera& camera);



int main()
{
	// initialize camera
	Error error;
	Camera camera;
	CameraInfo camInfo;
	int sleeptime = 30;

	// Connect the camera
	error = camera.Connect(0);
	error = camera.StartCapture();
	camera.GetCameraInfo(&camInfo);

	std::cout << camInfo.vendorName << " "
		      << camInfo.modelName << " "
		      << camInfo.serialNumber << std::endl;

	// change shutter using camera internal unit
	Property shutter;
	shutter.type = SHUTTER;
	shutter.absControl = false;
	shutter.valueA = 2000;
	camera.SetProperty(&shutter);

	// Get the image
	Image monoImage;
	Sleep(sleeptime);
	camera.RetrieveBuffer(&monoImage);


	// convert to OpenCV Mat
	unsigned int rowBytes = (double)monoImage.GetReceivedDataSize() / (double)monoImage.GetRows();
	cv::Mat image = cv::Mat(monoImage.GetRows(), monoImage.GetCols(), CV_8UC1, monoImage.GetData(), rowBytes);


	//cv::namedWindow("win");
	//cv::imshow("win", image);
	//cv::waitKey();

	// initialize the kpz101

	// Build list of the connected device
	TLI_BuildDeviceList();

	//get device list size
	short n = TLI_GetDeviceListSize();
	printf("Found Device matched: %d!\r\n", n);
	char *context = NULL;
	//get BBD serial numbers
	char serialNos[100] = { '0' };
	TLI_GetDeviceListByTypeExt(serialNos, 100, 29);
	std::vector<std::string> serialNo_total(0);

	//output list of matching devices
	char *p = strtok_s(serialNos, ",", &context);
	while (p != NULL)
	{
		TLI_DeviceInfo deviceInfo;
		//get device info form device
		TLI_GetDeviceInfo(p, &deviceInfo);
		//get strings from device info structure
		char desc[65];
		strncpy_s(desc, deviceInfo.description, 64);
		desc[64] = '\0';
		char serialNo[9];
		strncpy_s(serialNo, deviceInfo.serialNo, 8);
		serialNo[8] = '\0';
		std::string serialNo_temp;
		serialNo_temp = std::string(serialNo);
		serialNo_total.push_back(serialNo_temp);
		// output
		printf("Found Device %s=%s : %s\r\n", p, serialNo, desc);
		p = strtok_s(NULL, ",", &context);
	}
	char testSerialNo[2][9] = { '\0' };
	short maximumoutput = 150;
	short setmaximumoutput = maximumoutput * 10;
	for (short i = 0; i < n; i++)
	{
		strncpy_s(testSerialNo[i], serialNo_total[i].c_str(), serialNo_total[i].length());
		PCC_Open(testSerialNo[i]);
		PCC_CheckConnection(testSerialNo[i]);
		PCC_Identify(testSerialNo[i]);
		PCC_Enable(testSerialNo[i]);
		PCC_SetZero(testSerialNo[i]);
		PCC_SetPositionControlMode(testSerialNo[i], PZ_ControlModeTypes::PZ_OpenLoop);
		PCC_SetVoltageSource(testSerialNo[i], PZ_InputSourceFlags::PZ_Potentiometer);
		PCC_SetMaxOutputVoltage(testSerialNo[i], setmaximumoutput);
	}




	// set initial voltage
	double outputvoltage0 = double(75);
	double outputvoltage1 = double(75);
	short setoutputvoltage0 = short(outputvoltage0 / double(maximumoutput) * 32767.0);
	short setoutputvoltage1 = short(outputvoltage1 / double(maximumoutput) * 32767.0);
	PCC_SetOutputVoltage(testSerialNo[0], setoutputvoltage0);
	PCC_SetOutputVoltage(testSerialNo[1], setoutputvoltage1);

	Sleep(sleeptime);

	// get init position for the spot
	cv::Point2f initMassCenter = GetSpotCenter(camera);

	cout << "initial position: " << initMassCenter << endl;



	int calib_iter = 50;
	int calib_iter_start = 1;
	// initialize the drift val table
	vector<cv::Point2d> drift_tab(0);

	// calibration loop for pizo_0
	for (int j = calib_iter_start; j <= calib_iter; j++) // calibrate with calib_iter 
	{

		// change voltage
		double outputvoltage = double(j);
		short setoutputvoltage = short(outputvoltage / double(maximumoutput) * 32767.0);
		PCC_SetOutputVoltage(testSerialNo[0], setoutputvoltage);

		Sleep(sleeptime);

		cv::Point2f tempMassCenter = GetSpotCenter(camera);

		cout << "temp position: " << tempMassCenter << endl;

		// save current drift to drift table
		drift_tab.push_back(tempMassCenter - initMassCenter);

	}

	// save calibration data to file
	std::vector<double> offset_x(0);
	std::vector<double> offset_y(0);

	fstream outputFile;
	outputFile.open("outputFile_controller0.txt", std::ios::out);
	for (size_t ii = 0; ii < drift_tab.size(); ++ii)
	{
		outputFile << drift_tab[ii].x << "," << drift_tab[ii].y << std::endl;
		offset_x.push_back(drift_tab[ii].x);
		offset_y.push_back(drift_tab[ii].y);
	}
	outputFile.close();


	// find the correspondence between the port and offset 
	cv::Mat x_mean_mat, y_mean_mat, x_std_mat, y_std_mat;
	double x_std, y_std, x_mean, y_mean;
	cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
	cv::meanStdDev(offset_y, y_mean_mat, y_std_mat);
	x_std = x_std_mat.at<double>(0, 0);
	y_std = y_std_mat.at<double>(0, 0);
	x_mean = x_mean_mat.at<double>(0, 0);
	y_mean = y_mean_mat.at<double>(0, 0);

	short sign_x = 1, sign_y = 1;
	// if the controller[0] controls y axis
	if (x_std < y_std)
	{
		char testSerialNo_temp[9] = { '\0' };
		strcpy_s(testSerialNo_temp, testSerialNo[0]);
		strcpy_s(testSerialNo[0], testSerialNo[1]);
		strcpy_s(testSerialNo[1], testSerialNo_temp);
		if (y_mean < 0)
		{
			sign_y = -1;
		}
	}
	else
	{
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}

	cout << "calibrate next controller: " << endl;

	if (x_std < y_std)
	{
		for (int j = calib_iter_start; j <= calib_iter; j++) // calibrate with calib_iter 
		{

			// change voltage
			double outputvoltage = double(j);
			short setoutputvoltage = short(outputvoltage / double(maximumoutput) * 32767.0);
			PCC_SetOutputVoltage(testSerialNo[0], setoutputvoltage);

			Sleep(sleeptime);

			cv::Point2f tempMassCenter = GetSpotCenter(camera);

			// x,y location of the mass center
			double tempxloc = tempMassCenter.x;
			double tempyloc = tempMassCenter.y;

			cout << "temp position: " << tempMassCenter << endl;

			// save current drift to drift table
			drift_tab.push_back(tempMassCenter - initMassCenter);

		}
		cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
		x_mean = x_mean_mat.at<double>(0, 0);
		y_mean = y_mean_mat.at<double>(0, 0);
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}
	else
	{
		for (int j = calib_iter_start; j <= calib_iter; j++) // calibrate with calib_iter 
		{

			// change voltage
			double outputvoltage = double(j);
			short setoutputvoltage = short(outputvoltage / double(maximumoutput) * 32767.0);
			PCC_SetOutputVoltage(testSerialNo[1], setoutputvoltage);

			Sleep(sleeptime);

			cv::Point2f tempMassCenter = GetSpotCenter(camera);

			// x,y location of the mass center
			double tempxloc = tempMassCenter.x;
			double tempyloc = tempMassCenter.y;

			cout << "temp position: " << tempMassCenter << endl;

			// save current drift to drift table
			drift_tab.push_back(tempMassCenter - initMassCenter);

		}
		cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
		x_mean = x_mean_mat.at<double>(0, 0);
		y_mean = y_mean_mat.at<double>(0, 0);
		if (y_mean < 0)
		{
			sign_x = -1;
		}
	}

	outputFile.open("outputFile_controller1.txt", std::ios::out);
	for (size_t ii = 0; ii < drift_tab.size(); ++ii)
	{
		outputFile << drift_tab[ii].x << "," << drift_tab[ii].y << std::endl;
		offset_x.push_back(drift_tab[ii].x);
		offset_y.push_back(drift_tab[ii].y);
	}
	outputFile.close();

	// start ATP from a bad position
	outputvoltage0 = double(35);
	outputvoltage1 = double(125);
	setoutputvoltage0 = short(outputvoltage0 / double(maximumoutput) * 32767.0);
	setoutputvoltage1 = short(outputvoltage1 / double(maximumoutput) * 32767.0);
	PCC_SetOutputVoltage(testSerialNo[0], setoutputvoltage0);
	PCC_SetOutputVoltage(testSerialNo[1], setoutputvoltage1);

	Sleep(sleeptime);

	// capture loop
	char key = 0;
	while (key != 'q')
	{
		cv::Point2f currentMassCenter = GetSpotCenter(camera);

		// x,y location of the mass center
		double xloc = currentMassCenter.x;
		double yloc = currentMassCenter.y;


		double error_tolerance = 0.1;
		//double position_error_x = 10.0, position_error_y = 10.0;
		double kp_0 = 5.5, kp_1 = 5.5;    // Proportion
		double ki_0 = 0.8, ki_1 = 0.8; // Integral    
		double kd_0 = 0.0, kd_1 = 0.0; // Derivative
		double x_error_present = xloc - initMassCenter.x, y_error_present = yloc - initMassCenter.y;
		double delta_v0 = 0, delta_v1 = 0, v0 = outputvoltage0, v1 = outputvoltage1;
		double x_error_last = 0, y_error_last = 0;
		double x_error_previous = 0, y_error_previous = 0;
		std::vector<double> position_error_x(0);
		std::vector<double> position_error_y(0);

		int run_times_1 = 0;
		while (abs(x_error_present) > error_tolerance && run_times_1 < 500)
		{
			delta_v0 = kp_0 * (x_error_present - x_error_last)
				+ ki_0 * x_error_present
				+ kd_0 * (x_error_present - 2 * x_error_last + x_error_previous);

			v0 += delta_v0;
			run_times_1++;

			if (v0 >= 150)
			{
				v0 = 150;
				cout << "voltage too high" << endl;
			}

			if (v0 <= 0)
			{
				v0 = 0;
				cout << "voltage too low" << endl;
			}

			// set output voltage
			double outputvoltage0 = double(v0);
			short setoutputvoltage0 = short(outputvoltage0 / double(maximumoutput) * 32767.0);
			PCC_SetOutputVoltage(testSerialNo[0], setoutputvoltage0);
			Sleep(sleeptime);

			// read corrected position
			cv::Point2f correctedMassCenter = GetSpotCenter(camera);

			// x,y location of the corrected mass center
			xloc = correctedMassCenter.x;

			x_error_previous = x_error_last;

			x_error_last = x_error_present;

			x_error_present = xloc - initMassCenter.x;
			position_error_x.push_back(x_error_present);

		}

		int run_times_2 = 0;
		while (abs(y_error_present) > error_tolerance && run_times_2 < 500)
		{
			delta_v1 = kp_1 * (y_error_present - y_error_last)
				+ ki_1 * y_error_present
				+ kd_1 * (y_error_present - 2 * y_error_last + y_error_previous);

			v1 -= delta_v1;
			run_times_2++;

			if (v1 >= 150)
			{
				v1 = 150;
				cout << "voltage too high" << endl;
			}

			if (v1 <= 0)
			{
				v1 = 0;
				cout << "voltage too low" << endl;
			}

			// set output voltage
			double outputvoltage1 = double(v1);
			short setoutputvoltage1 = short(outputvoltage1 / double(maximumoutput) * 32767.0);
			PCC_SetOutputVoltage(testSerialNo[1], setoutputvoltage1);

			Sleep(sleeptime);

			// read corrected position
			cv::Point2f correctedMassCenter = GetSpotCenter(camera);

			// x,y location of the corrected mass center
			xloc = correctedMassCenter.x;
			yloc = correctedMassCenter.y;

			y_error_previous = y_error_last;

			y_error_last = y_error_present;

			y_error_present = yloc - initMassCenter.y;

			position_error_y.push_back(y_error_present);

		}

		//for debug 
		//fstream outputFile;
		//outputFile.open("output_error.txt", std::ios::out);
		//for (short ii = 0; ii < position_error.size(); ii++)
		//{
		//	outputFile << position_error[ii] << std::endl;
		//}
		//outputFile.close();

	}


	error = camera.StopCapture();
	if (error != PGRERROR_OK)
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}

	for (int m = 0; m < n; m++)
	{
		PCC_Close(testSerialNo[m]);
	}


	return 0;
}








cv::Point2f GetSpotCenter(Camera& camera)
{

	Image monoImage;
	camera.RetrieveBuffer(&monoImage);
	//Sleep(100);

	// convert to OpenCV Mat
	unsigned int rowBytes = (double)monoImage.GetReceivedDataSize() / (double)monoImage.GetRows();
	cv::Mat image = cv::Mat(monoImage.GetRows(), monoImage.GetCols(), CV_8UC1, monoImage.GetData(), rowBytes);


	// blur it slightly
	cv::Mat blurred;
	cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0);

	// threshold it
	cv::Mat thresh;
	cv::threshold(blurred, thresh, 100, 255, cv::THRESH_BINARY);

	// find contours in the thresholded image
	cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// find the largest contour
	int largest_area = 0;
	int largest_contour_index = 0;

	for (size_t i = 0; i < contours.size(); i++)  // iterate through each contour.
	{
		double area = contourArea(contours[i]);  //  Find the area of contour

		if (area > largest_area)
		{
			largest_area = area;
			largest_contour_index = i;           //Store the index of largest contour
		}
	}



	for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
	{
		cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
		drawContours(dst, contours, idx, color, CV_FILLED, 8, hierarchy);
	}

	// Get the moments
	vector<cv::Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}


	// Get the mass centers:
	vector<cv::Point2f> mc(contours.size());
	//for (int i = 0; i < contours.size(); i++)
	for (int i = largest_contour_index; i <= largest_contour_index; i++) // only excute once
	{
		mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

		cv::circle(image, mc[i], 2, (255, 255, 255), -1);

		cv::Point Orig;
		Orig.x = mc[i].x - 20;
		Orig.y = mc[i].y - 20;

		cv::putText(image, "center", Orig, cv::FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1);

	}

	cv::drawContours(image, contours, -1, (255, 255, 0), 2);

	//cv::namedWindow("win");
	//cv::imshow("win", image);
	//cv::waitKey();

	//camera.StopCapture();

	return mc[largest_contour_index];
}

// massCenter = getSpotCenter(camera);

// xloc = massCenter.x;
// yloc = massCenter.y;



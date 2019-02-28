#include "Minimap.hpp"

#include <opencv2/opencv.hpp>
#include <stdio.h>

int main(int argc, char *argv[])
{
	printf("USAGE: This program helps to test the Minimap module."
			"\n"
			"W - step forward\n"
			"A - rotate left\n"
			"D - rotate right\n"
			"S - rotate around\n"
			"P - toggle path visibility\n"
			"5 - Print path coordinates\n"
			"C - Capture Map\n"
			"\n"
			"setTile() is called after every stepForward\n"
			"draw() is called in a loop to update the screen\n\n");

	int cameraIndex = (argc - 1 > 0) ? atoi(argv[1]) : 0;
	cv::VideoCapture cap{cameraIndex};

	// Create a new minimap
	Minimap minimap{10, 10, 128};

	cv::Mat screen{512, 600, CV_8UC3};

	{
		cv::Mat startTile{512, 512, CV_8UC3};
		startTile.setTo(cv::Scalar{255, 255, 255});
		cv::putText(startTile, "START", {0, 300}, cv::FONT_HERSHEY_COMPLEX, 5, {0, 0, 0}, 10);
		minimap.setTile(startTile, {0, 0}, {511, 0}, {0, 511}, {511, 511});
	}

	int pressed_key = 0;

	while (pressed_key != 'q')
	{
		cv::Mat frame;
		cap >> frame;

		int minSize = std::min(frame.rows, frame.cols);
		int rowMargin = (frame.rows - minSize) / 2;
		int colMargin = (frame.cols - minSize) / 2;
		cv::Point2f topLeft		{(float)0 + colMargin,			(float)rowMargin};
		cv::Point2f topRight	{(float)minSize + colMargin,	(float)rowMargin};
		cv::Point2f botLeft		{(float)0 + colMargin,			(float)rowMargin + minSize};
		cv::Point2f botRight	{(float)minSize + colMargin,	(float)rowMargin + minSize};

		screen.setTo(cv::Scalar{255, 0, 255});
		minimap.draw(screen);
		cv::imshow("w", screen);

		pressed_key = cv::waitKey(1);

		switch(pressed_key)
		{
			case 'w':
				minimap.stepForward();
				minimap.setTile(frame, topLeft, topRight, botLeft, botRight);
				break;
			case 'a':
				minimap.rotateLeft();
				break;
			case 's':
				minimap.rotateAround();
				break;
			case 'd':
				minimap.rotateRight();
				break;
			case 'p':
				minimap.togglePath();
				break;
			case '5':
				minimap.printPath();
				break;
			case 'c':
				minimap.saveMap("img/my_map.jpg");
				break;
		}
	}

	return 0;
}

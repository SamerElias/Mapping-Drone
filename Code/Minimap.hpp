#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class Minimap{
    private:
        cv::Size mapSize;
        int tileSize;
        int direction; //the direction the drone is facing
        cv::Point2f currentPoint;   //current location
        cv::Point2f prevPoint;      //previous point
        cv::Point2i currentCoords;  //current coordinates
        cv::Mat fullBoard;          //our map
        cv::Mat fullBoardWithPath;  //our map with path

        cv::Mat pathLayer;      //layer for path only
        cv::Mat pathGray;       //same as pathLayer but grayscale
        cv::Mat pathMask;       //mask layer; only non transparent area
        bool pathFlag;          //path visibility flag
        std::vector<cv::Point2f> path;	//to save path coordinates

        void drawPath();        //draws path
        void drawLegend(cv::Mat &screen);   //draws legend in corner of screen

    public:
        // width - the number of tiles on the X axis
        // height - the number of tiles on the Y axis
        // tileSize - the size of each tile in pixels
        Minimap(int width, int height, int tileSize);

		// Draws the map on the screen matrix.
		// screen will be of type CV_8UC3
		// Different calls to draw may have different screen sizes.
		void draw(cv::Mat &screen);

		// Places the photo into the current tile.
		// photo - a CV_8UC3 photo from the drone.
		// The points that follow - the postiion of the four tile corners
		// in the current frame, in pixel coordinates.
        void setTile(const cv::Mat &photo,
                    cv::Point2f topLeft,
                    cv::Point2f topRight,
                    cv::Point2f botLeft,
                    cv::Point2f botRight);

		// The drone moves on a 2D grid.
		// stepForward moves the drone forward one tile
		// rotateLeft rotates the drone 90 degrees counter-clockwise
		// rotateRight rotates the drone 90 degrees clockwise
		// rotateAround rotates the drone 180 degrees
		void stepForward();
		void rotateLeft();
		void rotateRight();
		void rotateAround();

        // Saves Map depending on received location.
        void saveMap(const std::string& path);

        // Switches between showing/hiding path
        void togglePath();

        // Prints coordinates to console.
        // Prints distance from starting point
        void printPath();
};
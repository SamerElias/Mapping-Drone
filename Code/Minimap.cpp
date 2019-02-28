#include "Minimap.hpp"
#include <sstream>
#include <iomanip>

using namespace std;
using namespace cv;

Minimap::Minimap(int width, int height, int tileSize){
    //Set width and height of map from given input:
    //  Given the size of a single tile, we multiply it by the number of
    //  tiles on the X axis to set the total map width in pixels, and by
    //  number of tiles on the Y axis to set map height.
    //  To keep a centralized tile, we added and extra tile to width/height
    //  if the given width/height is even.
    this->tileSize = tileSize;
    if(!(width % 2))
        mapSize.width = (width+1)*tileSize;
    else
        mapSize.width = width*tileSize;
    if(!(height % 2))
        mapSize.height = (height+1)*tileSize;
    else
        mapSize.height = height*tileSize;
    //Set current point:
    //  currentPoint starts exactly at the center of our Map, this
    //  assumption was varified by Andrey.
    //  currentPoint is mainly used to keep track of drones current
    //  location in pixels.
    currentPoint = Point2f((mapSize.width/2),(mapSize.height/2));
    //Set previous point:
    //  We start with prevPoint being equal to currentPoint.
    //  prevPoint is used when drawing the path from previous to
    //  current point, and for recovering previous location if
    //  the drone exits area's bound.
    prevPoint = currentPoint;
    //Set current coordination:
    //  Since the drone is assumed to start at point (0,0) we give
    //  that value to currentCoords.
    //  currentCoords is used for showing readable and understandable
    //  output to user.
    currentCoords = Point2i(0,0);
    //push point (0,0) to path vector
    // param path saves all points reached by the drone
    path.push_back(currentCoords);
    //create Map board and fill area with gray
    //  We have two matrices:
    //      fullBoard: holds the map.
    //      fullBoardWithPath: hold the map with the path.
    fullBoard = Mat(mapSize,CV_8UC3,Scalar(60,60,60));
	//create Map board copy which includes path lines
    fullBoardWithPath = fullBoard.clone();
    //create path layers
    //  the following layers will hold the path lines.
    //  More detailes on how they are used are in drawPath function.
    pathLayer = Mat(mapSize, CV_8UC3);     //creat layer for path only
    pathGray = Mat(mapSize, CV_8UC3);      //path gray
    pathMask = Mat(mapSize, CV_8UC3);      //path mask
    //initiate path visibility to be false
    pathFlag = false;
    //drone's starting direction is now our North.
    //Available values are: 0=North, 1=East, 2=South, 3=West.
    direction = 0;
}


void Minimap::draw(Mat &screen){
    /*  screen is received as output and we need to draw our map in it,
    *   since screen can change size mid running, it’s size needs to be
    *   checked in each iteration.
    */
    screen.setTo(Scalar(0,0,0));    //set screen to black
    //create marginBoard.
    //  Copying from an available are will crash the program, that's why
    //  marginBoard is a Mat the size of the map plus twice the size of screen,
    //  this will make the area we copy from always exists, preventing a fatal error
    Mat marginBoard = Mat(mapSize.height+(2*screen.rows),mapSize.width+(2*screen.cols),CV_8UC3,Scalar(0,0,0));
    //get map depending on path visibility
    Mat finalMap = pathFlag?fullBoardWithPath:fullBoard;
    //copy map to center of marginBoard
    finalMap.copyTo(marginBoard(Rect(screen.cols,screen.rows,mapSize.width,mapSize.height)));
    //copy marginBoard to screen
    //  visBox selects to area to copy from
    Rect visBox = (Rect(currentPoint.x+(screen.cols/2),currentPoint.y+(screen.rows/2),screen.cols,screen.rows));
    marginBoard(visBox).copyTo(screen(Rect(0,0,screen.cols,screen.rows)));
    //draw legend in corner, more detailes in drawLegend function.
    drawLegend(screen);
    

    /** Alternative code **
     *   The following code does what is needed from draw function without creating marginBoard Matrix.
     *   instead, only the needed areas are copied directly from our map to screen, which makes it overall
     *   more efficient, yet we opted for the code above since it was thoroughly tested and looks more clean.
     *   In the alternative option we had to address each corner separately, which resulted in a
     *   complicated diagram of conditions that are yet to cover all possible situations, for example:
     *   two/four corners are visible at the same time. */
    /*
    screen.setTo(0);
    Mat finalMap = pathFlag ? fullBoardWithPath : fullBoard;
    Point2f fromTRcorner,fromBLcorner,toTRcorner,toBLcorner;
    Rect from, to;
    fromTRcorner = Point2f(max(0.0f,currentPoint.x-(screen.cols/2)),max(0.0f,currentPoint.y-(screen.rows/2)));
    fromBLcorner = Point2f(min((float)mapSize.width,currentPoint.x+(screen.cols/2)),min((float)mapSize.height,currentPoint.y+(screen.rows/2)));
	//finding the corners of rect 'to' varies by current coordinates
    if(currentCoords.x<=0 && currentCoords.y>=0){
	    from = Rect(fromTRcorner,fromBLcorner);
        toTRcorner = Point2f(screen.cols-from.width,screen.rows-from.height);
        toBLcorner = Point2f(screen.cols,screen.rows);
    }
    if(currentCoords.x>0 && currentCoords.y>=0){
        from = Rect(fromTRcorner,fromBLcorner);
        toTRcorner = Point2f(0,screen.rows-from.height);
        toBLcorner = Point2f(from.width,screen.rows);
    }
    if(currentCoords.x>=0 && currentCoords.y<=0){
	    from = Rect(fromTRcorner,fromBLcorner);
        toTRcorner = Point2f(0,0);
        toBLcorner = Point2f(from.width,from.height);
    }
    if(currentCoords.x<0 && currentCoords.y<0){
	    from = Rect(fromTRcorner,fromBLcorner);
        toTRcorner = Point2f(screen.cols-from.width,0);
        toBLcorner = Point2f(screen.cols,from.height);
    }
	to = Rect(toTRcorner,toBLcorner);
	finalMap(from).copyTo(screen(to));
    drawLegend(screen);
    */
}

void Minimap::setTile(const cv::Mat &photo, cv::Point2f topLeft, cv::Point2f topRight,
                      cv::Point2f botLeft, cv::Point2f botRight){
    /*  we receive four points(corners) and a frame, the four points creates
    *   a quadrilateral, a polygon with four edges, and we need to transform
    *   this polygon to it’s correct location on map.
    */
    //array with received points
    Point2f pts1[] = {topLeft, topRight, botRight, botLeft};
    //array with corners of a general tile, duplicated to make it circular
    Point2f pts2[] = {
        Point2f(0,0),
        Point2f(tileSize,0),
        Point2f(tileSize,tileSize),
        Point2f(0,tileSize),

        Point2f(0,0),
        Point2f(tileSize,0),
        Point2f(tileSize,tileSize),
        Point2f(0,tileSize)
    };
    //fix tile tansformation; move each corner to it's correct point,
    //and rotate tile depending on direction.
    //  getPerspectiveTransform calculates tranformation matrix.
    //  Since pts2 array is circular, this will ease rotation based on
    //  direction, e.g. if direction is 2(South), getPerspectiveTransform
    //  will skip the first two points in the array and consider the bottom
    //  right corner to be the first corner, thus rotating the frame as intended.
    Mat geoTrans = getPerspectiveTransform(pts1, pts2 + direction);
    //create a new mat to hold fixed frame
    Mat fixedFrame = Mat(tileSize,tileSize,CV_8UC3,Scalar(0,0,0));
    //apply tranformation matrix calculated by getPerspectiveTransform
    warpPerspective(photo, fixedFrame, geoTrans, Size(tileSize,tileSize));
    //create focus area where new tile should go
    Rect newTile = Rect(currentPoint.x-tileSize/2,currentPoint.y-tileSize/2,tileSize,tileSize);
    //copy tile to full board
    fixedFrame.copyTo(fullBoard(newTile));
    fixedFrame.copyTo(fullBoardWithPath(newTile));
    //draw new path
    drawPath();
}

//The following three functions change the direction of drone,
//Available direction values are: 0=North, 1=East, 2=South, 3=West.
void Minimap::rotateRight(){
    direction = (direction + 1) % 4;
}
void Minimap::rotateAround(){
    direction = (direction + 2) % 4;
}
void Minimap::rotateLeft(){
    direction = (direction + 3) % 4;
}

void Minimap::stepForward(){
    // When moving the drone forward one tile, we first need to check if
    // it is still within the bounds of our map.
    Point2i prevCoords = currentCoords;
    prevPoint = currentPoint;
    if (direction == 0) {  //facing North
        currentPoint.y -= tileSize;	//move current point right
        currentCoords.y++;	//edit coordinates
    }
    if (direction == 1) {  //facing East
        currentPoint.x += tileSize;	//move current point up
        currentCoords.x++;
    }
    if (direction == 2) {  //facing South
        currentPoint.y += tileSize;	//move current point down
        currentCoords.y--;
    }
    if (direction == 3) {  //facing West
        currentPoint.x -= tileSize;	//move current point left
        currentCoords.x--;
    }
    //check if out of bound
    if (currentPoint.x-tileSize/2.0 < 0 ||
        currentPoint.y-tileSize/2.0 < 0 ||
        currentPoint.x+tileSize/2.0 > mapSize.width ||
        currentPoint.y+tileSize/2.0 > mapSize.height){
        cout << "out of bound" << std::endl;
        currentCoords = prevCoords; //recover previous location
        currentPoint = prevPoint;
    }
    //add new coordinates to path vector
    path.push_back(Point2i(currentCoords.x,currentCoords.y));
}

void Minimap::drawPath(){
    /*  To copy a partially transparent mat, we need a mask layer,
    *   mask will only select pixels based on their alpha channel,
    *   a bigger alpha value means more visibility. Moreover, given
    *   a gray-scaled image, in mask terms, a black pixel is translated
    *   as 0 visibility, and a white pixel is fully visible, thus we need
    *   a gray-scaled mat that is essentially the alpha channel of our path layer.
    */
    //add most recent move to path layer
    line( pathLayer, prevPoint, currentPoint, Scalar(0,0,255), 2, LINE_8 );
    //convert path image to gray-scaled
    cvtColor(pathLayer ,pathGray ,CV_BGR2GRAY );
    //create mask to seperate transparent parts of path from red lines
    threshold(pathGray,pathMask,0,255,0);
    //copy path to board in considiration to mask
    pathLayer.copyTo(fullBoardWithPath,pathMask);
}

void Minimap::togglePath(){
    // Toggle path visibility
    pathFlag = !pathFlag;
}

void Minimap::saveMap(const std::string& path){
    //get map depending on path visibility
	Mat finalMap = pathFlag?fullBoardWithPath:fullBoard;
    //save map to disk
    imwrite(path, finalMap);
}

void Minimap::printPath(){
    //iterate over path vestor and print each point
    for (auto it = path.begin(); it != path.end(); ++it)
        std::cout << *it << " , ";
    //in the end, print distance from (0,0)
    std::cout << std::endl << "You are " << sqrt(pow((path[path.size() - 1]).x,2)+pow((path[path.size() - 1]).y,2));
    std::cout << " pt away from [0,0]" << std::endl;
}

// drawLegend will draw a small guide in the top left corner of the screen
// giving user frindly output on drone's direction and location with each move.
void Minimap::drawLegend(Mat &screen){
    //if screen is smaller than leegend itself then do not continue
	if(screen.rows < 100 || screen.cols < 400)
        return;
	Scalar myBlue = Scalar(191,157,69); //our color of choice
    //draw two rectangles. They are the background of legend.
	rectangle(screen,Point2f(0,0),Point2f(50,50),myBlue,CV_FILLED);
	rectangle(screen,Point2f(50,0),Point2f(300,50),Scalar(0,0,0),CV_FILLED);
    //draw an arrowed line presenting drones direction.
    //needs to set arrow head and base, based on direction value.
    Point2f arrowBase, arrowHead;
    if (direction == 0) {
        arrowBase = Point2f(25,35);
        arrowHead = Point2f(25,15);
    }
    if (direction == 1) {
        arrowBase = Point2f(15,25);
        arrowHead = Point2f(35,25);
    }
    if (direction == 2) {
        arrowBase = Point2f(25,15);
        arrowHead = Point2f(25,35);
    }
    if (direction == 3) {
        arrowBase = Point2f(35,25);
        arrowHead = Point2f(15,25);
    }
    //draw arrow
	arrowedLine(screen,arrowBase,arrowHead,Scalar(0,0,0),2,8,0,1.1);
    ostringstream strLine1, strLine2;
    strLine1 << "Location: " << (path[path.size() - 1]);
	String line1 = strLine1.str();
    strLine2 << "Distance: " << std::setprecision(2) << (sqrt(pow((path[path.size() - 1]).x,2)+pow((path[path.size() - 1]).y,2)))
    << " pts away from [0,0]" ;
	String line2 = strLine2.str();
    //draw text presenting location in coordinates
	putText(screen,line1,Point2f(60,20),FONT_HERSHEY_SIMPLEX,0.4,myBlue,1,LINE_AA);
    //draw text presenting distance from (0,0)
	putText(screen,line2,Point2f(60,40),FONT_HERSHEY_SIMPLEX,0.4,myBlue,1,LINE_AA);
}
#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include <algorithm>
#include <vector>
#include <stdio.h>
#include "ofxJSON.h"
#include <iostream>





using namespace std;
using namespace cv;

class ofApp : public ofBaseApp {
public:
    void drawLineBtwnObjects(int indexOfStart, int indexOfFinish, array<int, 10> contourSize, array<ofxCv::ContourFinder, 10> contourFinders, int targetColorIx, ofImage road, ofImage plane);
	void setup();
	void update();
	void draw();
	void mousePressed(int x, int y, int button);
   
	
	ofVideoGrabber cam;
    ofVideoPlayer vid;
	//ofxCv::ContourFinder contourFinder;
	//ofColor targetColor;
    std::array<ofColor, 10> targetColors;
    //std::array<ofxCv::ContourFinderofVec2f, 1>, 10> contourFinders;
    
    array<ofxCv::ContourFinder, 10> contourFinders;
    array<ofVec2f, 2> thingsToDraw;
    array<int, 10> tempN;
    array<int, 10> contourSize;
    
    float centers [10][10][2];
    
    
    //std::array<ofVec2f, 10> centers;
    array<bool, 10> changeFlag;
    int targetColorIx;
    
    ofxPanel gui;
    ofParameter<float> threshold;
    ofParameter<float> farThresholdSlider;
    ofParameter<float> nearThresholdSlider;
    ofParameter<bool> trackHs;
    ofParameter<bool> holes;
    
    ofImage road;
    ofImage plane;
    
    array<ofVec2f, 2> connectIx;
    
    struct Connect{
        float angle;
        float distance;
        ofVec2f startV;
        int indexOfColors, indexOfColorf, indexOfContour;
        
        
        
    }CONNECT;
    
    vector<Connect> connectToDraw;
    ofxKinect kinect;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar;
    // the far thresholded image
    ofxCvColorImage colorImg;
    ofxCvColorImage resizedColorImg;
    ofxCvGrayscaleImage redImg, blueImg, greenImg;
    int nearThreshold;
    int farThreshold;
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    
    
    bool isInDepthPixel[640*480];
    bool isInDistancePixel[640*480];
    int **pixels;
    
    bool isInDepthRect(bool isInDepthPixel[], int temp);
    bool isInDistanceRect(bool isInDistancePixel[], int temp);
    int angle;
    
    void keyPressed(int key);
    ofFloatPixels distancePixel;
    
    
    int n, m, start;
    
    
    
    cv::Point2f getNearestContour(array<ofxCv::ContourFinder, 10> contourFinders, int contourFindersIx, int contourIx);
    
    
    int contourSizes[10];
    
    bool contourSizeBool[10];
    
    ofJson data;
    bool setupOrigin;
    bool setupLength;
    bool setupTopRight;
    bool setupBottomLeft;
    
    float originX, originY, topRightX, topRightY, bottomLeftX, bottomLeftY, printMouse;
    float screenLengthX, screenLengthY;
    float displayLengthX, displayLengthY;
    ofFile file;
    ofJson emptyData;
    
    int tempJsonIndexIndex;
    array<int, 10> tempJsonIndex;
    
    
    
};

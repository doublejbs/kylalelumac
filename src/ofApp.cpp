#include "ofApp.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace ofxCv;
using namespace cv;
using namespace std;

void ofApp::setup() {
    tempJsonIndexIndex = 0;
    printMouse = false;
    setupOrigin = false;
    setupLength = false;
    
    screenLengthX = 500;
    screenLengthY = 300;
    
    displayLengthX = 1600;
    displayLengthY = 900;
    
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    //kinect.enableDepthNearValueWhite(false);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    
    grayImage.allocate(kinect.width, kinect.height);
    colorImg.allocate(kinect.width, kinect.height);
    redImg.allocate(kinect.width, kinect.height);
    greenImg.allocate(kinect.width, kinect.height);
    blueImg.allocate(kinect.width, kinect.height);
    resizedColorImg.allocate(1600, 900);
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }  ;
    
    
    
    road.load("images/road.png");
    vid.load("viedos/video.mov");
    plane.load("images/plane.png");
    
    
    gui.setup();
    gui.add(threshold.set("Threshold", 30, 0, 255));
    gui.add(trackHs.set("Track Hue/Saturation", false));
    gui.add(holes.set("Holes", false));
    gui.add(farThresholdSlider.set("Far Threshold", 118, 0, 355));
    gui.add(nearThresholdSlider.set("Near Threshold", 127, 0, 355));
    
    targetColorIx = 0;
    
    
    
    
    connectIx[0] = ofVec2f(0,1);
    connectIx[1] = ofVec2f(1,2);
    
    
    nearThreshold = 130;
    farThreshold = 60;
    
    for (int i=0; i<10; i++) {
        
        contourSizeBool[i] = false;
    }
    
    for (int i=0; i<10; i++) {
        contourSizes[i] = 0;
    }
    
    
    
    
    
   
}

void ofApp::update() {
    
    
    
    
    nearThreshold = nearThresholdSlider;
    farThreshold = farThresholdSlider;
    
    if (setupOrigin || setupTopRight || setupBottomLeft) return;
	
    kinect.update();
    if(kinect.isFrameNew()) {
        for (int i=0; i<targetColorIx; i++) {
            contourFinders[i].setMinAreaRadius(5);
            contourFinders[i].setMaxAreaRadius(150);
            contourFinders[i].setTargetColor(targetColors[i], trackHs ? TRACK_COLOR_HS : TRACK_COLOR_RGB);
            contourFinders[i].setThreshold(threshold);
            contourFinders[i].setFindHoles(holes);
            contourFinders[i].findContours(resizedColorImg);
            contourSize[i] = contourFinders[i].size();
            contourFinders[i].setJsonIndex(tempJsonIndex[i]);
        }
        
        // load grayscale depth image from the kinect source
        //grayImage.setFromPixels(kinect.getDepthPixels());
        //grayImage.setFromPixels(kinect.getDistancePixels());
        //distancePixel = kinect.getDistancePixels();
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        
            
            // or we do it ourselves - show people how they can work with the pixels
        //ofPixels & pix = grayImage.getPixels();
        
        
        colorImg.setFromPixels(kinect.getPixels());
        grayImage.setFromPixels(kinect.getDepthPixels());
        colorImg.convertToGrayscalePlanarImages(redImg, greenImg, blueImg);
        
        
        ofPixels & redPix = redImg.getPixels();
        ofPixels & greenPix = greenImg.getPixels();
        ofPixels & bluePix = blueImg.getPixels();
        ofPixels & depthPix = grayImage.getPixels();
        ofPixels & pix = colorImg.getPixels();
        
        
        
        for(int i = 0; i < depthPix.size(); i++) {
            if(depthPix[i] < nearThreshold && depthPix[i] > farThreshold) {
                //depthPix[i] = 255;
                
            } else {
                redPix[i] = 0;
                greenPix[i] = 0;
                bluePix[i] = 0;
            }
        }
        
        //cout<<depthPix[ofGetWidth()/2 + ofGetWidth()*(ofGetHeight()-1)]<<"\n";
        colorImg.setFromGrayscalePlanarImages(redImg, greenImg, blueImg);
        resizedColorImg.scaleIntoMe(colorImg);
        file.open("sample.json");
        if(targetColorIx != 0){
            
            for (int i=0; i<targetColorIx; i++) {
                
                
                ofxCv::ContourFinder tempCon = contourFinders[i];
                ofJson inData;
                /*if(tempCon.size() == 0){
                    array<int, 2> tempCoordinates;
                    tempCoordinates[0] = -1;
                    tempCoordinates[1] = -1;
                    inData[0] = tempCoordinates;
                    data[ofToString(tempCon.jsonIndex)] = inData;
                    continue;
                }*/
                
                int temp = 0;
                for (int j=0; j<tempCon.size(); j++) {
                    
                    if(tempCon.getCenter(j).x > topRightX || tempCon.getCenter(j).x < originX) continue;
                    if(tempCon.getCenter(j).y > bottomLeftY || tempCon.getCenter(j).y < originY) continue;
                    
                    array<int, 2> coordinates;
                    coordinates[0] = (int)tempCon.getCenter(j).x;
                    coordinates[1] = (int)tempCon.getCenter(j).y;
                    
                    inData[temp++] = coordinates;
                    
                }
                data[ofToString(tempCon.jsonIndex)] = inData;
                
                
            }
            ofSavePrettyJson("sample.json", data);
            file.close();
            
        }
        
    }
    
    
}

void ofApp::draw() {
	
	resizedColorImg.draw(0, 0);

    for(int i=0; i<targetColorIx; i++){
        
        contourFinders[i].draw();
        
    }
    
    
    gui.draw();
    
	ofTranslate(8, 90);
	ofFill();
	
    
    
    
    
    
}

void ofApp::mousePressed(int x, int y, int button) {
    
    if (setupOrigin) {
        originX = mouseX;
        originY = mouseY;
    }
    
    if (setupTopRight){
        topRightX = mouseX;
        topRightY = mouseY;
    }
    
    if (setupBottomLeft){
        bottomLeftX = mouseX;
        bottomLeftY = mouseY;
        screenLengthX = topRightX - originX;
        screenLengthY = bottomLeftY - originY;
    }
    
    if (!setupOrigin && !setupTopRight && !setupBottomLeft){
        cout<<"yes\n";
        targetColors[targetColorIx++] = resizedColorImg.getPixels().getColor(x, y);
    }
    
    
    if (printMouse){
        cout<<"mouseX: "<<mouseX<<" mouseY: "<<mouseY<<"\n";
    }
	
}

void ofApp::keyPressed (int key) {
    switch(key) {
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_LEFT:
            setupOrigin = true;
            //farThreshold = 80;
            ofLogNotice() << originX;
            break;
        case OF_KEY_RIGHT:
            setupOrigin = false;
            setupTopRight = true;
            break;
        case '9':
            setupTopRight = false;
            setupBottomLeft = true;
            break;
            
        case OF_KEY_BACKSPACE:
            setupOrigin = false;
            setupTopRight = false;
            setupBottomLeft = false;
            break;
            
        
        case '0':
            tempJsonIndex[tempJsonIndexIndex++] = 0;
            break;
            
        case '1':
            tempJsonIndex[tempJsonIndexIndex++] = 1;
            break;
    
        case '2':
            tempJsonIndex[tempJsonIndexIndex++] = 2;
            break;
            
        case '3':
            tempJsonIndex[tempJsonIndexIndex++] = 3;
            break;
        case '4':
            tempJsonIndex[tempJsonIndexIndex++] = 4;
            break;
        case '5':
            tempJsonIndex[tempJsonIndexIndex++] = 5;
            break;
        case '6':
            tempJsonIndex[tempJsonIndexIndex++] = 6;
            break;
        case '7':
            tempJsonIndex[tempJsonIndexIndex++] = 7;
            break;
        
    }
    
}

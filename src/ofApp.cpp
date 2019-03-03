#include "ofApp.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace ofxCv;
using namespace cv;
using namespace std;






cv::Point2f ofApp::getNearestContour(array<ofxCv::ContourFinder, 10> contourFinders, int contourFindersIx, int contourIx){
    ofxCv::ContourFinder tempCon = contourFinders[contourFindersIx];
    cv::Point2f tempCenter(tempCon.getCenter(contourIx).x, tempCon.getCenter(contourIx).y);
    float dist = 1000000;
    int tempIx = 0;
    for (int i=0; i<tempCon.size(); i++) {
        
        float tempDist = ofDist(tempCon.getCenter(i).x, tempCon.getCenter(i).y, tempCenter.x, tempCenter.y);
        if(i == contourIx) continue;
        
        if (tempDist < dist) {
            dist = tempDist;
            tempIx = i;
        }
        
    }
    return tempCon.getCenter(tempIx);
}




void ofApp::drawLineBtwnObjects(int indexOfStart, int indexOfFinish, array<int, 10> contourSize, array<ofxCv::ContourFinder, 10> contourFinders, int targetColorIx, ofImage road, ofImage plane){
    //ofSetColor(0,0,0);
    //ofSetLineWidth(10.0);
    ofVec2f origin(1,0);
   
    
    if(targetColorIx > 3){
        for (int i=0; i<contourSize[indexOfStart]; i++) {
            for (int j=0; j<contourSize[indexOfFinish]; j++) {
                
                ofVec2f startV(contourFinders[indexOfStart].getCenter(i).x, contourFinders[indexOfStart].getCenter(i).y);
                ofVec2f finishV(contourFinders[indexOfFinish].getCenter(j).x, contourFinders[indexOfFinish].getCenter(j).y);
                //cout<<startV.x<<startV.y;
                ofSetColor(255);
                ofDrawRectangle(0, 0, 2560, 1440);
                ofSetColor(targetColors[indexOfStart]);
                ofDrawCircle(startV.x, startV.y, 20);
                ofSetColor(targetColors[indexOfFinish]);
                ofDrawCircle(finishV.x, finishV.y, 20);
                
                ofPushMatrix();
                ofTranslate(startV.x, startV.y);
                ofRotateDeg(180-(-1*(finishV-startV)).angle(startV)+origin.angle(startV));
                ofDrawLine(0, 0, finishV.x, finishV.y);
                road.draw(0, 0, ofDist(startV.x, startV.y, finishV.x, finishV.y), 100);
                for(int x=0; x<finishV.x; x+=2){
                    int y = (finishV.y-startV.y)*(x-startV.x)/(finishV.x-finishV.y)+startV.y;
                    plane.draw(x, y, 10, 10);
                }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
                ofDrawBitmapString(ofToString(startV.angle(finishV)), 0, 0);
                //ofDrawLine(contourFinders[indexOfStart].getCenter(i).x, contourFinders[indexOfStart].getCenter(i).y, contourFinders[indexOfFinish].getCenter(j).x, contourFinders[indexOfFinish].getCenter(j).y);
                ofPopMatrix();
            }
        }
    }
    
    else{
        return;
    }
    
}

bool ofApp::isInDepthRect(bool isInDepthPixel[], int temp){
    for (int i=-10; i<=10; i++) {
        for (int j=-10; j<=10; j++) {
            if(isInDepthPixel[temp+640*i+j]){
                return true;
            }
        }
    }
    return false;
}


bool ofApp::isInDistanceRect(bool isInDistancePixel[], int temp){
    for (int i=-10; i<=10; i++) {
        for (int j=-10; j<=10; j++) {
            if(isInDistancePixel[temp+640*i+j]){
                return true;
            }
        }
    }
    return false;
}

void ofApp::setup() {
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
    gui.add(farThresholdSlider.set("Far Threshold", 130, 0, 355));
    gui.add(nearThresholdSlider.set("Near Threshold", 60, 0, 355));
    
    targetColorIx = 0;
    targetColors[0] = ofColor::black;
    
    
    
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
    
    
    ofFile file("sample.json");
    
    
   
}

void ofApp::update() {
    nearThreshold = nearThresholdSlider;
    farThreshold = farThresholdSlider;
    
    if (setupOrigin || setupTopRight || setupBottomLeft) return;
	
    kinect.update();
    if(kinect.isFrameNew()) {
        for (int i=0; i<targetColorIx; i++) {
            contourFinders[i].setMinAreaRadius(3);
            contourFinders[i].setMaxAreaRadius(150);
            contourFinders[i].setTargetColor(targetColors[i], trackHs ? TRACK_COLOR_HS : TRACK_COLOR_RGB);
            contourFinders[i].setThreshold(threshold);
            contourFinders[i].setFindHoles(holes);
            contourFinders[i].findContours(resizedColorImg);
            contourSize[i] = contourFinders[i].size();
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
        
        if(targetColorIx > 0){
            
            for (int i=0; i<targetColorIx; i++) {
                if (contourFinders[i].size() == 0) return;
                ofJson inData;
                ofxCv::ContourFinder tempCon = contourFinders[i];
                /*if (contourFinders[i].size() == 1){
                    array<float, 4> myselfCoordinate;
                    
                 
                    
                    myselfCoordinate[0] = ofMap(tempCon.getCenter(0).x, originX, originX + screenLengthX, 0, displayLengthX);
                    myselfCoordinate[1] = ofMap(tempCon.getCenter(0).y, originY, originY + screenLengthY, 0, displayLengthY);
                    myselfCoordinate[2] = ofMap(tempCon.getCenter(0).x, originX, originX + screenLengthX, 0, displayLengthX);
                    myselfCoordinate[3] = ofMap(tempCon.getCenter(0).y, originY, originY + screenLengthY, 0, displayLengthY);
                    
                    inData[0] = myselfCoordinate;
                    
                }
                
                int temp = 0;
                for (int j=1; j<contourFinders[i].size(); j++) {
                    
                    array<float, 4> coordinates;
                    cv::Point2f tempPoint = getNearestContour(contourFinders, i, j);
                    cout<<"connect : "<<i<<":"<<j<<"("<<contourFinders[i].getCenter(j).x<<", "<<contourFinders[i].getCenter(j).y<<") -> ("<<tempPoint.x<<", "<<tempPoint.y<<")\n";
                 
                    
                    coordinates[0] = ofMap(contourFinders[i].getCenter(j).x, originX, originX + screenLengthX, 0, displayLengthX);
                    coordinates[1] = ofMap(contourFinders[i].getCenter(j).y, originY, originY + screenLengthY, 0, displayLengthY);
                    coordinates[2] = ofMap(tempPoint.x, originX, originX + screenLengthX, 0, displayLengthX);
                    coordinates[3] = ofMap(tempPoint.y, originY, originY + screenLengthY, 0, displayLengthY);
                    
                    
                    inData[temp++] = coordinates;
                    
                }*/
                int temp = 0;
                for (int j=0; j<tempCon.size(); j++) {
                    array<float, 2> coordinates;
                    coordinates[0] = ofMap(tempCon.getCenter(j).x, originX, originX + screenLengthX, 0, displayLengthX);
                    coordinates[1] = ofMap(tempCon.getCenter(j).y, originY, originY + screenLengthY, 0, displayLengthY);
                    
                    inData[temp++] = coordinates;
                }
                data[ofToString(i)] = inData;
            }
            
        }
        ofSavePrettyJson("sample.json", data);
        
        
        /*for(int i = 0; i < numPixels; i++) {
            if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                isInDepthPixel[i] = true;
            } else {
                isInDepthPixel[i] = false;
            }
            
        }*/
        
    }
    
    
}

void ofApp::draw() {
	//ofSetColor(255);
    
    //grayImage.draw(0, 0);
	resizedColorImg.draw(0, 0);
    //colorImg.draw(0, 0);
    
    
    for(int i=0; i<targetColorIx; i++){
        
        contourFinders[i].draw();
        
        
    }
    
    
    //ofSetColor(0, 0, 0);
    //ofSetLineWidth(10.0);
    //contourFinder[0]과 contourFinder[1]은 검은색 선으로 이어져야 한다
    
    /*for (int i=0; i<connectToDraw.size(); i++) {
        ofPushMatrix();
        ofSetColor(255);
        ofTranslate(connectToDraw[i].startV.x, connectToDraw[i].startV.y);
        ofRotateDeg(connectToDraw[i].angle);
        //ofDrawLine(0, 0, finishV.x, finishV.y);
        road.draw(0, 0, connectToDraw[i].distance, 100);
        for(int x=0; x<finishV.x; x+=2){
            int y = (finishV.y-startV.y)*(x-startV.x)/(finishV.x-finishV.y)+startV.y;
            plane.draw(x, y, 10, 10);
        }
        ofDrawBitmapString(ofToString(connectToDraw[i].angle), 0, 0);
        //ofDrawLine(contourFinders[indexOfStart].getCenter(i).x, contourFinders[indexOfStart].getCenter(i).y, contourFinders[indexOfFinish].getCenter(j).x, contourFinders[indexOfFinish].getCenter(j).y);
        //connectToDraw.erase(connectToDraw.begin()+i);
        ofPopMatrix();
    }
    
    for (int i=0; i<targetColorIx; i++) {
        for (int j=0; j<contourFinders[i].size(); j++) {
            //contour마다의 depth값이 지정한 depth 범위에 해당하는지 확인.
            int x = contourFinders[i].getCenter(j).x;
            int y = contourFinders[i].getCenter(j).y;
            
            int temp = (x+1)*(y+1);
            
            
            
            
        }
    }*/
    
    //drawLineBtwnObjects(0, 1, contourSize, contourFinders, targetColorIx, road, plane);
    //drawLineBtwnObjects(0, 2, contourSize, contourFinders, targetColorIx, road, plane);
    
    
    
    //ofSetColor(0, 0, 0);
    //ofDrawLine(centers[0][0][0], centers[0][0][1], centers[1][0][0], centers[1][0][1]);
    
	/*
	ofNoFill();
    for(int j=0; j<targetColorIx; j++){
        int n = contourFinders[j].size();
        cout<<"size: "<<n;
        
     
        for(int i = 0; i < n; i++) {
            // smallest rectangle that fits the contour
            ofSetColor(cyanPrint);
            ofPolyline minAreaRect = toOf(contourFinders[j].getMinAreaRect(i));
            minAreaRect.draw();
            
            // ellipse that best fits the contour
            ofSetColor(magentaPrint);
            cv::RotatedRect ellipse = contourFinders[j].getFitEllipse(i);
            ofPushMatrix();e
            ofVec2f ellipseCenter = toOf(ellipse.center);
            ofVec2f ellipseSize = toOf(ellipse.size);
            ofTranslate(ellipseCenter.x, ellipseCenter.y);
            ofRotate(ellipse.angle);
            ofDrawEllipse(0, 0, ellipseSize.x, ellipseSize.y);
            ofPopMatrix();
            
            // minimum area circle that encloses the contour
            ofSetColor(cyanPrint);
            float circleRadius;
            ofVec2f circleCenter = toOf(contourFinders[j].getMinEnclosingCircle(i, circleRadius));
            ofDrawCircle(circleCenter, circleRadius);
            
            // convex hull of the contour
            ofSetColor(yellowPrint);
            ofPolyline convexHull = toOf(contourFinders[j].getConvexHull(i));
            convexHull.draw();
            
            // defects of the convex hull
            vector<cv::Vec4i> defects = contourFinders[j].getConvexityDefects(i);
            for(int j = 0; j < defects.size(); j++) {
                ofDrawLine(defects[j][0], defects[j][1], defects[j][2], defects[j][3]);
            }
            
            // some different styles of contour centers
            ofVec2f centroid = toOf(contourFinders[j].getCentroid(i));
            ofVec2f average = toOf(contourFinders[j].getAverage(i));
            ofVec2f center = toOf(contourFinders[j].getCenter(i));
            ofSetColor(cyanPrint);
            ofDrawCircle(centroid, 1);
            ofSetColor(magentaPrint);
            ofDrawCircle(average, 1);
            ofSetColor(yellowPrint);
            ofDrawCircle(center, 1);
            
            // you can also get the area and perimeter using ofPolyline:
            // ofPolyline::getArea() and ofPolyline::getPerimeter()
            double area = contourFinders[j].getContourArea(i);
            double length = contourFinders[j].getArcLength(i);
            
            // balance is useful for detecting when a shape has an "arm" sticking out
            // if balance.length() is small, the shape is more symmetric: like I, O, X...
            // if balance.length() is large, the shape is less symmetric: like L, P, F...
            ofVec2f balance = toOf(contourFinders[j].getBalance(i));
            ofPushMatrix();
            ofTranslate(centroid.x, centroid.y);
            ofScale(5, 5);
            ofDrawLine(0, 0, balance.x, balance.y);
            ofPopMatrix();
            
            if(contourFinders[j].getHole(i)) {
                ofDrawBitmapStringHighlight("hole", center.x, center.y);
            }
        }
    }*/
    
    gui.draw();
    
	ofTranslate(8, 90);
	ofFill();
	//ofSetColor(0);
	//ofDrawRectangle(-3, -3, 64+6, 64+6);
    /*for (int i=0; i<targetColorIx; i++) {
        ofSetColor(targetColors[i]);
        ofDrawRectangle(0+64*i, 0, 64, 64);
    }*/
    
    
    
    
    
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
	//targetColors[targetColorIx++] = cam.getPixels().getColor(x, y);
    
    
    //nearThreshold = kinect.getDistanceAt(x, y) - 1;
    //farThreshold = kinect.getDistanceAt(x, y) + 1;
    //cout<<"distance : "<<kinect.getDistanceAt(x, y)<<"\n";
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
            
        case '0':
            setupOrigin = true;
            //farThreshold = 80;
            ofLogNotice() << originX;
            break;
        case '1':
            setupOrigin = false;
            setupTopRight = true;
            break;
            
        case '2':
            setupTopRight = false;
            setupBottomLeft = true;
            break;
            
        case '3':
            setupOrigin = false;
            setupTopRight = false;
            setupBottomLeft = false;
            
            break;
        case '4':
            printMouse = true;
            break;
        
    }
    
}

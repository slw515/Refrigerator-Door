#include "ofApp.h"
#include "ofxCv.h"
#include "ofBitmapFont.h"
#include <iostream>
#include <string>
#include "ofxAssets.h"

using namespace ofxAssets;
using namespace std;
using namespace ofxCv;
using namespace cv;

void drawMarker(float size, const string letter){
//void drawMarker(float size, ofImage* alphabet){
	ofDrawAxis(size / 2);
	ofPushMatrix();
		// move up from the center by size*.5
		// to draw a box centered at that point
    ofTranslate(0,size*0.5,0);
    ofFill();
    ofSetColor(255);
    ofDrawBitmapString(letter, 0, 0);
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetWindowTitle("ofxAruco - example");
	ofSetVerticalSync(true);
	useVideo = false;
	string boardName = "boardConfiguration.yml";
    grabber.setGrabber(std::make_shared<ofxPS3EyeGrabber>());
    grabber.setup(640, 480);
    
	//aruco.setThreaded(false);
	showMarkers = true;
	showBoard = true;
	showBoardImage = false;

	ofEnableAlphaBlending();
    warpedColor.allocate(640, 480, OF_IMAGE_COLOR);
    
    movingPoint = false;
    saveMatrix = false;
    homographyReady = false;
    settingNewPoints = false;
    
    destPoints.push_back(ofVec2f(0,0));
    destPoints.push_back(ofVec2f(warpedColor.getWidth(),0));
    destPoints.push_back(ofVec2f(warpedColor.getWidth(),warpedColor.getHeight()));
    destPoints.push_back(ofVec2f(0,warpedColor.getHeight()));
    
    srcPoints.push_back(ofVec2f(0+warpedColor.getWidth()+100,30));
    srcPoints.push_back(ofVec2f(warpedColor.getWidth()+warpedColor.getWidth()-100,30));
    srcPoints.push_back(ofVec2f(warpedColor.getWidth()+warpedColor.getWidth()-100,warpedColor.getHeight()-30));
    srcPoints.push_back(ofVec2f(0+warpedColor.getWidth()+100,warpedColor.getHeight()-30));
    
    // load the previous homography if it's available
    ofFile previous("homography.yml");
    if(previous.exists()) {
        FileStorage fs(ofToDataPath("homography.yml"), FileStorage::READ);
        fs["homography"] >> homography;
        homographyReady = true;
    }
    ofSetVerticalSync(true);
    
    aruco.setup("intrinsics.int", warpedColor.getWidth(), warpedColor.getHeight(), boardName);
//    aruco.getBoardImage(board.getPixels());
//    aruco.findMarkers(board.getPixels());
    board.update();
    letter1.load("letter1.png");
    
    letters.insert({985, "A"});
    letters.insert({838, "B"});
    letters.insert({908, "D"});
    letters.insert({299, "E"});
    letters.insert({341, "T"});
    letters.insert({64, "G"});
    letters.insert({177, "N"});
    letters.insert({428, "H"});
    letters.insert({760, "S"});
    letters.insert({882, "R"});
    
    prevAX = prevAY = 0;
    prevBX = prevBY = 0;
    prevDX = prevDY = 0;
    prevEX = prevEY = 0;
    prevTX = prevTY = 0;
    prevGX = prevGY = 0;
    prevNX = prevNY = 0;
    prevHX = prevHY = 0;
    prevSX = prevSY = 0;
    prevRX = prevRY = 0;
    
    ofSetBackgroundColor(255);
    
    map.addQuad(600, 600);
    
    myFlower.setup(); // calling the object's setup method
    myFlower1.setup(); // calling the object's setup method
    myFlower2.setup(); // calling the object's setup method
    myFlower3.setup(); // calling the object's setup method
    myFlower4.setup(); // calling the object's setup method
    
    debugMode = true;


}

//--------------------------------------------------------------
void ofApp::update(){
    grabber.update();
//    cout<<aruco.getMarkers()<<endl;
    if(grabber.isFrameNew()){
        if(settingNewPoints) {
            vector<Point2f> sPoints, dPoints;
            for(int i = 0; i < destPoints.size(); i++) {
                sPoints.push_back(Point2f(srcPoints[i].x - warpedColor.getWidth(), srcPoints[i].y));
                dPoints.push_back(Point2f(destPoints[i].x, destPoints[i].y));
            }
                // generate a homography from the two sets of points
                homography = findHomography(Mat(sPoints), Mat(dPoints));
                homographyReady = true;
                
                if(saveMatrix) {
                    FileStorage fs(ofToDataPath("homography.yml"), FileStorage::WRITE);
                    fs << "homography" << homography;
                    saveMatrix = false;
                    settingNewPoints = false;
                }
            }
            
            if(homographyReady) {
                // this is how you warp one ofImage into another ofImage given the homography matrix
                // CV INTER NN is 113 fps, CV_INTER_LINEAR is 93 fps
                warpPerspective(grabber, warpedColor, homography, CV_INTER_LINEAR);
                warpedColor.update();
            }
    }

    aruco.detectBoards(warpedColor.getPixels());
    myFlower.update(); // calling the object's setup method
    myFlower1.update(); // calling the object's setup method
    myFlower2.update(); // calling the object's setup method
    myFlower3.update(); // calling the object's setup method
    myFlower4.update(); // calling the object's setup method

}

void ofApp::drawPoints(vector<ofVec2f>& points) {
    ofNoFill();
    for(int i = 0; i < points.size(); i++) {
        ofDrawCircle(points[i], 10);
        ofDrawCircle(points[i], 1);
    }
}

//--------------------------------------------------------------

void ofApp::draw(){
    ofFile previous("homography.yml");
//    if (previous.exists() == false) {
//    grabber.draw(640, 0);
//    }

    if(homographyReady) {
        warpedColor.draw(0, 0);
    }
    
    if (settingNewPoints){
        ofPushStyle();
        ofSetColor(0, 120, 240);
        drawPoints(srcPoints);
        ofSetColor(128);
        ofSetLineWidth(2);
        for(int i = 1; i < srcPoints.size(); i++) {
            ofDrawLine(srcPoints[i-1], srcPoints[i]);
        }
        ofDrawLine(srcPoints[srcPoints.size()-1], srcPoints[0]);
        ofPopStyle();
    }

//    ofSetBackgroundColor(255);
    string message = ofToString((int) ofGetFrameRate());
    message += "\nPress 'n' to set points\nPress 's' to save";
    ofDrawBitmapString(message, 10, 20);

    cout<<ofGetWidth()<<endl;
    map.begin(0);

    if (showMarkers) {

        ofBackground(255);
        ofSetBackgroundColor(255);

        for (auto m: aruco.getMarkers()) {
//            cout<<m.id<<endl;
            if (m.id == 838 || prevBX > 0) {
                ofSetColor(0, 255, 0);
                string letter = letters.at(838);
                
                if (m.id == 838) {
                    prevBX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                    prevBY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                }
                ofVec2f centerPos = ofVec2f(prevBX, prevBY);
                if (debugMode == true) {
                    font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                }
                ofSetColor(255, 134, 184);

                ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                ofVec2f aDirection = aCenterPos - centerPos;
                float aDistance = aDirection.length();
                
                if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                    ofVec2f thirdDistance = aDirection;
                    string otherLetter = "A";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                        ofVec2f fourthDistance = aDirection;
                        string otherLetter = "A";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                
                ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                ofVec2f dDirection = dCenterPos - centerPos;
                float dDistance = dDirection.length();
                
                if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                    ofVec2f thirdDistance = dDirection;
                    string otherLetter = "D";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                        ofVec2f fourthDistance = dDirection;
                        string otherLetter = "D";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                
                ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                ofVec2f eDirection = eCenterPos - centerPos;
                float eDistance = eDirection.length();
                
                if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                    ofVec2f thirdDistance = eDirection;
                    string otherLetter = "E";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                        ofVec2f fourthDistance = eDirection;
                        string otherLetter = "E";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                
                ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                ofVec2f tDirection = tCenterPos - centerPos;
                float tDistance = tDirection.length();
                cout<<tDirection.x<<"x"<<endl;
                cout<<tDirection.y<<"y"<<endl;

                if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                    ofVec2f thirdDistance = tDirection;
                    string otherLetter = "T";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;

                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                    }
                    else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                        ofVec2f fourthDistance = tDirection;
                        string otherLetter = "T";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                
                ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                ofVec2f gDirection = gCenterPos - centerPos;
                float gDistance = gDirection.length();
                
                if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                    ofVec2f thirdDistance = gDirection;
                    string otherLetter = "G";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                        ofVec2f fourthDistance = gDirection;
                        string otherLetter = "G";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }

                ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                ofVec2f nDirection = nCenterPos - centerPos;
                float nDistance = nDirection.length();
                
                if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                    ofVec2f thirdDistance = nDirection;
                    string otherLetter = "N";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                        ofVec2f fourthDistance = nDirection;
                        string otherLetter = "N";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                ofVec2f hDirection = hCenterPos - centerPos;
                float hDistance = hDirection.length();
                
                if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                    ofVec2f thirdDistance = hDirection;
                    string otherLetter = "H";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                        ofVec2f fourthDistance = hDirection;
                        string otherLetter = "H";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                
                ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                ofVec2f sDirection = sCenterPos - centerPos;
                float sDistance = sDirection.length();
                
                if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                    ofVec2f thirdDistance = sDirection;
                    string otherLetter = "S";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                        ofVec2f fourthDistance = sDirection;
                        string otherLetter = "S";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                
                ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                ofVec2f rDirection = rCenterPos - centerPos;
                float rDistance = rDirection.length();
                
                if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                    ofVec2f thirdDistance = rDirection;
                    string otherLetter = "R";
                    string middleLetter;
                    string firstLetter;
                    string lastLetter;
                    
                    for (auto i: threeLetter) {
                        firstLetter = i[0];
                        lastLetter = i[2];
                        if (firstLetter == letter && otherLetter == lastLetter) {
                            middleLetter = i[1];
                        }
                    }
                
                    thirdDistance /= 2;
                     
                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                      
                    }
                    else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                        ofVec2f fourthDistance = rDirection;
                        string otherLetter = "R";
                        string secondLetter;
                        string thirdLetter;
                        string firstLetter;
                        string lastLetter;

                        for (auto i: fourLetter) {
                            firstLetter = i[0];
                            lastLetter = i[3];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                secondLetter = i[1];
                                thirdLetter = i[2];
                            }
                        }

                        fourthDistance /= 3;
                         
                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                    }
                }
if (m.id == 985 || prevAX > 0) {
                    ofSetColor(0, 255, 0);
                    string letter = letters.at(985);
                    if (m.id == 985) {
                        prevAX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                        prevAY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                    }
                    ofVec2f centerPos = ofVec2f(prevAX, prevAY);
                    if (debugMode == true) {
                        font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                    }
                    ofSetColor(10, 67, 44);
                    ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                    ofVec2f bDirection = bCenterPos - centerPos;
                    float bDistance = bDirection.length();
                    
                    if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                        ofVec2f thirdDistance = bDirection;
                        string otherLetter = "B";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                            ofVec2f fourthDistance = bDirection;
                            string otherLetter = "B";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                    
                    ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                    ofVec2f dDirection = dCenterPos - centerPos;
                    float dDistance = dDirection.length();
                    
                    if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                        ofVec2f thirdDistance = dDirection;
                        string otherLetter = "D";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                            ofVec2f fourthDistance = dDirection;
                            string otherLetter = "D";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    
                    ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                    ofVec2f eDirection = eCenterPos - centerPos;
                    float eDistance = eDirection.length();
                    
                    if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                        ofVec2f thirdDistance = eDirection;
                        string otherLetter = "E";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                            ofVec2f fourthDistance = eDirection;
                            string otherLetter = "E";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    
                    ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                    ofVec2f tDirection = tCenterPos - centerPos;
                    float tDistance = tDirection.length();
                    
                    if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                        ofVec2f thirdDistance = tDirection;
                        string otherLetter = "T";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                            ofVec2f fourthDistance = tDirection;
                            string otherLetter = "T";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    
                    ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                    ofVec2f gDirection = gCenterPos - centerPos;
                    float gDistance = gDirection.length();
                    
                    if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                        ofVec2f thirdDistance = gDirection;
                        string otherLetter = "G";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                            ofVec2f fourthDistance = gDirection;
                            string otherLetter = "E";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }

                    ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                    ofVec2f nDirection = nCenterPos - centerPos;
                    float nDistance = nDirection.length();
                    
                    if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                        ofVec2f thirdDistance = nDirection;
                        string otherLetter = "N";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                            ofVec2f fourthDistance = nDirection;
                            string otherLetter = "N";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                    ofVec2f hDirection = hCenterPos - centerPos;
                    float hDistance = hDirection.length();
                    
                    if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                        ofVec2f thirdDistance = hDirection;
                        string otherLetter = "H";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                            ofVec2f fourthDistance = hDirection;
                            string otherLetter = "H";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    
                    ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                    ofVec2f sDirection = sCenterPos - centerPos;
                    float sDistance = sDirection.length();
                    
                    if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                        ofVec2f thirdDistance = sDirection;
                        string otherLetter = "S";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                            ofVec2f fourthDistance = sDirection;
                            string otherLetter = "S";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    
                    ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                    ofVec2f rDirection = rCenterPos - centerPos;
                    float rDistance = rDirection.length();
                    
                    if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                        ofVec2f thirdDistance = rDirection;
                        string otherLetter = "R";
                        string middleLetter;
                        string firstLetter;
                        string lastLetter;
                        
                        for (auto i: threeLetter) {
                            firstLetter = i[0];
                            lastLetter = i[2];
                            if (firstLetter == letter && otherLetter == lastLetter) {
                                middleLetter = i[1];
                            }
                        }
                    
                        thirdDistance /= 2;
                         
                        font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                          
                        }
                        else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                            ofVec2f fourthDistance = rDirection;
                            string otherLetter = "R";
                            string secondLetter;
                            string thirdLetter;
                            string firstLetter;
                            string lastLetter;

                            for (auto i: fourLetter) {
                                firstLetter = i[0];
                                lastLetter = i[3];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    secondLetter = i[1];
                                    thirdLetter = i[2];
                                }
                            }

                            fourthDistance /= 3;
                             
                            font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                            font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                        }
                    }
    if (m.id == 908 || prevDX > 0) {
                                ofSetColor(0, 255, 0);
                                string letter = letters.at(908);
                                
                                if (m.id == 908) {
                                    prevDX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                                    prevDY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                                }
                                
                                
                            ofVec2f centerPos = ofVec2f(prevDX, prevDY);
                            if (debugMode == true) {
                                font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                                
                            }
                                ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                                ofVec2f bDirection = bCenterPos - centerPos;
                                float bDistance = bDirection.length();
                                ofSetColor(240, 97, 38);

                                if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f thirdDistance = bDirection;
                                    string otherLetter = "B";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                        ofVec2f fourthDistance = bDirection;
                                        string otherLetter = "B";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                        }
                                
                                ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                                ofVec2f aDirection = aCenterPos - centerPos;
                                float aDistance = aDirection.length();
                                
                                if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f thirdDistance = aDirection;
                                    string otherLetter = "A";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                        ofVec2f fourthDistance = aDirection;
                                        string otherLetter = "A";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                
                                ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                                ofVec2f eDirection = eCenterPos - centerPos;
                                float eDistance = eDirection.length();
                                
                                if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f thirdDistance = eDirection;
                                    string otherLetter = "E";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                        ofVec2f fourthDistance = eDirection;
                                        string otherLetter = "E";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                
                                ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                                ofVec2f tDirection = tCenterPos - centerPos;
                                float tDistance = tDirection.length();
                                
                                if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                                    ofVec2f thirdDistance = tDirection;
                                    string otherLetter = "T";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                        ofVec2f fourthDistance = tDirection;
                                        string otherLetter = "T";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                
                                ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                                ofVec2f gDirection = gCenterPos - centerPos;
                                float gDistance = gDirection.length();
                                
                                if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                                    ofVec2f thirdDistance = gDirection;
                                    string otherLetter = "G";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                        ofVec2f fourthDistance = gDirection;
                                        string otherLetter = "E";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }

                                ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                                ofVec2f nDirection = nCenterPos - centerPos;
                                float nDistance = nDirection.length();
                                
                                if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                                    ofVec2f thirdDistance = nDirection;
                                    string otherLetter = "N";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                        ofVec2f fourthDistance = nDirection;
                                        string otherLetter = "N";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                                ofVec2f hDirection = hCenterPos - centerPos;
                                float hDistance = hDirection.length();
                                
                                if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                                    ofVec2f thirdDistance = hDirection;
                                    string otherLetter = "H";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                        ofVec2f fourthDistance = hDirection;
                                        string otherLetter = "H";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                
                                ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                                ofVec2f sDirection = sCenterPos - centerPos;
                                float sDistance = sDirection.length();
                                
                                if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                                    ofVec2f thirdDistance = sDirection;
                                    string otherLetter = "S";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                        ofVec2f fourthDistance = sDirection;
                                        string otherLetter = "S";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                                
                                ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                                ofVec2f rDirection = rCenterPos - centerPos;
                                float rDistance = rDirection.length();
                                
                                if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f thirdDistance = rDirection;
                                    string otherLetter = "R";
                                    string middleLetter;
                                    string firstLetter;
                                    string lastLetter;
                                    
                                    for (auto i: threeLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[2];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            middleLetter = i[1];
                                        }
                                    }
                                
                                    thirdDistance /= 2;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                      
                                    }
                                    else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                        ofVec2f fourthDistance = rDirection;
                                        string otherLetter = "R";
                                        string secondLetter;
                                        string thirdLetter;
                                        string firstLetter;
                                        string lastLetter;

                                        for (auto i: fourLetter) {
                                            firstLetter = i[0];
                                            lastLetter = i[3];
                                            if (firstLetter == letter && otherLetter == lastLetter) {
                                                secondLetter = i[1];
                                                thirdLetter = i[2];
                                            }
                                        }

                                        fourthDistance /= 3;
                                         
                                        font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                        font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            }
if (m.id == 299 || prevEX > 0) {
                       ofSetColor(0, 255, 0);
                       string letter = letters.at(299);
                       
                       if (m.id == 299) {
                           prevEX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                           prevEY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                       }
                       
                       ofVec2f centerPos = ofVec2f(prevEX, prevEY);
                       if (debugMode == true) {
                           font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                           
                       }
                        ofSetColor(229, 29, 116);
                        ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                        ofVec2f bDirection = bCenterPos - centerPos;
                        float bDistance = bDirection.length();
                        
                        if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                            ofVec2f thirdDistance = bDirection;
                            string otherLetter = "B";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f fourthDistance = bDirection;
                                string otherLetter = "B";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                        
                        ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                        ofVec2f dDirection = dCenterPos - centerPos;
                        float dDistance = dDirection.length();
                        
                        if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                            ofVec2f thirdDistance = dDirection;
                            string otherLetter = "D";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f fourthDistance = dDirection;
                                string otherLetter = "D";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                        ofVec2f aDirection = aCenterPos - centerPos;
                        float aDistance = aDirection.length();
                        
                        if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                            ofVec2f thirdDistance = aDirection;
                            string otherLetter = "A";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f fourthDistance = aDirection;
                                string otherLetter = "A";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                        ofVec2f tDirection = tCenterPos - centerPos;
                        float tDistance = tDirection.length();
                        
                        if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                            ofVec2f thirdDistance = tDirection;
                            string otherLetter = "T";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f fourthDistance = tDirection;
                                string otherLetter = "T";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                        ofVec2f gDirection = gCenterPos - centerPos;
                        float gDistance = gDirection.length();
                        
                        if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                            ofVec2f thirdDistance = gDirection;
                            string otherLetter = "G";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                ofVec2f fourthDistance = gDirection;
                                string otherLetter = "E";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }

                        ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                        ofVec2f nDirection = nCenterPos - centerPos;
                        float nDistance = nDirection.length();
                        
                        if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                            ofVec2f thirdDistance = nDirection;
                            string otherLetter = "N";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f fourthDistance = nDirection;
                                string otherLetter = "N";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                        ofVec2f hDirection = hCenterPos - centerPos;
                        float hDistance = hDirection.length();
                        
                        if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                            ofVec2f thirdDistance = hDirection;
                            string otherLetter = "H";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f fourthDistance = hDirection;
                                string otherLetter = "H";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                        ofVec2f sDirection = sCenterPos - centerPos;
                        float sDistance = sDirection.length();
                        
                        if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                            ofVec2f thirdDistance = sDirection;
                            string otherLetter = "S";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f fourthDistance = sDirection;
                                string otherLetter = "S";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                        ofVec2f rDirection = rCenterPos - centerPos;
                        float rDistance = rDirection.length();
                        
                        if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                            ofVec2f thirdDistance = rDirection;
                            string otherLetter = "R";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f fourthDistance = rDirection;
                                string otherLetter = "R";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        }
    if (m.id == 341 || prevTX > 0) {
                            ofSetColor(0, 255, 0);
                            string letter = letters.at(341);
                            
                            if (m.id == 341) {
                                prevTX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                                prevTY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                            }
                            
                            ofVec2f centerPos = ofVec2f(prevTX, prevTY);
                            if (debugMode == true) {
                                font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                                
                            }
                            ofSetColor(249, 144, 229);
                            ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                            ofVec2f bDirection = bCenterPos - centerPos;
                            float bDistance = bDirection.length();
                            
                            if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f thirdDistance = bDirection;
                                string otherLetter = "B";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f fourthDistance = bDirection;
                                    string otherLetter = "B";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            
                            ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                            ofVec2f dDirection = dCenterPos - centerPos;
                            float dDistance = dDirection.length();
                            
                            if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f thirdDistance = dDirection;
                                string otherLetter = "D";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                    ofVec2f fourthDistance = dDirection;
                                    string otherLetter = "D";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                            ofVec2f eDirection = eCenterPos - centerPos;
                            float eDistance = eDirection.length();
                            
                            if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f thirdDistance = eDirection;
                                string otherLetter = "E";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f fourthDistance = eDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                            ofVec2f aDirection = aCenterPos - centerPos;
                            float aDistance = aDirection.length();
                            
                            if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f thirdDistance = aDirection;
                                string otherLetter = "A";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f fourthDistance = aDirection;
                                    string otherLetter = "A";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                            ofVec2f gDirection = gCenterPos - centerPos;
                            float gDistance = gDirection.length();
                            
                            if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                                ofVec2f thirdDistance = gDirection;
                                string otherLetter = "G";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                    ofVec2f fourthDistance = gDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }

                            ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                            ofVec2f nDirection = nCenterPos - centerPos;
                            float nDistance = nDirection.length();
                            
                            if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f thirdDistance = nDirection;
                                string otherLetter = "N";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                    ofVec2f fourthDistance = nDirection;
                                    string otherLetter = "N";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                            ofVec2f hDirection = hCenterPos - centerPos;
                            float hDistance = hDirection.length();
                            
                            if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f thirdDistance = hDirection;
                                string otherLetter = "H";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                    ofVec2f fourthDistance = hDirection;
                                    string otherLetter = "H";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                            ofVec2f sDirection = sCenterPos - centerPos;
                            float sDistance = sDirection.length();
                            
                            if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f thirdDistance = sDirection;
                                string otherLetter = "S";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                    ofVec2f fourthDistance = sDirection;
                                    string otherLetter = "S";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                            ofVec2f rDirection = rCenterPos - centerPos;
                            float rDistance = rDirection.length();
                            
                            if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f thirdDistance = rDirection;
                                string otherLetter = "R";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f fourthDistance = rDirection;
                                    string otherLetter = "R";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            }
if (m.id == 64 || prevGX > 0) {
                            ofSetColor(0, 255, 0);
                            string letter = letters.at(64);
                            
                            if (m.id == 64) {
                                prevGX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                                prevGY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                            }
                            
                            ofVec2f centerPos = ofVec2f(prevGX, prevGY);
                            if (debugMode == true) {
                                font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                                
                            }
                            ofSetColor(140, 235, 227);
                            ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                            ofVec2f bDirection = bCenterPos - centerPos;
                            float bDistance = bDirection.length();

                            
                            if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f thirdDistance = bDirection;
                                string otherLetter = "B";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f fourthDistance = bDirection;
                                    string otherLetter = "B";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            
                            ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                            ofVec2f dDirection = dCenterPos - centerPos;
                            float dDistance = dDirection.length();
                            
                            if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f thirdDistance = dDirection;
                                string otherLetter = "D";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                    ofVec2f fourthDistance = dDirection;
                                    string otherLetter = "D";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                            ofVec2f eDirection = eCenterPos - centerPos;
                            float eDistance = eDirection.length();
                            
                            if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f thirdDistance = eDirection;
                                string otherLetter = "E";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f fourthDistance = eDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                            ofVec2f tDirection = tCenterPos - centerPos;
                            float tDistance = tDirection.length();
                            
                            if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f thirdDistance = tDirection;
                                string otherLetter = "T";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                    ofVec2f fourthDistance = tDirection;
                                    string otherLetter = "T";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                            ofVec2f aDirection = aCenterPos - centerPos;
                            float aDistance = aDirection.length();
                            
                            if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f thirdDistance = aDirection;
                                string otherLetter = "A";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f fourthDistance = aDirection;
                                    string otherLetter = "A";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }

                            ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                            ofVec2f nDirection = nCenterPos - centerPos;
                            float nDistance = nDirection.length();
                            
                            if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f thirdDistance = nDirection;
                                string otherLetter = "N";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                    ofVec2f fourthDistance = nDirection;
                                    string otherLetter = "N";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                            ofVec2f hDirection = hCenterPos - centerPos;
                            float hDistance = hDirection.length();
                            
                            if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f thirdDistance = hDirection;
                                string otherLetter = "H";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                    ofVec2f fourthDistance = hDirection;
                                    string otherLetter = "H";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                            ofVec2f sDirection = sCenterPos - centerPos;
                            float sDistance = sDirection.length();
                            
                            if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f thirdDistance = sDirection;
                                string otherLetter = "S";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                    ofVec2f fourthDistance = sDirection;
                                    string otherLetter = "S";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                            ofVec2f rDirection = rCenterPos - centerPos;
                            float rDistance = rDirection.length();
                            
                            if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f thirdDistance = rDirection;
                                string otherLetter = "R";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f fourthDistance = rDirection;
                                    string otherLetter = "R";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            }
if (m.id == 177 || prevNX > 0) {
                            ofSetColor(0, 255, 0);
                            string letter = letters.at(177);
                            
                            if (m.id == 177) {
                                prevNX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                                prevNY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                            }
                            
                            ofVec2f centerPos = ofVec2f(prevNX, prevNY);
                            if (debugMode == true) {
                                font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                                
                            }
                            ofSetColor(86, 77, 130);
                                ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                            ofVec2f bDirection = bCenterPos - centerPos;
                            float bDistance = bDirection.length();
    
                            ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                            ofVec2f gDirection = gCenterPos - centerPos;
                            float gDistance = gDirection.length();
                               
                           if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                               ofVec2f thirdDistance = gDirection;
                               string otherLetter = "G";
                               string middleLetter;
                               string firstLetter;
                               string lastLetter;
                               
                               for (auto i: threeLetter) {
                                   firstLetter = i[0];
                                   lastLetter = i[2];
                                   if (firstLetter == letter && otherLetter == lastLetter) {
                                       middleLetter = i[1];
                                   }
                               }
                           
                               thirdDistance /= 2;
                                
                               font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                 
                               }
                               else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                   ofVec2f fourthDistance = gDirection;
                                   string otherLetter = "G";
                                   string secondLetter;
                                   string thirdLetter;
                                   string firstLetter;
                                   string lastLetter;

                                   for (auto i: fourLetter) {
                                       firstLetter = i[0];
                                       lastLetter = i[3];
                                       if (firstLetter == letter && otherLetter == lastLetter) {
                                           secondLetter = i[1];
                                           thirdLetter = i[2];
                                       }
                                   }

                                   fourthDistance /= 3;
                                    
                                   font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                   font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                               }
                            
                            if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f thirdDistance = bDirection;
                                string otherLetter = "B";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f fourthDistance = bDirection;
                                    string otherLetter = "B";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            
                            ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                            ofVec2f dDirection = dCenterPos - centerPos;
                            float dDistance = dDirection.length();
                            
                            if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f thirdDistance = dDirection;
                                string otherLetter = "D";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                    ofVec2f fourthDistance = dDirection;
                                    string otherLetter = "D";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                            ofVec2f eDirection = eCenterPos - centerPos;
                            float eDistance = eDirection.length();
                            
                            if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f thirdDistance = eDirection;
                                string otherLetter = "E";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f fourthDistance = eDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                            ofVec2f tDirection = tCenterPos - centerPos;
                            float tDistance = tDirection.length();
                            
                            if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f thirdDistance = tDirection;
                                string otherLetter = "T";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                    ofVec2f fourthDistance = tDirection;
                                    string otherLetter = "T";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                            ofVec2f aDirection = aCenterPos - centerPos;
                            float aDistance = aDirection.length();
                            
                            if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f thirdDistance = aDirection;
                                string otherLetter = "A";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f fourthDistance = aDirection;
                                    string otherLetter = "A";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }

                            ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                            ofVec2f hDirection = hCenterPos - centerPos;
                            float hDistance = hDirection.length();
                            
                            if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f thirdDistance = hDirection;
                                string otherLetter = "H";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                    ofVec2f fourthDistance = hDirection;
                                    string otherLetter = "H";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                            ofVec2f sDirection = sCenterPos - centerPos;
                            float sDistance = sDirection.length();
                            
                            if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f thirdDistance = sDirection;
                                string otherLetter = "S";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                    ofVec2f fourthDistance = sDirection;
                                    string otherLetter = "S";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                            ofVec2f rDirection = rCenterPos - centerPos;
                            float rDistance = rDirection.length();
                            
                            if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f thirdDistance = rDirection;
                                string otherLetter = "R";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f fourthDistance = rDirection;
                                    string otherLetter = "R";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            }
if (m.id == 428 || prevHX > 0) {
                            ofSetColor(0, 255, 0);
                            string letter = letters.at(428);
                            
                            if (m.id == 428) {
                                prevHX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                                prevHY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                            }
                            
                            ofVec2f centerPos = ofVec2f(prevHX, prevHY);
                            if (debugMode == true) {
                                font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                                
                            }
                            ofSetColor(116, 160, 214);
                            ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                            ofVec2f bDirection = bCenterPos - centerPos;
                            float bDistance = bDirection.length();
    
                            ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                            ofVec2f gDirection = gCenterPos - centerPos;
                            float gDistance = gDirection.length();
                               
                           if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                               ofVec2f thirdDistance = gDirection;
                               string otherLetter = "G";
                               string middleLetter;
                               string firstLetter;
                               string lastLetter;
                               
                               for (auto i: threeLetter) {
                                   firstLetter = i[0];
                                   lastLetter = i[2];
                                   if (firstLetter == letter && otherLetter == lastLetter) {
                                       middleLetter = i[1];
                                   }
                               }
                           
                               thirdDistance /= 2;
                                
                               font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                 
                               }
                               else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                   ofVec2f fourthDistance = gDirection;
                                   string otherLetter = "G";
                                   string secondLetter;
                                   string thirdLetter;
                                   string firstLetter;
                                   string lastLetter;

                                   for (auto i: fourLetter) {
                                       firstLetter = i[0];
                                       lastLetter = i[3];
                                       if (firstLetter == letter && otherLetter == lastLetter) {
                                           secondLetter = i[1];
                                           thirdLetter = i[2];
                                       }
                                   }

                                   fourthDistance /= 3;
                                    
                                   font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                   font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                               }
                            
                            if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f thirdDistance = bDirection;
                                string otherLetter = "B";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f fourthDistance = bDirection;
                                    string otherLetter = "B";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            
                            ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                            ofVec2f dDirection = dCenterPos - centerPos;
                            float dDistance = dDirection.length();
                            
                            if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f thirdDistance = dDirection;
                                string otherLetter = "D";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                    ofVec2f fourthDistance = dDirection;
                                    string otherLetter = "D";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                            ofVec2f eDirection = eCenterPos - centerPos;
                            float eDistance = eDirection.length();
                            
                            if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f thirdDistance = eDirection;
                                string otherLetter = "E";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f fourthDistance = eDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                            ofVec2f tDirection = tCenterPos - centerPos;
                            float tDistance = tDirection.length();
                            
                            if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f thirdDistance = tDirection;
                                string otherLetter = "T";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                    ofVec2f fourthDistance = tDirection;
                                    string otherLetter = "T";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                            ofVec2f aDirection = aCenterPos - centerPos;
                            float aDistance = aDirection.length();
                            
                            if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f thirdDistance = aDirection;
                                string otherLetter = "A";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f fourthDistance = aDirection;
                                    string otherLetter = "A";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }

                            ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                            ofVec2f nDirection = nCenterPos - centerPos;
                            float nDistance = nDirection.length();
                            
                            if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f thirdDistance = nDirection;
                                string otherLetter = "N";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                    ofVec2f fourthDistance = nDirection;
                                    string otherLetter = "N";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                        
                            ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                            ofVec2f sDirection = sCenterPos - centerPos;
                            float sDistance = sDirection.length();
                            
                            if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f thirdDistance = sDirection;
                                string otherLetter = "S";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                    ofVec2f fourthDistance = sDirection;
                                    string otherLetter = "S";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                            ofVec2f rDirection = rCenterPos - centerPos;
                            float rDistance = rDirection.length();
                            
                            if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f thirdDistance = rDirection;
                                string otherLetter = "R";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f fourthDistance = rDirection;
                                    string otherLetter = "R";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            }
if (m.id == 760 || prevSX > 0) {
                          string letter = letters.at(760);
                          
                          if (m.id == 760) {
                              prevSX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
                              prevSY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
                          }
                          
                          ofVec2f centerPos = ofVec2f(prevSX, prevSY);
                          ofSetColor(0, 255, 0);
                          if (debugMode == true) {
                              font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
                              
                          }
                            ofSetColor(87, 48, 11);
                            ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                            ofVec2f bDirection = bCenterPos - centerPos;
                            float bDistance = bDirection.length();
    
                            ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                            ofVec2f gDirection = gCenterPos - centerPos;
                            float gDistance = gDirection.length();
                               
                           if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                               ofVec2f thirdDistance = gDirection;
                               string otherLetter = "G";
                               string middleLetter;
                               string firstLetter;
                               string lastLetter;
                               
                               for (auto i: threeLetter) {
                                   firstLetter = i[0];
                                   lastLetter = i[2];
                                   if (firstLetter == letter && otherLetter == lastLetter) {
                                       middleLetter = i[1];
                                   }
                               }
                           
                               thirdDistance /= 2;
                                
                               font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                 
                               }
                               else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                                   ofVec2f fourthDistance = gDirection;
                                   string otherLetter = "G";
                                   string secondLetter;
                                   string thirdLetter;
                                   string firstLetter;
                                   string lastLetter;

                                   for (auto i: fourLetter) {
                                       firstLetter = i[0];
                                       lastLetter = i[3];
                                       if (firstLetter == letter && otherLetter == lastLetter) {
                                           secondLetter = i[1];
                                           thirdLetter = i[2];
                                       }
                                   }

                                   fourthDistance /= 3;
                                    
                                   font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                   font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                               }
                            
                            if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f thirdDistance = bDirection;
                                string otherLetter = "B";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                    ofVec2f fourthDistance = bDirection;
                                    string otherLetter = "B";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                    }
                            
                            ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                            ofVec2f dDirection = dCenterPos - centerPos;
                            float dDistance = dDirection.length();
                            
                            if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f thirdDistance = dDirection;
                                string otherLetter = "D";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                    ofVec2f fourthDistance = dDirection;
                                    string otherLetter = "D";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                            ofVec2f eDirection = eCenterPos - centerPos;
                            float eDistance = eDirection.length();
                            
                            if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f thirdDistance = eDirection;
                                string otherLetter = "E";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                    ofVec2f fourthDistance = eDirection;
                                    string otherLetter = "E";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                            ofVec2f tDirection = tCenterPos - centerPos;
                            float tDistance = tDirection.length();
                            
                            if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f thirdDistance = tDirection;
                                string otherLetter = "T";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                    ofVec2f fourthDistance = tDirection;
                                    string otherLetter = "T";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                            ofVec2f aDirection = aCenterPos - centerPos;
                            float aDistance = aDirection.length();
                            
                            if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f thirdDistance = aDirection;
                                string otherLetter = "A";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                    ofVec2f fourthDistance = aDirection;
                                    string otherLetter = "A";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }

                            ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                            ofVec2f nDirection = nCenterPos - centerPos;
                            float nDistance = nDirection.length();
                            
                            if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f thirdDistance = nDirection;
                                string otherLetter = "N";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                    ofVec2f fourthDistance = nDirection;
                                    string otherLetter = "N";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                            ofVec2f hDirection = hCenterPos - centerPos;
                            float hDistance = hDirection.length();
                            
                            if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f thirdDistance = hDirection;
                                string otherLetter = "H";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                    ofVec2f fourthDistance = hDirection;
                                    string otherLetter = "H";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            
                            ofVec2f rCenterPos = ofVec2f(prevRX, prevRY);
                            ofVec2f rDirection = rCenterPos - centerPos;
                            float rDistance = rDirection.length();
                            
                            if (rDistance <= 190 && rDistance > 70 && rDirection.x < 10 && rDirection.y > 10) {
                                ofVec2f thirdDistance = rDirection;
                                string otherLetter = "R";
                                string middleLetter;
                                string firstLetter;
                                string lastLetter;
                                
                                for (auto i: threeLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[2];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        middleLetter = i[1];
                                    }
                                }
                            
                                thirdDistance /= 2;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                                  
                                }
                                else if (rDistance > 210  && rDistance < 250 && rDirection.x < 10 && rDirection.y > 10) {
                                    ofVec2f fourthDistance = rDirection;
                                    string otherLetter = "R";
                                    string secondLetter;
                                    string thirdLetter;
                                    string firstLetter;
                                    string lastLetter;

                                    for (auto i: fourLetter) {
                                        firstLetter = i[0];
                                        lastLetter = i[3];
                                        if (firstLetter == letter && otherLetter == lastLetter) {
                                            secondLetter = i[1];
                                            thirdLetter = i[2];
                                        }
                                    }

                                    fourthDistance /= 3;
                                     
                                    font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                    font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                            }
if (m.id == 882 || prevRX > 0) {
        ofSetColor(0, 255, 0);
        string letter = letters.at(882);

        if (m.id == 882) {
            prevRX = ofMap(m.getCenter().x, 0, 640, 0, ofGetWindowWidth());
            prevRY = ofMap(m.getCenter().y, 0, 480, 0, ofGetWindowHeight());
        }
        ofVec2f centerPos = ofVec2f(prevRX, prevRY);
        if (debugMode == true) {
            font("AlphaFridgeMagnets", 24).drawString(letter, centerPos.x, centerPos.y);
            
        }
        ofSetColor(57, 182, 9);
                            ofVec2f bCenterPos = ofVec2f(prevBX, prevBY);
                        ofVec2f bDirection = bCenterPos - centerPos;
                        float bDistance = bDirection.length();

                        ofVec2f gCenterPos = ofVec2f(prevGX, prevGY);
                        ofVec2f gDirection = gCenterPos - centerPos;
                        float gDistance = gDirection.length();
                           
                       if (gDistance <= 190 && gDistance > 70 && gDirection.x < 10 && gDirection.y > 10) {
                           ofVec2f thirdDistance = gDirection;
                           string otherLetter = "G";
                           string middleLetter;
                           string firstLetter;
                           string lastLetter;
                           
                           for (auto i: threeLetter) {
                               firstLetter = i[0];
                               lastLetter = i[2];
                               if (firstLetter == letter && otherLetter == lastLetter) {
                                   middleLetter = i[1];
                               }
                           }
                       
                           thirdDistance /= 2;
                            
                           font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                             
                           }
                           else if (gDistance > 210  && gDistance < 250 && gDirection.x < 10 && gDirection.y > 10) {
                               ofVec2f fourthDistance = gDirection;
                               string otherLetter = "G";
                               string secondLetter;
                               string thirdLetter;
                               string firstLetter;
                               string lastLetter;

                               for (auto i: fourLetter) {
                                   firstLetter = i[0];
                                   lastLetter = i[3];
                                   if (firstLetter == letter && otherLetter == lastLetter) {
                                       secondLetter = i[1];
                                       thirdLetter = i[2];
                                   }
                               }

                               fourthDistance /= 3;
                                
                               font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                               font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                           }
                        
                        if (bDistance <= 190 && bDistance > 70 && bDirection.x < 10 && bDirection.y > 10) {
                            ofVec2f thirdDistance = bDirection;
                            string otherLetter = "B";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (bDistance > 210  && bDistance < 250 && bDirection.x < 10 && bDirection.y > 10) {
                                ofVec2f fourthDistance = bDirection;
                                string otherLetter = "B";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                                }
                        
                        ofVec2f dCenterPos = ofVec2f(prevDX, prevDY);
                        ofVec2f dDirection = dCenterPos - centerPos;
                        float dDistance = dDirection.length();
                        
                        if (dDistance <= 190 && dDistance > 70 && dDirection.x < 10 && dDirection.y > 10) {
                            ofVec2f thirdDistance = dDirection;
                            string otherLetter = "D";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (dDistance > 210  && dDistance < 250 && dDirection.x < 10 && dDirection.y > 10) {
                                ofVec2f fourthDistance = dDirection;
                                string otherLetter = "D";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f eCenterPos = ofVec2f(prevEX, prevEY);
                        ofVec2f eDirection = eCenterPos - centerPos;
                        float eDistance = eDirection.length();
                        
                        if (eDistance <= 190 && eDistance > 70 && eDirection.x < 10 && eDirection.y > 10) {
                            ofVec2f thirdDistance = eDirection;
                            string otherLetter = "E";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (eDistance > 210  && eDistance < 250 && eDirection.x < 10 && eDirection.y > 10) {
                                ofVec2f fourthDistance = eDirection;
                                string otherLetter = "E";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f tCenterPos = ofVec2f(prevTX, prevTY);
                        ofVec2f tDirection = tCenterPos - centerPos;
                        float tDistance = tDirection.length();
                        
                        if (tDistance <= 190 && tDistance > 70 && tDirection.x < 10 && tDirection.y > 10) {
                            ofVec2f thirdDistance = tDirection;
                            string otherLetter = "T";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (tDistance > 210  && tDistance < 250 && tDirection.x < 10 && tDirection.y > 10) {
                                ofVec2f fourthDistance = tDirection;
                                string otherLetter = "T";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f aCenterPos = ofVec2f(prevAX, prevAY);
                        ofVec2f aDirection = aCenterPos - centerPos;
                        float aDistance = aDirection.length();
                        
                        if (aDistance <= 190 && aDistance > 70 && aDirection.x < 10 && aDirection.y > 10) {
                            ofVec2f thirdDistance = aDirection;
                            string otherLetter = "A";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (aDistance > 210  && aDistance < 250 && aDirection.x < 10 && aDirection.y > 10) {
                                ofVec2f fourthDistance = aDirection;
                                string otherLetter = "A";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }

                        ofVec2f nCenterPos = ofVec2f(prevNX, prevNY);
                        ofVec2f nDirection = nCenterPos - centerPos;
                        float nDistance = nDirection.length();
                        
                        if (nDistance <= 190 && nDistance > 70 && nDirection.x < 10 && nDirection.y > 10) {
                            ofVec2f thirdDistance = nDirection;
                            string otherLetter = "N";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (nDistance > 210  && nDistance < 250 && nDirection.x < 10 && nDirection.y > 10) {
                                ofVec2f fourthDistance = nDirection;
                                string otherLetter = "N";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        ofVec2f hCenterPos = ofVec2f(prevHX, prevHY);
                        ofVec2f hDirection = hCenterPos - centerPos;
                        float hDistance = hDirection.length();
                        
                        if (hDistance <= 190 && hDistance > 70 && hDirection.x < 10 && hDirection.y > 10) {
                            ofVec2f thirdDistance = hDirection;
                            string otherLetter = "H";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (hDistance > 210  && hDistance < 250 && hDirection.x < 10 && hDirection.y > 10) {
                                ofVec2f fourthDistance = hDirection;
                                string otherLetter = "H";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        
                        ofVec2f sCenterPos = ofVec2f(prevSX, prevSY);
                        ofVec2f sDirection = sCenterPos - centerPos;
                        float sDistance = sDirection.length();
                        
                        if (sDistance <= 190 && sDistance > 70 && sDirection.x < 10 && sDirection.y > 10) {
                            ofVec2f thirdDistance = sDirection;
                            string otherLetter = "S";
                            string middleLetter;
                            string firstLetter;
                            string lastLetter;
                            
                            for (auto i: threeLetter) {
                                firstLetter = i[0];
                                lastLetter = i[2];
                                if (firstLetter == letter && otherLetter == lastLetter) {
                                    middleLetter = i[1];
                                }
                            }
                        
                            thirdDistance /= 2;
                             
                            font("AlphaFridgeMagnets", 28).drawString(middleLetter, centerPos.x+(thirdDistance.x), centerPos.y+(thirdDistance.y));
                              
                            }
                            else if (sDistance > 210  && sDistance < 250 && sDirection.x < 10 && sDirection.y > 10) {
                                ofVec2f fourthDistance = sDirection;
                                string otherLetter = "S";
                                string secondLetter;
                                string thirdLetter;
                                string firstLetter;
                                string lastLetter;

                                for (auto i: fourLetter) {
                                    firstLetter = i[0];
                                    lastLetter = i[3];
                                    if (firstLetter == letter && otherLetter == lastLetter) {
                                        secondLetter = i[1];
                                        thirdLetter = i[2];
                                    }
                                }

                                fourthDistance /= 3;
                                 
                                font("AlphaFridgeMagnets", 28).drawString(secondLetter, centerPos.x+(fourthDistance.x), centerPos.y+(fourthDistance.y));
                                font("AlphaFridgeMagnets", 28).drawString(thirdLetter, centerPos.x+(fourthDistance.x * 2), centerPos.y+(fourthDistance.y * 2));
                            }
                        }

                }

    }
    map.end(0);
    myFlower.draw();
    myFlower1.draw();
    myFlower2.draw();
    myFlower3.draw();
    myFlower4.draw();
    
    ofSetColor(0);
    
    ofDrawBitmapString("markers detected: " + ofToString(aruco.getNumMarkers()), 20, 160);
    ofDrawBitmapString("fps " + ofToString(ofGetFrameRate()), 20, 180);
    ofDrawBitmapString("m toggles markers", 20, 60);
    ofDrawBitmapString("b toggles board", 20, 80);
    ofDrawBitmapString("i toggles board image", 20, 100);
    ofDrawBitmapString("s saves board image", 20, 120);
    ofDrawBitmapString("0-9 saves marker image", 20, 140);
}


bool ofApp::movePoint(vector<ofVec2f>& points, ofVec2f point) {
    for(int i = 0; i < points.size(); i++) {
        if(points[i].distance(point) < 20) {
            movingPoint = true;
            curPoint = &points[i];
            return true;
        }
    }
    return false;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'm') showMarkers = !showMarkers;
	if (key == 'b') showBoard = !showBoard;
	if (key == 'i') showBoardImage = !showBoardImage;
	if (key == 'l') board.save("boardimage.png");
	if (key >= '0' && key <= '9') {
		// there's 1024 different markers
		int markerID = key - '0';
        aruco.getMarkerImage(markerID, 240, marker);
		marker.save("marker" + ofToString(markerID) + ".png");
	}
    if(key == 's') {
        saveMatrix = true;
    }
    if(key == 'n') {
        settingNewPoints = true;
    }
    if (key=='d') {
        map.toggleDebug();
    }
    
    if (key == 'e') {
        debugMode = !debugMode;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if(movingPoint) {
        curPoint->set(x, y);
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    ofVec2f cur(x, y);
    ofVec2f rightOffset(warpedColor.getWidth(), 0);
    movePoint(srcPoints, cur);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    movingPoint = false;
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

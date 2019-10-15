#pragma once

#include "ofMain.h"
#include "ofxAruco.h"
#include "ofxPS3EyeGrabber.h"
#include "ofxProjectionMapping.h"
#include "Flower.hpp"


class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
        bool movePoint(vector<ofVec2f>& points, ofVec2f point);
        void drawPoints(vector<ofVec2f>& points);
		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mousePressed(int x, int y, int button);
        void mouseDragged(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
//        auto markerArray[];
        ofImage warpedColor;
        vector<ofVec2f> destPoints, srcPoints;
        bool movingPoint;
        ofVec2f* curPoint;
        bool saveMatrix;
        bool homographyReady;
        bool settingNewPoints;
        int currentPoint;
    
    int prevAX, prevBX, prevAY, prevBY, prevDX, prevDY, prevEX, prevEY, prevTX, prevTY, prevGX, prevGY, prevNX, prevNY, prevHX, prevHY, prevSX, prevSY, prevRX, prevRY;
//        string wordArray;
        string wordArray[7];
        string fourLetterWords[2];
    
    string threeLetter[70] = {"ADD", "AXE", "ANT", "ANN", "ASH", "AIR", "ARB", "BOA", "BAD", "BEE", "BAT", "BUG", "BIN", "BUS", "BAR", "DAB", "DOE", "DOT", "DOG", "DEN", "ERA", "EBB", "END", "EAT", "EGG", "EON", "EAR", "TEA", "TUB", "TAD", "TIE", "TUG", "TON", "TAR", "GOA", "GOD", "GEE", "GET", "GUN", "GAS", "GAR", "NAB", "NOD", "NUT", "NAG", "NOR", "HUB", "HID", "HUE", "HAT", "HUG", "HEN", "HIS", "HER", "SEA", "SOB", "SAD", "SEE", "SIT", "SAG", "SUN", "SIR", "RIB", "RUB", "ROD", "RED", "RYE", "RAT", "RUG", "RUN"};
    
    string fourLetter[84] = {"AQUA", "ACID", "AXLE", "AUNT", "AGOG", "AXON", "ARGH", "AXIS", "AJAR", "BOBA", "BIRD", "BIKE", "BEAT", "BRAG", "BURN", "BATH", "BOSS", "BEAR", "DIVA", "DUMB", "DIME", "DART", "DRUG", "DAWN", "DISH", "DUDS", "DEER", "EMMA", "EYED", "EMIT", "EARN", "EACH", "EELS", "EVER", "TOGA", "TIED", "TIME", "TILT", "TWIG", "TWIN", "TECH", "TOSS", "GIGA", "GLOB", "GRID", "GLUE", "GIFT", "GOAT", "GRIN", "GEMS", "NOVA", "NUMB", "NERD", "NEST", "NEWS", "NEAR", "HULA", "HERB", "HEAD", "HOLE", "HUNT", "HANG", "HORN", "HISS", "HAIR", "SCAR", "STAR", "SEMI", "SWAN", "SPIN", "SLUG", "SOFT", "SALT", "SIDE", "SAME", "SAND", "SWAB", "SODA", "READ", "ROSE", "RAFT", "RING", "ROTI", "RIBS"};


        cv::Mat homography;
    
		ofVideoGrabber grabber;

		ofBaseVideoDraws * video;

		ofxAruco aruco;
		bool useVideo;
		bool showMarkers;
		bool showBoard;
		bool showBoardImage;
		ofImage board;
		ofImage marker;
    
        ofImage letter1;
        map<int, string> letters;
    
        ofxProjectionMapping map;
    
        Flower myFlower, myFlower1, myFlower2, myFlower3, myFlower4;
    
        bool debugMode;
};

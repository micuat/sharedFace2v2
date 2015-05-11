#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxCv.h"
#include "ofxXmlSettings.h"

// listen on port 12345
#define PORT 57121
#define NUM_MSG_STRINGS 20

class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxOscReceiver receiver;
	ofVboMesh mesh, meshTemplate;
	ofEasyCam cam;
	ofVec3f centroid;

	cv::Mat proIntrinsics, proExtrinsics;
	ofxCv::Intrinsics proCalibration;
	cv::Size proSize;

	ofxXmlSettings XML;

	ofFbo fbo;
	
	ofVec2f viewShift;
};

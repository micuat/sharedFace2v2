#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxCv.h"
#include "ofxXmlSettings.h"
#include "ofxFluid.h"
#include "ofxGpuParticles.h"

// listen on port 12345
#define PORT 57121
#define PORT_HDFACE 57122
#define NUM_MSG_STRINGS 20

#define SURFACE_WIDTH 1920
#define SURFACE_HEIGHT 1080
#define PROJECTOR_WIDTH 1024
#define PROJECTOR_HEIGHT 768

class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void updateMesh(ofxOscMessage &m);
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

	enum RenderSwitch {
		Fluid,
		Particles
	};

	RenderSwitch renderSwitch;

	ofxOscReceiver receiver;
	ofVboMesh mesh, meshTemplate;
	ofEasyCam cam;
	ofVec3f centroid;

	cv::Mat proIntrinsics, proExtrinsics;
	ofxCv::Intrinsics proCalibration;
	cv::Size proSize;

	ofxXmlSettings XML;

	ofFbo fbo;

    ofxFluid fluid;

	ofVec2f viewShift;

	vector<ofVec3f> trackedTips;

	struct closestVertex {
		int index;
		float distanceSquared;
		float distance() {return sqrtf(distanceSquared);};
		bool updated;
		closestVertex() : index(0), distanceSquared(100000), updated(false) {};
	};
	vector<closestVertex> closestVertices;
	ofVec3f contactPoint;
	ofVec2f contactCoord, contactCoordPrev;

	ofFloatColor curColor;

    void onParticlesUpdate(ofShader& shader);
    
    ofxGpuParticles particles;
};

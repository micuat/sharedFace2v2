#pragma once
    
// flags to skip features for efficient debugging
#define WITH_FLUID
#define WITH_APPLE
//#define WITH_PARTICLES
//#define WITH_SKULL

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxCv.h"
#include "ofxXmlSettings.h"
#include "ofxFluid.h"
#include "ofxGpuParticles.h"

#ifdef WITH_SKULL
#include "ofxVolumetrics.h"
#endif

// listen on port 12345
#define PORT 57121
#define PORT_HDFACE 57122
#define PORT_SPEECH 57123

#define SURFACE_WIDTH 1920
#define SURFACE_HEIGHT 1080
#define PROJECTOR_WIDTH 1024
#define PROJECTOR_HEIGHT 768

class ofApp : public ofBaseApp {
public:

	void setup();
	void setupProjector();
    void setupFluid();
    void setupApple();
    void setupParticles();
	void setupSkull();
    void update();
    void updateApple();
    void updateMesh(ofxOscMessage &m);
    void draw();
    void drawApple();

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
        Apple,
		Particles,
        Skull
	};

	RenderSwitch renderSwitch;

	ofxOscReceiver receiver;
	ofVboMesh mesh, meshTemplate, meshTex;
	ofEasyCam cam;
	ofVec3f centroid, centroidTemplate;

	cv::Mat proIntrinsics, proExtrinsics;
	ofxCv::Intrinsics proCalibration;
	cv::Size proSize;

	ofxXmlSettings XML;

	ofFbo fbo;

    ofxFluid fluid;

#ifdef WITH_SKULL
    ofxVolumetrics myVolume;
#endif

    ofVec3f viewShift;

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

    ofQuaternion quaternion;
    ofxCv::KalmanEuler kalman;
    ofxCv::KalmanPosition kalmanP;

    ofShader lensShader;

    string hexColor, command;

    int happy;
    int trackingId;

    class AnApple
    {
    public:
        ofVec2f position;
        void update()
        {
            position.y += 10;
        }
        void draw()
        {
            ofPushStyle();
            ofSetColor(ofColor::red);
            ofCircle(position, 50);
            ofPopStyle();
        }
    };
    vector<AnApple> apples;
};

// flags to skip features for efficient debugging
#define WITH_FLUID
#define WITH_APPLE
//#define WITH_PARTICLES
//#define WITH_SKULL

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxCv.h"
#include "ofxXmlSettings.h"
#include "ofxGui.h"
#include "ofxFluid.h"
#include "ofxPubSubOsc.h"
#include "ofxSecondWindow.h"

#ifdef WITH_PARTICLES
#include "ofxGpuParticles.h"
#endif

#ifdef WITH_SKULL
#include "ofxVolumetrics.h"
#endif

// listen on port 12345
#define PORT 57121
#define PORT_HDFACE 57122
#define PORT_SPEECH 57123
#define PORT_PD 8001

#define SURFACE_WIDTH 1920
#define SURFACE_HEIGHT 1080
#define PROJECTOR_WIDTH 1024
#define PROJECTOR_HEIGHT 768

static ofVec2f nosePosition;

struct closestVertex {
    int index;
    float distanceSquared;
    float distance() { return sqrtf(distanceSquared); };
    bool updated;
    closestVertex() : index(0), distanceSquared(100000), updated(false) {};
};

struct InputStatus
{
    ofVec2f contactCoord;
    float contactDistance;
    int mouthOpen;
    int happy;
    bool buttonMouthOpen;
};

class AppleController
{
public:
    ofVec2f position;
    float ripeness;
    enum DieType { Eaten, Captured, Dropped };
    DieType dieType;
    int dieCount;
    int dieMax;
    static const int appleSpeed = 5;
    static const int appleRadius = 50;
    enum StateMachine {Emerge, Drop, DieAnimation, Dead};
    StateMachine stateMachine;
    float lastStateChangedTime;
    float appleCurRadius;

    AppleController() : dieCount(0), dieMax(30), stateMachine(Emerge), appleCurRadius(0) {
        ripeness = 0;
        float p = ofRandom(0, 3);
        position.y = nosePosition.y;
        if (p < 1)
        {
            position.x = 1024 * 0.5f;
        }
        else if (p < 2)
        {
            position.x = 1024 * 0.5f - 200;
        }
        else
        {
            position.x = 1024 * 0.5f + 200;
        }

        lastStateChangedTime = ofGetElapsedTimef();
    }

    void update(InputStatus status, float velocity)
    {
        float curTime = ofGetElapsedTimef();

        switch (stateMachine)
        {
        case Emerge:
            if ((status.happy >= 1) || status.buttonMouthOpen)
            {
                ripeness += 0.02f;
                ripeness = ofClamp(ripeness, 0, 1);
            }
            else
            {
                ripeness -= 0.02f;
                ripeness = ofClamp(ripeness, 0, 1);
            }

            if (ripeness < 0.9999f)
            {
                appleCurRadius = ofMap(ripeness, 0, 1, 30, appleRadius);
            }
            else
            {
                stateMachine = Drop;
            }
            break;
        case Drop:
            position.y += velocity;
            break;
        case DieAnimation:
            if (dieCount >= dieMax)
            {
                stateMachine = Dead;
                lastStateChangedTime = ofGetElapsedTimef();
            }
            dieCount++;
            break;
        case Dead:
            break;
        }
    }

    ofFloatColor getColor()
    {
        ofFloatColor color;
        color.setHsb(ofMap(ripeness, 0, 1, 0.5f, 0), 1, 1);
        return color;
    }

    void draw()
    {
        ofPushStyle();
        ofSetColor(getColor());

        switch (stateMachine)
        {
        case Emerge:
            ofCircle(position, appleCurRadius);
            break;
        case Drop:
            ofCircle(position, appleRadius);
            break;
        case DieAnimation:
            if (dieType == Eaten)
            {
                ofCircle(position, appleRadius);
                ofSetColor(ofColor::black);
                ofCircle(position + ofVec2f(ofLerp(appleRadius, 0, (float)dieCount / dieMax), 0), appleRadius);
            }
            break;
        case Dead:
            break;
        }
        ofPopStyle();
    }

    void kill(DieType _dieType)
    {
        if (stateMachine == Emerge || stateMachine == Drop)
        {
            stateMachine = DieAnimation;
            lastStateChangedTime = ofGetElapsedTimef();
            dieCount = 1;
            ofxPublishRegisteredOsc("localhost", PORT_PD, "/sdt/bubble");
            dieType = _dieType;
        }
    }

    bool isCompletelyDead()
    {
        return stateMachine == Dead;
    }

    bool isAtLeastLightlyDead()
    {
        return stateMachine == DieAnimation || stateMachine == Dead;
    }
};

class GameController
{
public:
    vector<AppleController> apples;
    int appleLife;
    const int appleMaxLife = 8;
    const int appleDefaultLife = 3;
    int width, height;
    enum StateMachine {WaitHappy, WaitTouch, Play, Gameover, Clear} stateMachine;
    float lastStateChangedTime;
    ofxFluid* fluid;
    int waitCount;

    GameController() : appleLife(appleDefaultLife), stateMachine(WaitHappy), lastStateChangedTime(0), waitCount(0) {}

    void setup(int _width, int _height)
    {
        width = _width;
        height = _height;
        nosePosition = ofVec2f(width / 2, 400);
    }

    void update(InputStatus status)
    {
        switch (stateMachine)
        {
        case WaitHappy:
            if (status.happy >= 1 || status.buttonMouthOpen)
            {
                waitCount++;
                if (waitCount > 60)
                {
                    stateMachine = WaitTouch;
                    lastStateChangedTime = ofGetElapsedTimef();
                    appleLife = appleDefaultLife;
                    waitCount = 0;
                }
            }
            else
            {
                waitCount--;
                if (waitCount < 0)
                    waitCount = 0;
            }
            break;
        case WaitTouch:
            if (status.contactDistance < 0.03f && status.contactCoord.distanceSquared(nosePosition) < 50 * 50)
            {
                stateMachine = Play;
                lastStateChangedTime = ofGetElapsedTimef();
                appleLife = appleDefaultLife;
                waitCount = 0;
            }
            break;
        case Play:
            if (apples.size() == 0)
            {
                AppleController a;
                apples.push_back(a);
                fluid->clear();
            }
            for (int i = 0; i < apples.size(); i++)
            {
                auto& apple = apples.at(i);
                float yOld = apple.position.y;
                apple.update(status, ofMap(appleLife, 0, appleMaxLife, 3, 10));
                if (apple.isAtLeastLightlyDead()) continue;

                float threshold = 75 * 75;
                bool isCrossedChin = yOld < 650 && apple.position.y >= 650;
                bool isTouched = status.contactDistance < 0.03 && status.contactCoord.distanceSquared(apple.position) < threshold;

                ofFloatColor color = apple.getColor();

                if (apple.stateMachine == AppleController::Drop)
                {
                    if (isTouched)
                    {
                        appleLife += 1;
                        apple.kill(AppleController::Captured);
                        for (int i = 0; i < 8; i++)
                        {
                            ofVec2f d = ofVec2f(1, 0).getRotated(i * 45);
                            fluid->addTemporalForce(apple.position + d * 50, d * 50, color, 5, 15, 2);
                        }
                    }
                    else if (isCrossedChin)
                    {
                        apple.kill(AppleController::Eaten);
                        appleLife -= 1;
                   }
                }
            }
            for (int i = 0; i < apples.size(); )
            {
                if (apples.at(i).isCompletelyDead())
                {
                    apples.erase(apples.begin() + i);
                }
                else
                {
                    i++;
                }
            }
            if (appleLife == 0)
            {
                stateMachine = Gameover;
                lastStateChangedTime = ofGetElapsedTimef();
                ofxPublishRegisteredOsc("localhost", PORT_PD, "/sdt/disappointed");
                apples.clear();
            }
            else if (appleLife == appleMaxLife)
            {
                stateMachine = Clear;
                lastStateChangedTime = ofGetElapsedTimef();
                ofxPublishRegisteredOsc("localhost", PORT_PD, "/sdt/applause");
                apples.clear();
            }
            break;
        case Gameover:
            if (ofGetElapsedTimef() - lastStateChangedTime > 5)
            {
                stateMachine = WaitHappy;
                appleLife = appleDefaultLife;
                lastStateChangedTime = ofGetElapsedTimef();
                fluid->clear();
            }
            break;
        case Clear:
            if (ofGetElapsedTimef() - lastStateChangedTime > 5)
            {
                stateMachine = WaitHappy;
                appleLife = appleDefaultLife;
                lastStateChangedTime = ofGetElapsedTimef();
                fluid->clear();
            }
            break;
        }
    }

    void draw()
    {
        ofPushStyle();
        ofPushMatrix();

        for (int i = 0; i < apples.size(); i++)
        {
            apples.at(i).draw();
        }

        if (stateMachine == WaitHappy)
        {
            ofFloatColor color;
            float ripeness = ofMap(waitCount, 0, 60, 0, 1);
            color.setHsb(ofMap(ripeness, 0, 1, 0.5f, 0), 1, 1);
            ofSetColor(color);
            float alpha = ofMap(sinf(ofGetElapsedTimef() * 3.1415f), -1, 1, 0, 1);
            ofCircle(nosePosition, ofMap(alpha, 0, 1, 10, AppleController::appleRadius));
        }
        else if (stateMachine == WaitTouch)
        {
            ofFloatColor color;
            float ripeness = 1.0f;
            color.setHsb(ofMap(ripeness, 0, 1, 0.5f, 0), 1, 1);
            ofSetColor(color);
            float alpha = ofMap(sinf(ofGetElapsedTimef() * 3.1415f), -1, 1, 0, 1);
            ofCircle(nosePosition, ofMap(alpha, 0, 1, 10, AppleController::appleRadius));
        }
        else if (stateMachine == Gameover)
        {
            ofVec2f d = ofVec2f(0, 1);
            fluid->addTemporalForce(ofVec2f(width / 2 - 150, 330) + d * 100, d * 100, ofFloatColor(0, 0.1, 0.1), 5, 5, 20);
            fluid->addTemporalForce(ofVec2f(width / 2 + 150, 330) + d * 100, d * 100, ofFloatColor(0, 0.1, 0.1), 5, 5, 20);
        }
        else if (stateMachine == Clear)
        {
            for (int i = 0; i < 8; i++)
            {
                ofVec2f d = ofVec2f(1, 0).getRotated(i * 45);
                fluid->addTemporalForce(ofVec2f(width / 2 - 150, 330) + d * 120, d * 100, ofFloatColor(0.1, 0, 0), 3, 5, 20);
                fluid->addTemporalForce(ofVec2f(width / 2 + 150, 330) + d * 120, d * 100, ofFloatColor(0, 0.1, 0), 3, 5, 20);
            }
        }

        ofTranslate(512, 300);
        for (int i = 0; i < appleLife; i++)
        {
            ofSetColor(ofFloatColor::fromHsb(ofMap(i, 0, appleMaxLife, 0.5f, 0), 1, 1));
            ofRect(-30, -5, 60, 10);
            ofTranslate(0, -15);
        }
        ofPopStyle();
        ofPopMatrix();
    }
};

class ofApp : public ofBaseApp {
public:

    void setup();
    void setupProjector();
    void setupFluid();
    void setupApple();
    void setupParticles();
    void setupSkull();
    void update();
    void updateCursor();
    void updateFluid();
    void updateApple();
    void updateMesh(ofxOscMessage &m);
    void onParticlesUpdate(ofShader& shader);
    void draw();
    void drawSkull();
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

    ofxSecondWindow projectorWindow;

    ofxToggle toggleDebugInput;
    ofxToggle buttonMouthOpen;
    ofxIntSlider numApples;
    ofxPanel gui;

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

    vector<closestVertex> closestVertices;
    ofVec3f contactPoint;
    ofVec2f contactCoord, contactCoordPrev;
    float contactDistance;

    InputStatus inputStatus;

    ofFloatColor curColor;

#ifdef WITH_PARTICLES
    ofxGpuParticles particles;
#endif

    ofQuaternion quaternion;
    ofxCv::KalmanEuler kalman;
    ofxCv::KalmanPosition kalmanP;

    ofShader lensShader;

    string hexColor, command;

    int happy;
    int trackingId;
    int mouthOpen;

    GameController gameController;
    
    int oscBubble, oscApplause, oscDisappointed;
};

//--------------------------------------------------------------
int main() {
    ofSetupOpenGL(SURFACE_WIDTH, SURFACE_HEIGHT, OF_WINDOW);
    //ofSetupOpenGL(SURFACE_WIDTH + PROJECTOR_WIDTH, SURFACE_HEIGHT, OF_GAME_MODE);
    ofRunApp(new ofApp());
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/hdface", this, &ofApp::updateMesh);
    ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/face/rotation", quaternion);
    ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/face/happy", happy);
    ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/face/id", trackingId);
    ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/face/mouthopen", mouthOpen);
    ofxSubscribeOsc(PORT_SPEECH, "/speech/color", hexColor);
    ofxSubscribeOsc(PORT_SPEECH, "/speech/command", command);
    ofxSubscribeOsc(PORT, "/sharedface/finger", trackedTips);
    ofxRegisterPublishingOsc("localhost", PORT_PD, "/sdt/bubble", oscBubble);
    ofxRegisterPublishingOsc("localhost", PORT_PD, "/sdt/applause", oscApplause);
    ofxRegisterPublishingOsc("localhost", PORT_PD, "/sdt/disappointed", oscDisappointed);

    hexColor = "FFFF00";
    command = "";

    ofSetWindowShape(1024, 768);
    projectorWindow.setup("Projector window", SURFACE_WIDTH, 0, PROJECTOR_WIDTH, PROJECTOR_HEIGHT, true);
    projectorWindow.show();

    gui.setup(); // most of the time you don't need a name
    gui.add(toggleDebugInput.setup("Debug Input", true));
    gui.add(buttonMouthOpen.setup("Open Mouth", false));
    gui.add(numApples.setup("Number of Apples", 0, 0, 10));

	ofSetFrameRate(30);

    //ofLogToFile(ofToDataPath(ofGetTimestampString()), true);
    ofSetLogLevel(OF_LOG_VERBOSE);

	proSize.width = PROJECTOR_WIDTH;
	proSize.height = PROJECTOR_HEIGHT;

	if( XML.loadFile(ofToDataPath("calibration.xml")) ){
		ofLogNotice() << "loaded!";
	}else{
		ofLogError() << "unable to load";
	}

	meshTemplate.load(ofToDataPath("hdfaceTex.ply"));
    meshTex = meshTemplate;
    for(int i = 0; i < meshTemplate.getNumTexCoords(); i++) {
        ofVec2f v = meshTemplate.getTexCoord(i);
        v -= ofVec2f(0.5, 0.5);
        v.rotate(-87);
        v += ofVec2f(0.5, 0.5);
        v *= ofVec2f(1024, 768);
        meshTemplate.setTexCoord(i, v);
        meshTex.setVertex(i, v);
    }
    centroidTemplate = meshTemplate.getCentroid();
	mesh = meshTemplate;
    //ofEnableNormalizedTexCoords();

	renderSwitch = Apple;

    contactDistance = 100;

	fbo.allocate(1024, 768);

   	kalman.init(1e-6, 1); // invert of (smoothness, rapidness)
   	kalmanP.init(1e-6, 1); // invert of (smoothness, rapidness)

    setupProjector();
	setupFluid();
	setupParticles();
	setupSkull();
    setupApple();
}

void ofApp::setupProjector(){
    cv::Mat depthToColor = cv::Mat1d(4, 4);
    cv::Mat distCoeffs = cv::Mat1d(5, 1);
    XML.pushTag("ProjectorCameraEnsemble");
	XML.pushTag("cameras");
	ofLogError() << XML.getNumTags("Camera");
	XML.pushTag("Camera", 0);
	XML.pushTag("calibration");
	XML.pushTag("depthToColorTransform");
	XML.pushTag("ValuesByColumn");
	XML.pushTag("ArrayOfDouble", 0);
	depthToColor.at<double>(0, 0) = XML.getValue("double", 0.0, 0);
	depthToColor.at<double>(1, 0) = XML.getValue("double", 0.0, 1);
	depthToColor.at<double>(2, 0) = XML.getValue("double", 0.0, 2);
	depthToColor.at<double>(3, 0) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 1);
	depthToColor.at<double>(0, 1) = XML.getValue("double", 0.0, 0);
	depthToColor.at<double>(1, 1) = XML.getValue("double", 0.0, 1);
	depthToColor.at<double>(2, 1) = XML.getValue("double", 0.0, 2);
	depthToColor.at<double>(3, 1) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 2);
	depthToColor.at<double>(0, 2) = XML.getValue("double", 0.0, 0);
	depthToColor.at<double>(1, 2) = XML.getValue("double", 0.0, 1);
	depthToColor.at<double>(2, 2) = XML.getValue("double", 0.0, 2);
	depthToColor.at<double>(3, 2) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 3);
    depthToColor.at<double>(0, 3) = XML.getValue("double", 0.0, 0);
    depthToColor.at<double>(1, 3) = XML.getValue("double", 0.0, 1);
    depthToColor.at<double>(2, 3) = XML.getValue("double", 0.0, 2);
	depthToColor.at<double>(3, 3) = XML.getValue("double", 0.0, 3);
	XML.popTag();

	XML.popTag();
	XML.popTag();
	XML.popTag();
	XML.popTag();
	XML.popTag();
	XML.popTag();

	proIntrinsics = cv::Mat1d(3, 3);
	XML.pushTag("ProjectorCameraEnsemble");
	XML.pushTag("projectors");
	ofLogError() << XML.getNumTags("Projector");
	XML.pushTag("Projector", 0);
	XML.pushTag("cameraMatrix");
	XML.pushTag("ValuesByColumn");
	XML.pushTag("ArrayOfDouble", 0);
	proIntrinsics.at<double>(0, 0) = XML.getValue("double", 0.0, 0);
	proIntrinsics.at<double>(1, 0) = XML.getValue("double", 0.0, 1);
	proIntrinsics.at<double>(2, 0) = XML.getValue("double", 0.0, 2);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 1);
	proIntrinsics.at<double>(0, 1) = XML.getValue("double", 0.0, 0);
	proIntrinsics.at<double>(1, 1) = XML.getValue("double", 0.0, 1);
	proIntrinsics.at<double>(2, 1) = XML.getValue("double", 0.0, 2);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 2);
	proIntrinsics.at<double>(0, 2) = XML.getValue("double", 0.0, 0);
    proIntrinsics.at<double>(1, 2) = XML.getValue("double", 0.0, 1);// *1 + proSize.height;
	proIntrinsics.at<double>(2, 2) = XML.getValue("double", 0.0, 2);
	XML.popTag();
	XML.popTag();
	XML.popTag();

	proExtrinsics = cv::Mat1d(4, 4);
	XML.pushTag("pose");
	XML.pushTag("ValuesByColumn");
	XML.pushTag("ArrayOfDouble", 0);
	proExtrinsics.at<double>(0, 0) = XML.getValue("double", 0.0, 0);
	proExtrinsics.at<double>(1, 0) = XML.getValue("double", 0.0, 1);
	proExtrinsics.at<double>(2, 0) = XML.getValue("double", 0.0, 2);
	proExtrinsics.at<double>(3, 0) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 1);
	proExtrinsics.at<double>(0, 1) = XML.getValue("double", 0.0, 0);
	proExtrinsics.at<double>(1, 1) = XML.getValue("double", 0.0, 1);
	proExtrinsics.at<double>(2, 1) = XML.getValue("double", 0.0, 2);
	proExtrinsics.at<double>(3, 1) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 2);
	proExtrinsics.at<double>(0, 2) = XML.getValue("double", 0.0, 0);
	proExtrinsics.at<double>(1, 2) = XML.getValue("double", 0.0, 1);
	proExtrinsics.at<double>(2, 2) = XML.getValue("double", 0.0, 2);
	proExtrinsics.at<double>(3, 2) = XML.getValue("double", 0.0, 3);
	XML.popTag();
	XML.pushTag("ArrayOfDouble", 3);
	proExtrinsics.at<double>(0, 3) = XML.getValue("double", 0.0, 0);
	proExtrinsics.at<double>(1, 3) = XML.getValue("double", 0.0, 1);
	proExtrinsics.at<double>(2, 3) = XML.getValue("double", 0.0, 2);
	proExtrinsics.at<double>(3, 3) = XML.getValue("double", 0.0, 3);
	XML.popTag();
    XML.popTag();
    XML.popTag();

    XML.pushTag("lensDistortion");
    XML.pushTag("ValuesByColumn");
    XML.pushTag("ArrayOfDouble", 0);
    distCoeffs.at<double>(0, 0) = XML.getValue("double", 0.0, 0);
    distCoeffs.at<double>(1, 0) = XML.getValue("double", 0.0, 1);
    distCoeffs.at<double>(2, 0) = XML.getValue("double", 0.0, 2);
    distCoeffs.at<double>(3, 0) = XML.getValue("double", 0.0, 3);
    distCoeffs.at<double>(4, 0) = XML.getValue("double", 0.0, 4);
    XML.popTag();
    XML.popTag();
    XML.popTag();

    cv::Mat cameraToDepth = (cv::Mat1d(4,4) << 1, 0, 0, -69,
    0, 1, 0, -38,
    0, 0, 1, 0,
    0, 0, 0, 1);

    // set parameters for projection
	proCalibration.setup(proIntrinsics, proSize);
	proExtrinsics = cameraToDepth.t() * depthToColor.t() * proExtrinsics.t();

	ofLogVerbose() << proIntrinsics;
    ofLogVerbose() << proExtrinsics;
    ofLogVerbose() << depthToColor;

    lensShader.load("shaders/lens.vert", "shaders/lens.frag");
    lensShader.begin();
    lensShader.setUniform1f("k1", distCoeffs.at<double>(0));
    lensShader.setUniform1f("k2", distCoeffs.at<double>(1));
    lensShader.setUniform1f("p1", distCoeffs.at<double>(2));
    lensShader.setUniform1f("p2", distCoeffs.at<double>(3));
    lensShader.setUniform1f("k3", distCoeffs.at<double>(4));
    lensShader.end();

    ofLogVerbose() << distCoeffs;
}

void ofApp::setupFluid(){
#ifdef WITH_FLUID
	fluid.allocate(1024, 768, 0.25);
	fluid.dissipation = 0.95;
	fluid.velocityDissipation = 0.99;
	fluid.setGravity(ofVec2f(0.0,0.0));

	//curColor.setHsb(0, 1, 1);

	closestVertices.resize(3);
#endif
}

void ofApp::setupApple() {
#ifdef WITH_APPLE
    gameController.setup(fbo.getWidth(), fbo.getHeight());
    gameController.fluid = &fluid;
#endif
}

void ofApp::setupParticles(){
#ifdef WITH_PARTICLES
	unsigned w = meshTemplate.getNumIndices()/3;
	unsigned h = 3 * 50;
	
	particles.init(w, h, OF_PRIMITIVE_POINTS, true, 3);
	
	// initial positions
	// use new to allocate 4,000,000 floats on the heap rather than
	// the stack
	float* particlePosns = new float[w * h * 3];
	for (unsigned y = 0; y < h; ++y)
	{
		for (unsigned x = 0; x < w; ++x)
		{
			unsigned idx = y * w + x;
			int yIdx = y / 50;
			auto pStart = meshTemplate.getTexCoord(meshTemplate.getIndex(x * 3 + yIdx));
			auto pEnd = meshTemplate.getTexCoord(meshTemplate.getIndex(x * 3 + (yIdx + 1) % 3));
			auto p = pStart.interpolated(pEnd, (y % 50) * 0.02);
			particlePosns[idx * 3] = p.x + ofRandom(-2, 2);//ofMap(x, 0, w, 0, 1024); // particle x
			particlePosns[idx * 3 + 1] = p.y + ofRandom(-2, 2);//ofMap(y, 0, h, 0, 768); // particle y
			particlePosns[idx * 3 + 2] = 0.f; // particle z
		}
	}
	particles.loadDataTexture(ofxGpuParticles::POSITION, particlePosns);
	
	// initial velocities
	particles.zeroDataTexture(ofxGpuParticles::VELOCITY);
	
	particles.loadDataTexture(2, particlePosns);
	delete[] particlePosns;

	// listen for update event to set additonal update uniforms
	ofAddListener(particles.updateEvent, this, &ofApp::onParticlesUpdate);
#endif
}

void ofApp::setupSkull(){
#ifdef WITH_SKULL
    unsigned char * volumeData;
    int volWidth, volHeight, volDepth;
    ofxImageSequencePlayer imageSequence;

        imageSequence.init("volumes/head/cthead-8bit",3,".tif", 1);
    volWidth = imageSequence.getWidth();
    volHeight = imageSequence.getHeight();
    volDepth = imageSequence.getSequenceLength();

    cout << "setting up volume data buffer at " << volWidth << "x" << volHeight << "x" << volDepth <<"\n";

    volumeData = new unsigned char[volWidth*volHeight*volDepth*4];

    ofVec2f offset = 0.5 * ofVec2f(volWidth, volHeight) - ofVec2f(102, 141);

    for(int z=0; z<volDepth; z++)
    {
        imageSequence.loadFrame(z);
        for(int x=0; x<volWidth; x++)
        {
            for(int y=0; y<volHeight; y++)
            {
                // convert from greyscale to RGBA, false color
                int i4 = ((x+volWidth*(z))+(volHeight-y-1)*volWidth*volDepth)*4;

                if(x - offset.x < 0 || x - offset.x >= volWidth ||
                   y - offset.y < 0 || y - offset.y >= volHeight) {
                    volumeData[i4] = 0;
                    volumeData[i4+1] = 0;
                    volumeData[i4+2] = 0;
                    volumeData[i4+3] = 0;
                }
                else {
                    int sample = imageSequence.getPixels()[x - static_cast<int>(offset.x) + (y - static_cast<int>(offset.y)) * volWidth];
                    ofColor c;
                    c.setHsb(sample, 255-sample, sample);
                    volumeData[i4] = c.r;
                    volumeData[i4+1] = c.g;
                    volumeData[i4+2] = c.b;
                    volumeData[i4+3] = sample * 0.7;
                }
            }
        }
    }

    myVolume.setup(volWidth, volDepth, volHeight, ofVec3f(1,2,1),true);
    myVolume.updateVolumeData(volumeData,volWidth,volDepth,volHeight,0,0,0);
    myVolume.setRenderSettings(0.5, 0.5, 0.3, 0.4);
    //myVolume.setSlice(ofVec3f(0, 0.2, 0), ofVec3f(0, -1, 0));

    myVolume.setVolumeTextureFilterMode(GL_LINEAR);
#endif
}

//--------------------------------------------------------------
void ofApp::updateMesh(ofxOscMessage &m){
    ofBuffer buf = m.getArgAsBlob(0);
    mesh.clearVertices();
    centroid = ofVec3f();
    short* index = (short *)buf.getBinaryBuffer();
    while (mesh.getNumVertices() != meshTemplate.getNumVertices())
    {
        short* x = index++;
        short* y = index++;
        short* z = index++;
        ofVec3f v(-*x, -*y, -*z);
        v *= 0.001f;
        mesh.addVertex(v);
        centroid += v;
    }
    centroid *= 1.0 / mesh.getNumVertices();
}

//--------------------------------------------------------------
void ofApp::update(){
    // apply speech command
    if (command == "Paint")
    {
        renderSwitch = Fluid;
    }
    else if (command == "Apple")
    {
        renderSwitch = Apple;
    }
    else if (command == "Web")
    {
        renderSwitch = Particles;
    }
    else if (command == "Skull")
    {
        renderSwitch = Skull;
    }
    kalman.update(quaternion);
    updateCursor();

	switch(renderSwitch) {
	case Fluid:
        updateFluid();
		break;
    case Apple:
        updateApple();
        break;
    case Particles:
#ifdef WITH_PARTICLES
        particles.update();
#endif
        break;
    case Skull:
        break;
	}

	ofSetWindowTitle(ofToString(ofGetFrameRate()));

    // logging
    if (closestVertices.size() && contactDistance < 0.01)
    {
        ofLogNotice() << ofGetTimestampString() << "::" << trackingId << "::" << command << "::" << hexColor << "::" << contactCoord << "::" << closestVertices.at(0).distance() << "::" << happy;
    }
}

void ofApp::updateCursor()
{
    if (trackedTips.size() == 0) {
        closestVertices.at(0).updated = false;
        return;
    }

    auto vertices = vector<closestVertex>(closestVertices.size());
    for (int i = 0; i < mesh.getNumVertices(); i += 2) {
        float distanceSquared = trackedTips.at(0).distanceSquared(mesh.getVertex(i));
        for (int j = 0; j < vertices.size(); j += 2) {
            if (distanceSquared < vertices.at(j).distanceSquared) {
                for (int k = vertices.size() - 1; k > j; k--) {
                    vertices.at(k) = vertices.at(k - 1); // demote
                }
                vertices.at(j).index = i;
                vertices.at(j).distanceSquared = distanceSquared;
                vertices.at(j).updated = true;
                break;
            }
        }
    }
    if (vertices.at(0).updated) {
        closestVertices = vertices;
        contactPoint = ofVec3f();
        contactCoordPrev = contactCoord;
        contactCoord = ofVec2f();

        auto a0 = cv::Mat(3, 1, CV_32F, mesh.getVertex(vertices.at(0).index).getPtr());
        auto a1 = cv::Mat(3, 1, CV_32F, mesh.getVertex(vertices.at(1).index).getPtr());
        auto a2 = cv::Mat(3, 1, CV_32F, mesh.getVertex(vertices.at(2).index).getPtr());
        auto ax = cv::Mat(3, 1, CV_32F, trackedTips.at(0).getPtr());
        cv::Mat a01 = a1 - a0;
        cv::Mat a02 = a2 - a0;
        cv::Mat a0n = a01.cross(a02);
        cv::Mat a0x = ax - a0;
        a0n *= 1.0 / cv::norm(a0n);
        cv::Mat m(3, 3, CV_32F);
        a01.copyTo(m.col(0));
        a02.copyTo(m.col(1));
        a0n.copyTo(m.col(2));
        cv::Mat a0xProjected = m.inv() * a0x;

        // refine distance
        vertices.at(0).distanceSquared = a0xProjected.at<float>(2) * a0xProjected.at<float>(2);

        if (closestVertices.at(0).distanceSquared < 0.03)
            closestVertices.at(0).updated = false;

        contactDistance = vertices.at(0).distance();

        auto alpha = a0xProjected.at<float>(0);
        auto beta = a0xProjected.at<float>(1);
        cv::Mat contactPointMat = a0 + a01 * alpha + a02 * beta;
        contactPoint.x = contactPointMat.at<float>(0);
        contactPoint.y = contactPointMat.at<float>(1);
        contactPoint.z = contactPointMat.at<float>(2);
        auto & t0 = mesh.getTexCoord(vertices.at(0).index);
        auto & t1 = mesh.getTexCoord(vertices.at(1).index);
        auto & t2 = mesh.getTexCoord(vertices.at(2).index);
        contactCoord = t0 + (t1 - t0) * alpha + (t2 - t0) * beta;
    }
}

void ofApp::updateFluid()
{
    int x;
    std::stringstream ss;
    ss << std::hex << hexColor;
    ss >> x;
    curColor.setHex(x);
    if(contactDistance < 0.03)
        fluid.addTemporalForce(contactCoord, (contactCoord - contactCoordPrev) * 3, curColor * ofMap(contactDistance, 0.005, 0.03, 1, 0, true), 1.5f * 2, 20, 5);
    fluid.update();
}

void ofApp::updateApple()
{
    updateFluid();
    inputStatus.contactCoord = contactCoord;
    inputStatus.contactDistance = contactDistance;
    inputStatus.mouthOpen = mouthOpen;
    inputStatus.happy = happy;
    inputStatus.buttonMouthOpen = buttonMouthOpen;
    gameController.update(inputStatus);

    numApples = gameController.apples.size();
}

// set any update uniforms in this function
void ofApp::onParticlesUpdate(ofShader& shader)
{
	//ofVec3f mouse(mouseX,mouseY);
	ofVec3f mouse;
	if(closestVertices.size() > 0 && contactDistance < 0.05) {
		mouse = contactCoord;
	} else {
		mouse = ofVec3f(10000, 10000, 10000);
	}
	shader.setUniform3fv("mouse", mouse.getPtr());
	shader.setUniform1f("elapsed", ofGetLastFrameTime());
	shader.setUniform1f("radiusSquared", 200.f * 200.f);
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);

	fbo.begin();
	ofBackground(0);
	switch(renderSwitch) {
	case Fluid:
		fluid.draw();
		break;
    case Apple:
        drawApple();
        break;
    case Particles:
#ifdef WITH_PARTICLES
        particles.draw();
#endif
        break;
    case Skull:
        break;
	}
	fbo.end();

    ofPushStyle();
    fbo.draw(0, 0);
    meshTex.drawWireframe();
    ofPopStyle();

    gui.draw();

    // projector screen
    projectorWindow.begin();
    proCalibration.loadProjectionMatrix(0.01, 1000000.0);
    ofTranslate(viewShift);
	glMultMatrixd((GLdouble*)proExtrinsics.ptr(0, 0));

    ofBackground(0);
	ofViewport(0, 300, PROJECTOR_WIDTH, PROJECTOR_HEIGHT);
    //ofViewport(SURFACE_WIDTH + viewShift.x, viewShift.y, PROJECTOR_WIDTH, PROJECTOR_HEIGHT);

	ofSetColor(255);
	if(renderSwitch == Particles) ofSetColor(80);

	ofScale(1000, 1000, 1000);

	switch(renderSwitch) {
    case Fluid:
    case Apple:
    case Particles:
        lensShader.begin();
        lensShader.setUniformTexture("texture1", fbo.getTextureReference(), 0);
        lensShader.setUniform1f("alpha", renderSwitch == Particles ? 0.4 : 1);
        mesh.draw();
        lensShader.end();
        break;
    case Skull:
        drawSkull();
        break;
	}
    projectorWindow.end();

}

void ofApp::drawSkull()
{
#ifdef WITH_SKULL
    ofPushMatrix();

    ofTranslate(centroid);
    ofScale(0.001, 0.001, 0.001);
    ofScale(0.2, 0.2, 0.2);
    ofEnableAlphaBlending();

    ofVec3f euler = kalman.getPrediction().getEuler();
    ofRotateX(-euler.z + 180);
    ofRotateY(-euler.y);
    ofRotateZ(euler.x);

    ofTranslate(0, -80, -300);
    ofVec3f slice_p(0, 0, 0), slice_n(0, 0, -1);// = -m.getRowAsVec3f(2);
    if (closestVertices.size() && contactDistance < 0.03) {
        slice_p.z = (meshTemplate.getVertex(closestVertices.at(0).index) - centroid).z * 0.001 * 10;
    }
    kalmanP.update(slice_p);
    myVolume.setSlice(kalmanP.getPrediction(), slice_n);
    myVolume.drawVolume(SURFACE_WIDTH / 2 - 150, -100, 0, PROJECTOR_WIDTH, 0);
    ofPopMatrix();

    ofDisableAlphaBlending();
#endif
}

void ofApp::drawApple()
{
    fluid.draw();
    gameController.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch(key) {
    case 'f':
        ofToggleFullscreen();
        break;
	case '1':
		renderSwitch = Fluid;
		break;
	case '2':
		renderSwitch = Particles;
		break;
    case '3':
        renderSwitch = Skull;
        break;
    case '4':
        renderSwitch = Apple;
        break;
    case 'o':
        buttonMouthOpen = true;
        break;
    case 's':
        viewShift.z -= 1;
        break;
    case 'w':
        viewShift.z += 1;
        break;
    case OF_KEY_UP:
        viewShift.y -= 1;
        break;
    case OF_KEY_DOWN:
        viewShift.y += 1;
        break;
    case OF_KEY_LEFT:
		viewShift.x -= 1;
		break;
	case OF_KEY_RIGHT:
		viewShift.x += 1;
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    switch (key)
    {
    case 'o':
        buttonMouthOpen = false;
    }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if (toggleDebugInput)
    {
        contactDistance = 0.0001;
        contactCoordPrev = contactCoord;
        contactCoord.x = mouseX;
        contactCoord.y = mouseY;
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    if (toggleDebugInput)
    {
        contactDistance = 100;
    }
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

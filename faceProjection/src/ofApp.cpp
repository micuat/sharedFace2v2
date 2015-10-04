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
#include "ofxOscSubscriber.h"

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

#define SURFACE_WIDTH 1920
#define SURFACE_HEIGHT 1080
#define PROJECTOR_WIDTH 1024
#define PROJECTOR_HEIGHT 768

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
    bool buttonMouthOpen;
};

class AppleController
{
public:
    ofVec2f position;
    enum Type { Red, Blue };
    Type type;
    enum DieType { Eaten, Captured, Dropped };
    DieType dieType;
    int dieCount;
    const int dieMax = 30;

    AppleController() : dieCount(0) {
        float f = ofRandom(0, 2);
        if (f < 1)
        {
            type = Red;
        }
        else
        {
            type = Blue;
        }
    }

    void update()
    {
        if (isCompletelyDead()) return;

        if (dieCount > 0)
        {
            dieCount++;
            return;
        }
        position.y += 10;
    }
    void draw()
    {
        if (isCompletelyDead()) return;

        ofPushStyle();
        if (type == Red)
            ofSetColor(ofColor::red);
        else if (type == Blue)
            ofSetColor(ofColor::blue);

        if (dieCount > 0)
        {
            if (dieType == Eaten)
            {
                ofCircle(position, 50);
                ofSetColor(ofColor::black);
                ofCircle(position + ofVec2f(ofLerp(50, 0, (float)dieCount / dieMax), 0), 50);
            }
            else
            {
                ofPushMatrix();
                ofCircle(position, ofMap(dieCount, 0, dieMax, 50, 100));
                ofSetColor(ofColor::black);
                ofCircle(position, ofMap(dieCount * 2, 0, dieMax, 0, 100));
                ofPopMatrix();
            }
        }
        else
        {
            ofCircle(position, 50);
        }

        ofPopStyle();
    }

    void kill(DieType _dieType)
    {
        if(dieCount == 0)
            dieCount = 1;
        dieType = _dieType;
    }

    bool isCompletelyDead()
    {
        return dieCount == dieMax;
    }

    bool isAtLeastLightlyDead()
    {
        return dieCount > 0;
    }
};

class MouthController
{
public:
    int openness;
    MouthController() : openness(0)
    {
    }

    bool isOpen() { return openness > 25; }

    void update(InputStatus status)
    {
        if (status.mouthOpen == 2 || status.buttonMouthOpen)
        {
            openness = ofClamp(openness + 10, 0, 50);
        }
        else if (status.mouthOpen <= 0 || !status.buttonMouthOpen)
        {
            openness = ofClamp(openness - 10, 0, 50);
        }
    }

    void draw()
    {
        ofSetColor(ofColor::pink);
        int y = 526 - 20;
        y -= openness;
        ofRect(0, y, 1024, 20);
        y = 526 + 20;
        y += openness;
        ofRect(0, y, 1024, 20);
    }
};

class GameController
{
public:
    vector<AppleController> apples;
    MouthController mouthContoller;
    int appleLife;
    const int appleMaxLife = 20;
    int width, height;

    GameController() : appleLife(5) {}

    void setup(int _width, int _height)
    {
        width = _width;
        height = _height;
    }

    void update(InputStatus status)
    {
        mouthContoller.update(status);
        if (ofGetFrameNum() % 30 == 0)
        {
            AppleController a;
            a.position.x = ofRandom(300, width - 300);
            a.position.y = ofRandom(0, 100);
            apples.push_back(a);
        }

        for (int i = 0; i < apples.size(); i++)
        {
            auto& apple = apples.at(i);
            float yOld = apple.position.y;
            apple.update();
            if (apple.isAtLeastLightlyDead()) continue;

            float threshold = 75 * 75;
            bool isCrossedMouth = yOld < 526 && apple.position.y >= 526 && mouthContoller.isOpen();
            bool isCrossedChin = yOld < 600 && apple.position.y >= 600;
            bool isTouched = status.contactDistance < 0.03 && status.contactCoord.distanceSquared(apple.position) < threshold;
            if (apple.type == 0)
            {
                if (isCrossedMouth)
                {
                    apple.kill(AppleController::Eaten);
                    appleLife += 1;
                }
                else if (isTouched)
                {
                    apple.kill(AppleController::Captured);
                    appleLife -= 1;
                }
            }
            else if (apple.type == 1)
            {
                if (isTouched)
                {
                    appleLife += 1;
                    apple.kill(AppleController::Captured);
                }
                else if (isCrossedMouth)
                {
                    apple.kill(AppleController::Eaten);
                    appleLife -= 1;
                }
            }
            if (isCrossedChin)
            {
                apple.kill(AppleController::Dropped);
                appleLife -= 1;
            }
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

        mouthContoller.draw();

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

    ofxToggle toggleDebugInput;
    ofxToggle buttonMouthOpen;
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
};

//--------------------------------------------------------------
int main() {
    ofSetupOpenGL(SURFACE_WIDTH + PROJECTOR_WIDTH, SURFACE_HEIGHT, OF_WINDOW);
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

    hexColor = "FFFFFF";
    command = "";

    gui.setup(); // most of the time you don't need a name
    gui.add(toggleDebugInput.setup("Debug Input", true));
    gui.add(buttonMouthOpen.setup("Open Mouth", false));

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
	fluid.velocityDissipation = 0.95;
	fluid.setGravity(ofVec2f(0.0,0.0));

	//curColor.setHsb(0, 1, 1);

	closestVertices.resize(3);
#endif
}

void ofApp::setupApple() {
#ifdef WITH_APPLE
    gameController.setup(fbo.getWidth(), fbo.getHeight());
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
	for(int i = 0; i < buf.size() / 4 / 3; i++) {
		float* x = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 0);
		float* y = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 4);
		float* z = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 8);
		ofVec3f v(-*x, -*y, -*z);
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
    fluid.addTemporalForce(contactCoord, (contactCoordPrev - contactCoord) * 3, curColor * ofMap(contactDistance, 0.005, 0.03, 1, 0, true), 1.5f, 20, 5);
    fluid.update();
}

void ofApp::updateApple()
{
    updateFluid();
    inputStatus.contactCoord = contactCoord;
    inputStatus.contactDistance = contactDistance;
    inputStatus.mouthOpen = mouthOpen;
    inputStatus.buttonMouthOpen = buttonMouthOpen;
    gameController.update(inputStatus);
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
	proCalibration.loadProjectionMatrix(0.01, 1000000.0);
    ofTranslate(viewShift);
	glMultMatrixd((GLdouble*)proExtrinsics.ptr(0, 0));

	ofViewport(SURFACE_WIDTH, 0, PROJECTOR_WIDTH, PROJECTOR_HEIGHT);
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

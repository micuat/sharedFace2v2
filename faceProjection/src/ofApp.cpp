#include "ofApp.h"

#include "ofxOscSubscriber.h"

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

    hexColor = "FF0000";
    command = "";

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

	cout << proIntrinsics << endl;
	cout << proExtrinsics << endl;
	cout << depthToColor << endl;

    lensShader.load("shaders/lens.vert", "shaders/lens.frag");
    lensShader.begin();
    lensShader.setUniform1f("k1", distCoeffs.at<double>(0));
    lensShader.setUniform1f("k2", distCoeffs.at<double>(1));
    lensShader.setUniform1f("p1", distCoeffs.at<double>(2));
    lensShader.setUniform1f("p2", distCoeffs.at<double>(3));
    lensShader.setUniform1f("k3", distCoeffs.at<double>(4));
    lensShader.end();

    cout << distCoeffs << endl;
}

void ofApp::setupFluid(){
#ifdef WITH_FLUID
	fluid.allocate(1024, 768, 0.25);
	fluid.dissipation = 0.995;
	fluid.velocityDissipation = 0.99;
	fluid.setGravity(ofVec2f(0.0,0.0));

	//curColor.setHsb(0, 1, 1);

	closestVertices.resize(3);
#endif
}

void ofApp::setupApple() {
#ifdef WITH_APPLE
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
    //renderSwitch = Fluid;

	if(trackedTips.size() > 0) {
		auto vertices = vector<closestVertex>(closestVertices.size());
		for(int i = 0; i < mesh.getNumVertices(); i++) {
			float distanceSquared = trackedTips.at(0).distanceSquared(mesh.getVertex(i));
			for(int j = 0; j < vertices.size(); j++) {
				if(distanceSquared < vertices.at(j).distanceSquared) {
					for(int k = vertices.size() - 1; k > j; k--) {
						vertices.at(k) = vertices.at(k-1); // demote
					}
					vertices.at(j).index = i;
					vertices.at(j).distanceSquared = distanceSquared;
					vertices.at(j).updated = true;
					break;
				}
			}
		}
		if(vertices.at(0).updated) {
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

            if(closestVertices.at(0).distanceSquared < 0.03)
                closestVertices.at(0).updated = false;
			
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
            if (renderSwitch == Fluid)
            {
                int x;
                std::stringstream ss;
                ss << std::hex << hexColor;
                ss >> x;
                curColor.setHex(x);
                fluid.addTemporalForce(contactCoord, (contactCoordPrev - contactCoord) * 3, curColor * ofMap(vertices.at(0).distance(), 0.005, 0.03, 1, 0, true), 1.5f, 20, 5);
            }
		}
	} else {
        closestVertices.at(0).updated = false;
    }

	switch(renderSwitch) {
	case Fluid:
		fluid.update();
		break;
    case Apple:
        updateApple();
        break;
    case Particles:
        particles.update();
		break;
    case Skull:
        break;
	}

	ofSetWindowTitle(ofToString(ofGetFrameRate()));

    // logging
    if (closestVertices.size() && closestVertices.at(0).distanceSquared < 0.01 * 0.01)
    {
        ofLogNotice() << ofGetTimestampString() << "::" << trackingId << "::" << command << "::" << hexColor << "::" << contactCoord << "::" << closestVertices.at(0).distance() << "::" << happy;
    }
}

void ofApp::updateApple()
{
    if (ofGetFrameNum() % 30 == 0)
    {
        AnApple a;
        a.position.x = ofRandom(0, fbo.getWidth());
        a.position.y = ofRandom(0, 100);
        a.type = ofRandom(0, 2);
        apples.push_back(a);
    }

    for (int i = 0; i < apples.size(); i++)
    {
        auto& apple = apples.at(i);
        float yOld = apple.position.y;
        apple.update();

        float threshold = 50 * 50;
        if (apple.type == 0 &&
            apple.position.x > 427 && apple.position.x < 597 &&
            yOld < 526 && apple.position.y > 526)
        {
            apple.dead = true;
        }
        if (apple.type == 1 &&
            closestVertices.at(0).distanceSquared < 0.03 * 0.03 &&
            contactCoord.distanceSquared(apple.position) < threshold)
        {
            apple.dead = true;
        }
    }
}

// set any update uniforms in this function
void ofApp::onParticlesUpdate(ofShader& shader)
{
	//ofVec3f mouse(mouseX,mouseY);
	ofVec3f mouse;
	if(closestVertices.size() > 0 && closestVertices.at(0).distance() < 0.05) {
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

    /*
    ofViewport(0, 0, SURFACE_WIDTH, SURFACE_HEIGHT);
	int n = 16;
	for(int i = 0; i < n; i++) {
		ofSetColor(ofColor::fromHsb(i * 256 / n, 255, 255));
		if(i != n-1)
			ofRect(i * SURFACE_WIDTH / n, 0, SURFACE_WIDTH / n, SURFACE_HEIGHT);
		else {
			if( renderSwitch != Particles ) {
				ofPushMatrix();
				ofTranslate(i * SURFACE_WIDTH / n, 0);
				for( int j = 0; j < n; j++) {
					ofSetColor(255);
					ofLine(j * SURFACE_WIDTH / n / n, 0, j * SURFACE_WIDTH / n / n, SURFACE_HEIGHT);
				}
				ofPopMatrix();
			}
			else {
				ofMesh m;
				m.addVertex(ofVec2f(i * SURFACE_WIDTH / n, 0));
				m.addVertex(ofVec2f(n * SURFACE_WIDTH / n, 0));
				m.addVertex(ofVec2f(i * SURFACE_WIDTH / n, SURFACE_HEIGHT));
				m.addVertex(ofVec2f(n * SURFACE_WIDTH / n, 0));
				m.addVertex(ofVec2f(n * SURFACE_WIDTH / n, SURFACE_HEIGHT));
				m.addVertex(ofVec2f(i * SURFACE_WIDTH / n, SURFACE_HEIGHT));
				m.addColor(ofColor::white);
				m.addColor(ofColor::white);
				m.addColor(ofColor::black);
				m.addColor(ofColor::white);
				m.addColor(ofColor::black);
				m.addColor(ofColor::black);
				m.draw();
			}
		}
	}
    */

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
		particles.draw();
		break;
    case Skull:
        break;
	}
	fbo.end();

    ofPushStyle();
    fbo.draw(0, 0);
    meshTex.drawWireframe();
    ofSetColor(ofColor::red);
    if(closestVertices.size())
        ofCircle(contactCoord, ofMap(closestVertices.at(0).distance(), 0.005, 0.03, 10, 0, true));
    ofPopStyle();

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
        ofPushMatrix();

        ofTranslate(centroid);
        ofScale(0.001, 0.001, 0.001);
//        ofScale(0.225, 0.225, 0.225);
        ofScale(0.2, 0.2, 0.2);
        ofEnableAlphaBlending();
        
        ofVec3f euler = kalman.getPrediction().getEuler();
        ofRotateX(-euler.z + 180);
        ofRotateY(-euler.y);
        ofRotateZ(euler.x);
        //ofDrawAxis(100);

        ofTranslate(0, -80, -300);
        ofVec3f slice_p(0, 0, 0), slice_n(0, 0, -1);// = -m.getRowAsVec3f(2);
        if(closestVertices.size() && closestVertices.at(0).distance() < 0.03) {
            slice_p.z = (meshTemplate.getVertex(closestVertices.at(0).index) - centroid).z * 0.001 * 10;
        }
        kalmanP.update(slice_p);
#ifdef WITH_SKULL
        myVolume.setSlice(kalmanP.getPrediction(), slice_n);
        myVolume.drawVolume(SURFACE_WIDTH/2 - 150,-100,0, PROJECTOR_WIDTH, 0);
#endif
        ofPopMatrix();

        ofDisableAlphaBlending();
        break;
	}

}

void ofApp::drawApple()
{
    ofPushStyle();
    for (int i = 0; i < apples.size(); i++)
    {
        apples.at(i).draw();
    }

    //debug
    ofSetColor(ofColor::pink);
    if (closestVertices.size())
        ofCircle(contactCoord, ofMap(closestVertices.at(0).distance(), 0.005, 0.03, 10, 0, true));

    ofPopStyle();
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
	case 'f':
		ofToggleFullscreen();
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

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	if(x < SURFACE_WIDTH * 15.0 / 16.0) {
		//curColor.setHsb((float)x / SURFACE_WIDTH * 15.0 / 16.0, 1, 1);
	}
    fluid.addTemporalForce(ofVec2f(x, y), ofVec2f(), curColor, 1.5f, 20, 5);
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if(x < SURFACE_WIDTH * 15.0 / 16.0) {
		//curColor.setHsb((float)x / SURFACE_WIDTH * 15.0 / 16.0, 1, 1);
	}
	if(SURFACE_WIDTH * 15.0 / 16.0 < x && x < SURFACE_WIDTH) {
		if(renderSwitch == Fluid) renderSwitch = Particles;
		else renderSwitch = Fluid;
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

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

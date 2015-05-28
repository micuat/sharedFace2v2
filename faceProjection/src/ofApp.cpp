#include "ofApp.h"

#include "ofxOscSubscriber.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/hdface", this, &ofApp::updateMesh);
	ofxSubscribeOsc(PORT, "/sharedface/finger", trackedTips);

	ofSetFrameRate(60);

	proSize.width = PROJECTOR_WIDTH;
	proSize.height = PROJECTOR_HEIGHT;

	if( XML.loadFile(ofToDataPath("calibration.xml")) ){
		ofLogNotice() << "loaded!";
	}else{
		ofLogError() << "unable to load";
	}

	meshTemplate.load(ofToDataPath("hdfaceTex.ply"));
	mesh = meshTemplate;

	cv::Mat depthToColor = cv::Mat1d(4, 4);
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
	depthToColor.at<double>(0, 3) = XML.getValue("double", 0.0, 0) * 1000;
	depthToColor.at<double>(1, 3) = XML.getValue("double", 0.0, 1) * 1000;
	depthToColor.at<double>(2, 3) = XML.getValue("double", 0.0, 2) * 1000;
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
	proIntrinsics.at<double>(1, 2) = XML.getValue("double", 0.0, 1) * 1 + proSize.height;
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

	// set parameters for projection
	proCalibration.setup(proIntrinsics, proSize);
	proExtrinsics = depthToColor.t() * proExtrinsics.t();

	cout << proIntrinsics << endl;
	cout << proExtrinsics << endl;
	cout << depthToColor << endl;

	fbo.allocate(1024, 768);

	fluid.allocate(1024, 768, 0.25);
	fluid.dissipation = 0.99;
	fluid.velocityDissipation = 0.99;
	fluid.setGravity(ofVec2f(0.0,0.0));

	curColor.setHsb(0, 1, 1);

	closestVertices.resize(3);

	unsigned w = meshTemplate.getNumIndices()/3;
	unsigned h = 3 * 100;
	
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
			int yIdx = y / 100;
			auto pStart = meshTemplate.getTexCoord(meshTemplate.getIndex(x * 3 + yIdx));
			auto pEnd = meshTemplate.getTexCoord(meshTemplate.getIndex(x * 3 + (yIdx + 1) % 3));
			auto p = pStart.interpolated(pEnd, (y % 100) * 0.01);
			particlePosns[idx * 3] = p.x; // particle x
			particlePosns[idx * 3 + 1] = p.y; // particle y
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
			float denom = 0;
			for(auto it = closestVertices.begin(); it != closestVertices.end(); it++) {
				contactPoint += mesh.getVertex(it->index) / it->distanceSquared;
				contactCoord += mesh.getTexCoord(it->index) / it->distanceSquared;
				denom += 1 / it->distanceSquared;
			}
			contactPoint /= denom;
			contactCoord /= denom;

			fluid.addTemporalForce(contactCoord, contactCoordPrev - contactCoord, curColor * ofMap(vertices.at(0).distance(), 0.005, 0.03, 1, 0, true),2.0f, 20, 5);

		}
	}

	switch(renderSwitch) {
	case Fluid:
		fluid.update();
		break;
	case Particles:
		particles.update();
		break;
	}

	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

// set any update uniforms in this function
void ofApp::onParticlesUpdate(ofShader& shader)
{
	//ofVec3f mouse(mouseX,mouseY);
	ofVec3f mouse(contactCoord);
	shader.setUniform3fv("mouse", mouse.getPtr());
	shader.setUniform1f("elapsed", ofGetLastFrameTime());
	shader.setUniform1f("radiusSquared", 200.f * 200.f);
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);

	ofViewport(0, 0, SURFACE_WIDTH, SURFACE_HEIGHT);
	int n = 16;
	for(int i = 0; i < n; i++) {
		ofSetColor(ofColor::fromHsb(i * 256 / n, 255, 255));
		ofRect(i * SURFACE_WIDTH / n, 0, SURFACE_WIDTH / n, SURFACE_HEIGHT);
	}

	fbo.begin();
	ofBackground(0);
	switch(renderSwitch) {
	case Fluid:
		fluid.draw();
		break;
	case Particles:
		ofEnableBlendMode(OF_BLENDMODE_ADD);
		particles.draw();
		ofDisableBlendMode();
		break;
	}
	fbo.end();

	ofSetColor(255);
	
	proCalibration.loadProjectionMatrix(0.01, 1000000.0);
	glMultMatrixd((GLdouble*)proExtrinsics.ptr(0, 0));

	ofViewport(SURFACE_WIDTH + viewShift.x, viewShift.y, PROJECTOR_WIDTH, PROJECTOR_HEIGHT);

	ofSetColor(255);
	ofScale(1000, 1000, 1000);
	fbo.getTextureReference().bind();
	mesh.draw();
	fbo.getTextureReference().unbind();
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
	case 'f':
		ofToggleFullscreen();
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

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if(x < SURFACE_WIDTH) {
		curColor.setHsb((float)x / SURFACE_WIDTH, 1, 1);
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

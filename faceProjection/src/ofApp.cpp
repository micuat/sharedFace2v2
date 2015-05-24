#include "ofApp.h"

#include "ofxOscSubscriber.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofxSubscribeOsc(PORT_HDFACE, "/osceleton2/hdface", this, &ofApp::updateMesh);
	ofxSubscribeOsc(PORT, "/sharedface/finger", trackedTips);

	ofSetFrameRate(60);

	proSize.width = 1024;
	proSize.height = 768;

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
	fbo.begin();
	ofBackground(0);
	ofPushStyle();
	ofSetLineWidth(3);
	ofSetColor(255);
//	ofLine(512, 0, 512, 768);
	ofPopStyle();
	fbo.end();

    fluid.allocate(1024, 768, 0.25);
    fluid.dissipation = 0.99;
    fluid.velocityDissipation = 0.99;
    fluid.setGravity(ofVec2f(0.0,0.0));

	closestVertices.resize(3);
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

			fluid.addTemporalForce(contactCoord, contactCoordPrev - contactCoord, ofFloatColor(1, 0.5, 0.5) * ofMap(vertices.at(0).distance(), 0.005, 0.03, 1, 0, true),2.0f, 20, 5);

		}
	}

	fluid.update();
	
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}


//--------------------------------------------------------------
void ofApp::draw(){

	fbo.begin();
	fluid.draw();
	fbo.end();

	ofBackground(0);

	ofSetColor(255);
	
	proCalibration.loadProjectionMatrix(0.01, 1000000.0);
	glMultMatrixd((GLdouble*)proExtrinsics.ptr(0, 0));

	ofViewport(viewShift.x, viewShift.y);

	ofSetColor(255);
	//cam.begin();
	ofScale(1000, 1000, 1000);
	fbo.getTextureReference().bind();
	mesh.draw();
	fbo.getTextureReference().unbind();
	ofSetColor(50);
	mesh.drawWireframe();

//	ofSetColor(ofColor::mediumVioletRed);
//	ofCircle(contactPoint, 0.005);

//	ofSetColor(75);

//	for(int i = 0; i < trackedTips.size(); i++) {
//		ofCircle(trackedTips.at(i), 0.005);
//	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'f') {
		ofToggleFullscreen();
	}
	if (key == OF_KEY_UP) {
		viewShift.y -= 1;
	}
	if (key == OF_KEY_DOWN) {
		viewShift.y += 1;
	}
	if (key == OF_KEY_LEFT) {
		viewShift.x -= 1;
	}
	if (key == OF_KEY_RIGHT) {
		viewShift.x += 1;
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

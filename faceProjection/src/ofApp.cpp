#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	// listen on the given port
	cout << "listening for osc messages on port " << PORT << "\n";
	receiver.setup(PORT);
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofSetFrameRate(60);

	proSize.width = 1024;
	proSize.height = 768;

	if( XML.loadFile(ofToDataPath("calibration.xml")) ){
		ofLogNotice() << "loaded!";
	}else{
		ofLogError() << "unable to load";
	}

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
	
	//proIntrinsics = (cv::Mat1d(3, 3) << 1000, 0, 512, 0, 1000, 384, 0, 0, 1);
	
	cout << proIntrinsics << endl;
	cout << proExtrinsics << endl;

	// set parameters for projection
	proCalibration.setup(proIntrinsics, proSize);

	cv::Mat m = proExtrinsics;
	cv::Mat extrinsics = (cv::Mat1d(4, 4) <<
		m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2), m.at<double>(0, 3),
		m.at<double>(1, 0), m.at<double>(1, 1), m.at<double>(1, 2), m.at<double>(1, 3),
		m.at<double>(2, 0), m.at<double>(2, 1), m.at<double>(2, 2), m.at<double>(2, 3),
		0, 0, 0, 1);
	//proExtrinsics = extrinsics.t();
	proExtrinsics = proExtrinsics.t();
}

//--------------------------------------------------------------
void ofApp::update(){

	while(receiver.hasWaitingMessages()){
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(&m);

		// check for mouse moved message
		if(m.getAddress() == "/osceleton2/hdface"){
			ofBuffer buf = m.getArgAsBlob(0);
			mesh.clear();
			centroid = ofVec3f();
			for(int i = 0; i < buf.size() / 4 / 3; i++) {
				float* x = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 0);
				float* y = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 4);
				float* z = (float*)(buf.getBinaryBuffer() + i * 4 * 3 + 8);
				ofVec3f v(-*x, -*y, -*z);
				//v *= 1000;
				mesh.addVertex(v);
				centroid += v;
			}
			centroid *= 1.0 / mesh.getNumVertices();
		}
	}

	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}


//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);

	//ofSetupScreenPerspective(proSize.width, proSize.height);
	proCalibration.loadProjectionMatrix(0.01, 1000000.0);
	glMultMatrixd((GLdouble*)proExtrinsics.ptr(0, 0));
	
	ofSetColor(255);
	//cam.begin();
	ofScale(1000, 1000, 1000);
	//ofTranslate(-centroid);
	mesh.drawVertices();
	//cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'f') {
		ofToggleFullscreen();
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

#include "ofApp.h"
#include "ofxPubSubOsc.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30);

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodyIndexSource();
	kinect.initBodySource();

	irFinder.setThreshold(200);
	irFinder.setMinAreaRadius(1);
	irFinder.setMaxAreaRadius(10);
	irFinder.getTracker().setPersistence(15);
	irFinder.getTracker().setMaximumDistance(32);

	ofxPublishOsc("localhost", 57121, "/sharedface/finger", trackedTips);

	kalmanPosition.init(0.1, 0.01);

    roi = cv::Rect(128, 0, 256, 350);
}

//--------------------------------------------------------------
void ofApp::update(){
	using namespace cv;
	using namespace ofxCv;
	kinect.update();
	pixels = kinect.getDepthSource()->getPixelsRef();
	irPixels = kinect.getInfraredSource()->getPixelsRef();
	auto depthMatOrg = toCv(pixels);
	if(irPixels.isAllocated() && pixels.isAllocated()) {
        Mat depthMat(depthMatOrg, roi);

        trackedTips.clear();
        Mat m = toCv(irPixels);
        Mat mSub(m, roi);
		irFinder.findContours(mSub);
		for(int i = 0; i < irFinder.size(); i++) {
			auto bound = irFinder.getBoundingRect(i);
			Mat blob(depthMat, bound);
			float minVal = 1000, maxVal = -1000;
			ofVec2f minPoint;
			for(int y = 0; y < blob.rows; y++) {
				for(int x = 0; x < blob.cols; x++) {
					auto val = blob.at<float>(y, x);
					if(val > 0 && val < minVal) {
						minVal = val;
						minPoint.x = x;
						minPoint.y = y;
					}
					if(val > maxVal) maxVal = val;
				}
			}
			if(minVal > 0 && minVal < 1000) {
				minPoint.x += bound.x + roi.x;
				minPoint.y += bound.y + roi.y;
				DepthSpacePoint depthPoint = { 0 };
				depthPoint.X = irFinder.getCenter(i).x + roi.x;
				depthPoint.Y = irFinder.getCenter(i).y + roi.y;
				CameraSpacePoint cameraPoint = { 0 };
				UINT16 depth;
				depth = kinect.getDepthSource()->getPixels()[(int)minPoint.x + (int)minPoint.y * pixels.getWidth()];
				if(S_OK == kinect.getDepthSource()->coordinateMapper->MapDepthPointToCameraSpace(depthPoint, depth, &cameraPoint))
					trackedTips.push_back(ofVec3f(-cameraPoint.X, -cameraPoint.Y, cameraPoint.Z));

				if(i == 0) { // TODO: multiple blob tracking
					kalmanPosition.update(trackedTips.at(0));
					kalmanPosition.getPrediction();
					trackedTips.at(0) = kalmanPosition.getEstimation();
				}
			}
		}
	}

	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofPushMatrix();
	ofEnableAlphaBlending();
	ofScale(2, 2);
	ofSetColor(255);
	ofPushStyle();
	kinect.getInfraredSource()->draw(0, 0);
	ofSetColor(ofColor::red);

    ofTranslate(roi.x, roi.y);
	irFinder.draw();
	ofPopStyle();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

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

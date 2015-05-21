#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodyIndexSource();
	kinect.initBodySource();

	contourFinder.setThreshold(150);
}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();
	pixels = kinect.getDepthSource()->getPixelsRef();
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void ofApp::draw(){
	cv::Rect roi;
	float depthInterest = 0;
	bool foundBody = false;
	if(kinect.getBodySource()) {
		auto bodies = kinect.getBodySource()->getBodies();
		for(int i = 0; i < bodies.size(); i++ ) {
			auto body = bodies.at(i);
			if(body.tracked && body.joints.size()) {
				auto p = body.joints.at(JointType_Head).getProjected(kinect.getDepthSource()->coordinateMapper, ofxKFW2::ProjectionCoordinates::DepthCamera);
				p.y -= 10;
				int d = 100;
				roi.x = p.x - d;
				if(roi.x < 0) break;
				roi.y = p.y - d;
				if(roi.y < 0) break;
				roi.width = d * 2;
				if(roi.x + roi.width > pixels.getWidth()) break;
				roi.height = d * 2;
				if(roi.y + roi.height > pixels.getHeight()) break;

				depthInterest = pixels.getColor(roi.x + roi.width/2, roi.y + roi.width/2).r;

				foundBody = true;
				break;
			}
		}
	}
	if(foundBody) {
//		cv::Mat depthMat(roi.height, roi.width, CV_8U);
		cv::Mat depthMat(roi.height, roi.width, CV_32F);
		for(int i = 0; i < roi.width; i++){
			for(int j = 0; j < roi.height; j++){
				float f = pixels.getColor(i + roi.x, j + roi.y).r;
				float d = 0.0005;
//				depthMat.at<unsigned char>(j, i) = ofMap(f, depthInterest - d, depthInterest, 0, 255, true);
				depthMat.at<float>(j, i) = ofMap(f, depthInterest - d, depthInterest, 0, 1, true);
//				if(f == 0) depthMat.at<unsigned char>(j, i) = 255;
				if(f == 0) depthMat.at<float>(j, i) = 1;
			}
		}
		enum TrackState {none, leftEdge, flat, rightEdge, ending};
		ofImage labelMap;
		int labelIndex = 0;
		labelMap.allocate(roi.width, roi.height, OF_IMAGE_COLOR_ALPHA);
		cv::Mat label = cv::Mat1i(roi.width, roi.height, -1);
//		ofxCv::Canny(depthMat, img3, 100, 200);
		
		cv::blur(depthMat, depthMat, cv::Size(3, 3));
		cv::Mat xdiff, ydiff;
		cv::Sobel(depthMat, xdiff, CV_32F, 1, 0);
		cv::Sobel(depthMat, ydiff, CV_32F, 0, 1);
		for(int j = xdiff.cols - 1; j >= 0; j--){
			bool filled = false;
			TrackState trackState = none;
			int trackStart[4];

			for(int i = 0; i < xdiff.rows; i++){
				labelMap.setColor(i, j, ofColor::black);
				int &labelCur = label.at<int>(j, i);
				
				if(depthMat.at<float>(j, i) == 1 && trackState == none) {
					trackState = none;
					continue;
				}

				// update
				auto oldTrackState = trackState;
				float th = 0.3, th2 = 0.5;
				switch(trackState) {
				case none:
					if(xdiff.at<float>(j, i) < -th) {
						trackState = leftEdge;
						trackStart[(int)leftEdge] = i;
					}
					break;
				case leftEdge:
					if(abs(xdiff.at<float>(j, i)) <= th2) {
						trackState = flat;
						trackStart[(int)flat] = i;
					}
					else if(!(xdiff.at<float>(j, i) < -th2)) {
						trackState = none;
					}
					break;
				case flat:
					if(xdiff.at<float>(j, i) > th) {
						trackState = rightEdge;
						trackStart[(int)rightEdge] = i;
					}
					else if(!(abs(xdiff.at<float>(j, i)) <= th2)) {
						trackState = none;
					}
					break;
				case rightEdge:
					if(!(xdiff.at<float>(j, i) > th)) {
						trackState = ending;
					}
					break;
				}

				// render
				switch(trackState) {
				case none:
					break;
				case leftEdge:
//					labelMap.setColor(i, j, ofColor::lightBlue);
					break;
				case flat:
//					labelMap.setColor(i, j, ofColor::dimGrey);
					break;
				case rightEdge:
//					labelMap.setColor(i, j, ofColor::indianRed);
					break;
				case ending:
					if((trackStart[(int)rightEdge] - trackStart[(int)flat]) < 10) {
						// looks like a finger
						int center = (trackStart[(int)flat] + trackStart[(int)rightEdge]-1)*0.5;
						auto & labelCenter = label.at<int>(j, center);
						if(j < depthMat.cols - 3 && center > 1 && center < depthMat.rows - 2) {
							for(int ii = 0; ii < 3 && labelCenter < 0; ii++) {
								labelCenter = label.at<int>(j + ii, center);
								if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center - 1);
								if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center + 1);
								if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center - 2);
								if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center + 2);
							}
							if(labelCenter < 0) {
								labelCenter = ++labelIndex;
							}
							labelMap.setColor(center, j, ofFloatColor::fromHsb(labelCenter / 16.0, 1, 1));
						}
					}
					trackState = none;
					break;
				}
				
			}
		}

		ofPushMatrix();
		ofScale(2, 2);
		ofSetColor(255);
		labelMap.update();
		labelMap.draw(0, 0);
//		ofxCv::drawMat(depthMatFloat, 0, 0);
		ofPopMatrix();
	}

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

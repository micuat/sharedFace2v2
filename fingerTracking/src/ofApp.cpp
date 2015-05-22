#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodyIndexSource();
	kinect.initBodySource();

	tracker.setPersistence(15);
	tracker.setMaximumDistance(32);

	ofxPublishOsc("localhost", 57121, "/sharedface/finger/", trackedTips);
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
		struct LabelInfo {
			int index;
			ofVec2f begin;
			ofVec2f end;
			int count;
			int penalty;
			LabelInfo() {index = 0; count = 0; penalty = 0;};
		};
		vector<LabelInfo> labelInfos;
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
				labelMap.setColor(i, j, ofFloatColor(0));
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
					break;
				case flat:
					break;
				case rightEdge:
					break;
				case ending:
					if((trackStart[(int)rightEdge] - trackStart[(int)flat]) < 10) {
						// looks like a finger
						int center = (trackStart[(int)flat] + trackStart[(int)rightEdge]-1)*0.5;
						auto & labelCenter = label.at<int>(j, center);
						if(j < depthMat.cols - 4 && center > 2 && center < depthMat.rows - 3) {
							int ii, jj;
							for(ii = 1; ii < 5 && labelCenter < 0; ii++) {
								labelCenter = label.at<int>(j + ii, center);
								for(jj = 1; jj < 4 && labelCenter < 0; jj++) {
									if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center - jj);
									if(labelCenter < 0) labelCenter = label.at<int>(j + ii, center + jj);
								}
							}
							if(labelCenter < 0 || labelInfos.at(labelCenter).penalty > 1) {
								// new label
								labelCenter = labelIndex;
								LabelInfo li;
								li.index = labelIndex;
								li.begin.x = center;
								li.begin.y = j;
								li.end.x = center;
								li.end.y = j;
								li.count = 1;
								labelInfos.push_back(li);
								labelIndex++;
							} else {
								// update
								labelInfos.at(labelCenter).end.x = center;
								labelInfos.at(labelCenter).end.y = j;
								labelInfos.at(labelCenter).count++;
								int penalty = 0;
								// penalize jumping
								if(ii > 1 || jj > 1) penalty += 1;
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
		ofEnableAlphaBlending();
		ofScale(2, 2);
		ofSetColor(255);
		labelMap.update();
		labelMap.draw(0, 0);
		ofDisableAlphaBlending();

		ofPushStyle();
		int detectCount = 0;
		vector<cv::Point2f> tips;
		for(auto it = labelInfos.begin(); it != labelInfos.end(); it++) {
			if(it->count > 15 && detectCount < 5) {
				detectCount++;
				tips.push_back(cv::Point2f(it->end.x, it->end.y));
			}
		}
		ofPopStyle();

		tracker.track(tips);

		trackedTips.clear();
		for(auto it = tracker.getCurrentLabels().begin(); it != tracker.getCurrentLabels().end(); it++) {
			ofPoint center(tracker.getCurrent(*it).x, tracker.getCurrent(*it).y);

			DepthSpacePoint depthPoint = { 0 };
			depthPoint.X = tracker.getCurrent(*it).x + roi.x;
			depthPoint.Y = tracker.getCurrent(*it).y + roi.y;
			CameraSpacePoint cameraPoint = { 0 };
			UINT16* depth;
			float depthFloat = pixels.getColor(depthPoint.X, depthPoint.Y).r;
			depth = (UINT16*)&depthFloat;
			kinect.getDepthSource()->coordinateMapper->MapDepthPointToCameraSpace(depthPoint, *depth, &cameraPoint);

			trackedTips.push_back(ofVec3f(cameraPoint.X, cameraPoint.Y, cameraPoint.Z));
			ofPushMatrix();
			ofTranslate(center.x, center.y);
			int label = *it;
			string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
			ofDrawBitmapString(msg, 0, 0);
			ofPopMatrix();
		}

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

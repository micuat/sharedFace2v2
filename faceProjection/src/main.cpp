#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
	//ofSetupOpenGL(SURFACE_WIDTH + PROJECTOR_WIDTH, SURFACE_HEIGHT, OF_WINDOW);			// <-------- setup the GL context
	ofSetupOpenGL(SURFACE_WIDTH + PROJECTOR_WIDTH, SURFACE_HEIGHT, OF_GAME_MODE);			// <-------- setup the GL context
	// always has to be 1: surface (no magnification) 2: projector

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}

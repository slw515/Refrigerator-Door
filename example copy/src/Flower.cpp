//
//  Flower.cpp
//  example copy
//
//  Created by Steven Wyks on 10/7/19.
//

#include "Flower.hpp"
Flower::Flower(){
}

void Flower::setup(){
    x = ofRandom(0, ofGetWidth());      // give some random positioning
    y = ofRandom(0, ofGetHeight());

    speedX = ofRandom(-1, 1);           // and random speed and direction
    speedY = ofRandom(-1, 1);

    dim = 20;

    color.set(ofRandom(255),ofRandom(255),ofRandom(255)); // one way of defining digital color is by addressing its 3 components individually (Red, Green, Blue) in a value from 0-255, in this example we're setting each to a random value
}

void Flower::update(){
    if(x < 0 ){
        x = 0;
        speedX *= -1;
    } else if(x > ofGetWidth()){
        x = ofGetWidth();
        speedX *= -1;
    }

    if(y < 0 ){
        y = 0;
        speedY *= -1;
    } else if(y > ofGetHeight()){
        y = ofGetHeight();
        speedY *= -1;
    }

    x+=speedX;
    y+=speedY;
    
    frameRate++;

}

void Flower::draw(){
    ofPushMatrix();
    ofTranslate(x, y);

    // rotate canvas using frame count and mouse position
    ofRotateDeg(frameRate);
    // draw 5 petals, rotating after each one
    ofSetColor(0, 255, 50, 180); // green
    for (int i = 0; i < 6; i++) {
      ofDrawEllipse(0, -20, 15, 25);
      ofRotateDeg(60);
    }
    // centre circle
    ofSetColor(254, 255, 0); // light yellow
    ofDrawEllipse(0, 0, 20, 20);
    ofPopMatrix();
    frameRate+=0.0005;
}

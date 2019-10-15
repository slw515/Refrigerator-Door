//
//  Flower.hpp
//  example copy
//
//  Created by Steven Wyks on 10/7/19.
//

#ifndef _FLOWER // if this class hasn't been defined, the program can define it
#define _FLOWER // by using this if statement you prevent the class to be called more than once which would confuse the compiler
#include "ofMain.h" // we need to include this to have a reference to the openFrameworks framework
class Flower {
public:
    // methods, equivalent to specific functions of your class objects
    void setup();    // setup method, use this to setup your object's initial state
    void update();  // update method, used to refresh your objects properties
    void draw();    // draw method, this where you'll do the object's drawing

    // variables
    float x;        // position
    float y;
    float speedY;   // speed and direction
    float speedX;
    int dim;
    int frameRate;
    // size
    ofColor color;  // color using ofColor type

    Flower();  // constructor - used to initialize an object, if no properties are passed the program sets them to the default value
}; // don't forget the semicolon!
#endif

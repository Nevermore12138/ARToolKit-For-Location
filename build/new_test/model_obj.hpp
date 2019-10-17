#ifndef Model_OBJ_hpp
#define Model_OBJ_hpp

// Parameters for the Virtual Object drawing
#define POINTS_PER_VERTEX 3
#define TOTAL_FLOATS_IN_TRIANGLE 9

#include <stdio.h>
#include <string>
#include <cmath>
#include <fstream>
// #include "GL/glut.h"
#include <GL/glut.h>
#include <iostream>

using namespace std;


class model_obj
{
	public:
  		model_obj();
  		int TotalConnectedPoints;				// Stores the total number of connected verteces
    	int TotalConnectedTriangles;			// Stores the total number of connected triangles
    	float* calculateNormal(float* coord1, float* coord2, float* coord3);
	    int Load(const char *filename);	// Loads the model
	    void Draw();					// Draws the model on the screen
	    //void Model_OBJ::Release();				// Release the model
	    
	    float* normals;							// Stores the normals
	    float* Faces_Triangles;					// Stores the triangles
	    float* vertexBuffer;					// Stores the points which make the object
};

#endif /* Model_OBJ_hpp */
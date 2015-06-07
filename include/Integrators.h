#pragma once

#include "cinder\Vector.h"
#include "cinder\gl\gl.h"
#include <vector>
#include <algorithm>
#include "cinder\Matrix.h"


struct State {
	/* constant quantities */
	ci::Matrix33f mInitI;
	ci::Matrix33f mInitInvI;
	
    /* State variables */
    ci::Vec3f mCOM;
	ci::Matrix33f mR;
    ci::Vec3f mP;
	ci::Vec3f mL;

	/* Derived quantities */
	ci::Matrix33f mI;
	ci::Matrix33f mInvI;
};


struct Derivative {
	ci::Vec3f V;
	ci::Matrix33f dOmegaR;
	ci::Vec3f force;
	ci::Vec3f torque;
};


inline ci::Matrix33f Star(const ci::Vec3f &omega) {
	return ci::Matrix33f(0.0f, -omega.z, omega.y,
						 omega.z, 0.0f, -omega.x,
						 -omega.y, omega.x, 0.0f);
}


/* Forward declare Object class */
class Bird;

Derivative evaluate(Bird *boid, const State &oldState, float dTime, const Derivative &dFunc);

void integrateByEulerMethod(Bird *boid, State &state, float dTime, const Derivative &dFunc);
void integrateByRungeKutta4(Bird *boid, State &state, float dTime, const Derivative &dFunc);

typedef void (*IntegratorType)(Bird *boid, State &state, float dTime, const Derivative &dFunc);


struct Scene_Properties {
	GLfloat gravity;
    ci::Vec3f g;

	ci::Vec3f wind;
	GLfloat windForce;
	GLfloat rho;

    GLfloat dT;
	IntegratorType integrate;

	float level;
};
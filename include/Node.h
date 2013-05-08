#pragma once

#include "Mesh.h"
#include "Integrators.h"
#include "cinder\Matrix.h"
#include "cinder\Vector.h"
#include <vector>

const float PI		=	3.141592653589793f;
const float dWING	=	0.02617993875f;
const float dAOA	=	0.0174532925f;

class Node;
ci::Matrix33f outerProduct(const ci::Vec3f &v);

class Link {
public:
	Node *nodePtr;
	ci::Matrix44f Lmatrix;
	ci::Matrix44f Amatrix;
	Link *sibling;
};

class Node
{
public:
	Mesh shape;
	ci::Matrix44<float> Tmatrix;
	std::vector<Link*> childs;
};

typedef std::vector<Link*>::iterator vArcIter;
typedef std::vector<Link*>::const_iterator CvArcIter;
typedef std::vector<ci::Vec3f>::iterator Vec3fIter;
typedef std::vector<ci::Vec3f>::const_iterator CVec3fIter;

class Bird {
private:
	std::vector<ci::Vec3f> transformData(const ci::Matrix44<float> &matrix, const std::vector<ci::Vec3f> &vertices);
	void draw(const std::vector<ci::Vec3f> &newVertices, const Mesh &shape, bool lighting);
public:
	Bird(const ci::Vec3f &linearVelocity, const ci::Vec3f &AngularVelocity, Scene_Properties *prop){
		this->prop = prop;		
		M = (2*WING_WT + BODY_WT);
		root = new Link();
		loadGeometry(root);

		// Bird COM and MOI are set inside this function
		compute_COM_Inertia();
		mState.mR = ci::Matrix33f::identity();
		mState.mP = (M*linearVelocity);
		mState.mL = (mState.mInitI * AngularVelocity);
		mState.mI = mState.mInitI;
		mState.mInvI = mState.mI.inverted();

		// Compute wing area based on the span
		Mesh *wing = &(root->nodePtr->childs[0]->nodePtr->shape);
		ci::Vec3f dimsW = wing->extents[1] - wing->extents[0];
		WingArea = dimsW.x * dimsW.z;

		// Articulation Parameters
		swing = 0.0f;
		AOA = 15.0f;
		LWing_Norm.set(0.0f,1.0f,0.0f);
		RWing_Norm.set(0.0f,1.0f,0.0f);
		upThsld = 45.0f;
		downThsld = -45.0f;
		atkUThlsd = 5.0f;
		atkDThlsd = -10.0f;
		t = 0.5;
		upstroke = true;
		gnrt_Swing_AOA();
	}
	void loadGeometry(Link *arc);
	void updateMatrices();
	void drawGeometry(bool lighting);
	void drawGeometry(const Link *arc, ci::Matrix44<float> matrix, bool lighting);
	ci::Vec3f getCenter() { return mState.mCOM; }
	ci::Vec3f getDirectionOfMovement() {
		ci::Vec3f dir(mState.mP/M);
		dir.normalize();
		return dir;
	}
	ci::Vec3f getLeftWingFaceNormal();
	ci::Vec3f getRightWingFaceNormal();

	// State update functions
	void compute_COM_Inertia();
	void gnrt_Swing_AOA();
	Derivative accelerate(const State &st);
	void accelerate(Derivative &delta);
	void updateState(const State &st);

	//Attributes
	Link *root;
	State mState;
	float M;
	float WingArea;
	Scene_Properties *prop;
	// Articulation parameters
	float t;
	bool upstroke;
	float swing, AOA;	// radians
	ci::Vec3f LWing_Norm;
	ci::Vec3f RWing_Norm;
	float upThsld;
	float downThsld;
	float atkUThlsd;
	float atkDThlsd;
};

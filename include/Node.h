#pragma once

#include "Mesh.h"
#include "Integrators.h"
#include "cinder\Matrix.h"
#include "cinder\Vector.h"
#include <vector>

const float PI		=	3.141592653589793f;
const float dWING	=	0.02617993875f;
const float dAOA	=	0.0174532925f;

struct Node;

ci::Matrix33f outerProduct(const ci::Vec3f &v);

struct Link {
	Node *nodePtr;
	ci::Matrix44f Lmatrix;
	ci::Matrix44f Amatrix;
	Link *sibling;
};

struct Node {
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
    /* Attributes */
    Link*       mObjectRoot;
    State       mState;
    float       mMass;
    float       mWingArea;
    Scene_Properties *mProp;

    /* Articulation parameters */
    float       mCurrentTime;
    bool        mUpStroke;
    float       mSwing, mAOA;	/* radians */
    ci::Vec3f   mLeftWingNorm;
    ci::Vec3f   mRightWingNorm;
    float       mUpThreshold;
    float       mDownThreshold;
    float       mATKUpThreshold;
    float       mATKDownThreshold;

    void loadGeometry(Link *arc);
    void compute_COM_Inertia();
    void updateMatrices();
	std::vector<ci::Vec3f> transformData(const ci::Matrix44<float> &matrix, const std::vector<ci::Vec3f> &vertices);
    void draw(const std::vector<ci::Vec3f> &newVertices, const Mesh &shape, bool lighting);
    void drawGeometry(const Link *arc, ci::Matrix44<float> matrix, bool lighting);

public:
    Bird(const ci::Vec3f &linearVelocity, const ci::Vec3f &AngularVelocity, Scene_Properties *mProp);

    const ci::Vec3f& getCenter() const;
    const State& getState() const;

    ci::Vec3f getDirectionOfMovement();
	ci::Vec3f getLeftWingFaceNormal();
	ci::Vec3f getRightWingFaceNormal();

	/* State update functions */
    Derivative accelerate(const State &st);
	void accelerate(Derivative &delta);
    void updateState(const State &st);
    void gnrt_Swing_AOA();

    /* rendering functions */
    void drawGeometry(bool lighting);
};

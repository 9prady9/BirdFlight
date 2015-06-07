#pragma once

#include "cinder\Matrix.h"
#include "cinder\Vector.h"
#include "cinder\Ray.h"
#include <vector>

/* Weight in Kilograms */
const int WING_WT = 1;
const int BODY_WT = 2;

typedef ci::Vec3<unsigned int> Vec3ui;

enum BODY_PART { BODY, WING };

class Mesh {
private:
    /* Attributes */
    ci::Vec3f               mCOM;
    ci::Matrix33f           mInitI;
    std::vector<ci::Vec3f>  mVertices;
    std::vector<Vec3ui>     mFaceIndices;
    std::vector<ci::Vec3f>  mFaceNormals;
    
    /* AABB (Axis Aligned Bounding Box) attributes */
    ci::Vec3f               mExtents[2];
    ci::Vec3f               mCorners[8];

    /* Volume mPoints */
    BODY_PART               mPartType;
    float                   mPointMass;
    ci::Matrix33f           mInitR;
    std::vector<ci::Vec3f>  mInteriorPoints;

    /* 
     * Mesh Class Helper Member Functions 
     */
    /* Below helper is used to compute mesh face mNormals */
    void computeFaceNormals();

    /* Below helper is used to compute mesh's 
       Axis Aligned Bounding Box */
    ci::Matrix33f computeAABB();

    void generateInsiders();

    /* Below helper is used by computerInertia compute
       the object's Center of Mass */
    ci::Vec3f computeCOM();

    /* Below helper is used by computeInertia function
      to compute the object's intial moment of inertia */
    ci::Matrix33f computeInitialInertia();

    /* Below helper is used by loadMesh */
    void computeInertia();

    /* Below helper is used to findIntersection if given
    face(index) contains the point Ph */
    bool insideTriangle(const ci::Vec3f &Ph, unsigned int index);

public:
	bool loadMesh(const char* filename, BODY_PART partType);
	std::vector<float> findIntersection(const ci::Ray &ry);

    const int getNumTriangles() const;
    const std::vector<Vec3ui>& getIndices() const;
    const std::vector<ci::Vec3f>& getVertices() const;
    const std::vector<ci::Vec3f>& getNormals() const;
    const ci::Vec3f& extent(const unsigned index) const;
    const BODY_PART& bodyPart() const;
    const ci::Vec3f& COM() const;
    const ci::Matrix33f& initialInteria() const;

};

typedef std::vector<ci::Vec3f>::iterator Vec3fIter;
typedef std::vector<ci::Vec3f>::const_iterator CVec3fIter;
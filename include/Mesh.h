#pragma once

#include "cinder\Matrix.h"
#include "cinder\Vector.h"
#include "cinder\Ray.h"
#include <vector>

// Weight in Kilograms
const int WING_WT = 1;
const int BODY_WT = 2;

typedef ci::Vec3<unsigned int> Vec3ui;
enum BODY_PART { BODY, WING };

class Mesh{
public:
	bool loadMesh(const char* filename, BODY_PART partType);
	void computeFaceNormals();
	ci::Matrix33f computeAABB();
	bool insideTriangle(const ci::Vec3f &Ph, unsigned int index);
	std::vector<float> findIntersection(const ci::Ray &ry);
	void generateInsiders();
	ci::Vec3f computeCOM();
	ci::Matrix33f computeInitialInertia();
	void computeInertia();

	const int getNumTriangles() const { return faceIndices.size(); }
	const std::vector<Vec3ui>& getIndices() const { return faceIndices; } 
	const std::vector<ci::Vec3f>& getVertices() const { return myVertices; }
	const std::vector<ci::Vec3f>& getNormals() const { return faceNormals; }

	// Attributes
	ci::Vec3f COM;
	ci::Matrix33f mInitI;
	std::vector<ci::Vec3f> myVertices;
    std::vector<Vec3ui> faceIndices;
    std::vector<ci::Vec3f> faceNormals;
	// AABB attributes
	ci::Vec3f extents[2];
	ci::Vec3f corners[8];
	// Volume points
	BODY_PART part;
	float pointMass;
	ci::Matrix33f initR;
	std::vector<ci::Vec3f> insiders;
};

typedef std::vector<ci::Vec3f>::iterator Vec3fIter;
typedef std::vector<ci::Vec3f>::const_iterator CVec3fIter;
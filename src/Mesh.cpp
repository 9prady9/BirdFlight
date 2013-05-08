#include "Mesh.h"
#include "cinder\Vector.h"
#include "cinder\app\AppBasic.h"
#include <sstream>
#include <fstream>
#include <vector>

using namespace std;
using namespace ci;

bool Mesh::loadMesh(const char* filename, BODY_PART partType)
{
	this->part = partType;
	char type;
    string inputLine;
    ifstream ObjFile(filename,ios::in);

    while(getline(ObjFile,inputLine))
    {
        stringstream ss(inputLine);
        ss>>type;
        switch(type) {
        case 'v': {
            float x,y,z; ss>>x>>y>>z;
            myVertices.push_back(Vec3f(x,y,z));
        }
            break;
        case 'f': {
            unsigned int indices[3];
            ss>>indices[0]>>indices[1]>>indices[2];
			indices[0]--; indices[1]--; indices[2]--;
            faceIndices.push_back(Vec3ui(indices[0],indices[1],indices[2]));
        }
            break;
        case '#':		// Do nothing
            break;
        case ' ':		// Do nothing
            break;
        default:		// Do nothing
            break;
        }
		type='#';
    }
	ObjFile.close();
	computeFaceNormals();
	computeInertia();
	app::console()<<"AABB's dimensions : "<<(extents[1] - extents[0])<<std::endl;
	return true;
}

void Mesh::computeFaceNormals()
{
	for( vector<Vec3ui>::iterator face = faceIndices.begin(); face != faceIndices.end(); face++ )
    {
        Vec3ui indices = *face;
        Vec3f norm = cross(myVertices[indices[1]]-myVertices[indices[0]], myVertices[indices[2]]-myVertices[indices[0]]);
		norm.normalize();
        faceNormals.push_back(norm);
    }
}

ci::Matrix33f Mesh::computeAABB()
{
	extents[0].set(10000.0f,10000.0f,10000.0f);
	extents[1].set(-10000.0f,-10000.0f,-10000.0f);
	for(Vec3fIter it = myVertices.begin(); it!= myVertices.end(); it++) {
		Vec3f currV = *it;
		// Min. extraction
		extents[0][0] = ( currV[0] < extents[0][0] ? currV[0] : extents[0][0]);
		extents[0][1] = ( currV[1] < extents[0][1] ? currV[1] : extents[0][1]);
		extents[0][2] = ( currV[2] < extents[0][2] ? currV[2] : extents[0][2]);
		// Max. extraction
		extents[1][0] = ( currV[0] > extents[1][0] ? currV[0] : extents[1][0]);
		extents[1][1] = ( currV[1] > extents[1][1] ? currV[1] : extents[1][1]);
		extents[1][2] = ( currV[2] > extents[1][2] ? currV[2] : extents[1][2]);
	}
	extents[0] -= Vec3f(EPSILON,EPSILON,EPSILON);
	extents[1] += Vec3f(EPSILON,EPSILON,EPSILON);
	corners[0] = extents[0];
	corners[1].set(extents[1][0],extents[0][1],extents[0][2]);
	corners[2].set(extents[1][0],extents[1][1],extents[0][2]);
	corners[3].set(extents[0][0],extents[1][1],extents[0][2]);
	corners[4].set(extents[0][0],extents[1][1],extents[1][2]);
	corners[5].set(extents[0][0],extents[0][1],extents[1][2]);
	corners[6].set(extents[1][0],extents[0][1],extents[1][2]);
	corners[7] = extents[1];
	Vec3f X = (corners[1]-corners[0]).normalized();
	X.normalize();
	Vec3f Y = (corners[3]-corners[0]).normalized();
	Y.normalize();
	Vec3f Z = (corners[5]-corners[0]).normalized();
	Z.normalize();
	return Matrix33f(X,Y,Z);
}

bool Mesh::insideTriangle(const Vec3f &Ph, unsigned int index)
{
    float a,b,c;
    float mu,mv;
    float ut,vt,wt;

    Vec3f A = cross( myVertices[faceIndices[index].y]-myVertices[faceIndices[index].x],
					 myVertices[faceIndices[index].z]-myVertices[faceIndices[index].x] );
	Vec3f A1 = cross( myVertices[faceIndices[index].y]-Ph, myVertices[faceIndices[index].z]-Ph );
	Vec3f A2 = cross( myVertices[faceIndices[index].z]-Ph, myVertices[faceIndices[index].x]-Ph );

    /* u computation */
    a = (A1.x < 0.0f) ? A1.x*-1.0f : A1.x;
    b = (A1.y < 0.0f) ? A1.y*-1.0f : A1.y;
    c = (A1.z < 0.0f) ? A1.z*-1.0f : A1.z;
    mu = max(a,max(b,c));
    if( a == mu) ut = A1.x/A.x;
    else if(b == mu) ut = A1.y/A.y;
    else ut = A1.z/A.z;
    /* check for a miss to return preempt uneccessary calculations */
    if(ut<0 || ut>1) return false;
    /* v computation */
    a = (A2.x < 0.0f) ? A2.x*-1.0f : A2.x;
    b = (A2.y < 0.0f) ? A2.y*-1.0f : A2.y;
    c = (A2.z < 0.0f) ? A2.z*-1.0f : A2.z;
    mv = max(a,max(b,c));
    if( a == mv) vt = A2.x/A.x;
    else if(b == mv) vt = A2.y/A.y;
    else vt = A2.z/A.z;
    /* check for a miss to return preempt uneccessary calculations */
    if(vt<0 || vt>1) return false;
    /* w computation */
    wt = 1 - ut -vt;
    if( 0 <= wt && wt <= 1 )
		return true;
	else
		return false;
}

std::vector<float> Mesh::findIntersection(const ci::Ray &ry)
{
	vector<float> ret_val;
    for(unsigned int i=0; i<faceIndices.size(); i++)
    {
		float tmp = dot(faceNormals[i],ry.getDirection());
		if( tmp > 1.0e-6 || tmp < -1.0e-6 ) {
			float t = dot(faceNormals[i], myVertices[faceIndices[i].x]-ry.getOrigin()) / tmp;
			Vec3f Ph = ry.getOrigin() + t*ry.getDirection();
			if( insideTriangle( Ph, i) )
				ret_val.push_back(t);
		}
    }
    return ret_val;
}

void Mesh::generateInsiders()
{
	Vec3f X = initR.getColumn(0);
	Vec3f Y = initR.getColumn(1);
	Vec3f Z = initR.getColumn(2);
	float xLen = (corners[1]-corners[0]).length();
	float yLen = (corners[3]-corners[0]).length();
	float zLen = (corners[5]-corners[0]).length();
	int xGrids = ( xLen/2.0f > 1.0f ? (int)(xLen/2.0) : 4);
	int yGrids = ( yLen/2.0f > 1.0f ? (int)(yLen/2.0) : 4);
	int zGrids = ( zLen/2.0f > 1.0f ? (int)(zLen/2.0) : 4);
	float xStep = xLen/xGrids;
	float yStep = yLen/yGrids;
	float zStep = zLen/zGrids;

	for( int j=1; j<=yGrids; j++ )
	{
		for( int k=1; k<=zGrids; k++ )
		{
			Vec3f src = corners[0] + j*yStep*Y + k*zStep*Z;
			Ray currRay(src,X);
			vector<float> tS = this->findIntersection(currRay);
			for( vector<float>::iterator tf = tS.begin(); tf != tS.end(); tf=tf+2 ) {
				float t = *tf;
				if( tf+1 != tS.end() ) {
					float tNxt = *(tf+1);
					Vec3f Pt = currRay.getOrigin() + t * currRay.getDirection() - corners[0];
					Vec3f PtNxt = currRay.getOrigin() + tNxt * currRay.getDirection() - corners[0];
					int txInd = (int)(Pt.x/xStep);
					int txNxtInd = (int)(PtNxt.x/xStep);
					if( txInd < txNxtInd ) {
						for( int i = txInd+1; i<=txNxtInd; i++ )
							insiders.push_back(src+i*xStep*X);
					} else {
						for( int i = txNxtInd+1; i<=txInd; i++ )
							insiders.push_back(src+i*xStep*X);
					}
				} else
					break;
			}
		}
	}
}

ci::Vec3f Mesh::computeCOM()
{
	int pointCount = myVertices.size() + insiders.size();
	float Mi;
	if( this->part == BODY_PART::BODY ) {
		this->pointMass = BODY_WT;
		this->pointMass /= (float)pointCount;
	}
	if( this->part == BODY_PART::WING ) {
		this->pointMass = WING_WT;
		this->pointMass /= (float)pointCount;
	}
	Matrix33f I(Matrix33f::zero());

	Mi = this->pointMass;
	Vec3f acc(Vec3f::zero());
	for( Vec3fIter iter = myVertices.begin(); iter != myVertices.end(); iter++ )
		acc += (Mi*(*iter));
	for( Vec3fIter inIter = insiders.begin(); inIter != insiders.end(); inIter++ )
		acc += (Mi*(*inIter));
	acc /= (pointCount * this->pointMass);

	// Update Mesh coordinates; insiders such the COM is center of local frame of reference
	for( Vec3fIter v = myVertices.begin(); v != myVertices.end(); v++ )
		(*v) -= acc;
	for( Vec3fIter in = insiders.begin(); in != insiders.end(); in++ )
		(*in) -= acc;
	// update face normals as well
	computeFaceNormals();

	return acc;
}

ci::Matrix33f Mesh::computeInitialInertia()
{
	Matrix33f I(Matrix33f::zero());
	for( Vec3fIter in = insiders.begin(); in != insiders.end(); in++ )
	{
		Vec3f ri = *in;
		I.at(0,0) += ri.y*ri.y + ri.z * ri.z;
		I.at(0,1) += -ri.x * ri.y;
		I.at(0,2) += -ri.x * ri.z;
		I.at(1,0) += -ri.y * ri.x;
		I.at(1,1) += ri.x*ri.x + ri.z * ri.z;
		I.at(1,2) += -ri.y * ri.z;
		I.at(2,0) += -ri.x * ri.z;
		I.at(2,1) += -ri.y * ri.z;
		I.at(2,2) += ri.x*ri.x + ri.y * ri.y;
	}
	for( Vec3fIter it = myVertices.begin(); it != myVertices.end(); it++ )
	{
		Vec3f ri = *it;
		I.at(0,0) += ri.y*ri.y + ri.z * ri.z;
		I.at(0,1) += -ri.x * ri.y;
		I.at(0,2) += -ri.x * ri.z;
		I.at(1,0) += -ri.y * ri.x;
		I.at(1,1) += ri.x*ri.x + ri.z * ri.z;
		I.at(1,2) += -ri.y * ri.z;
		I.at(2,0) += -ri.x * ri.z;
		I.at(2,1) += -ri.y * ri.z;
		I.at(2,2) += ri.x*ri.x + ri.y * ri.y;
	}
	Matrix33f massScale(Matrix33f::createScale(pointMass));
	I*=massScale;
	return I;
}

void Mesh::computeInertia()
{	
	initR = computeAABB();
	generateInsiders();
	COM = computeCOM();
	mInitI = computeInitialInertia();
	app::console()<<"COM : "<<COM<<std::endl;
	app::console()<<"Inertia : "<<std::endl<<mInitI<<std::endl;
}
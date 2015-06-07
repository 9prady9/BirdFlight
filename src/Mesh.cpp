#include "Mesh.h"
#include "cinder\Vector.h"
#include "cinder\app\AppBasic.h"
#include <sstream>
#include <fstream>
#include <vector>

using namespace std;
using namespace ci;

void Mesh::computeFaceNormals()
{
    for (vector<Vec3ui>::iterator face = mFaceIndices.begin(); face != mFaceIndices.end(); ++face) {
        Vec3ui indices = *face;
        Vec3f norm = cross(mVertices[indices[1]] - mVertices[indices[0]], mVertices[indices[2]] - mVertices[indices[0]]);
        norm.normalize();
        mFaceNormals.push_back(norm);
    }
}

ci::Matrix33f Mesh::computeAABB()
{
    mExtents[0].set(10000.0f, 10000.0f, 10000.0f);
    mExtents[1].set(-10000.0f, -10000.0f, -10000.0f);
    for (Vec3fIter it = mVertices.begin(); it != mVertices.end(); it++) {
        Vec3f currV = *it;
        // Min. extraction
        mExtents[0][0] = (currV[0] < mExtents[0][0] ? currV[0] : mExtents[0][0]);
        mExtents[0][1] = (currV[1] < mExtents[0][1] ? currV[1] : mExtents[0][1]);
        mExtents[0][2] = (currV[2] < mExtents[0][2] ? currV[2] : mExtents[0][2]);
        // Max. extraction
        mExtents[1][0] = (currV[0] > mExtents[1][0] ? currV[0] : mExtents[1][0]);
        mExtents[1][1] = (currV[1] > mExtents[1][1] ? currV[1] : mExtents[1][1]);
        mExtents[1][2] = (currV[2] > mExtents[1][2] ? currV[2] : mExtents[1][2]);
    }
    mExtents[0] -= Vec3f((float)EPSILON, (float)EPSILON, (float)EPSILON);
    mExtents[1] += Vec3f((float)EPSILON, (float)EPSILON, (float)EPSILON);
    mCorners[0] = mExtents[0];
    mCorners[1].set(mExtents[1][0], mExtents[0][1], mExtents[0][2]);
    mCorners[2].set(mExtents[1][0], mExtents[1][1], mExtents[0][2]);
    mCorners[3].set(mExtents[0][0], mExtents[1][1], mExtents[0][2]);
    mCorners[4].set(mExtents[0][0], mExtents[1][1], mExtents[1][2]);
    mCorners[5].set(mExtents[0][0], mExtents[0][1], mExtents[1][2]);
    mCorners[6].set(mExtents[1][0], mExtents[0][1], mExtents[1][2]);
    mCorners[7] = mExtents[1];
    Vec3f X = (mCorners[1] - mCorners[0]).normalized();
    X.normalize();
    Vec3f Y = (mCorners[3] - mCorners[0]).normalized();
    Y.normalize();
    Vec3f Z = (mCorners[5] - mCorners[0]).normalized();
    Z.normalize();
    return Matrix33f(X, Y, Z);
}

void Mesh::generateInsiders()
{
    Vec3f X = mInitR.getColumn(0);
    Vec3f Y = mInitR.getColumn(1);
    Vec3f Z = mInitR.getColumn(2);
    float xLen = (mCorners[1] - mCorners[0]).length();
    float yLen = (mCorners[3] - mCorners[0]).length();
    float zLen = (mCorners[5] - mCorners[0]).length();
    int xGrids = (xLen / 2.0f > 1.0f ? (int)(xLen / 2.0) : 4);
    int yGrids = (yLen / 2.0f > 1.0f ? (int)(yLen / 2.0) : 4);
    int zGrids = (zLen / 2.0f > 1.0f ? (int)(zLen / 2.0) : 4);
    float xStep = xLen / xGrids;
    float yStep = yLen / yGrids;
    float zStep = zLen / zGrids;

    for (int j = 1; j <= yGrids; j++)
    {
        for (int k = 1; k <= zGrids; k++)
        {
            Vec3f src = mCorners[0] + j*yStep*Y + k*zStep*Z;
            Ray currRay(src, X);
            vector<float> tS = this->findIntersection(currRay);
            for (vector<float>::iterator tf = tS.begin(); tf != tS.end(); tf = tf + 2) {
                float mCurrentTime = *tf;
                if (tf + 1 != tS.end()) {
                    float tNxt = *(tf + 1);
                    Vec3f Pt = currRay.getOrigin() + mCurrentTime * currRay.getDirection() - mCorners[0];
                    Vec3f PtNxt = currRay.getOrigin() + tNxt * currRay.getDirection() - mCorners[0];
                    int txInd = (int)(Pt.x / xStep);
                    int txNxtInd = (int)(PtNxt.x / xStep);
                    if (txInd < txNxtInd) {
                        for (int i = txInd + 1; i <= txNxtInd; i++)
                            mInteriorPoints.push_back(src + i*xStep*X);
                    }
                    else {
                        for (int i = txNxtInd + 1; i <= txInd; i++)
                            mInteriorPoints.push_back(src + i*xStep*X);
                    }
                }
                else
                    break;
            }
        }
    }
}

ci::Vec3f Mesh::computeCOM()
{
    int pointCount = mVertices.size() + mInteriorPoints.size();
    float Mi;
    if (this->mPartType == BODY_PART::BODY) {
        this->mPointMass = BODY_WT;
        this->mPointMass /= (float)pointCount;
    }
    if (this->mPartType == BODY_PART::WING) {
        this->mPointMass = WING_WT;
        this->mPointMass /= (float)pointCount;
    }
    Matrix33f I(Matrix33f::zero());

    Mi = this->mPointMass;
    Vec3f acc(Vec3f::zero());
    for (Vec3fIter iter = mVertices.begin(); iter != mVertices.end(); iter++)
        acc += (Mi*(*iter));
    for (Vec3fIter inIter = mInteriorPoints.begin(); inIter != mInteriorPoints.end(); inIter++)
        acc += (Mi*(*inIter));
    acc /= (pointCount * this->mPointMass);

    /* Update Mesh coordinates; mInteriorPoints such the
       mCOM is center of local frame of reference */
    for (Vec3fIter v = mVertices.begin(); v != mVertices.end(); v++)
        (*v) -= acc;
    for (Vec3fIter in = mInteriorPoints.begin(); in != mInteriorPoints.end(); in++)
        (*in) -= acc;
    /* update face mNormals as well */
    computeFaceNormals();

    return acc;
}

ci::Matrix33f Mesh::computeInitialInertia()
{
    Matrix33f I(Matrix33f::zero());

    for (Vec3fIter in = mInteriorPoints.begin(); in != mInteriorPoints.end(); in++) {
        Vec3f ri = *in;
        I.at(0, 0) += ri.y*ri.y + ri.z * ri.z;
        I.at(0, 1) += -ri.x * ri.y;
        I.at(0, 2) += -ri.x * ri.z;
        I.at(1, 0) += -ri.y * ri.x;
        I.at(1, 1) += ri.x*ri.x + ri.z * ri.z;
        I.at(1, 2) += -ri.y * ri.z;
        I.at(2, 0) += -ri.x * ri.z;
        I.at(2, 1) += -ri.y * ri.z;
        I.at(2, 2) += ri.x*ri.x + ri.y * ri.y;
    }

    for (Vec3fIter it = mVertices.begin(); it != mVertices.end(); it++) {
        Vec3f ri = *it;
        I.at(0, 0) += ri.y*ri.y + ri.z * ri.z;
        I.at(0, 1) += -ri.x * ri.y;
        I.at(0, 2) += -ri.x * ri.z;
        I.at(1, 0) += -ri.y * ri.x;
        I.at(1, 1) += ri.x*ri.x + ri.z * ri.z;
        I.at(1, 2) += -ri.y * ri.z;
        I.at(2, 0) += -ri.x * ri.z;
        I.at(2, 1) += -ri.y * ri.z;
        I.at(2, 2) += ri.x*ri.x + ri.y * ri.y;
    }

    Matrix33f massScale(Matrix33f::createScale(mPointMass));
    I *= massScale;
    return I;
}

void Mesh::computeInertia()
{
    mInitR = computeAABB();
    generateInsiders();
    mCOM = computeCOM();
    mInitI = computeInitialInertia();
    app::console() << "mCOM : " << mCOM << std::endl;
    app::console() << "Inertia : " << std::endl << mInitI << std::endl;
}

bool Mesh::loadMesh(const char* filename, BODY_PART partType)
{
	this->mPartType = partType;
	char type;
    string inputLine;
    ifstream ObjFile(filename,ios::in);

    while(getline(ObjFile,inputLine)) {
        stringstream ss(inputLine);
        ss>>type;
        switch(type) {
        case 'v': {
            float x,y,z; ss>>x>>y>>z;
            mVertices.push_back(Vec3f(x,y,z));
        }
            break;
        case 'f': {
            unsigned int indices[3];
            ss>>indices[0]>>indices[1]>>indices[2];
			indices[0]--; indices[1]--; indices[2]--;
            mFaceIndices.push_back(Vec3ui(indices[0],indices[1],indices[2]));
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
	app::console()<<"AABB's dimensions : "<<(mExtents[1] - mExtents[0])<<std::endl;
	return true;
}

std::vector<float> Mesh::findIntersection(const ci::Ray &ry)
{
    vector<float> ret_val;
    for (unsigned int i = 0; i<mFaceIndices.size(); i++)
    {
        float tmp = dot(mFaceNormals[i], ry.getDirection());
        if (tmp > 1.0e-6 || tmp < -1.0e-6) {
            float mCurrentTime = dot(mFaceNormals[i], mVertices[mFaceIndices[i].x] - ry.getOrigin()) / tmp;
            Vec3f Ph = ry.getOrigin() + mCurrentTime*ry.getDirection();
            if (insideTriangle(Ph, i))
                ret_val.push_back(mCurrentTime);
        }
    }
    return ret_val;
}

bool Mesh::insideTriangle(const Vec3f &Ph, unsigned int index)
{
    float a,b,c;
    float mu,mv;
    float ut,vt,wt;

    Vec3f A = cross( mVertices[mFaceIndices[index].y]-mVertices[mFaceIndices[index].x],
					 mVertices[mFaceIndices[index].z]-mVertices[mFaceIndices[index].x] );
	Vec3f A1 = cross( mVertices[mFaceIndices[index].y]-Ph, mVertices[mFaceIndices[index].z]-Ph );
	Vec3f A2 = cross( mVertices[mFaceIndices[index].z]-Ph, mVertices[mFaceIndices[index].x]-Ph );

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

const int Mesh::getNumTriangles() const
{
    return mFaceIndices.size();
}
const std::vector<Vec3ui>& Mesh::getIndices() const
{
    return mFaceIndices;
}

const std::vector<ci::Vec3f>& Mesh::getVertices() const
{
    return mVertices;
}

const std::vector<ci::Vec3f>& Mesh::getNormals() const
{
    return mFaceNormals;
}

const ci::Vec3f& Mesh::extent(const unsigned index) const
{
    return mExtents[index];
}

const BODY_PART& Mesh::bodyPart() const
{
    return mPartType;
}

const ci::Vec3f& Mesh::COM() const
{
    return mCOM;
}

const ci::Matrix33f& Mesh::initialInteria() const
{
    return mInitI;
}
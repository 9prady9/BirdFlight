
#include "Node.h"
#include "Mesh.h"
#include "cinder\Matrix.h"
#include "cinder\Vector.h"
#include "cinder\gl\gl.h"
#include "cinder\app\AppBasic.h"
#include "cinder\CinderMath.h"
#include <vector>

using namespace ci;
using namespace std;

Matrix33f outerProduct(const Vec3f &r)
{
	return Matrix33f(r.x*r.x, r.x*r.y, r.x*r.z,
						r.x*r.y, r.y*r.y, r.y*r.z,
						r.x*r.z, r.y*r.z, r.z*r.z);
}

void Bird::loadGeometry(Link *arc)
{
	// Load Arc to body
	arc->Lmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);
	arc->Amatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					0.0f, 1.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 1.0f, 0.0f,
					0.0f, 0.0f, 0.0f, 1.0f);
	arc->sibling = 0;
	arc->nodePtr = new Node();
	// get the pointer for processing
	Node *B_Node  = arc->nodePtr;
	// Load Body Node
	std::string strPath = app::getAssetPath("meshes/Body_Bird.obj").string();
	B_Node->shape.loadMesh( strPath.c_str(), BODY );
	B_Node->Tmatrix = Matrix44<float>::identity();
	B_Node->childs.push_back( new Link() );	// Left Wing
	B_Node->childs.push_back( new Link() );	// Right Wing
	Link *LArc = B_Node->childs[0];
	Link *RArc = B_Node->childs[1];

	// Load Left Wing Arc	
	LArc->Lmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f, 0.0f,
					  0.0f, 0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 0.0f, 1.0f); //4.0f, 2.5f, -23.8f, 1.0f);
	LArc->Amatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f, 0.0f,
					  0.0f, 0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 0.0f, 1.0f);
	LArc->sibling = RArc;
	LArc->nodePtr = new Node();
	Node *LWNode = LArc->nodePtr;
	// Load Left Wing Node
	strPath = app::getAssetPath("meshes/LeftWing_Bird.obj").string();
	LWNode->shape.loadMesh( strPath.c_str(), WING );
	LWNode->Tmatrix = Matrix44<float>::identity();

	// Load Right Wing Arc
	RArc->Lmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f, 0.0f,
					  0.0f, 0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 0.0f, 1.0f); //5.0f, 2.5f, 28.0f, 1.0f);
	RArc->Amatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f, 0.0f,
					  0.0f, 0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 0.0f, 1.0f);
	RArc->sibling = LArc;
	RArc->nodePtr = new Node();
	Node *RWNode = RArc->nodePtr;
	// Load Right Wing Node
	strPath = app::getAssetPath("meshes/RightWing_Bird.obj").string();
	RWNode->shape.loadMesh( strPath.c_str(), WING );
	RWNode->Tmatrix = Matrix44<float>::identity();
}

void Bird::drawGeometry(const Link *arc, Matrix44<float> matrix, bool lighting)
{
	// get transformations associated with arc and concatenate to current matrix
	matrix = matrix * arc->Lmatrix;
	matrix = matrix * arc->Amatrix;
	// process data at node
	Node *nodePtr = arc->nodePtr;
	Matrix44f hold(matrix);
	matrix = matrix * nodePtr->Tmatrix;
	vector<Vec3f> articulatedData = transformData( matrix, nodePtr->shape.getVertices() );
	draw(articulatedData, nodePtr->shape, lighting);
	matrix = hold;

	// process children of the node
	if( nodePtr->childs.size() > 0 )
	{
		for( vArcIter vait = nodePtr->childs.begin(); vait != nodePtr->childs.end(); vait++ )
		{
			Link *nextArc = *vait;
			drawGeometry(nextArc, matrix, lighting);
		}
	}
}

Vec3f Bird::getLeftWingFaceNormal()
{
	Matrix44f twist(  cos(AOA), sin(AOA), 0.0f, 0.0f,
					 -sin(AOA), cos(AOA), 0.0f, 0.0f,
					      0.0f,     0.0f, 1.0f, 0.0f,
						  0.0f,     0.0f, 0.0f, 1.0f);	
	Link *leftArc = root->nodePtr->childs[0];
	Matrix44f transform = root->Amatrix * (leftArc->Amatrix * twist);
	Vec3f result = transform * this->LWing_Norm;
	result.normalize();
	return result;
}

Vec3f Bird::getRightWingFaceNormal()
{
	Matrix44f twist(  cos(AOA), sin(AOA), 0.0f, 0.0f,
					 -sin(AOA), cos(AOA), 0.0f, 0.0f,
					      0.0f,     0.0f, 1.0f, 0.0f,
						  0.0f,     0.0f, 0.0f, 1.0f);
	Link *rightArc = root->nodePtr->childs[1];
	Matrix44f transform = root->Amatrix * (rightArc->Amatrix * twist);
	Vec3f result = transform * this->RWing_Norm;
	result.normalize();
	return result;
}

vector<Vec3f> Bird::transformData(const Matrix44f &matrix, const vector<Vec3f> &vertices)
{
	vector<Vec3f> result;
	for( CVec3fIter vit = vertices.begin(); vit != vertices.end(); vit++ )
	{
		Vec4f v(*vit,1.0f);
		result.push_back( (matrix * v).xyz() );
	}
	return result;
}

void Bird::draw(const vector<Vec3f> &newVertices, const Mesh &shape, bool lighting)
{
	GLfloat mat_amb[] = {0.05f,0.05f,0.05f,1.0f};
    GLfloat mat_diff[4];
    GLfloat mat_spec[] = {1.0f,1.0f,1.0f,1.0f};
    GLfloat mat_shininess[] = {100.0f};

	int numTriangles = shape.getNumTriangles();
	const vector<Vec3ui> indices = shape.getIndices();
	const vector<Vec3f> norms = shape.getNormals();
	if( shape.part == BODY ) {
		glColor4f(0.72f,0.23f,0.04f,1.0f);
		mat_diff[0] = 0.55f;
		mat_diff[1] = 0.27f;
		mat_diff[2] = 0.08f;
		mat_diff[3] = 1.0f;
	}
	else if( shape.part == WING ) {
		glColor4f(0.72f,0.53f,0.04f,1.0f);
		mat_diff[0] = 0.72f;
		mat_diff[1] = 0.53f;
		mat_diff[2] = 0.04f;
		mat_diff[3] = 1.0f;
	}
	glPushMatrix();
		if(lighting) {
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mat_amb);
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diff);
			glMaterialfv(GL_FRONT,GL_SPECULAR,mat_spec);
			glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);
		} else {
			glColor4fv(mat_diff);
		}
		glBegin(GL_TRIANGLES);
		for( int triag=0; triag<numTriangles; triag++ )
		{
			glNormal3f( norms[triag] );
			glVertex3f( newVertices[ indices[triag].x ] );
			glVertex3f( newVertices[ indices[triag].y ] );
			glVertex3f( newVertices[ indices[triag].z ] );
		}
		glEnd();
	glPopMatrix();
}

void Bird::updateMatrices()
{
	// Move the bird to new COM
	root->Lmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f, 0.0f,
					  0.0f, 0.0f, 1.0f, 0.0f,
					  mState.mCOM.x, mState.mCOM.y, mState.mCOM.z, 1.0f);
	// Orient the bird according to new orientation
	root->Amatrix = mState.mR;

	//Wing Twist about z-axis
	Matrix44f twist(  cos(AOA), sin(AOA), 0.0f, 0.0f,
					 -sin(AOA), cos(AOA), 0.0f, 0.0f,
					      0.0f,     0.0f, 1.0f, 0.0f,
						  0.0f,     0.0f, 0.0f, 1.0f);

	//update left wing orientation
	Link *LArc = root->nodePtr->childs[0];
	LArc->Amatrix.set(1.0f,        0.0f,       0.0f, 0.0f,
					  0.0f,  cos(swing), sin(swing), 0.0f,
					  0.0f, -sin(swing), cos(swing), 0.0f,
					  0.0f,        0.0f,       0.0f, 1.0f);
	LArc->Amatrix *= twist;
	LArc->nodePtr->Tmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
							   0.0f, 1.0f, 0.0f, 0.0f,
							   0.0f, 0.0f, 1.0f, 0.0f,
							   4.0f, 2.5f, -23.8f, 1.0f);

	//update right wing orientation
	Link *RArc = root->nodePtr->childs[1];
	RArc->Amatrix.set(1.0f,       0.0f,        0.0f, 0.0f,
					  0.0f, cos(swing), -sin(swing), 0.0f,
					  0.0f, sin(swing), cos(swing), 0.0f,
					  0.0f,       0.0f,        0.0f, 1.0f);
	RArc->Amatrix *= twist;
	RArc->nodePtr->Tmatrix.set(1.0f, 0.0f, 0.0f, 0.0f,
							   0.0f, 1.0f, 0.0f, 0.0f,
							   0.0f, 0.0f, 1.0f, 0.0f,
							   5.0f, 2.5f, 25.8f, 1.0f);
}

void Bird::drawGeometry(bool lighting)
{
	drawGeometry( root, Matrix44f::identity(), lighting );
}

void Bird::compute_COM_Inertia()
{
	app::console()<<"----------------------------------"<<std::endl;
	app::console()<<"Bird model COM and Inertia details"<<std::endl;

	// COM of whole bird model
	Vec3f body = root->nodePtr->shape.COM;
	Vec3f lwing = root->nodePtr->childs[0]->nodePtr->shape.COM;
	Vec3f rwing = root->nodePtr->childs[1]->nodePtr->shape.COM;
	mState.mCOM = ( WING_WT * lwing + WING_WT * rwing + BODY_WT * body )/( (2*WING_WT) +BODY_WT );

	// Moment of Inertia	
	Vec3f LeftWing(4.0f, 2.5f, -23.8f);
	Vec3f RightWing(5.0f, 2.5f, 28.0f);
	// Left Wing MOI
	Matrix33f leftI = root->nodePtr->childs[0]->nodePtr->shape.mInitI;
	Matrix33f lE = Matrix33f::identity();
	lE *= dot(LeftWing,LeftWing);
    leftI += ( (lE - outerProduct(LeftWing)) * WING_WT );
	// Right Wing MOI
	Matrix33f rightI = root->nodePtr->childs[1]->nodePtr->shape.mInitI;
	Matrix33f rE = Matrix33f::identity();
	rE *= dot(RightWing,RightWing);
	rightI += ( (rE - outerProduct(RightWing)) * WING_WT );
	// Total MOI
	mState.mInitI = root->nodePtr->shape.mInitI + leftI + rightI;
	mState.mInitInvI = mState.mInitI.inverted();

	app::console()<<"COM : "<<mState.mCOM<<std::endl;
	app::console()<<"Inertia : "<<std::endl<<mState.mInitI<<std::endl;
	app::console()<<"Inertia Mat deter : "<<std::endl<<mState.mInitI.determinant()<<std::endl;
	app::console()<<"Inverse Inertia : "<<std::endl<<mState.mInitInvI<<std::endl;
	app::console()<<"----------------------------------"<<std::endl;
}

void Bird::gnrt_Swing_AOA()
{
	if( (mState.mCOM.y -  prop->level ) < 0.0f )
	{
		swing = (PI/180)*( upThsld*t + (1-t)*downThsld );
		AOA = (PI/180)*( atkUThlsd*t + (1-t)*atkDThlsd );
		if(upstroke) {
			t = t + 0.1f;
			if(t >= 1.0f)
				upstroke = false;
		} else {
			t = t - 0.1f;
			if( t<=0.0f )
				upstroke = true;
		}

	} else {
		swing = 0.0f;
		AOA = 0.0f;
	}
	updateMatrices();
	app::console()<<"t value : "<<t<<std::endl;
	app::console()<<"Angle of attack : "<<(AOA*180.0f)/PI<<std::endl;
	app::console()<<"swing angle : "<<(swing*180.0f)/PI<<std::endl;
}

Derivative Bird::accelerate(const State &st)
{
	Derivative delta;
	delta.V = mState.mP/M;
	Vec3f omega = mState.mInvI * mState.mL;
	delta.dOmegaR = Star(omega)*mState.mR;
	delta.force = Vec3f::zero();
	delta.torque = Vec3f::zero();
	// gravity force
	delta.force = M*prop->g;
	//Find angle of attack and calculate lift force
	float Cl = 2 * PI * abs(AOA) * 100;
	Vec3f relV = (mState.mP/M) - prop->windForce * prop->wind;
	float Vr2 = relV.lengthSquared();
	Vec3f lLiftDir = this->getLeftWingFaceNormal();
	Vec3f leftLift = (0.5f * Cl * WingArea * prop->rho * Vr2) * lLiftDir;
	Vec3f rLiftDir = this->getRightWingFaceNormal();
	Vec3f rightLift = (0.5f * Cl * WingArea * prop->rho * Vr2) * rLiftDir;
	Vec3f lR(4.0f, 2.5f, -23.8f);
	Vec3f rR(4.0f, 2.5f, 23.8f);
	Vec3f Total_Lift  = leftLift + rightLift;
	//update force and torque
	delta.force += Total_Lift;
	//delta.torque += cross(lR,leftLift) + cross(rR,rightLift);
	return delta;
}

void Bird::accelerate(Derivative &delta)
{
	delta.V = mState.mP/M;
	Vec3f omega = mState.mInvI * mState.mL;
	delta.dOmegaR = Star(omega)*mState.mR;
	delta.force = Vec3f::zero();
	delta.torque = Vec3f::zero();
	// gravity force
	delta.force = M*prop->g;
	//Find angle of attack and calculate lift force
	float Cl = 2 * PI * abs(AOA) * 120;
	Vec3f relV = (mState.mP/M) - prop->windForce * prop->wind;
	float Vr2 = relV.lengthSquared();
	Vec3f lLiftDir = this->getLeftWingFaceNormal();
	Vec3f leftLift = (0.5f * Cl * WingArea * prop->rho * Vr2) * lLiftDir;
	Vec3f rLiftDir = this->getRightWingFaceNormal();
	Vec3f rightLift = (0.5f * Cl * WingArea * prop->rho * Vr2) * rLiftDir;
	Vec3f lR(4.0f, 2.5f, -23.8f);
	Vec3f rR(4.0f, 2.5f, 23.8f);
	Vec3f Total_Lift  = leftLift + rightLift;
	//update force and torque
	delta.force += Total_Lift;
	//delta.torque += cross(lR,leftLift) + cross(rR,rightLift);
	/*
	app::console()<<"Downward pull : "<<(M*prop->g).length()<<std::endl;
	app::console()<<"Total Lift : "<<Total_Lift<<"; Magnitude :"<<Total_Lift.length()<<std::endl;
	app::console()<<"Total force : "<<delta.force<<"; Magnitude :"<<delta.force.length()<<std::endl;
	*/
}

void Bird::updateState(const State &st)
{
	mState.mCOM		=	st.mCOM;
	mState.mR		=	st.mR;
	mState.mP		=	st.mP;
	mState.mL		=	st.mL;
	mState.mInvI	=	st.mInvI;
	// Now re-orthogonize the orientation matrix
	Vec3f X = mState.mR.getColumn(0);
	Vec3f Y = mState.mR.getColumn(1);
	Vec3f Z = mState.mR.getColumn(2);
	Vec3f Z_ = cross(X,Y);
	Vec3f Y_ = cross(Z_,X);
	X.normalize(); Y_.normalize(); Z_.normalize();
	mState.mR.set(X.x,X.y,X.z,Y_.x,Y_.y,Y_.z,Z_.x,Z_.y,Z_.z);
}
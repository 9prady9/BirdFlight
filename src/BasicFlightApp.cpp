#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder\Camera.h"
#include "cinder/params/Params.h"
#include "cinder\TriMesh.h"
#include "Resources.h"
#include "cinder/ImageIo.h"
#include "cinder\Quaternion.h"
#include "boost\range\numeric.hpp"
#include <sstream>
#include "Node.h"
#include "terrain.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BasicFlightApp : public AppBasic {
public:
	void prepareSettings( Settings *settings );
	void loadTextures();
	void setup();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();

	// PARAMS
	params::InterfaceGlRef	mParams;
	bool wireframe;
	bool lighting;
	bool runSimulation;
	bool isSetupDone;
	std::string integrator;
	Vec3f gNorm;
	int frame;

	// CAMERA
	CameraPersp			mCam;
	Vec3f				mEye, mCenter, mUp;
	float				mCameraDistance;
	GLfloat				mLightPos[4];

	// Scene properties
	Scene_Properties	prop;
	GLfloat				time;
	Bird*				boid;
	Terrain*			terr;
};

void BasicFlightApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 55.0f );

	prop.gravity = 980.0f;
	prop.g.set(0.0f,-1.0f,0.0f);
	prop.windForce = 447.0f;
	prop.wind = Vec3f(-1.0f,0.0f,0.0f);
	prop.rho = 1.229 * 1.0e-6;
	prop.g *= prop.gravity;
	prop.dT = 0.001f;
	gNorm = prop.g.normalized();
	prop.integrate = integrateByEulerMethod;
	prop.level = 5.0f;

	integrator = "Mr. Euler";
	wireframe = false;
	runSimulation = false;
	lighting = true;
	isSetupDone = false;
	frame = 58;
}

void BasicFlightApp::setup()
{
	GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };

	// SETUP PARAMS
	mParams = params::InterfaceGl::create(getWindow(), "Basic Flight", Vec2i( 250, 100 ) );
	mParams->addParam("Which Integration ?", &integrator);

	// Setup Scence parameters
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);     // Background => dark blue
	glShadeModel(GL_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
	glEnable( GL_DEPTH_TEST );
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	mLightPos[0] = mEye.x; mLightPos[1] = mEye.y; mLightPos[2] = mEye.z; mLightPos[3] = 0.0f;
	glLightfv(GL_LIGHT0, GL_POSITION, mLightPos);

	// Object initilization
	boid = new Bird(Vec3f(1000.0f,0.0f,0.0f),Vec3f(0.0f, 0.0f, 0.0f),&prop);

	/*terr = new Terrain(1024,1024,0.0f,100.0f,0.3f,0.75f);
	terr->prepareTerrain();*/

	terr = new Terrain(1025,1025,0.0f,100.0f,0.3f,0.75f);
	std::string strPath1 = app::getAssetPath("HeightMap_Projects/HeightMap_2BaseTexture.bmp").string();
	std::string strPath2 = app::getAssetPath("HeightMap_Projects/HeightMap2.bmp").string();
	terr->loadTerrain(strPath2.c_str(),strPath1.c_str());

	// SETUP CAMERA
	Vec3f dir	= Vec3f(1,0,0);
	mCenter		= boid->getCenter() + Vec3f( 0.0f, 40.0f, 0.0f );
	mEye		= mCenter - 100.0f * dir;
	mUp			= Vec3f::yAxis();

	mCam.setPerspective( 60.0f, getWindowAspectRatio(), 0.1f, 2000.0f );
	gl::setMatrices( mCam );
	isSetupDone = false;
}

void BasicFlightApp::mouseDown( MouseEvent event )
{
}

void BasicFlightApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'w' || event.getChar()=='W' ) {
		if(wireframe) {
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			wireframe = false;
		} else {
			glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
			wireframe = true;
		}
	} else if( event.getChar() == 'l' || event.getChar() == 'L' ) {
		if(lighting) {
			lighting = false;
			glDisable(GL_LIGHTING);
			glDisable(GL_LIGHT0);
		} else {
			lighting = true;
			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
		}
	} else if( event.getChar() == 'r' || event.getChar() == 'R' ) {
		std::cout<<"Simulation Started"<<std::endl;
		runSimulation = true;
	} else if( event.getChar() == 's' || event.getChar() == 'S' ) {
		runSimulation = false;
	} else if( event.getChar() == 'e' || event.getChar() == 'E' ) {
		prop.integrate = integrateByEulerMethod;
		integrator = "Mr. Euler";
	} else if( event.getChar() == 'k' || event.getChar() == 'K' ) {
		prop.integrate = integrateByRungeKutta4;
		integrator = "Mr. RK4";
	}
}

void BasicFlightApp::update()
{
	if(isSetupDone)
	{
		// Update Camera Setup
		Vec3f dir	= Vec3f(1,0,0);
		mCenter		= boid->getCenter() + Vec3f( 0.0f, 40.0f, 0.0f );
		mEye		= mCenter - 100.0f * dir;
		mUp			= Vec3f::yAxis();

		if(lighting) {
			mLightPos[0] = mEye.x; mLightPos[1] = mEye.y; mLightPos[2] = mEye.z; mLightPos[3] = 0.0f;
			glLightfv(GL_LIGHT0, GL_POSITION, mLightPos);
		}	
		mCam.lookAt( mEye, mCenter, mUp );
		mCam.setPerspective( 60.0f, getWindowAspectRatio(), 0.1f, 2000.0f );
		gl::setMatrices( mCam );

		// Run updates
		if(runSimulation)
		{		
			State newState = boid->mState;

			// Print current state

			/*console()<<" Current state : "<<std::endl<<
			newState.mCOM<<std::endl<<
			newState.mR<<std::endl<<
			newState.mP<<std::endl<<
			newState.mL<<std::endl<<
			"---------------------------"<<std::endl;*/

			Derivative accAccum;
			boid->accelerate( accAccum );
			/*
			console()<<" Derivate state : "<<std::endl<<
			accAccum.V<<std::endl<<
			accAccum.dOmegaR<<std::endl<<
			accAccum.force<<std::endl<<
			accAccum.torque<<std::endl<<
			"---------------------------"<<std::endl;
			*/
			prop.integrate( boid, newState, prop.dT, accAccum );

			// Print new state
			/*
			console()<<" New state : "<<std::endl<<
			newState.mCOM<<std::endl<<
			newState.mR<<std::endl<<
			newState.mP<<std::endl<<
			newState.mL<<std::endl<<
			"---------------------------"<<std::endl;
			*/
			boid->updateState(newState);
			boid->gnrt_Swing_AOA();
			time += prop.dT;
			//console()<<"--------------------------------------One time step done--------------------------------------"<<std::endl;
		}
	}
}

void BasicFlightApp::draw()
{
	if(isSetupDone)
	{
		// clear out the window with black
		gl::clear( Color( 0, 0, 0 ) ); 
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		terr->renderTerrain(lighting);
		// Draw bird	
		boid->drawGeometry(lighting);
		// Draw the interface
		mParams->draw();
		// Write output image	
		/*if(runSimulation) {
		stringstream out;
		out<<frame++<<".png";
		writeImage(out.str(),copyWindowSurface());
		}*/
	}
}

CINDER_APP_BASIC( BasicFlightApp, RendererGl )

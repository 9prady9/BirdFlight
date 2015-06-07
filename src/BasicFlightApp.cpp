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

static const uint8_t MAX_TERRAIN_MAPS = 6;

class BasicFlightApp : public AppBasic {
private:
    /* UI Parameters */
    params::InterfaceGlRef	mParams;
    bool wireframe;
    bool lighting;
    bool runSimulation;
    bool isSetupDone;
    std::string integrator;

    /* Simulation tracking parameters*/
    Vec3f gNorm;
    int frame;

    /* Camera Parameters */
    CameraPersp	    mCam;
    Vec3f		    mEye, mCenter, mUp;
    float		    mCameraDistance;

    /* Scene objects & time properties */
    GLfloat		        mLightPos[4];
    Scene_Properties	mProp;
    GLfloat				time;
    Bird*				boid;
    Terrain*			terr[MAX_TERRAIN_MAPS];
    std::vector<std::string> terrBMapPaths;
    std::vector<std::string> terrHMapPaths;
    Terrain*            activeTerrain;
    uint8_t             activeTerrainId;

public:
    /* Below function overrides base class function 
    and it is used to setup the application*/
    void prepareSettings(Settings *settings);

    /* Below function overrides base class function
    and it is used to initialize OpenGL state*/
    void setup();

    /* UI Event handlers */
	void mouseDown(MouseEvent event);
	void keyDown(KeyEvent event);

    /* Below function overrides base class function
    and it is used to update the state variables
    of the simulation in progress */
	void update();

    /* Below function overrides base class function
    and it is where our draw calls have to be written */
	void draw();
};

void BasicFlightApp::prepareSettings(Settings *settings)
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 55.0f );

	mProp.gravity    = 980.0f;
	mProp.g.set(0.0f,-1.0f,0.0f);
	mProp.windForce  = 447.0f;
	mProp.wind       = Vec3f(-1.0f,0.0f,0.0f);
	mProp.rho        = 1.229f * 1.0e-6f;
	mProp.g          *= mProp.gravity;
	mProp.dT         = 0.001f;
	gNorm           = mProp.g.normalized();
	mProp.integrate  = integrateByEulerMethod;
	mProp.level      = 5.0f;

	integrator      = "Mr. Euler";
	wireframe       = false;
	runSimulation   = false;
	lighting        = true;
	frame           = 58;

    for (uint8_t i = 0; i < MAX_TERRAIN_MAPS; ++i) {
        std::string tmp1 = "HeightMap_Projects/HeightMap_" + std::to_string(i + 1) + "BaseTexture.bmp";
        std::string tmp2 = "HeightMap_Projects/HeightMap" + std::to_string(i + 1) + ".bmp";
        terrBMapPaths.push_back(app::getAssetPath(tmp1).string());
        terrHMapPaths.push_back(app::getAssetPath(tmp1).string());
    }
    activeTerrainId = 0;
}

void terrainInitHelper(Terrain** ter, std::string baseMap, std::string heightMap)
{
    /*terr = new Terrain(1024,1024,0.0f,100.0f,0.3f,0.75f);
    terr->prepareTerrain();*/

    *ter = new Terrain(1025, 1025, 0.0f, 100.0f, 0.3f, 0.75f);
    std::string strPath1 = app::getAssetPath("HeightMap_Projects/HeightMap_2BaseTexture.bmp").string();
    std::string strPath2 = app::getAssetPath("HeightMap_Projects/HeightMap2.bmp").string();
    (*ter)->prepareTerrain(baseMap.c_str(), heightMap.c_str());
}

void BasicFlightApp::setup()
{
	GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };

	/* SETUP PARAMS */
	mParams = params::InterfaceGl::create(getWindow(), "Basic Flight", Vec2i( 250, 100 ) );
	mParams->addParam("Which Integration ?", &integrator);

	/* Setup Scence parameters */
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);     /* Background => dark blue */
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

	/* Object initilization */
	boid = new Bird(Vec3f(1000.0f,0.0f,0.0f),Vec3f(0.0f, 0.0f, 0.0f),&mProp);

    for (uint8_t i = 0; i < MAX_TERRAIN_MAPS; ++i) {
        terrainInitHelper(&terr[i], terrBMapPaths[i], terrHMapPaths[i]);
    }
    activeTerrain = terr[activeTerrainId];

	/* Setup Camera properties */
	Vec3f dir	= Vec3f(1,0,0);
	mCenter		= boid->getCenter() + Vec3f( 0.0f, 40.0f, 0.0f );
	mEye		= mCenter - 100.0f * dir;
	mUp			= Vec3f::yAxis();

	mCam.setPerspective( 60.0f, getWindowAspectRatio(), 0.1f, 2000.0f );
	gl::setMatrices( mCam );
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
		mProp.integrate = integrateByEulerMethod;
		integrator = "Mr. Euler";
	} else if( event.getChar() == 'k' || event.getChar() == 'K' ) {
		mProp.integrate = integrateByRungeKutta4;
		integrator = "Mr. RK4";
    } else if (event.getChar() == 'm' || event.getChar() == 'M') {
        activeTerrainId = (activeTerrainId + 1) % MAX_TERRAIN_MAPS;
        activeTerrain = terr[activeTerrainId];
    }
}

void BasicFlightApp::update()
{
	/* Update Camera Setup */
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

	/* Run updates */
	if(runSimulation)
	{		
		State newState = boid->getState();

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
		mProp.integrate( boid, newState, mProp.dT, accAccum );

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
		time += mProp.dT;
		//console()<<"-------------------One time step done-------------------"<<std::endl;
	}
}

void BasicFlightApp::draw()
{
	/* clear out the window with black */
	gl::clear(Color( 0, 0, 0 )); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    activeTerrain->renderTerrain(lighting);
	/* Draw bird */
	boid->drawGeometry(lighting);
	/* Draw the interface */
	mParams->draw();
	/* Write output image */
	/*
    if(runSimulation) {
	stringstream out;
	out<<frame++<<".png";
	writeImage(out.str(),copyWindowSurface());
	}
    */
}

CINDER_APP_BASIC( BasicFlightApp, RendererGl )
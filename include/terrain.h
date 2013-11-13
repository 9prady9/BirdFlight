#pragma once

#include "cinder\Vector.h"
#include "cinder\Color.h"

const ci::ColorA GREEN(0.333f,0.420f,0.184f);						//(R,G,B)=(85,107,47)
const ci::ColorA BROWN(0.957f,0.643f,0.376f);						//(R,G,B)=(244,164,96)
const ci::ColorA SEA_GREEN(0.561f,0.737f,0.561f);					//(R,G,B)=(143,188,143)

/**
 * Terrain class implements http://en.wikipedia.org/wiki/Diamond-square_algorithm
 * points of min height are give color GREEN
 * points of max height are give color BROWN
 * any points with height between min and max height have a interpolated color of GREEN & BROWN
 * terrain algorithm i implemented is a fractal based one to which link is given.
 *
 * To use the terrain my class produces, call prepareTerrain(), iterateByDiamondSquare(), 
 * applySmoothingFilter() in the same order during setup of the application
 *
 * However anyone who wants to use different terrain can load a terrain texture with it's height map encoded in alpha chanel.
 */
class Terrain{
public:
	Terrain( int zDim, int xDim, float minHeight, float maxHeight, float roughness, float smoothy);
	void loadTerrain(const char *heightmap,const char* colormap);	// prepare the terrain using image
	void prepareTerrain();											// prepare init data for creating a terrain
	void computeNormals();											// call this function before rendering Terrain if lighting is enabled
	void renderTerrain(bool lighting);								// render based on lighting requirement
private:
    ci::ColorA getColor(float height) {
		float frc = (height - minH)*invHDiff;
		return (BROWN*frc + GREEN*(1-frc));
	}
	void DiaSqrHelper(int x1_idx, int x2_idx, int z1_idx, int z2_idx, int iteration, int Threshold);
	void iterateByDiamondSquare(int how_many_iterations=9);			// call this method if prepareTerrain was used to initialize terrain
	void applySmoothingFilter();

	int len,wid;
	float minH,maxH;
	float invHDiff;
	float rough;
	float smth;
	ci::Vec3f **points;
	ci::ColorA **colors;
	ci::Vec3f **normals;
};
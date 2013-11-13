#include "terrain.h"
#include "cinder\Rand.h"
#include "cinder\Surface.h"
#include "cinder\ImageIo.h"
#include "cinder\gl\gl.h"
#include "Resources.h"

using namespace ci;

Terrain::Terrain(int Z, int X, float minHeight, float maxHeight, float roughness, float smoothy)
{
	len = Z;
	wid = X;
	minH = minHeight;
	maxH = maxHeight;
	invHDiff = 1/(maxH-minH);
	rough = roughness;
	smth = smoothy;
	points = new Vec3f*[wid];
	colors = new ColorA*[wid];
	normals = new Vec3f*[wid];
	for(int i=0; i<wid; i++) {
		points[i] = new Vec3f[len];
		colors[i] = new ColorA[len];
		normals[i] = new Vec3f[len];
	}
	Rand::randomize();	// resets the static random generator seed
}

void Terrain::loadTerrain(const char *heightmap,const char* colormap)
{
	const Surface8u heightXZ( loadImage( loadFile( heightmap ) ) );
	const Surface8u colorXZ( loadImage( loadFile( colormap ) ) );
	// Assuming alpha channel availability
	for(int i=0; i<wid; i++)
	{
		for(int k=0; k<len; k++)
		{
			points[i][k].x = -wid/2.0f + 2.0f*i;
			points[i][k].y = (*heightXZ.getDataRed(Vec2i(i,k)))-200.0f;
			points[i][k].z = -len/2.0f + 2.0f*k;
			colors[i][k] = colorXZ.getPixel(Vec2i(i,k));
		}
	}
	// smooth the generated terrain
	for(int i=0;i<9;++i) applySmoothingFilter();
	computeNormals();
}

void Terrain::prepareTerrain()
{
	int mid_i = wid/2;
	int mid_k = len/2;
	for(int i=0; i<wid; i++)
	{
		for(int k=0; k<len; k++)
		{
			points[i][k].x = -wid/2.0f + 3.0f*i;
			points[i][k].y = Rand::randFloat(minH,maxH);
			points[i][k].z = -len/2.0f + 3.0f*k;
			colors[i][k]   = getColor(points[i][k].y);
		}
	}
	// run iterations to generate a finer mesh points
	iterateByDiamondSquare(9);
	// smooth the generated terrain
	for(int i=0;i<9;++i) applySmoothingFilter();
	computeNormals();
}

void Terrain::computeNormals()
{
	int widThreshold = wid-1;
	int lenThreshold = len-1;
	Vec3f n1,n2,n3,n4,n5,n6,n7,n8;
	n1 = n2 = n3 = n4 = n5 = n6 = n7 = n8 = Vec3f::zero();
	for(int i=0; i<wid; i++)
	{
		for(int k=0; k<len; k++)
		{
			if(k>0 && i>0)
				n1 = cross(points[i-1][k] - points[i][k], points[i-1][k-1] - points[i][k]);
			if(k>0 && i>0)
				n2 = cross(points[i-1][k-1] - points[i][k], points[i][k-1] - points[i][k]);
			if(k>0 && i<widThreshold)
				n3 = cross(points[i][k-1] - points[i][k], points[i+1][k-1] - points[i][k]);
			if(k>0 && i<widThreshold)
				n4 = cross(points[i+1][k-1] - points[i][k], points[i+1][k] - points[i][k]);
			if(k<lenThreshold && i<widThreshold)
				n5 = cross(points[i+1][k] - points[i][k], points[i+1][k+1] - points[i][k]);
			if(k<lenThreshold && i<widThreshold)
				n6 = cross(points[i+1][k+1] - points[i][k], points[i][k+1] - points[i][k]);
			if(k<lenThreshold && i>0)
				n7 = cross(points[i][k+1] - points[i][k], points[i-1][k+1] - points[i][k]);
			if(k<lenThreshold && i>0)
				n8 = cross(points[i-1][k+1] - points[i][k], points[i-1][k] - points[i][k]);

			normals[i][k] = (n1+n2+n3+n4+n5+n6+n7+n8).normalized();
		}
	}
}

void Terrain::renderTerrain(bool lighting)
{
	int widThreshold = wid-1;
	int lenThreshold = len-1;
	if(lighting) {
		glEnable(GL_COLOR_MATERIAL);
		for(int i=0; i<widThreshold; i++)
		{
			glBegin(GL_TRIANGLE_STRIP);
			for(int k=0; k<lenThreshold; k++)
			{
				glColor3f(colors[i][k].r,colors[i][k].g,colors[i][k].b);
				glNormal3f(normals[i][k].x,normals[i][k].y,normals[i][k].z);
				glVertex3f(points[i][k].x,points[i][k].y,points[i][k].z);
				glColor3f(colors[i+1][k].r,colors[i+1][k].g,colors[i+1][k].b);
				glNormal3f(normals[i+1][k].x,normals[i+1][k].y,normals[i+1][k].z);
				glVertex3f(points[i+1][k].x,points[i+1][k].y,points[i+1][k].z);
			}
			glEnd();
		}
		glDisable(GL_COLOR_MATERIAL);
	} else {
		for(int i=0; i<widThreshold; i++)
		{
			glBegin(GL_TRIANGLE_STRIP);
			for(int k=0; k<lenThreshold; k++)
			{
				glColor3f(colors[i][k].r,colors[i][k].g,colors[i][k].b);
				glVertex3f(points[i][k].x,points[i][k].y,points[i][k].z);
				glColor3f(colors[i+1][k].r,colors[i+1][k].g,colors[i+1][k].b);
				glVertex3f(points[i+1][k].x,points[i+1][k].y,points[i+1][k].z);
			}
			glEnd();
		}
	}
}

void Terrain::iterateByDiamondSquare(int nums)
{
	DiaSqrHelper(0,wid-1,0,len-1,0,nums);
}

void Terrain::DiaSqrHelper(int x1_idx, int x2_idx, int z1_idx, int z2_idx, int iteration, int Threshold)
{
	if( iteration < Threshold )
	{
		/* left to right - X axis; top to bottom - Z axis */
		// Get Corner points
		Vec3f leftTop		= points[x1_idx][z1_idx];
		Vec3f leftBottom	= points[x1_idx][z2_idx];
		Vec3f rightBottom	= points[x2_idx][z2_idx];
		Vec3f rightTop		= points[x2_idx][z1_idx];
		// find mid point indices
		int mid_x_idx = (x1_idx + x2_idx)/2;
		int mid_z_idx = (z1_idx + z2_idx)/2;
		// assign new height to mid point
		points[mid_x_idx][mid_z_idx].y = ((leftTop.y + leftBottom.y + rightBottom.y + rightTop.y)/4) + Rand::randFloat(minH,maxH);
		// assign new heights to mid points of sides of square
		if( z1_idx>0 ) {
			points[mid_x_idx][z1_idx].y = ((leftTop.y + points[mid_x_idx][mid_z_idx].y + 
											rightTop.y + points[mid_x_idx][z1_idx-1].y)/4) + Rand::randFloat(minH,maxH);
		} else {
			points[mid_x_idx][z1_idx].y = ((leftTop.y + points[mid_x_idx][mid_z_idx].y + rightTop.y)/3) + Rand::randFloat(minH,maxH);
		}
		if( z2_idx < len-1 ) {
			points[mid_x_idx][z2_idx].y = ((leftBottom.y + points[mid_x_idx][mid_z_idx].y + 
											rightBottom.y + points[mid_x_idx][z2_idx+1].y)/4) + Rand::randFloat(minH,maxH);
		} else {
			points[mid_x_idx][z2_idx].y = ((leftBottom.y + points[mid_x_idx][mid_z_idx].y + rightBottom.y)/3) + Rand::randFloat(minH,maxH);
		}
		if( x1_idx>0 ) {
			points[x1_idx][mid_z_idx].y = ((leftTop.y + points[mid_x_idx][mid_z_idx].y + 
											leftBottom.y + points[x1_idx-1][mid_z_idx].y)/4) + Rand::randFloat(minH,maxH);
		} else {
			points[x1_idx][mid_z_idx].y = ((leftTop.y + points[mid_x_idx][mid_z_idx].y + leftBottom.y)/3) + Rand::randFloat(minH,maxH);
		}
		if( x2_idx<wid-1 ) {
			points[x2_idx][mid_z_idx].y = ((rightTop.y + points[mid_x_idx][mid_z_idx].y + 
											rightBottom.y + points[x2_idx+1][mid_z_idx].y)/4) + Rand::randFloat(minH,maxH);
		} else {
			points[x2_idx][mid_z_idx].y = ((rightTop.y + points[mid_x_idx][mid_z_idx].y + rightBottom.y)/3) + Rand::randFloat(minH,maxH);
		}
		iteration++;
		if( iteration < Threshold ) {
			minH *= pow(2,-rough);
			maxH *= pow(2,-rough);
			DiaSqrHelper(x1_idx,mid_x_idx,z1_idx,mid_z_idx,iteration,Threshold);
			DiaSqrHelper(mid_x_idx,x2_idx,z1_idx,mid_z_idx,iteration,Threshold);
			DiaSqrHelper(x1_idx,mid_x_idx,mid_z_idx,z2_idx,iteration,Threshold);
			DiaSqrHelper(mid_x_idx,x2_idx,mid_z_idx,z2_idx,iteration,Threshold);
		}
	}
}

void Terrain::applySmoothingFilter()
{
	/* Rows, left to right */
	for(int x = 1;x < wid; x++)
	    for (int z = 0;z < len; z++)
			points[x][z].y = points[x-1][z].y * (1-smth) + points[x][z].y * smth;

	/* Rows, right to left*/
	for(int x = wid-2;x < -1; x--)
	    for (int z = 0;z < len; z++)
		points[x][z].y = points[x+1][z].y * (1-smth) + points[x][z].y * smth;

	/* Columns, bottom to top */
	for(int x = 0;x < wid; x++)
	    for (int z = 1;z < len; z++)
		points[x][z].y = points[x][z-1].y * (1-smth) + points[x][z].y * smth;

	/* Columns, top to bottom */
	for(int x = 0;x < wid; x++)
	    for (int z = len; z < -1; z--)
		points[x][z].y = points[x][z+1].y * (1-smth) + points[x][z].y * smth;
}
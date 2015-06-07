#include "terrain.h"
#include "cinder\Rand.h"
#include "cinder\Surface.h"
#include "cinder\ImageIo.h"
#include "cinder\gl\gl.h"
#include "Resources.h"

using namespace ci;


void Terrain::DiaSqrHelper(int x1_idx, int x2_idx, int z1_idx, int z2_idx, int iteration, int Threshold)
{
    if (iteration < Threshold)
    {
        /* left to right - X axis; top to bottom - Z axis */
        // Get Corner mPoints
        Vec3f leftTop = mPoints[x1_idx][z1_idx];
        Vec3f leftBottom = mPoints[x1_idx][z2_idx];
        Vec3f rightBottom = mPoints[x2_idx][z2_idx];
        Vec3f rightTop = mPoints[x2_idx][z1_idx];
        // find mid point indices
        int mid_x_idx = (x1_idx + x2_idx) / 2;
        int mid_z_idx = (z1_idx + z2_idx) / 2;
        // assign new height to mid point
        mPoints[mid_x_idx][mid_z_idx].y = ((leftTop.y + leftBottom.y + rightBottom.y + rightTop.y) / 4) + Rand::randFloat(mMinHeight, mMaxHeight);
        // assign new heights to mid mPoints of sides of square
        if (z1_idx>0) {
            mPoints[mid_x_idx][z1_idx].y = ((leftTop.y + mPoints[mid_x_idx][mid_z_idx].y +
                rightTop.y + mPoints[mid_x_idx][z1_idx - 1].y) / 4) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        else {
            mPoints[mid_x_idx][z1_idx].y = ((leftTop.y + mPoints[mid_x_idx][mid_z_idx].y + rightTop.y) / 3) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        if (z2_idx < mTerrainLength - 1) {
            mPoints[mid_x_idx][z2_idx].y = ((leftBottom.y + mPoints[mid_x_idx][mid_z_idx].y +
                rightBottom.y + mPoints[mid_x_idx][z2_idx + 1].y) / 4) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        else {
            mPoints[mid_x_idx][z2_idx].y = ((leftBottom.y + mPoints[mid_x_idx][mid_z_idx].y + rightBottom.y) / 3) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        if (x1_idx>0) {
            mPoints[x1_idx][mid_z_idx].y = ((leftTop.y + mPoints[mid_x_idx][mid_z_idx].y +
                leftBottom.y + mPoints[x1_idx - 1][mid_z_idx].y) / 4) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        else {
            mPoints[x1_idx][mid_z_idx].y = ((leftTop.y + mPoints[mid_x_idx][mid_z_idx].y + leftBottom.y) / 3) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        if (x2_idx<mTerrainWidth - 1) {
            mPoints[x2_idx][mid_z_idx].y = ((rightTop.y + mPoints[mid_x_idx][mid_z_idx].y +
                rightBottom.y + mPoints[x2_idx + 1][mid_z_idx].y) / 4) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        else {
            mPoints[x2_idx][mid_z_idx].y = ((rightTop.y + mPoints[mid_x_idx][mid_z_idx].y + rightBottom.y) / 3) + Rand::randFloat(mMinHeight, mMaxHeight);
        }
        iteration++;
        if (iteration < Threshold) {
            mMinHeight *= pow(2.0f, -mRoughFactor);
            mMaxHeight *= pow(2.0f, -mRoughFactor);
            DiaSqrHelper(x1_idx, mid_x_idx, z1_idx, mid_z_idx, iteration, Threshold);
            DiaSqrHelper(mid_x_idx, x2_idx, z1_idx, mid_z_idx, iteration, Threshold);
            DiaSqrHelper(x1_idx, mid_x_idx, mid_z_idx, z2_idx, iteration, Threshold);
            DiaSqrHelper(mid_x_idx, x2_idx, mid_z_idx, z2_idx, iteration, Threshold);
        }
    }
}

void Terrain::iterateByDiamondSquare(int nums)
{
    DiaSqrHelper(0, mTerrainWidth - 1, 0, mTerrainLength - 1, 0, nums);
}

void Terrain::applySmoothingFilter()
{
    /* Rows, left to right */
    for (int x = 1; x < mTerrainWidth; x++)
        for (int z = 0; z < mTerrainLength; z++)
            mPoints[x][z].y = mPoints[x - 1][z].y * (1 - mSmoothFactor) + mPoints[x][z].y * mSmoothFactor;

    /* Rows, right to left*/
    for (int x = mTerrainWidth - 2; x < -1; x--)
        for (int z = 0; z < mTerrainLength; z++)
            mPoints[x][z].y = mPoints[x + 1][z].y * (1 - mSmoothFactor) + mPoints[x][z].y * mSmoothFactor;

    /* Columns, bottom to top */
    for (int x = 0; x < mTerrainWidth; x++)
        for (int z = 1; z < mTerrainLength; z++)
            mPoints[x][z].y = mPoints[x][z - 1].y * (1 - mSmoothFactor) + mPoints[x][z].y * mSmoothFactor;

    /* Columns, top to bottom */
    for (int x = 0; x < mTerrainWidth; x++)
        for (int z = mTerrainLength; z < -1; z--)
            mPoints[x][z].y = mPoints[x][z + 1].y * (1 - mSmoothFactor) + mPoints[x][z].y * mSmoothFactor;
}

void Terrain::computeNormals()
{
    int widThreshold = mTerrainWidth - 1;
    int lenThreshold = mTerrainLength - 1;

    Vec3f n1, n2, n3, n4, n5, n6, n7, n8;
    n1 = n2 = n3 = n4 = n5 = n6 = n7 = n8 = Vec3f::zero();

    for (int i = 0; i<mTerrainWidth; i++) {
        for (int k = 0; k<mTerrainLength; k++) {
            if (k>0 && i>0)
                n1 = cross(mPoints[i - 1][k] - mPoints[i][k], mPoints[i - 1][k - 1] - mPoints[i][k]);
            if (k>0 && i>0)
                n2 = cross(mPoints[i - 1][k - 1] - mPoints[i][k], mPoints[i][k - 1] - mPoints[i][k]);
            if (k>0 && i<widThreshold)
                n3 = cross(mPoints[i][k - 1] - mPoints[i][k], mPoints[i + 1][k - 1] - mPoints[i][k]);
            if (k>0 && i<widThreshold)
                n4 = cross(mPoints[i + 1][k - 1] - mPoints[i][k], mPoints[i + 1][k] - mPoints[i][k]);
            if (k<lenThreshold && i<widThreshold)
                n5 = cross(mPoints[i + 1][k] - mPoints[i][k], mPoints[i + 1][k + 1] - mPoints[i][k]);
            if (k<lenThreshold && i<widThreshold)
                n6 = cross(mPoints[i + 1][k + 1] - mPoints[i][k], mPoints[i][k + 1] - mPoints[i][k]);
            if (k<lenThreshold && i>0)
                n7 = cross(mPoints[i][k + 1] - mPoints[i][k], mPoints[i - 1][k + 1] - mPoints[i][k]);
            if (k<lenThreshold && i>0)
                n8 = cross(mPoints[i - 1][k + 1] - mPoints[i][k], mPoints[i - 1][k] - mPoints[i][k]);

            mNormals[i][k] = (n1 + n2 + n3 + n4 + n5 + n6 + n7 + n8).normalized();
        }
    }
}

Terrain::Terrain(int Z, int X, float minHeight, float maxHeight, float roughness, float smoothy)
{
	mTerrainLength  = Z;
	mTerrainWidth   = X;
	mMinHeight      = minHeight;
	mMaxHeight      = maxHeight;
	mHeightDiffInvVal = 1/(mMaxHeight-mMinHeight);
	mRoughFactor    = roughness;
	mSmoothFactor   = smoothy;

	mPoints     = new Vec3f*[mTerrainWidth];
	mColors     = new ColorA*[mTerrainWidth];
	mNormals    = new Vec3f*[mTerrainWidth];

	for(int i=0; i<mTerrainWidth; i++) {
		mPoints[i] = new Vec3f[mTerrainLength];
		mColors[i] = new ColorA[mTerrainLength];
		mNormals[i] = new Vec3f[mTerrainLength];
	}
    /* reset the static random generator seed */
    Rand::randomize();
}

void Terrain::prepareTerrain(const char *heightmap, const char* colormap)
{
	const Surface8u heightXZ( loadImage( loadFile( heightmap ) ) );
	const Surface8u colorXZ( loadImage( loadFile( colormap ) ) );
	/* Assuming alpha channel availability */
	for(int i=0; i<mTerrainWidth; i++) {
		for(int k=0; k<mTerrainLength; k++) {
			mPoints[i][k].x = -mTerrainWidth/2.0f + 2.0f*i;
			mPoints[i][k].y = (*heightXZ.getDataRed(Vec2i(i,k)))-200.0f;
			mPoints[i][k].z = -mTerrainLength/2.0f + 2.0f*k;
			mColors[i][k] = colorXZ.getPixel(Vec2i(i,k));
		}
	}
	/* smooth the generated terrain */
	for(int i=0;i<7;++i)
        applySmoothingFilter();
	computeNormals();
}

void Terrain::prepareTerrain()
{
	int mid_i = mTerrainWidth/2;
	int mid_k = mTerrainLength/2;
	for(int i=0; i<mTerrainWidth; i++) {
		for(int k=0; k<mTerrainLength; k++) {
			mPoints[i][k].x = -mTerrainWidth/2.0f + 3.0f*i;
			mPoints[i][k].y = Rand::randFloat(mMinHeight,mMaxHeight);
			mPoints[i][k].z = -mTerrainLength/2.0f + 3.0f*k;
			mColors[i][k]   = getColor(mPoints[i][k].y);
		}
	}
	/* run iterations to generate a finer mesh mPoints */
	iterateByDiamondSquare(9);
	/* smooth the generated terrain */
	for(int i=0;i<9;++i)
        applySmoothingFilter();
	computeNormals();
}

void Terrain::renderTerrain(bool lighting)
{
    int widThreshold = mTerrainWidth - 1;
    int lenThreshold = mTerrainLength - 1;
    if (lighting) {
        glEnable(GL_COLOR_MATERIAL);
        for (int i = 0; i<widThreshold; i++)
        {
            glBegin(GL_TRIANGLE_STRIP);
            for (int k = 0; k<lenThreshold; k++)
            {
                glColor3f(mColors[i][k].r, mColors[i][k].g, mColors[i][k].b);
                glNormal3f(mNormals[i][k].x, mNormals[i][k].y, mNormals[i][k].z);
                glVertex3f(mPoints[i][k].x, mPoints[i][k].y, mPoints[i][k].z);
                glColor3f(mColors[i + 1][k].r, mColors[i + 1][k].g, mColors[i + 1][k].b);
                glNormal3f(mNormals[i + 1][k].x, mNormals[i + 1][k].y, mNormals[i + 1][k].z);
                glVertex3f(mPoints[i + 1][k].x, mPoints[i + 1][k].y, mPoints[i + 1][k].z);
            }
            glEnd();
        }
        glDisable(GL_COLOR_MATERIAL);
    }
    else {
        for (int i = 0; i<widThreshold; i++)
        {
            glBegin(GL_TRIANGLE_STRIP);
            for (int k = 0; k<lenThreshold; k++)
            {
                glColor3f(mColors[i][k].r, mColors[i][k].g, mColors[i][k].b);
                glVertex3f(mPoints[i][k].x, mPoints[i][k].y, mPoints[i][k].z);
                glColor3f(mColors[i + 1][k].r, mColors[i + 1][k].g, mColors[i + 1][k].b);
                glVertex3f(mPoints[i + 1][k].x, mPoints[i + 1][k].y, mPoints[i + 1][k].z);
            }
            glEnd();
        }
    }
}
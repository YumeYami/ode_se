/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#include "icosahedron_geom.h"
//#include "volumeIntegration/volInt.c"

using namespace std;

//<---- Convex Object
dReal planes[] = // planes for a cube, these should coincide with the face array
{
	1.0f, 0.0f, 0.0f, 0.25f,
	0.0f, 1.0f, 0.0f, 0.25f,
	0.0f, 0.0f, 1.0f, 0.25f,
	-1.0f, 0.0f, 0.0f, 0.25f,
	0.0f, -1.0f, 0.0f, 0.25f,
	0.0f, 0.0f, -1.0f, 0.25f
	/*
	1.0f ,0.0f ,0.0f ,2.0f,
	0.0f ,1.0f ,0.0f ,1.0f,
	0.0f ,0.0f ,1.0f ,1.0f,
	0.0f ,0.0f ,-1.0f,1.0f,
	0.0f ,-1.0f,0.0f ,1.0f,
	-1.0f,0.0f ,0.0f ,0.0f
	*/
};
const unsigned int planecount = 6;

dReal points[] = // points for a cube
{
	0.25f, 0.25f, 0.25f,  //  point 0
	-0.25f, 0.25f, 0.25f, //  point 1

	0.25f, -0.25f, 0.25f, //  point 2
	-0.25f, -0.25f, 0.25f,//  point 3

	0.25f, 0.25f, -0.25f, //  point 4
	-0.25f, 0.25f, -0.25f,//  point 5

	0.25f, -0.25f, -0.25f,//  point 6
	-0.25f, -0.25f, -0.25f,// point 7 
};
const unsigned int pointcount = 8;
unsigned int polygons[] = //Polygons for a cube (6 squares)
{
	4, 0, 2, 6, 4, // positive X
	4, 1, 0, 4, 5, // positive Y
	4, 0, 1, 3, 2, // positive Z
	4, 3, 1, 5, 7, // negative X 
	4, 2, 3, 7, 6, // negative Y
	4, 5, 4, 6, 7, // negative Z
};
dVector3 zeroPos = {0, 0, 0};
const double zeroRot[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
//----> Convex Object

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#endif

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

// some constants
#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects

#define GPB 3			// maximum number of geometries per body
//#define PART_NUM 3
#define PART_NUM 2
#define MAX_CONTACTS 8          // maximum number of contact points per body
#define MAX_FEEDBACKNUM 20
#define GRAVITY         REAL(0.5)
#define USE_GEOM_OFFSET 1

/// base const
#define SCALING_X 0.6
#define SCALING_Y 0.6
#define SCALING_Z 0.4

/// input const
#define MODEL_SCALING_FACTOR (1.0/10.0)
#define DENSITY_MODEL_FACE	1.380386254
#define DENSITY_MODEL		0.208269525*DENSITY_MODEL_FACE * 0

#define DENSITY_BASE_FACE	1.380386254
#define DENSITY_BASE		1.380386254

#define MODEL_THICK			1.130922896*MODEL_SCALING_FACTOR
/// iron 110g/100 = 
///		0.00820489234215678779661524500649 g/mm^3
/// PLA = 
///		0.0013753128593654873841574781844 g/mm^3
/// iron/PLA = 5.9658369994025843958125135902092

/// calculation const
#define BASE_HEIGHT_OFFSET		2.0*MODEL_SCALING_FACTOR
#define BASE_ERROR_OFFSET		1.0*MODEL_SCALING_FACTOR
#define MIN_BINARY_SEARCH 0.0001
#define OBJECT_HEIGHT_FROM_MODEL_BASE -0.2

//#define TILT_ANGLE (1.1)
#define START_HEIGHT 4.0
//#define TILT_ANGLE (M_PI_4 + M_PI/4)
#define TILT_ANGLE 75.0/180*M_PI
#define TILT_ANGLE_INIT (M_PI)

/// file reading const
#define MAX_VERTEX 9999
#define MAX_INDEX 9999
#define MODEL_FILE "mons1.2.stl"
#define BASE_MESH_FILE "sphere_cut.stl"
#define BASE_WEIGHT_FILE "cube.stl"
#define READ_CLOCKWISE 1

// dynamics and collision objects

struct MyObject {
	dBodyID body;			// the body
	dGeomID geom[GPB];		// geometries representing this body
};

static int num = 0;		// number of objects in simulation
static int nextobj = 0;		// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];
static dJointGroupID contactgroup;
static int selected = -1;	// selected object
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int random_pos = 1;	// drop objects from random position?
static int write_world = 0;
static int show_body = 0;

struct MyFeedback {
	dJointFeedback fb;
	bool first;
};
static int doFeedback = 0;
static MyFeedback feedbacks[MAX_FEEDBACKNUM];
static int fbnum = 0;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	int i;
	// if (o1->body && o2->body) return;

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if ( b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact) ) return;

	dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
	for ( i = 0; i < MAX_CONTACTS; i++ ) {
		contact[i].surface.mode = dContactBounce | dContactSoftCFM;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = 0.1;
		contact[i].surface.bounce_vel = 0.1;
		contact[i].surface.soft_cfm = 0.01;
	}
	if ( int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
		sizeof(dContact)) ) {
		dMatrix3 RI;
		dRSetIdentity(RI);
		const dReal ss[3] = {0.02, 0.02, 0.02};
		for ( i = 0; i < numc; i++ ) {
			dJointID c = dJointCreateContact(world, contactgroup, contact + i);
			dJointAttach(c, b1, b2);
			if ( show_contacts ) dsDrawBox(contact[i].geom.pos, RI, ss);

			if ( doFeedback && (b1 == obj[selected].body || b2 == obj[selected].body) ) {
				if ( fbnum < MAX_FEEDBACKNUM ) {
					feedbacks[fbnum].first = b1 == obj[selected].body;
					dJointSetFeedback(c, &feedbacks[fbnum++].fb);
				}
				else fbnum++;
			}
		}
	}
}


// start simulation - set viewpoint

static void start() {
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {10.0f, 0.0f, 5.0f};
	static float hpr[3] = {180.000f, -17.0000f, 0.0000f};
	dsSetViewpoint(xyz, hpr);
	printf("To drop another object, press:\n");
	printf("   b for box.\n");
	printf("   s for sphere.\n");
	printf("   c for capsule.\n");
	printf("   y for cylinder.\n");
	printf("   v for a convex object.\n");
	printf("   x for a composite object.\n");
	printf("To select an object, press space.\n");
	printf("To disable the selected object, press d.\n");
	printf("To enable the selected object, press e.\n");
	printf("To dump transformation data for the selected object, press p.\n");
	printf("To toggle showing the geom AABBs, press a.\n");
	printf("To toggle showing the contact points, press t.\n");
	printf("To toggle dropping from random position/orientation, press r.\n");
	printf("To save the current state to 'state.dif', press 1.\n");
	printf("To show joint feedbacks of selected object, press f.\n");
}


char locase(char c) {
	if ( c >= 'A' && c <= 'Z' ) return c - ('a' - 'A');
	else return c;
}

dGeomID TriMesh1;
dGeomID TriMesh2;
dGeomID TriMesh3;
static dTriMeshDataID TriData1, TriData2, TriData3;  // reusable static trimesh data
int VertexCount0 = 0, IndexCount0 = 0, VertexCount1 = 0, IndexCount1 = 0, VertexCount2 = 0, IndexCount2 = 0;
dVector3 Vertices0[MAX_VERTEX];
dVector3 Vertices1[MAX_VERTEX];
dVector3 Vertices2[MAX_VERTEX];
int Indices0[MAX_INDEX];
int Indices1[MAX_INDEX];
int Indices2[MAX_INDEX];

void stlLoad(string fileName, int &VertexCount, int &IndexCount, int* Indices, dVector3* Vertices, int reading_method) {
	cout << "\nstart loading " << fileName << "\n";
	/// build trimesh data
	long i = 0;
	ifstream file;
	file.open(fileName);
	if ( file.is_open() ) {

		string line;
		getline(file, line);
		//cout << line;
		istringstream iss(line);
		string word;
		iss >> word;
		if ( word != "facet" ) {
			getline(file, line);
			istringstream iss2(line);
			iss2 >> word;
		}
		while ( word == "facet" ) {
			getline(file, line);
			getline(file, line);
			istringstream iss3(line);
			iss3 >> word >> Vertices[i][0] >> Vertices[i][1] >> Vertices[i][2];
			if ( reading_method == 1 ) {
				getline(file, line);
				istringstream iss4(line);
				iss4 >> word >> Vertices[i + 2][0] >> Vertices[i + 2][1] >> Vertices[i + 2][2];
				getline(file, line);
				istringstream iss5(line);
				iss5 >> word >> Vertices[i + 1][0] >> Vertices[i + 1][1] >> Vertices[i + 1][2];
			}
			else {
				getline(file, line);
				istringstream iss4(line);
				iss4 >> word >> Vertices[i + 1][0] >> Vertices[i + 1][1] >> Vertices[i + 1][2];
				getline(file, line);
				istringstream iss5(line);
				iss5 >> word >> Vertices[i + 2][0] >> Vertices[i + 2][1] >> Vertices[i + 2][2];
			}
			getline(file, line);
			getline(file, line);
			getline(file, line);
			istringstream iss6(line);
			iss6 >> word;

			for ( int j = 0; j < 3; j++ ) {
				Indices[IndexCount] = IndexCount;
				IndexCount++;
				//cout << "vertex " << i + j << ": " << Vertices[i + j][0] << "\t" << Vertices[i + j][1] << "\t" << Vertices[i + j][2] << "\n";
			}
			i += 3;
			if ( IndexCount >= MAX_INDEX - 9 ) {
				cout << "vertex reach max\n";
				break;
			}
		}
		VertexCount = i;
		//cout << "vertex: " << VertexCount << "\n index: " << IndexCount << "\n";
	}
	file.close();
}

void scaleMesh(double x, double y, double z, dVector3* Vertices, int VertexCount) {
	for ( int i = 0; i < VertexCount; i++ ) {
		Vertices[i][2] *= x;
		Vertices[i][1] *= y;
		Vertices[i][0] *= z;
	}
}

void calModelProperty(double &minHeight, double &minRadius, dMass m2, int VertexCount, int IndexCount, dVector3* Vertices, int* Indices) {
	minHeight = Vertices[0][2];
	for ( int i = 1; i < VertexCount; i++ ) {
		if ( Vertices[i][2] < minHeight ) {
			minHeight = Vertices[i][2];
		}
	}
	double cmx = m2.c[0], cmy = m2.c[1];
	minRadius = 0;
	for ( int i = 0; i < VertexCount; i++ ) {
		if ( Vertices[i][2] < minHeight + BASE_HEIGHT_OFFSET ) {
			double radius = SQR(Vertices[i][0] - cmx) + SQR(Vertices[i][1] - cmy);
			if ( radius > minRadius ) {
				minRadius = radius;
			}
		}
	}
	minRadius = sqrt(minRadius);
	// 	minHeight += m2.c[2];
}

void calTrimeshFaceMass(dMass &mFace, dReal modelThickness, dReal densityModelFace, int VertexCount, int IndexCount, dVector3* Vertices, int* Indices) {
	mFace.setZero();
	/// cal mass and cm
	cout << "vertex count: " << VertexCount << "\n";
	for ( int i = 0; i < VertexCount; i += 3 ) {
		dMass tmp;
		tmp.setZero();
		dVector3 *va = &Vertices[i];
		dVector3 *vb = &Vertices[i + 1];
		dVector3 *vc = &Vertices[i + 2];
		dVector3 v1 = {
			*va[0] - *vb[0],
			*va[1] - *vb[1],
			*va[2] - *vb[2],
			*va[3] - *vb[3]};
		dVector3 v2 = {
			*va[0] - *vc[0],
			*va[1] - *vc[1],
			*va[2] - *vc[2],
			*va[3] - *vc[3]};
		dVector3 vcross = {
			v1[1] * v2[2] - v1[2] * v2[1],
			v1[2] * v2[0] - v1[0] * v2[2],
			v1[0] * v2[1] - v1[1] * v2[0],
			0};
		//cout << "vcross: " << vcross[0] << " " << vcross[1] << " " << vcross[2] << "\n";
		dReal mass = (0.5 * sqrt(vcross[0] * vcross[0] + vcross[1] * vcross[1] + vcross[2] * vcross[2]) * modelThickness * densityModelFace);
		if ( mass <= 0 ) {
			continue;
		}
		dVector3 cm = {
			*va[0] + (v1[0] + v2[0]) / 2,
			*va[1] + (v1[1] + v2[1]) / 2,
			*va[2] + (v1[2] + v2[2]) / 2,
			0};
		dReal i11 = mass * (SQR(cm[1]) + SQR(cm[2]));
		dReal i22 = mass * (SQR(cm[0]) + SQR(cm[2]));
		dReal i33 = mass * (SQR(cm[1]) + SQR(cm[0]));
		// 		dReal i12 = mass * (SQR(cm[2]) + SQR(cm[0] - cm[1]) / 2);
		// 		dReal i13 = mass * (SQR(cm[1]) + SQR(cm[0] - cm[2]) / 2);
		// 		dReal i23 = mass * (SQR(cm[0]) + SQR(cm[2] - cm[1]) / 2);
		//cout << "I " << i11 << " " << i22 << " " << i33 << " " << i12 << " " << i13 << " " << i23 << "\n";
		// 		if ( i11 <= 0 ) {
		// 			cout << "i0 inertia error\n";
		// 		}
		// 		if ( i22 <= 0 ) {
		// 			cout << "i1 inertia error\n";
		// 		}
		// 		if ( i33 <= 0 ) {
		// 			cout << "i2 inertia error\n";
		// 		}
		dMassSetParameters(&tmp, abs(mass), cm[0], cm[1], cm[2], i11, i22, i33, 0, 0, 0);
		//cout << "cm: " << cm[0] << " " << cm[1] << " " << cm[2] << "\n\n";
		dMassAdd(&mFace, &tmp);
		//cout << "mass: " << mass << "\n";
	}
}

void binarySearchBaseSize(double &baseHeight, double tiltAngle, double minRadius, double modelMass, double modelCmHeight) {
	if ( tiltAngle >= M_PI_2 ) {
		return;
	}
	modelCmHeight = abs(modelCmHeight);
	double minB = BASE_HEIGHT_OFFSET, maxB = baseHeight;
	double min_slope = sqrt(SQR((modelMass + 2.0 / 3 * M_PI*minRadius*minRadius*minB*DENSITY_BASE) * (SQR(minRadius) - SQR(minB)) / (modelMass*modelCmHeight - M_PI / 4 * SQR(minRadius)*SQR(minB)*DENSITY_BASE)) - SQR(minB)) / minRadius;
	cout << "min slope " << min_slope << "\n";
	cout << "min tilt angle " << atan(min_slope) << " \t" << atan(min_slope) * 180 / M_PI << "\n\n";
	if ( atan(min_slope) > tiltAngle ) {
		cout << "tilt angle must be more than " << min_slope << "\n";
		return;
	}
	maxB = baseHeight;
	minB = maxB;
	double m2 = 2.0 / 3 * M_PI*SQR(minRadius)*minB*DENSITY_BASE;
	double h = (modelMass*modelCmHeight - m2 * 3 / 8 * minB);
	double til = sqrt(SQR((SQR(minRadius) - SQR(minB)) / h) - SQR(minB)) / minRadius;
	double til_rad = atan(til);
	while ( til_rad > tiltAngle ) {
		minB -= 0.02;
		m2 = 2.0 / 3 * M_PI*SQR(minRadius)*minB*DENSITY_BASE;
		h = (modelMass*modelCmHeight - m2 * 3 / 8 * minB);
		til = sqrt(SQR((SQR(minRadius) - SQR(minB)) / h) - SQR(minB)) / minRadius;
		til_rad = atan(til);
		cout << "til_rad " << til_rad << "\n";
	}
	while ( abs(minB - maxB) >= MIN_BINARY_SEARCH ) {
		double tmpH = (minB + maxB) / 2;
		m2 = 2.0 / 3 * M_PI*SQR(minRadius)*tmpH*DENSITY_BASE;
		h = (modelMass*modelCmHeight - m2 * 3 / 8 * tmpH);
		til = sqrt(SQR((SQR(minRadius) - SQR(tmpH)) / h) - SQR(tmpH)) / minRadius;
		til_rad = atan(til);
		cout << "min: " << minB << " tmpH: " << tmpH << " max: " << maxB << "\n";
		if ( til_rad != til_rad ) {
			cout << "error---------------------------------------\n";
		}
		else if ( til_rad > tiltAngle ) {
			maxB = tmpH;
		}
		else if ( til_rad < tiltAngle ) {
			minB = tmpH;
		}
		else {
			break;
		}
		cout << "input: " << tiltAngle << " tilt: " << atan(til) << "\n";
	}
	baseHeight = maxB;

	/// debugging section
	double bh = 0.01;
	while ( bh < minRadius ) {
		m2 = 2.0 / 3 * M_PI*SQR(minRadius)*bh*DENSITY_BASE;
		h = (modelMass*modelCmHeight - m2 * 3 / 8 * bh);
		til = sqrt(SQR((SQR(minRadius) - SQR(bh)) / h) - SQR(bh)) / minRadius;
		cout << "bh: " << bh << " m2: " << m2 << " h: " << h << " til: " << til << "\n";
		//cout << "bh: " << bh << " \ttan: " << til << " \tangle: " << atan(til) << " \tangle(d): " << atan(til) * 180 / M_PI << "\n";
		bh += 0.02;
	}
}

static void command(int cmd) {
	size_t i;
	int j, k;
	dReal sides[3];
	dMass m;
	int setBody;

	cmd = locase(cmd);
	if ( cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y' || cmd == 'v' || cmd == 'm' || cmd == 'n' || cmd == 'k' || cmd == 'l' || cmd == 'z' || cmd == 'h' ) {
		setBody = 0;
		if ( num < NUM ) {
			i = num;
			num++;
		}
		else {
			i = nextobj;
			nextobj++;
			if ( nextobj >= num ) nextobj = 0;

			// destroy the body and geoms for slot i
			dBodyDestroy(obj[i].body);
			for ( k = 0; k < GPB; k++ ) {
				if ( obj[i].geom[k] ) dGeomDestroy(obj[i].geom[k]);
			}
			memset(&obj[i], 0, sizeof(obj[i]));
		}

		obj[i].body = dBodyCreate(world);
		for ( k = 0; k < 3; k++ ) sides[k] = dRandReal()*0.5 + 0.1;

		dMatrix3 R;
		if ( random_pos ) {
			dBodySetPosition(obj[i].body, dRandReal() * 2, dRandReal() * 2, START_HEIGHT);
			//dRFromAxisAndAngle(R, dRandReal()*2.0 - 1.0, dRandReal()*2.0 - 1.0, dRandReal()*2.0 - 1.0, dRandReal()*10.0 - 5.0);
			dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, TILT_ANGLE);
		}
		else {
			dReal maxheight = 0;
			for ( k = 0; k<num; k++ ) {
				const dReal *pos = dBodyGetPosition(obj[k].body);
				if ( pos[2] > maxheight ) maxheight = pos[2];
			}
			dBodySetPosition(obj[i].body, 0, 0, maxheight + 1);
			dRSetIdentity(R);
			//dRFromAxisAndAngle (R,0,0,1,/*dRandReal()*10.0-5.0*/0);
		}
		dBodySetRotation(obj[i].body, R);
		dBodySetData(obj[i].body, (void*)i);

		if ( cmd == 'b' ) {
			dMassSetBox(&m, DENSITY, sides[0], sides[1], sides[2]);
			obj[i].geom[0] = dCreateBox(space, sides[0], sides[1], sides[2]);
		}
		else if ( cmd == 'c' ) {
			sides[0] *= 0.5;
			dMassSetCapsule(&m, DENSITY, 3, sides[0], sides[1]);
			obj[i].geom[0] = dCreateCapsule(space, sides[0], sides[1]);
		}
		//<---- Convex Object    
		else if ( cmd == 'v' ) {
			dMassSetBox(&m, DENSITY, 0.25, 0.25, 0.25);
#if 0
			obj[i].geom[0] = dCreateConvex(space,
										   planes,
										   planecount,
										   points,
										   pointcount,
										   polygons);
#else
			obj[i].geom[0] = dCreateConvex(space,
										   Sphere_planes,
										   Sphere_planecount,
										   Sphere_points,
										   Sphere_pointcount,
										   Sphere_polygons);
#endif
		}
		//----> Convex Object
		else if ( cmd == 'y' ) {
			dMassSetCylinder(&m, DENSITY, 3, sides[0], sides[1]);
			obj[i].geom[0] = dCreateCylinder(space, sides[0], sides[1]);
		}
		else if ( cmd == 's' ) {
			sides[0] *= 0.5;
			dMassSetSphere(&m, DENSITY, sides[0]);
			obj[i].geom[0] = dCreateSphere(space, sides[0]);
		}
		else if ( cmd == 'x' && USE_GEOM_OFFSET ) {
			setBody = 1;
			// start accumulating masses for the encapsulated geometries
			dMass m2;
			dMassSetZero(&m);

			dReal dpos[GPB][3];	// delta-positions for encapsulated geometries
			dMatrix3 drot[GPB];

			// set random delta positions
			for ( j = 0; j < GPB; j++ ) {
				for ( k = 0; k < 3; k++ ) dpos[j][k] = dRandReal()*0.3 - 0.15;
			}

			for ( k = 0; k < GPB; k++ ) {
				if ( k == 0 ) {
					dReal radius = dRandReal()*0.25 + 0.05;
					obj[i].geom[k] = dCreateSphere(space, radius);
					dMassSetSphere(&m2, DENSITY, radius);
				}
				else if ( k == 1 ) {
					obj[i].geom[k] = dCreateBox(space, sides[0], sides[1], sides[2]);
					dMassSetBox(&m2, DENSITY, sides[0], sides[1], sides[2]);
				}
				else {
					dReal radius = dRandReal()*0.1 + 0.05;
					dReal length = dRandReal()*1.0 + 0.1;
					obj[i].geom[k] = dCreateCapsule(space, radius, length);
					dMassSetCapsule(&m2, DENSITY, 3, radius, length);
				}

				dRFromAxisAndAngle(drot[k], dRandReal()*2.0 - 1.0, dRandReal()*2.0 - 1.0,
								   dRandReal()*2.0 - 1.0, dRandReal()*10.0 - 5.0);
				dMassRotate(&m2, drot[k]);

				dMassTranslate(&m2, dpos[k][0], dpos[k][1], dpos[k][2]);

				// add to the total mass
				dMassAdd(&m, &m2);

			}
			for ( k = 0; k < GPB; k++ ) {
				dGeomSetBody(obj[i].geom[k], obj[i].body);
				dGeomSetOffsetPosition(obj[i].geom[k],
									   dpos[k][0] - m.c[0],
									   dpos[k][1] - m.c[1],
									   dpos[k][2] - m.c[2]);
				dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
			}
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(obj[i].body, &m);
		}

		else if ( cmd == 'm' ) {
			TriData1 = dGeomTriMeshDataCreate();
			stlLoad(MODEL_FILE, VertexCount0, IndexCount0, &Indices0[0], &Vertices0[0], 1);
			dGeomTriMeshDataBuildSimple(TriData1, (dReal*)Vertices0, VertexCount0, (dTriIndex*)Indices0, IndexCount0);
			obj[i].geom[0] = dCreateTriMesh(space, TriData1, 0, 0, 0);
			dGeomSetData(obj[i].geom[0], TriData1);
			dMassSetTrimesh(&m, DENSITY, obj[i].geom[0]);
			cout << "model mass: " << m.c[0] << " " << m.c[1] << " " << m.c[2] << "\n";
			//dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);

		}
		else if ( cmd == 'n' ) {
			TriData2 = dGeomTriMeshDataCreate();
			stlLoad(BASE_MESH_FILE, VertexCount1, IndexCount1, &Indices1[0], &Vertices1[0], 0);
			dGeomTriMeshDataBuildSimple(TriData2, (dReal*)Vertices1, VertexCount1, (dTriIndex*)Indices1, IndexCount1);
			obj[i].geom[0] = dCreateTriMesh(space, TriData2, 0, 0, 0);
			dGeomSetData(obj[i].geom[0], TriData2);
			dMassSetTrimesh(&m, DENSITY, obj[i].geom[0]);
			cout << "base mass: " << m.c[0] << " " << m.c[1] << " " << m.c[2] << "\n";
			//dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
		}
		else if ( cmd == 'k' ) {
			TriData3 = dGeomTriMeshDataCreate();
			stlLoad(BASE_WEIGHT_FILE, VertexCount2, IndexCount2, &Indices2[0], &Vertices2[0], 1);
			dGeomTriMeshDataBuildSimple(TriData3, (dReal*)Vertices2, VertexCount2, (dTriIndex*)Indices2, IndexCount2);
			obj[i].geom[0] = dCreateTriMesh(space, TriData3, 0, 0, 0);
			dGeomSetData(obj[i].geom[0], TriData3);
			dMassSetTrimesh(&m, DENSITY, obj[i].geom[0]);
			cout << "base weight mass: " << m.c[0] << " " << m.c[1] << " " << m.c[2] << "\n";
			//dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dGeomSetPosition(obj[i].geom[0], -m.c[0], -m.c[1], -m.c[2]);
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
		}

		//////////////////////////////////////////////////////////////////////////
		/// create model object ///
		else if ( cmd == 'l' ) {
			setBody = 1;
			// start accumulating masses for the encapsulated geometries
			dMass m2;
			dMassSetZero(&m);

			dReal dpos[GPB][3];	// delta-positions for encapsulated geometries
			dMatrix3 drot[GPB];

			// set random delta positions
			// 			for ( j = 0; j < PART_NUM; j++ ) {
			// 				for ( k = 0; k < 3; k++ ) dpos[j][k] = dRandReal()*0.3 - 0.15;
			// 			}
			for ( int i = 0; i < GPB; i++ ) {
				dpos[i][0] = 0;
				dpos[i][1] = 0;
			}
			dpos[0][2] = 0;
			dpos[1][2] = -0.5;
			dpos[2][2] = -0.5;
			double modelCmHeight = 0;
			double minRadius = 1;
			double modelMass = 0;
			///--------------------------------------------------------------------------------------------------------- add model
			for ( k = 0; k < PART_NUM; k++ ) {
				if ( k == 0 ) {/// add model
					TriData1 = dGeomTriMeshDataCreate();
					/// load and scale model 
					stlLoad(MODEL_FILE, VertexCount0, IndexCount0, &Indices0[0], &Vertices0[0], 1);
					scaleMesh(MODEL_SCALING_FACTOR, MODEL_SCALING_FACTOR, MODEL_SCALING_FACTOR, Vertices0, VertexCount0);

					/// build mesh data
					dGeomTriMeshDataBuildSimple(TriData1, (dReal*)Vertices0, VertexCount0, (dTriIndex*)Indices0, IndexCount0);
					obj[i].geom[0] = dCreateTriMesh(space, TriData1, 0, 0, 0);
					dGeomSetData(obj[i].geom[0], TriData1);
					if ( DENSITY_MODEL != 0 ) {
						dMassSetTrimesh(&m2, DENSITY_MODEL_FACE, obj[i].geom[0]);
					}
					/// calculate triangle face property
					dMass mFace;
					calTrimeshFaceMass(mFace, MODEL_THICK, DENSITY_MODEL_FACE, VertexCount0, IndexCount0, Vertices0, Indices0);
					cout << "\nface mass: " << mFace.mass << "\n";
					cout << "model mass: " << m2.mass << "\n";
					dMassAdd(&m2, &mFace);
					/// calculate model property
					calModelProperty(modelCmHeight, minRadius, m2, VertexCount0, IndexCount0, Vertices0, Indices0); /// calculate model for min height and radius(contact area)
					dpos[1][0] = m2.c[0];
					dpos[1][1] = m2.c[1];
					dpos[1][2] = modelCmHeight;
					dpos[2][0] = m2.c[0];
					dpos[2][1] = m2.c[1];
					dpos[2][2] = modelCmHeight;
					cout << "minHeight: " << modelCmHeight << " minRadius: " << minRadius << "\n";
					modelMass = m2.mass;
					dGeomSetPosition(obj[i].geom[0], m2.c[0], m2.c[1], m.c[2]);
					//dMassTranslate(&m2, -m2.c[0], -m2.c[1], -m2.c[2]);
					dRFromAxisAndAngle(drot[k], 1, 1, 1, 0);
				}
				else if ( k == 1 ) {/// add base
					TriData2 = dGeomTriMeshDataCreate();

					/// load and scale base size
					stlLoad(BASE_MESH_FILE, VertexCount1, IndexCount1, &Indices1[0], &Vertices1[0], 0);
					double minBH = 0, minBR = 0;

					/// calculate case total cm == 0
					double baseHeight = 2.0 / minRadius * sqrt(modelMass * abs(modelCmHeight) / DENSITY_BASE / M_PI);
					cout << "minRadius " << minRadius << "\tmodelMass " << modelMass << "\nmodelCmHeight " << modelCmHeight << "\tmaxBaseHeight " << baseHeight << "\n";
					if ( baseHeight >= minRadius - BASE_HEIGHT_OFFSET ) {
						cout << "cannot use these density and angle to make rocking base\n";
					}
					else {
						binarySearchBaseSize(baseHeight, TILT_ANGLE, minRadius, modelMass, modelCmHeight);
						cout << "minBaseHeight: " << baseHeight << "\n";
					}
					/// scaling the base size
					{
						//baseHeight += BASE_ERROR_OFFSET * 4;
						scaleMesh(minRadius, minRadius, baseHeight, Vertices1, VertexCount1);
					}
					dGeomTriMeshDataBuildSimple(TriData2, (dReal*)Vertices1, VertexCount1, (dTriIndex*)Indices1, IndexCount1);
					obj[i].geom[1] = dCreateTriMesh(space, TriData2, 0, 0, 0);
					dGeomSetData(obj[i].geom[1], TriData2);

					/// calculate mass property of basement
					{
						dReal basemass = 2.0 / 3 * M_PI*SQR(minRadius)*baseHeight * DENSITY_BASE;
						dReal gz = 3.0 / 8 * baseHeight;
						cout << "base mass: " << basemass << "\n";
						dReal i1 = basemass / 5.0*(SQR(minRadius) + SQR(baseHeight));
						dReal i11 = i1 - basemass*SQR(gz);
						dReal i22 = i11;
						dReal i33 = basemass / 5.0 * 2.0 * SQR(minRadius);
						m2.setParameters(basemass, -gz, 0.0, 0.0, i33, i22, i11, 0, 0, 0);
						// 						cout << "cm: " << m2.c[0] << " " << m2.c[1] << " " << m2.c[2] << "\n";
						// 						cout << "I: ";
						// 						for ( int i = 0; i < 12; i++ ) {
						// 							cout << m2.I[i] << " ";
						// 						}
					}
					dGeomSetPosition(obj[i].geom[1], m2.c[0], m2.c[1], m.c[2]);
					//dMassTranslate(&m2, -m2.c[0], -m2.c[1], -m2.c[2]);
					dRFromAxisAndAngle(drot[k], 0, -1, 0, M_PI_2);
				}
				else if ( k == 2 ) {/// not use
					TriData3 = dGeomTriMeshDataCreate();
					stlLoad(BASE_WEIGHT_FILE, VertexCount2, IndexCount2, &Indices2[0], &Vertices2[0], 1);
					scaleMesh(0.0001, 0.0001, 0.0001, Vertices2, VertexCount2);
					dGeomTriMeshDataBuildSimple(TriData3, (dReal*)Vertices2, VertexCount2, (dTriIndex*)Indices2, IndexCount2);
					obj[i].geom[2] = dCreateTriMesh(space, TriData3, 0, 0, 0);
					dGeomSetData(obj[i].geom[2], TriData3);
					//dMassSetTrimesh(&m2, 0.00001, obj[i].geom[2]);
					m2.setZero();
					//dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
					dGeomSetPosition(obj[i].geom[2], m2.c[0], m2.c[1], m.c[2]);
					//dMassTranslate(&m2, -m2.c[0], -m2.c[1], -m2.c[2]);
					dRFromAxisAndAngle(drot[k], 1, 0, 0, 3.14);
				}
				else {
					cout << "error: ....\n";
				}
				//dRFromAxisAndAngle(drot[k], dRandReal()*2.0 - 1.0, dRandReal()*2.0 - 1.0, dRandReal()*2.0 - 1.0, dRandReal()*10.0 - 5.0);
				//dRFromAxisAndAngle(drot[k], 1, 1, 1, 0);
				dMassRotate(&m2, drot[k]);
				dMassTranslate(&m2, dpos[k][0], dpos[k][1], dpos[k][2]);
				// add to the total mass
				dMassAdd(&m, &m2);
			}
			//cout << "set body:::\n";
			for ( k = 0; k < PART_NUM; k++ ) {
				dGeomSetBody(obj[i].geom[k], obj[i].body);
				dGeomSetOffsetPosition(obj[i].geom[k],
									   dpos[k][0] - m.c[0],
									   dpos[k][1] - m.c[1],
									   dpos[k][2] - m.c[2]);
				dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
			}
			cout << "\nsum mass: " << m.mass << "\n";
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(obj[i].body, &m);
		}
		if ( !setBody )
			for ( k = 0; k < GPB; k++ ) {
				cout << "c6\n";
				if ( obj[i].geom[k] ) dGeomSetBody(obj[i].geom[k], obj[i].body);
			}
		//cout << "c7\n";
		dBodySetMass(obj[i].body, &m);
		//cout << "c8\n";
	}

	if ( cmd == ' ' ) {
		selected++;
		if ( selected >= num ) selected = 0;
		if ( selected < 0 ) selected = 0;
	}
	else if ( cmd == 'd' && selected >= 0 && selected < num ) {
		dBodyDisable(obj[selected].body);
	}
	else if ( cmd == 'e' && selected >= 0 && selected < num ) {
		dBodyEnable(obj[selected].body);
	}
	else if ( cmd == 'a' ) {
		show_aabb ^= 1;
	}
	else if ( cmd == 't' ) {
		show_contacts ^= 1;
	}
	else if ( cmd == 'r' ) {
		random_pos ^= 1;
	}
	else if ( cmd == '1' ) {
		write_world = 1;
	}
	else if ( cmd == 'p'&& selected >= 0 ) {
		const dReal* pos = dGeomGetPosition(obj[selected].geom[0]);
		const dReal* rot = dGeomGetRotation(obj[selected].geom[0]);
		printf("POSITION:\n\t[%f,%f,%f]\n\n", pos[0], pos[1], pos[2]);
		printf("ROTATION:\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\t[%f,%f,%f,%f]\n\n",
			   rot[0], rot[1], rot[2], rot[3],
			   rot[4], rot[5], rot[6], rot[7],
			   rot[8], rot[9], rot[10], rot[11]);
	}
	else if ( cmd == 'f' && selected >= 0 && selected < num ) {
		if ( dBodyIsEnabled(obj[selected].body) )
			doFeedback = 1;
	}

}

// draw a geom
void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb) {
	int i;

	if ( !g ) return;
	if ( !pos ) pos = dGeomGetPosition(g);
	if ( !R ) R = dGeomGetRotation(g);

	int type = dGeomGetClass(g);
	if ( type == dBoxClass ) {
		dVector3 sides;
		dGeomBoxGetLengths(g, sides);
		dsDrawBox(pos, R, sides);
	}
	else if ( type == dSphereClass ) {
		dsDrawSphere(pos, R, dGeomSphereGetRadius(g));
	}
	else if ( type == dCapsuleClass ) {
		dReal radius, length;
		dGeomCapsuleGetParams(g, &radius, &length);
		dsDrawCapsule(pos, R, length, radius);
	}
	//<---- Convex Object
	else if ( type == dConvexClass ) {
#if 0
		dsDrawConvex(pos, R, planes,
					 planecount,
					 points,
					 pointcount,
					 polygons);
#else
		dsDrawConvex(pos, R,
					 Sphere_planes,
					 Sphere_planecount,
					 Sphere_points,
					 Sphere_pointcount,
					 Sphere_polygons);
#endif
	}
	//----> Convex Object
	else if ( type == dCylinderClass ) {
		dReal radius, length;
		dGeomCylinderGetParams(g, &radius, &length);
		dsDrawCylinder(pos, R, length, radius);
	}
	else if ( type == dGeomTransformClass ) {
		dGeomID g2 = dGeomTransformGetGeom(g);
		const dReal *pos2 = dGeomGetPosition(g2);
		const dReal *R2 = dGeomGetRotation(g2);
		dVector3 actual_pos;
		dMatrix3 actual_R;
		dMultiply0_331(actual_pos, R, pos2);
		actual_pos[0] += pos[0];
		actual_pos[1] += pos[1];
		actual_pos[2] += pos[2];
		dMultiply0_333(actual_R, R, R2);
		drawGeom(g2, actual_pos, actual_R, 0);
	}
	else if ( type == dTriMeshClass ) {
		//cout << "draw trimesh\n";
		int tcount = dGeomTriMeshGetTriangleCount(g);
		dVector3 v0, v1, v2;
		for ( int i = 0; i < tcount; i++ ) {
			dGeomTriMeshGetTriangle(g, i, &v0, &v1, &v2);
			dsDrawTriangle(zeroPos, zeroRot, v0, v1, v2, 1);
		}
	}
	if ( show_body ) {
		dBodyID body = dGeomGetBody(g);
		if ( body ) {
			const dReal *bodypos = dBodyGetPosition(body);
			const dReal *bodyr = dBodyGetRotation(body);
			dReal bodySides[3] = {0.1, 0.1, 0.1};
			dsSetColorAlpha(0, 1, 0, 1);
			dsDrawBox(bodypos, bodyr, bodySides);
		}
	}
	if ( show_aabb ) {
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB(g, aabb);
		dVector3 bbpos;
		for ( i = 0; i < 3; i++ ) bbpos[i] = 0.5*(aabb[i * 2] + aabb[i * 2 + 1]);
		dVector3 bbsides;
		for ( i = 0; i < 3; i++ ) bbsides[i] = aabb[i * 2 + 1] - aabb[i * 2];
		dMatrix3 RI;
		dRSetIdentity(RI);
		dsSetColorAlpha(1, 0, 0, 0.5);
		dsDrawBox(bbpos, RI, bbsides);
	}
}

// simulation loop
static void simLoop(int pause) {
	dsSetColor(0, 0, 2);
	dSpaceCollide(space, 0, &nearCallback);
	if ( !pause ) dWorldQuickStep(world, 0.02);

	if ( write_world ) {
		FILE *f = fopen("state.dif", "wt");
		if ( f ) {
			dWorldExportDIF(world, f, "X");
			fclose(f);
		}
		write_world = 0;
	}


	if ( doFeedback ) {
		if ( fbnum > MAX_FEEDBACKNUM )
			printf("joint feedback buffer overflow!\n");
		else {
			dVector3 sum = {0, 0, 0};
			printf("\n");
			for ( int i = 0; i < fbnum; i++ ) {
				dReal* f = feedbacks[i].first ? feedbacks[i].fb.f1 : feedbacks[i].fb.f2;
				printf("%f %f %f\n", f[0], f[1], f[2]);
				sum[0] += f[0];
				sum[1] += f[1];
				sum[2] += f[2];
			}
			printf("Sum: %f %f %f\n", sum[0], sum[1], sum[2]);
			dMass m;
			dBodyGetMass(obj[selected].body, &m);
			printf("Object G=%f\n", GRAVITY*m.mass);
		}
		doFeedback = 0;
		fbnum = 0;
	}

	// remove all contact joints
	dJointGroupEmpty(contactgroup);

	dsSetColor(1, 1, 0);
	dsSetTexture(DS_WOOD);
	for ( int i = 0; i < num; i++ ) {
		for ( int j = 0; j < GPB; j++ ) {
			if ( i == selected ) {
				dsSetColor(0, 0.7, 1);
			}
			else if ( !dBodyIsEnabled(obj[i].body) ) {
				dsSetColor(1, 0.8, 0);
			}
			else {
				dsSetColor(1, 1, 0);
			}
			drawGeom(obj[i].geom[j], 0, 0, show_aabb);
		}
	}
}


int main(int argc, char **argv) {
	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(world, 0, 0, -GRAVITY);
	dWorldSetCFM(world, 1e-5);
	dWorldSetAutoDisableFlag(world, 1);

#if 1

	dWorldSetAutoDisableAverageSamplesCount(world, 10);

#endif

	dWorldSetLinearDamping(world, 0.00001);
	dWorldSetAngularDamping(world, 0.005);
	dWorldSetMaxAngularSpeed(world, 200);

	dWorldSetContactMaxCorrectingVel(world, 0.1);
	dWorldSetContactSurfaceLayer(world, 0.001);
	dCreatePlane(space, 0, 0, 1, 0);
	memset(obj, 0, sizeof(obj));

	dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
	dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
	dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
	// dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
	dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

	// run simulation
	dsSimulationLoop(argc, argv, 352, 288, &fn);

	dThreadingImplementationShutdownProcessing(threading);
	dThreadingFreeThreadPool(pool);
	dWorldSetStepThreadingImplementation(world, NULL, NULL);
	dThreadingFreeImplementation(threading);

	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	return 0;
}

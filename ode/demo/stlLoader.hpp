// /***************************************************************************
// tribarrier.h  -  description
// -------------------
// begin                : Mit Sep 25 13:11:41 CEST 2002
// copyright            : (C) 2002 by Harald Krippel
// email                : harald@hte-develop.net
// ***************************************************************************/
// 
// /***************************************************************************
// *                                                                         *
// *   This program is free software; you can redistribute it and/or modify  *
// *   it under the terms of the GNU General Public License as published by  *
// *   the Free Software Foundation; either version 2 of the License, or     *
// *   (at your option) any later version.                                   *
// *                                                                         *
// ***************************************************************************/
// #ifndef TRIBARRIER_H
// #define TRIBARRIER_H
// 
// /**
// @author Harald Krippel <harald@the-develop.net>
// */
// 
// #include <plib/ssg.h>
// #include <ode/ode.h>
// 
// #define  MAXVERTEX  32000
// #define  MAXINDEX  (32000*3)
// 
// class TriBarrier {
// public:
// 	TriBarrier(void);
// 	void   createTriMesh(dSpaceID odspace, const char *file);
// 	void   update();
// 	void   setTransform(ssgTransform *objtrans, const dReal pos[3], const dReal R[12], dVector3 dxyz);
// 	void   SetPosition();
// 	void   setScale(const float sx, const float sy, const float sz);
// 	void   Print();
// 	void   GetPosition(sgCoord  *bodypos);
// 	~TriBarrier(void);
// 
// 	sgCoord      pos;
// 	ssgTransform *objtrans;
// 	ssgTransform *boxobjtrans;
// 
// 
// protected:
// 	void buildTriMeshData(const char *file);
// 	/* ode */
// 	dTriMeshDataID data;
// 	dGeomID TriMesh;
// 	long VertexCount;
// 	long IndexCount;
// #ifdef dDOUBLE
// 	dVector3 Vertices[MAXVERTEX];
// 	dVector3 VerticesScale[MAXVERTEX];
// #endif
// #ifdef dSINGLE
// 	float Vertices[MAXVERTEX][3];
// 	float VerticesScale[MAXVERTEX][3];
// #endif
// 	int Indices[MAXINDEX];
// 	dReal scale[3];
// };
// 
// #endif
// 

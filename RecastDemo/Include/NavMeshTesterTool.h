//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef NAVMESHTESTERTOOL_H
#define NAVMESHTESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class NavMeshTesterTool : public SampleTool
{
	Sample* m_sample;
	
	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;

	dtQueryFilter m_filter;

	dtStatus m_pathFindStatus;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_FOLLOW,
		TOOLMODE_PATHFIND_STRAIGHT,
		TOOLMODE_PATHFIND_SLICED,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_IN_CIRCLE,
		TOOLMODE_FIND_POLYS_IN_SHAPE,
		TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD,
	};
	
	ToolMode m_toolMode;

	int m_straightPathOptions;
	
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	
	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys;
	Fix16 m_straightPath[MAX_POLYS*3];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];
	int m_nstraightPath;
	Fix16 m_polyPickExt[3];
	Fix16 m_smoothPath[MAX_SMOOTH*3];
	int m_nsmoothPath;
	Fix16 m_queryPoly[4*3];

	static const int MAX_RAND_POINTS = 64;
	Fix16 m_randPoints[MAX_RAND_POINTS*3];
	int m_nrandPoints;
	bool m_randPointsInCircle;
	
	Fix16 m_spos[3];
	Fix16 m_epos[3];
	Fix16 m_hitPos[3];
	Fix16 m_hitNormal[3];
	bool m_hitResult;
	Fix16 m_distanceToWall;
	Fix16 m_neighbourhoodRadius;
	Fix16 m_randomRadius;
	bool m_sposSet;
	bool m_eposSet;

	int m_pathIterNum;
	dtPolyRef m_pathIterPolys[MAX_POLYS]; 
	int m_pathIterPolyCount;
	Fix16 m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];
	
	static const int MAX_STEER_POINTS = 10;
	Fix16 m_steerPoints[MAX_STEER_POINTS*3];
	int m_steerPointCount;
	
public:
	NavMeshTesterTool();

	virtual int type() { return TOOL_NAVMESH_TESTER; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const Fix16* s, const Fix16* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const Fix16 dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

	void recalc();
	void drawAgent(const Fix16* pos, Fix16 r, Fix16 h, Fix16 c, const unsigned int col);
};

#endif // NAVMESHTESTERTOOL_H
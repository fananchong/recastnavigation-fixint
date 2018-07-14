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

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

static const Fix16 Fix16_0 = Fix16(0);
static const Fix16 Fix16_1 = Fix16(1.0f);

/// @par 
/// 
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
/// are marked as unwalkable.
///
/// This method is usually called immediately after the heightfield has been built.
///
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedTimer timer(ctx, RC_TIMER_ERODE_AREA);
	
	unsigned char* dist = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dist)
	{
		ctx->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", chf.spanCount);
		return false;
	}
	
	// Init distance.
	memset(dist, 0xff, sizeof(unsigned char)*chf.spanCount);
	
	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] == RC_NULL_AREA)
				{
					dist[i] = 0;
				}
				else
				{
					const rcCompactSpan& s = chf.spans[i];
					int nc = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int nx = x + rcGetDirOffsetX(dir);
							const int ny = y + rcGetDirOffsetY(dir);
							const int nidx = (int)chf.cells[nx+ny*w].index + rcGetCon(s, dir);
							if (chf.areas[nidx] != RC_NULL_AREA)
							{
								nc++;
							}
						}
					}
					// At least one missing neighbour.
					if (nc != 4)
						dist[i] = 0;
				}
			}
		}
	}
	
	unsigned char nd;
	
	// Pass 1
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (-1,-1)
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (1,-1)
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}
	
	// Pass 2
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (1,1)
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (-1,1)
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}
	
	const unsigned char thr = (unsigned char)(radius*2);
	for (int i = 0; i < chf.spanCount; ++i)
		if (dist[i] < thr)
			chf.areas[i] = RC_NULL_AREA;
	
	rcFree(dist);
	
	return true;
}

static void insertSort(unsigned char* a, const int n)
{
	int i, j;
	for (i = 1; i < n; i++)
	{
		const unsigned char value = a[i];
		for (j = i - 1; j >= 0 && a[j] > value; j--)
			a[j+1] = a[j];
		a[j+1] = value;
	}
}

/// @par
///
/// This filter is usually applied after applying area id's using functions
/// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
/// 
/// @see rcCompactHeightfield
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedTimer timer(ctx, RC_TIMER_MEDIAN_AREA);
	
	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		ctx->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).", chf.spanCount);
		return false;
	}
	
	// Init distance.
	memset(areas, 0xff, sizeof(unsigned char)*chf.spanCount);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA)
				{
					areas[i] = chf.areas[i];
					continue;
				}
				
				unsigned char nei[9];
				for (int j = 0; j < 9; ++j)
					nei[j] = chf.areas[i];
				
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						if (chf.areas[ai] != RC_NULL_AREA)
							nei[dir*2+0] = chf.areas[ai];
						
						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir+1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
							if (chf.areas[ai2] != RC_NULL_AREA)
								nei[dir*2+1] = chf.areas[ai2];
						}
					}
				}
				insertSort(nei, 9);
				areas[i] = nei[4];
			}
		}
	}
	
	memcpy(chf.areas, areas, sizeof(unsigned char)*chf.spanCount);
	
	rcFree(areas);
	
	return true;
}

/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkBoxArea(rcContext* ctx, const Fix16* bmin, const Fix16* bmax, unsigned char areaId,
				   rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_MARK_BOX_AREA);

	int minx = (int)((bmin[0]-chf.bmin[0])/chf.cs);
	int miny = (int)((bmin[1]-chf.bmin[1])/chf.ch);
	int minz = (int)((bmin[2]-chf.bmin[2])/chf.cs);
	int maxx = (int)((bmax[0]-chf.bmin[0])/chf.cs);
	int maxy = (int)((bmax[1]-chf.bmin[1])/chf.ch);
	int maxz = (int)((bmax[2]-chf.bmin[2])/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;

	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					if (chf.areas[i] != RC_NULL_AREA)
						chf.areas[i] = areaId;
				}
			}
		}
	}
}


static int pointInPoly(int nvert, const Fix16* verts, const Fix16* p)
{
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++)
	{
		const Fix16* vi = &verts[i*3];
		const Fix16* vj = &verts[j*3];
		if (((vi[2] > p[2]) != (vj[2] > p[2])) &&
			(p[0] < (vj[0]-vi[0]) * (p[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
	}
	return c;
}

/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// The y-values of the polygon vertices are ignored. So the polygon is effectively 
/// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkConvexPolyArea(rcContext* ctx, const Fix16* verts, const int nverts,
						  const Fix16 hmin, const Fix16 hmax, unsigned char areaId,
						  rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA);

	Fix16 bmin[3], bmax[3];
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < nverts; ++i)
	{
		rcVmin(bmin, &verts[i*3]);
		rcVmax(bmax, &verts[i*3]);
	}
	bmin[1] = hmin;
	bmax[1] = hmax;

	int minx = (int)((bmin[0]-chf.bmin[0])/chf.cs);
	int miny = (int)((bmin[1]-chf.bmin[1])/chf.ch);
	int minz = (int)((bmin[2]-chf.bmin[2])/chf.cs);
	int maxx = (int)((bmax[0]-chf.bmin[0])/chf.cs);
	int maxy = (int)((bmax[1]-chf.bmin[1])/chf.ch);
	int maxz = (int)((bmax[2]-chf.bmin[2])/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;
	
	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	
	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA)
					continue;
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					Fix16 p[3];
					p[0] = chf.bmin[0] + Fix16(x+0.5f)*chf.cs; 
					p[1] = 0;
					p[2] = chf.bmin[2] + Fix16(z+0.5f)*chf.cs;

					if (pointInPoly(nverts, verts, p))
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}
}

int rcOffsetPoly(const Fix16* verts, const int nverts, const Fix16 offset,
				 Fix16* outVerts, const int maxOutVerts)
{
	const Fix16	MITER_LIMIT = 1.20f;

	int n = 0;

	for (int i = 0; i < nverts; i++)
	{
		const int a = (i+nverts-1) % nverts;
		const int b = i;
		const int c = (i+1) % nverts;
		const Fix16* va = &verts[a*3];
		const Fix16* vb = &verts[b*3];
		const Fix16* vc = &verts[c*3];
		Fix16 dx0 = vb[0] - va[0];
		Fix16 dy0 = vb[2] - va[2];
		Fix16 d0 = dx0*dx0 + dy0*dy0;
		if (d0 > 1e-6f)
		{
			d0 = Fix16_1/rcSqrt(d0);
			dx0 *= d0;
			dy0 *= d0;
		}
		Fix16 dx1 = vc[0] - vb[0];
		Fix16 dy1 = vc[2] - vb[2];
		Fix16 d1 = dx1*dx1 + dy1*dy1;
		if (d1 > 1e-6f)
		{
			d1 = Fix16_1/rcSqrt(d1);
			dx1 *= d1;
			dy1 *= d1;
		}
		const Fix16 dlx0 = Fix16_0-dy0;
		const Fix16 dly0 = dx0;
		const Fix16 dlx1 = Fix16_0-dy1;
		const Fix16 dly1 = dx1;
		Fix16 cross = dx1*dy0 - dx0*dy1;
		Fix16 dmx = (dlx0 + dlx1) * 0.5f;
		Fix16 dmy = (dly0 + dly1) * 0.5f;
		Fix16 dmr2 = dmx*dmx + dmy*dmy;
		bool bevel = dmr2 * MITER_LIMIT*MITER_LIMIT < 1.0f;
		if (dmr2 > 1e-6f)
		{
			const Fix16 scale = Fix16_1 / dmr2;
			dmx *= scale;
			dmy *= scale;
		}

		if (bevel && cross < 0.0f)
		{
			if (n+2 >= maxOutVerts)
				return 0;
			Fix16 d = (Fix16_1 - (dx0*dx1 + dy0*dy1))*0.5f;
			outVerts[n*3+0] = vb[0] + (Fix16_0-dlx0+dx0*d)*offset;
			outVerts[n*3+1] = vb[1];
			outVerts[n*3+2] = vb[2] + (Fix16_0-dly0+dy0*d)*offset;
			n++;
			outVerts[n*3+0] = vb[0] + (Fix16_0-dlx1-dx1*d)*offset;
			outVerts[n*3+1] = vb[1];
			outVerts[n*3+2] = vb[2] + (Fix16_0-dly1-dy1*d)*offset;
			n++;
		}
		else
		{
			if (n+1 >= maxOutVerts)
				return 0;
			outVerts[n*3+0] = vb[0] - dmx*offset;
			outVerts[n*3+1] = vb[1];
			outVerts[n*3+2] = vb[2] - dmy*offset;
			n++;
		}
	}
	
	return n;
}


/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkCylinderArea(rcContext* ctx, const Fix16* pos,
						const Fix16 r, const Fix16 h, unsigned char areaId,
						rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_MARK_CYLINDER_AREA);
	
	Fix16 bmin[3], bmax[3];
	bmin[0] = pos[0] - r;
	bmin[1] = pos[1];
	bmin[2] = pos[2] - r;
	bmax[0] = pos[0] + r;
	bmax[1] = pos[1] + h;
	bmax[2] = pos[2] + r;
	const Fix16 r2 = r*r;
	
	int minx = (int)((bmin[0]-chf.bmin[0])/chf.cs);
	int miny = (int)((bmin[1]-chf.bmin[1])/chf.ch);
	int minz = (int)((bmin[2]-chf.bmin[2])/chf.cs);
	int maxx = (int)((bmax[0]-chf.bmin[0])/chf.cs);
	int maxy = (int)((bmax[1]-chf.bmin[1])/chf.ch);
	int maxz = (int)((bmax[2]-chf.bmin[2])/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;
	
	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				
				if (chf.areas[i] == RC_NULL_AREA)
					continue;
				
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					const Fix16 sx = chf.bmin[0] + Fix16(x+0.5f)*chf.cs;
					const Fix16 sz = chf.bmin[2] + Fix16(z+0.5f)*chf.cs;
					const Fix16 dx = sx - pos[0];
					const Fix16 dz = sz - pos[2];
					
					if (dx*dx + dz*dz < r2)
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}
}

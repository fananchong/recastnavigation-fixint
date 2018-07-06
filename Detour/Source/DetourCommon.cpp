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

#include "DetourCommon.h"
#include "DetourMath.h"

//////////////////////////////////////////////////////////////////////////////////////////

void dtClosestPtPointTriangle(Fix16* closest, const Fix16* p,
							  const Fix16* a, const Fix16* b, const Fix16* c)
{
	// Check if P in vertex region outside A
	Fix16 ab[3], ac[3], ap[3];
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	Fix16 d1 = dtVdot(ab, ap);
	Fix16 d2 = dtVdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	Fix16 bp[3];
	dtVsub(bp, p, b);
	Fix16 d3 = dtVdot(ab, bp);
	Fix16 d4 = dtVdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	Fix16 vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		Fix16 v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}
	
	// Check if P in vertex region outside C
	Fix16 cp[3];
	dtVsub(cp, p, c);
	Fix16 d5 = dtVdot(ab, cp);
	Fix16 d6 = dtVdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	Fix16 vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		Fix16 w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	Fix16 va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		Fix16 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	Fix16 denom = Fix16_One / (va + vb + vc);
	Fix16 v = vb * denom;
	Fix16 w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

bool dtIntersectSegmentPoly2D(const Fix16* p0, const Fix16* p1,
							  const Fix16* verts, int nverts,
							  Fix16& tmin, Fix16& tmax,
							  int& segMin, int& segMax)
{
	static const Fix16 EPS = 0.00000001f;
	
	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	
	Fix16 dir[3];
	dtVsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		Fix16 edge[3], diff[3];
		dtVsub(edge, &verts[i*3], &verts[j*3]);
		dtVsub(diff, p0, &verts[j*3]);
		const Fix16 n = dtVperp2D(edge, diff);
		const Fix16 d = dtVperp2D(dir, edge);
		if (dtMathFabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		const Fix16 t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

Fix16 dtDistancePtSegSqr2D(const Fix16* pt, const Fix16* p, const Fix16* q, Fix16& t)
{
	Fix16 pqx = q[0] - p[0];
	Fix16 pqz = q[2] - p[2];
	Fix16 dx = pt[0] - p[0];
	Fix16 dz = pt[2] - p[2];
	Fix16 d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}

void dtCalcPolyCenter(Fix16* tc, const unsigned short* idx, int nidx, const Fix16* verts)
{
	tc[0] = 0.0f;
	tc[1] = 0.0f;
	tc[2] = 0.0f;
	for (int j = 0; j < nidx; ++j)
	{
		const Fix16* v = &verts[idx[j]*3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const Fix16 s = 1.0f / nidx;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}

bool dtClosestHeightPointTriangle(const Fix16* p, const Fix16* a, const Fix16* b, const Fix16* c, Fix16& h)
{
	Fix16 v0[3], v1[3], v2[3];
	dtVsub(v0, c,a);
	dtVsub(v1, b,a);
	dtVsub(v2, p,a);
	
	const Fix16 dot00 = dtVdot2D(v0, v0);
	const Fix16 dot01 = dtVdot2D(v0, v1);
	const Fix16 dot02 = dtVdot2D(v0, v2);
	const Fix16 dot11 = dtVdot2D(v1, v1);
	const Fix16 dot12 = dtVdot2D(v1, v2);
	
	// Compute barycentric coordinates
	const Fix16 invDenom = Fix16_One / (dot00 * dot11 - dot01 * dot01);
	const Fix16 u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	const Fix16 v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	static const Fix16 EPS = 1e-4f;
    static const Fix16 NEGATIVE_EPS = -(1e-4f);
	
	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= NEGATIVE_EPS && v >= NEGATIVE_EPS && (u+v) <= Fix16_One+EPS)
	{
		h = a[1] + v0[1]*u + v1[1]*v;
		return true;
	}
	
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
bool dtPointInPolygon(const Fix16* pt, const Fix16* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Fix16* vi = &verts[i*3];
		const Fix16* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
	}
	return c;
}

bool dtDistancePtPolyEdgesSqr(const Fix16* pt, const Fix16* verts, const int nverts,
							  Fix16* ed, Fix16* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Fix16* vi = &verts[i*3];
		const Fix16* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const Fix16* axis, const Fix16* poly, const int npoly,
						Fix16& rmin, Fix16& rmax)
{
	rmin = rmax = dtVdot2D(axis, &poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const Fix16 d = dtVdot2D(axis, &poly[i*3]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

inline bool overlapRange(const Fix16 amin, const Fix16 amax,
						 const Fix16 bmin, const Fix16 bmax,
						 const Fix16 eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool dtOverlapPolyPoly2D(const Fix16* polya, const int npolya,
						 const Fix16* polyb, const int npolyb)
{
	const Fix16 eps = 1e-4f;
	
	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const Fix16* va = &polya[j*3];
		const Fix16* vb = &polya[i*3];
		const Fix16 n[3] = { vb[2]-va[2], 0, Fix16_Zero -(vb[0]-va[0]) };
		Fix16 amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const Fix16* va = &polyb[j*3];
		const Fix16* vb = &polyb[i*3];
		const Fix16 n[3] = { vb[2]-va[2], 0, Fix16_Zero-(vb[0]-va[0]) };
		Fix16 amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
void dtRandomPointInConvexPoly(const Fix16* pts, const int npts, Fix16* areas,
							   const Fix16 s, const Fix16 t, Fix16* out)
{
    static const Fix16 Fix16_0dot001 = 0.001f;

	// Calc triangle araes
	Fix16 areasum = 0.0f;
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(&pts[0], &pts[(i-1)*3], &pts[i*3]);
		areasum += dtMax(Fix16_0dot001, areas[i]);
	}
	// Find sub triangle weighted by area.
	const Fix16 thr = s*areasum;
	Fix16 acc = 0.0f;
	Fix16 u = Fix16_One;
	int tri = npts - 1;
	for (int i = 2; i < npts; i++) {
		const Fix16 dacc = areas[i];
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}
	
	Fix16 v = dtMathSqrtf(t);
	
	const Fix16 a = Fix16_One - v;
	const Fix16 b = (Fix16_One - u) * v;
	const Fix16 c = u * v;
	const Fix16* pa = &pts[0];
	const Fix16* pb = &pts[(tri-1)*3];
	const Fix16* pc = &pts[tri*3];
	
	out[0] = a*pa[0] + b*pb[0] + c*pc[0];
	out[1] = a*pa[1] + b*pb[1] + c*pc[1];
	out[2] = a*pa[2] + b*pb[2] + c*pc[2];
}

inline Fix16 vperpXZ(const Fix16* a, const Fix16* b) { return a[0]*b[2] - a[2]*b[0]; }

bool dtIntersectSegSeg2D(const Fix16* ap, const Fix16* aq,
						 const Fix16* bp, const Fix16* bq,
						 Fix16& s, Fix16& t)
{
	Fix16 u[3], v[3], w[3];
	dtVsub(u,aq,ap);
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	Fix16 d = vperpXZ(u,v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}


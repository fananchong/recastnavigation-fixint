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

#ifndef DETOUROBSTACLEAVOIDANCE_H
#define DETOUROBSTACLEAVOIDANCE_H

#include <fix16.hpp>

struct dtObstacleCircle
{
	Fix16 p[3];				///< Position of the obstacle
	Fix16 vel[3];			///< Velocity of the obstacle
	Fix16 dvel[3];			///< Velocity of the obstacle
	Fix16 rad;				///< Radius of the obstacle
	Fix16 dp[3], np[3];		///< Use for side selection during sampling.
};

struct dtObstacleSegment
{
	Fix16 p[3], q[3];		///< End points of the obstacle segment
	bool touch;
};


class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();
	
	bool init(const int maxSamples);
	void reset();
	void addSample(const Fix16* vel, const Fix16 ssize, const Fix16 pen,
				   const Fix16 vpen, const Fix16 vcpen, const Fix16 spen, const Fix16 tpen);
	
	void normalizeSamples();
	
	inline int getSampleCount() const { return m_nsamples; }
	inline const Fix16* getSampleVelocity(const int i) const { return &m_vel[i*3]; }
	inline Fix16 getSampleSize(const int i) const { return m_ssize[i]; }
	inline Fix16 getSamplePenalty(const int i) const { return m_pen[i]; }
	inline Fix16 getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline Fix16 getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline Fix16 getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline Fix16 getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData&);
	dtObstacleAvoidanceDebugData& operator=(const dtObstacleAvoidanceDebugData&);

	int m_nsamples;
	int m_maxSamples;
	Fix16* m_vel;
	Fix16* m_ssize;
	Fix16* m_pen;
	Fix16* m_vpen;
	Fix16* m_vcpen;
	Fix16* m_spen;
	Fix16* m_tpen;
};

dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);


static const int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.
static const int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

struct dtObstacleAvoidanceParams
{
	Fix16 velBias;
	Fix16 weightDesVel;
	Fix16 weightCurVel;
	Fix16 weightSide;
	Fix16 weightToi;
	Fix16 horizTime;
	unsigned char gridSize;	///< grid
	unsigned char adaptiveDivs;	///< adaptive
	unsigned char adaptiveRings;	///< adaptive
	unsigned char adaptiveDepth;	///< adaptive
};

class dtObstacleAvoidanceQuery
{
public:
	dtObstacleAvoidanceQuery();
	~dtObstacleAvoidanceQuery();
	
	bool init(const int maxCircles, const int maxSegments);
	
	void reset();

	void addCircle(const Fix16* pos, const Fix16 rad,
				   const Fix16* vel, const Fix16* dvel);
				   
	void addSegment(const Fix16* p, const Fix16* q);

	int sampleVelocityGrid(const Fix16* pos, const Fix16 rad, const Fix16 vmax,
						   const Fix16* vel, const Fix16* dvel, Fix16* nvel,
						   const dtObstacleAvoidanceParams* params,
						   dtObstacleAvoidanceDebugData* debug = 0);

	int sampleVelocityAdaptive(const Fix16* pos, const Fix16 rad, const Fix16 vmax,
							   const Fix16* vel, const Fix16* dvel, Fix16* nvel,
							   const dtObstacleAvoidanceParams* params, 
							   dtObstacleAvoidanceDebugData* debug = 0);
	
	inline int getObstacleCircleCount() const { return m_ncircles; }
	const dtObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	inline int getObstacleSegmentCount() const { return m_nsegments; }
	const dtObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery&);
	dtObstacleAvoidanceQuery& operator=(const dtObstacleAvoidanceQuery&);

	void prepare(const Fix16* pos, const Fix16* dvel);

	Fix16 processSample(const Fix16* vcand, const Fix16 cs,
						const Fix16* pos, const Fix16 rad,
						const Fix16* vel, const Fix16* dvel,
						const Fix16 minPenalty,
						dtObstacleAvoidanceDebugData* debug);

	dtObstacleAvoidanceParams m_params;
	Fix16 m_invHorizTime;
	Fix16 m_vmax;
	Fix16 m_invVmax;

	int m_maxCircles;
	dtObstacleCircle* m_circles;
	int m_ncircles;

	int m_maxSegments;
	dtObstacleSegment* m_segments;
	int m_nsegments;
};

dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);


#endif // DETOUROBSTACLEAVOIDANCE_H

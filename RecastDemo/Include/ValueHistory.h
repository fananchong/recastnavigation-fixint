#ifndef VALUEHISTORY_H
#define VALUEHISTORY_H

#include <fix16.hpp>

class ValueHistory
{
	static const int MAX_HISTORY = 256;
	Fix16 m_samples[MAX_HISTORY];
	int m_hsamples;
public:
	ValueHistory();

	inline void addSample(const Fix16 val)
	{
		m_hsamples = (m_hsamples+MAX_HISTORY-1) % MAX_HISTORY;
		m_samples[m_hsamples] = val;
	}
	
	inline int getSampleCount() const
	{
		return MAX_HISTORY;
	}
	
	inline Fix16 getSample(const int i) const
	{
		return m_samples[(m_hsamples+i) % MAX_HISTORY];
	}
	
	Fix16 getSampleMin() const;
	Fix16 getSampleMax() const;
	Fix16 getAverage() const;
};

struct GraphParams
{
	void setRect(int ix, int iy, int iw, int ih, int ipad);
	void setValueRange(Fix16 ivmin, Fix16 ivmax, int indiv, const char* iunits);
	
	int x, y, w, h, pad;
	Fix16 vmin, vmax;
	int ndiv;
	char units[16];
};

void drawGraphBackground(const GraphParams* p);

void drawGraph(const GraphParams* p, const ValueHistory* graph,
			   int idx, const char* label, const unsigned int col);


#endif // VALUEHISTORY_H
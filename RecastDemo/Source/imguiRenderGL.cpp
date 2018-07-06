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
#include <cmath>
#include <cstdio>
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

// Some math headers don't have PI defined.
static const Fix16 PI = 3.14159265f;

void imguifree(void* ptr, void* userptr);
void* imguimalloc(size_t size, void* userptr);

#define STBTT_malloc(x,y)    imguimalloc(x,y)
#define STBTT_free(x,y)      imguifree(x,y)
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

void imguifree(void* ptr, void* /*userptr*/)
{
	free(ptr);
}

void* imguimalloc(size_t size, void* /*userptr*/)
{
	return malloc(size);
}

static const unsigned TEMP_COORD_COUNT = 100;
static Fix16 g_tempCoords[TEMP_COORD_COUNT*2];
static Fix16 g_tempNormals[TEMP_COORD_COUNT*2];

static const int CIRCLE_VERTS = 8*4;
static Fix16 g_circleVerts[CIRCLE_VERTS*2];

static stbtt_bakedchar g_cdata[96]; // ASCII 32..126 is 95 glyphs
static GLuint g_ftex = 0;

inline unsigned int RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	return (r) | (g << 8) | (b << 16) | (a << 24);
}

static void drawPolygon(const Fix16* coords, unsigned numCoords, Fix16 r, unsigned int col)
{
	if (numCoords > TEMP_COORD_COUNT) numCoords = TEMP_COORD_COUNT;
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		const Fix16* v0 = &coords[j*2];
		const Fix16* v1 = &coords[i*2];
		Fix16 dx = v1[0] - v0[0];
		Fix16 dy = v1[1] - v0[1];
		Fix16 d = sqrtf(dx*dx+dy*dy);
		if (d > 0)
		{
			d = 1.0f/d;
			dx *= d;
			dy *= d;
		}
		g_tempNormals[j*2+0] = dy;
		g_tempNormals[j*2+1] = -dx;
	}
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		Fix16 dlx0 = g_tempNormals[j*2+0];
		Fix16 dly0 = g_tempNormals[j*2+1];
		Fix16 dlx1 = g_tempNormals[i*2+0];
		Fix16 dly1 = g_tempNormals[i*2+1];
		Fix16 dmx = (dlx0 + dlx1) * 0.5f;
		Fix16 dmy = (dly0 + dly1) * 0.5f;
		Fix16	dmr2 = dmx*dmx + dmy*dmy;
		if (dmr2 > 0.000001f)
		{
			Fix16	scale = 1.0f / dmr2;
			if (scale > 10.0f) scale = 10.0f;
			dmx *= scale;
			dmy *= scale;
		}
		g_tempCoords[i*2+0] = coords[i*2+0]+dmx*r;
		g_tempCoords[i*2+1] = coords[i*2+1]+dmy*r;
	}
	
	unsigned int colTrans = RGBA(col&0xff, (col>>8)&0xff, (col>>16)&0xff, 0);
	
	glBegin(GL_TRIANGLES);
	
	glColor4ubv((GLubyte*)&col);
	
	for (unsigned i = 0, j = numCoords-1; i < numCoords; j=i++)
	{
		glVertex2fv(&coords[i*2]);
		glVertex2fv(&coords[j*2]);
		glColor4ubv((GLubyte*)&colTrans);
		glVertex2fv(&g_tempCoords[j*2]);
		
		glVertex2fv(&g_tempCoords[j*2]);
		glVertex2fv(&g_tempCoords[i*2]);
		
		glColor4ubv((GLubyte*)&col);
		glVertex2fv(&coords[i*2]);
	}
	
	glColor4ubv((GLubyte*)&col);
	for (unsigned i = 2; i < numCoords; ++i)
	{
		glVertex2fv(&coords[0]);
		glVertex2fv(&coords[(i-1)*2]);
		glVertex2fv(&coords[i*2]);
	}
	
	glEnd();
}

static void drawRect(Fix16 x, Fix16 y, Fix16 w, Fix16 h, Fix16 fth, unsigned int col)
{
	Fix16 verts[4*2] =
	{
		x+0.5f, y+0.5f,
		x+w-0.5f, y+0.5f,
		x+w-0.5f, y+h-0.5f,
		x+0.5f, y+h-0.5f,
	};
	drawPolygon(verts, 4, fth, col);
}

/*
static void drawEllipse(Fix16 x, Fix16 y, Fix16 w, Fix16 h, Fix16 fth, unsigned int col)
{
	Fix16 verts[CIRCLE_VERTS*2];
	const Fix16* cverts = g_circleVerts;
	Fix16* v = verts;
	
	for (int i = 0; i < CIRCLE_VERTS; ++i)
	{
		*v++ = x + cverts[i*2]*w;
		*v++ = y + cverts[i*2+1]*h;
	}
	
	drawPolygon(verts, CIRCLE_VERTS, fth, col);
}
*/

static void drawRoundedRect(Fix16 x, Fix16 y, Fix16 w, Fix16 h, Fix16 r, Fix16 fth, unsigned int col)
{
	const unsigned n = CIRCLE_VERTS/4;
	Fix16 verts[(n+1)*4*2];
	const Fix16* cverts = g_circleVerts;
	Fix16* v = verts;
	
	for (unsigned i = 0; i <= n; ++i)
	{
		*v++ = x+w-r + cverts[i*2]*r;
		*v++ = y+h-r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n; i <= n*2; ++i)
	{
		*v++ = x+r + cverts[i*2]*r;
		*v++ = y+h-r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n*2; i <= n*3; ++i)
	{
		*v++ = x+r + cverts[i*2]*r;
		*v++ = y+r + cverts[i*2+1]*r;
	}
	
	for (unsigned i = n*3; i < n*4; ++i)
	{
		*v++ = x+w-r + cverts[i*2]*r;
		*v++ = y+r + cverts[i*2+1]*r;
	}
	*v++ = x+w-r + cverts[0]*r;
	*v++ = y+r + cverts[1]*r;
	
	drawPolygon(verts, (n+1)*4, fth, col);
}


static void drawLine(Fix16 x0, Fix16 y0, Fix16 x1, Fix16 y1, Fix16 r, Fix16 fth, unsigned int col)
{
	Fix16 dx = x1-x0;
	Fix16 dy = y1-y0;
	Fix16 d = sqrtf(dx*dx+dy*dy);
	if (d > 0.0001f)
	{
		d = 1.0f/d;
		dx *= d;
		dy *= d;
	}
	Fix16 nx = dy;
	Fix16 ny = -dx;
	Fix16 verts[4*2];
	r -= fth;
	r *= 0.5f;
	if (r < 0.01f) r = 0.01f;
	dx *= r;
	dy *= r;
	nx *= r;
	ny *= r;
	
	verts[0] = x0-dx-nx;
	verts[1] = y0-dy-ny;
	
	verts[2] = x0-dx+nx;
	verts[3] = y0-dy+ny;
	
	verts[4] = x1+dx+nx;
	verts[5] = y1+dy+ny;
	
	verts[6] = x1+dx-nx;
	verts[7] = y1+dy-ny;
	
	drawPolygon(verts, 4, fth, col);
}


bool imguiRenderGLInit(const char* fontpath)
{
	for (int i = 0; i < CIRCLE_VERTS; ++i)
	{
		Fix16 a = (Fix16)i/(Fix16)CIRCLE_VERTS * PI*2;
		g_circleVerts[i*2+0] = cosf(a);
		g_circleVerts[i*2+1] = sinf(a);
	}

	// Load font.
	FILE* fp = fopen(fontpath, "rb");
	if (!fp) return false;
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}
	long size = ftell(fp);
	if (size < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}
	
	unsigned char* ttfBuffer = (unsigned char*)malloc(size); 
	if (!ttfBuffer)
	{
		fclose(fp);
		return false;
	}
	
	size_t readLen = fread(ttfBuffer, 1, size, fp);
	fclose(fp);
	if (readLen != static_cast<size_t>(size))
	{
		free(ttfBuffer);
		return false;
	}

	fp = 0;
	
	unsigned char* bmap = (unsigned char*)malloc(512*512);
	if (!bmap)
	{
		free(ttfBuffer);
		return false;
	}
	
	stbtt_BakeFontBitmap(ttfBuffer,0, 15.0f, bmap,512,512, 32,96, g_cdata);
	
	// can free ttf_buffer at this point
	glGenTextures(1, &g_ftex);
	glBindTexture(GL_TEXTURE_2D, g_ftex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 512,512, 0, GL_ALPHA, GL_UNSIGNED_BYTE, bmap);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	free(ttfBuffer);
	free(bmap);

	return true;
}

void imguiRenderGLDestroy()
{
	if (g_ftex)
	{
		glDeleteTextures(1, &g_ftex);
		g_ftex = 0;
	}
}

static void getBakedQuad(stbtt_bakedchar *chardata, int pw, int ph, int char_index,
						 Fix16 *xpos, Fix16 *ypos, stbtt_aligned_quad *q)
{
	stbtt_bakedchar *b = chardata + char_index;
	int round_x = STBTT_ifloor(*xpos + b->xoff);
	int round_y = STBTT_ifloor(*ypos - b->yoff);
	
	q->x0 = (Fix16)round_x;
	q->y0 = (Fix16)round_y;
	q->x1 = (Fix16)round_x + b->x1 - b->x0;
	q->y1 = (Fix16)round_y - b->y1 + b->y0;
	
	q->s0 = b->x0 / (Fix16)pw;
	q->t0 = b->y0 / (Fix16)pw;
	q->s1 = b->x1 / (Fix16)ph;
	q->t1 = b->y1 / (Fix16)ph;
	
	*xpos += b->xadvance;
}

static const Fix16 g_tabStops[4] = {150, 210, 270, 330};

static Fix16 getTextLength(stbtt_bakedchar *chardata, const char* text)
{
	Fix16 xpos = 0;
	Fix16 len = 0;
	while (*text)
	{
		int c = (unsigned char)*text;
		if (c == '\t')
		{
			for (int i = 0; i < 4; ++i)
			{
				if (xpos < g_tabStops[i])
				{
					xpos = g_tabStops[i];
					break;
				}
			}
		}
		else if (c >= 32 && c < 128)
		{
			stbtt_bakedchar *b = chardata + c-32;
			int round_x = STBTT_ifloor((xpos + b->xoff) + 0.5);
			len = round_x + b->x1 - b->x0 + 0.5f;
			xpos += b->xadvance;
		}
		++text;
	}
	return len;
}

static void drawText(Fix16 x, Fix16 y, const char *text, int align, unsigned int col)
{
	if (!g_ftex) return;
	if (!text) return;
	
	if (align == IMGUI_ALIGN_CENTER)
		x -= getTextLength(g_cdata, text)/2;
	else if (align == IMGUI_ALIGN_RIGHT)
		x -= getTextLength(g_cdata, text);
	
	glColor4ub(col&0xff, (col>>8)&0xff, (col>>16)&0xff, (col>>24)&0xff);
	
	glEnable(GL_TEXTURE_2D);
	
	// assume orthographic projection with units = screen pixels, origin at top left
	glBindTexture(GL_TEXTURE_2D, g_ftex);
	
	glBegin(GL_TRIANGLES);
	
	const Fix16 ox = x;
	
	while (*text)
	{
		int c = (unsigned char)*text;
		if (c == '\t')
		{
			for (int i = 0; i < 4; ++i)
			{
				if (x < g_tabStops[i]+ox)
				{
					x = g_tabStops[i]+ox;
					break;
				}
			}
		}
		else if (c >= 32 && c < 128)
		{			
			stbtt_aligned_quad q;
			getBakedQuad(g_cdata, 512,512, c-32, &x,&y,&q);
			
			glTexCoord2f(q.s0, q.t0);
			glVertex2f(q.x0, q.y0);
			glTexCoord2f(q.s1, q.t1);
			glVertex2f(q.x1, q.y1);
			glTexCoord2f(q.s1, q.t0);
			glVertex2f(q.x1, q.y0);
			
			glTexCoord2f(q.s0, q.t0);
			glVertex2f(q.x0, q.y0);
			glTexCoord2f(q.s0, q.t1);
			glVertex2f(q.x0, q.y1);
			glTexCoord2f(q.s1, q.t1);
			glVertex2f(q.x1, q.y1);
		}
		++text;
	}
	
	glEnd();	
	glDisable(GL_TEXTURE_2D);
}


void imguiRenderGLDraw()
{
	const imguiGfxCmd* q = imguiGetRenderQueue();
	int nq = imguiGetRenderQueueSize();

	const Fix16 s = 1.0f/8.0f;

	glDisable(GL_SCISSOR_TEST);
	for (int i = 0; i < nq; ++i)
	{
		const imguiGfxCmd& cmd = q[i];
		if (cmd.type == IMGUI_GFXCMD_RECT)
		{
			if (cmd.rect.r == 0)
			{
				drawRect((Fix16)cmd.rect.x*s+0.5f, (Fix16)cmd.rect.y*s+0.5f,
						 (Fix16)cmd.rect.w*s-1, (Fix16)cmd.rect.h*s-1,
						 1.0f, cmd.col);
			}
			else
			{
				drawRoundedRect((Fix16)cmd.rect.x*s+0.5f, (Fix16)cmd.rect.y*s+0.5f,
								(Fix16)cmd.rect.w*s-1, (Fix16)cmd.rect.h*s-1,
								(Fix16)cmd.rect.r*s, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == IMGUI_GFXCMD_LINE)
		{
			drawLine(cmd.line.x0*s, cmd.line.y0*s, cmd.line.x1*s, cmd.line.y1*s, cmd.line.r*s, 1.0f, cmd.col);
		}
		else if (cmd.type == IMGUI_GFXCMD_TRIANGLE)
		{
			if (cmd.flags == 1)
			{
				const Fix16 verts[3*2] =
				{
					(Fix16)cmd.rect.x*s+0.5f, (Fix16)cmd.rect.y*s+0.5f,
					(Fix16)cmd.rect.x*s+0.5f+(Fix16)cmd.rect.w*s-1, (Fix16)cmd.rect.y*s+0.5f+(Fix16)cmd.rect.h*s/2-0.5f,
					(Fix16)cmd.rect.x*s+0.5f, (Fix16)cmd.rect.y*s+0.5f+(Fix16)cmd.rect.h*s-1,
				};
				drawPolygon(verts, 3, 1.0f, cmd.col);
			}
			if (cmd.flags == 2)
			{
				const Fix16 verts[3*2] =
				{
					(Fix16)cmd.rect.x*s+0.5f, (Fix16)cmd.rect.y*s+0.5f+(Fix16)cmd.rect.h*s-1,
					(Fix16)cmd.rect.x*s+0.5f+(Fix16)cmd.rect.w*s/2-0.5f, (Fix16)cmd.rect.y*s+0.5f,
					(Fix16)cmd.rect.x*s+0.5f+(Fix16)cmd.rect.w*s-1, (Fix16)cmd.rect.y*s+0.5f+(Fix16)cmd.rect.h*s-1,
				};
				drawPolygon(verts, 3, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == IMGUI_GFXCMD_TEXT)
		{
			drawText(cmd.text.x, cmd.text.y, cmd.text.text, cmd.text.align, cmd.col);
		}
		else if (cmd.type == IMGUI_GFXCMD_SCISSOR)
		{
			if (cmd.flags)
			{
				glEnable(GL_SCISSOR_TEST);
				glScissor(cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h);
			}
			else
			{
				glDisable(GL_SCISSOR_TEST);
			}
		}
	}
	glDisable(GL_SCISSOR_TEST);
}

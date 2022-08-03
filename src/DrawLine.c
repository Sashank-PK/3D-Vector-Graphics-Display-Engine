/*
===============================================================================
 Name        : DrawLine.c
 Author      : $RJ
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>



/* Be careful with the port number and location number, because

some of the location may not exist in that port. */

#define PORT_NUM            0

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];

#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29


#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values

#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0xF8001F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF
#define GREY 0xDDDDDD
#define YELLOW 0xFFFF33
#define SUNSET 0xFBCCBB
int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;

void spiwrite(uint8_t c)

{

 int pnum = 0;

 src_addr[0] = c;

 SSP_SSELToggle( pnum, 0 );

 SSPSend( pnum, (uint8_t *)src_addr, 1 );

 SSP_SSELToggle( pnum, 1 );

}

void writecommand(uint8_t c)

{

 LPC_GPIO0->FIOCLR |= (0x1<<21);

 spiwrite(c);

}

void writedata(uint8_t c)

{

 LPC_GPIO0->FIOSET |= (0x1<<21);

 spiwrite(c);

}

void writeword(uint16_t c)

{

 uint8_t d;

 d = c >> 8;

 writedata(d);

 d = c & 0xFF;

 writedata(d);

}

void write888(uint32_t color, uint32_t repeat)

{

 uint8_t red, green, blue;

 int i;

 red = (color >> 16);

 green = (color >> 8) & 0xFF;

 blue = color & 0xFF;

 for (i = 0; i< repeat; i++) {

  writedata(red);

  writedata(green);

  writedata(blue);

 }

}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)

{

 writecommand(ST7735_CASET);

 writeword(x0);

 writeword(x1);

 writecommand(ST7735_RASET);

 writeword(y0);

 writeword(y1);

}

void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t i;

 int16_t width, height;

 width = x1-x0+1;

 height = y1-y0+1;

 setAddrWindow(x0,y0,x1,y1);

 writecommand(ST7735_RAMWR);

 write888(color,width*height);

}

void lcddelay(int ms)

{

 int count = 24000;

 int i;

 for ( i = count*ms; i--; i > 0);

}

void lcd_init()

{

 int i;
 printf("LCD Demo Begins!!!\n");
 // Set pins P0.16, P0.21, P0.22 as output
 LPC_GPIO0->FIODIR |= (0x1<<16);

 LPC_GPIO0->FIODIR |= (0x1<<21);

 LPC_GPIO0->FIODIR |= (0x1<<22);

 // Hardware Reset Sequence
 LPC_GPIO0->FIOSET |= (0x1<<22);
 //lcddelay(500);

 LPC_GPIO0->FIOCLR |= (0x1<<22);
 //lcddelay(500);

 LPC_GPIO0->FIOSET |= (0x1<<22);
 //lcddelay(500);

 // initialize buffers
 for ( i = 0; i < SSP_BUFSIZE; i++ )
 {

   src_addr[i] = 0;
   dest_addr[i] = 0;
 }

 // Take LCD display out of sleep mode
 writecommand(ST7735_SLPOUT);
 //lcddelay(200);

 // Turn LCD display on
 writecommand(ST7735_DISPON);
 //lcddelay(200);

}

void drawPixel(int16_t x, int16_t y, uint32_t color)

{

 if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))

 return;

 setAddrWindow(x, y, x + 1, y + 1);

 writecommand(ST7735_RAMWR);

 write888(color, 1);

}

/*****************************************************************************


** Descriptions:        Draw line function

**

** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

** Returned value:        None

**

*****************************************************************************/

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)

{

 int16_t slope = abs(y1 - y0) > abs(x1 - x0);

 if (slope) {

  swap(x0, y0);

  swap(x1, y1);

 }

 if (x0 > x1) {

  swap(x0, x1);

  swap(y0, y1);

 }

 int16_t dx, dy;

 dx = x1 - x0;

 dy = abs(y1 - y0);

 int16_t err = dx / 2;

 int16_t ystep;

 if (y0 < y1) {

  ystep = 1;

 }

 else {

  ystep = -1;

 }

 for (; x0 <= x1; x0++) {

  if (slope) {

   drawPixel(y0, x0, color);

  }

  else {

   drawPixel(x0, y0, color);

  }

  err -= dy;

  if (err < 0) {

   y0 += ystep;

   err += dx;

  }

 }

}

// ==========================================================================================================
// END OF MAIN LCD FUNCTIONS
// ==========================================================================================================

/*
=======================3D World==============================================================================
*/

//------------------------------DATA TYPES-------------------------------------------------------------------

typedef enum {F,T} Bool;		// emulating a boolean data type
typedef enum {Portrait, Landscape, Portrait_Flipped, Landscape_Flipped} SCREEN_Mode;  // screen orientation type
typedef enum {TL,TM,TR,ML,MM,MR,BL,BM,BR} ORIGIN_Mode; // generalized Origin offsetand positions
typedef enum {Surf0,Surf1,Surf2,Surf3,Surf4,Surf5} SURFACE;


struct Point
{
	int X;
	int Y;
	int Z;
};

struct Pointf
{
	float X;
	float Y;
	float Z;
};

struct Reflectiv
{
	float R;
	float G;
	float B;
};

struct Screen
{
	SCREEN_Mode Orientation;
	ORIGIN_Mode O_Shift;
	Bool XYSwap;
};

struct Views
{
	struct Point World;
	struct Point Display;
};

struct Shape
{
	struct Views Pos[10];			// max 10 points allowed
	struct Reflectiv Ref[6];		// only for cube	6 surfaces
	Bool Solid[6];
	//struct
};
struct Shape Cube;

void SetCube(struct Point P,int side)
{
	Cube.Pos[0].World.X = P.X;
	Cube.Pos[0].World.Y = P.Y;
	Cube.Pos[0].World.Z = P.Z;

	Cube.Pos[1].World.X = P.X + side;
	Cube.Pos[1].World.Y = P.Y;
	Cube.Pos[1].World.Z = P.Z;

	Cube.Pos[2].World.X = P.X + side;
	Cube.Pos[2].World.Y = P.Y + side;
	Cube.Pos[2].World.Z = P.Z;

	Cube.Pos[3].World.X = P.X;
	Cube.Pos[3].World.Y = P.Y + side;
	Cube.Pos[3].World.Z = P.Z;

	Cube.Pos[4].World.X = P.X;
	Cube.Pos[4].World.Y = P.Y;
	Cube.Pos[4].World.Z = P.Z + side;

	Cube.Pos[5].World.X = P.X + side;
	Cube.Pos[5].World.Y = P.Y;
	Cube.Pos[5].World.Z = P.Z + side;

	Cube.Pos[6].World.X = P.X + side;
	Cube.Pos[6].World.Y = P.Y + side;
	Cube.Pos[6].World.Z = P.Z + side;

	Cube.Pos[7].World.X = P.X;
	Cube.Pos[7].World.Y = P.Y + side;
	Cube.Pos[7].World.Z = P.Z + side;
}

void SetSurfRefl(struct Reflectiv R, SURFACE s)
{
	if(s>-1 && s<6)
	{
		if(R.R < 1.0 || R.G < 1.0 ||R.B < 1.0)
		{
			Cube.Ref[s].R = R.R;
			Cube.Ref[s].G = R.G;
			Cube.Ref[s].B = R.B;
		}
		else
		{
			Cube.Ref[s].R = 0.0;
			Cube.Ref[s].G = 0.0;
			Cube.Ref[s].B = 0.0;
		}
	}

}

struct Cam
{
	struct Point Pos;
	float D;
};
struct Cam Camera;

void SetCamP(struct Point P)
{
	Camera.Pos.X = P.X;
	Camera.Pos.Y = P.Y;
	Camera.Pos.Z = P.Z;
}

void SetCamF(float F)
{
	Camera.D = F;
}

struct Axes
{
	struct Views Xax;
	struct Views Yax;
	struct Views Zax;
};

struct Origin
{
	struct Point O;
	struct Axes Ax;
	struct Screen SCR;
	struct Point Dir;
	struct Point Off;
};
struct Origin O;

void SetAxes(int L)
{
	O.Ax.Xax.World.X = O.O.X + L;
	O.Ax.Xax.World.Y = O.O.Y;
	O.Ax.Xax.World.Z = O.O.Z;

	O.Ax.Yax.World.X = O.O.X;
	O.Ax.Yax.World.Y = O.O.Y + L;
	O.Ax.Yax.World.Z = O.O.Z;

	O.Ax.Zax.World.X = O.O.X;
	O.Ax.Zax.World.Y = O.O.Y;
	O.Ax.Zax.World.Z = O.O.Z + L;
}

void SetOrigin()
{
	O.O.X = 0;
	O.O.Y = 0;
	O.O.Z = 0;
}

void DrawOrigin()
{
	struct Point Odisp;
	CalDisPnts(O.Ax.Xax.World,&O.Ax.Xax.Display);
	CalDisPnts(O.Ax.Yax.World,&O.Ax.Yax.Display);
	CalDisPnts(O.Ax.Zax.World,&O.Ax.Zax.Display);
	CalDisPnts(O.O,&Odisp);

	drawLine(Odisp.X,Odisp.Y,O.Ax.Xax.Display.X,O.Ax.Xax.Display.Y,RED);
	drawLine(Odisp.X,Odisp.Y,O.Ax.Yax.Display.X,O.Ax.Yax.Display.Y,BLUE);
	drawLine(Odisp.X,Odisp.Y,O.Ax.Zax.Display.X,O.Ax.Zax.Display.Y,GREEN);
}

void SetDisplay(SCREEN_Mode ori, ORIGIN_Mode om)
{
	O.SCR.Orientation = ori;
	O.SCR.O_Shift = om;

	switch(ori)
	{
	case 0:
	{
		O.Dir.X = 1.0;		// Physical X = Virtual X
		O.Dir.Y = -1.0;		// Physical Y = -Virtual Y
		O.SCR.XYSwap = F;			// X and Y axes are same alignment
		break;
	}
	case 1:
	{
		O.Dir.X = 1.0;		// Physical X = Virtual Y
		O.Dir.Y = 1.0;		// Physical Y = Virtual X
		O.SCR.XYSwap = T;					// X and Y axes are 90deg changed
		break;
	}
	case 2:
	{
		O.Dir.X = -1.0;		// Physical X = -Virtual X
		O.Dir.Y = 1.0;		// Physical Y = Virtual Y
		O.SCR.XYSwap = F;					// X and Y axes are same alignment
		break;
	}
	case 3:
	{
		O.Dir.X = -1.0;		// Physical X = Virtual X
		O.Dir.Y = -1.0;		// Physical Y = Virtual Y
		O.SCR.XYSwap = T;					// X and Y axes are same alignment
		break;
	}
	default:
	{
		O.Dir.X = 1.0;		// Physical X = Virtual X
		O.Dir.Y = -1.0;		// Physical Y = -Virtual Y
		O.SCR.XYSwap = F;					// X and Y axes are same alignment
		break;
	}

	}

	int W = ST7735_TFTWIDTH;
	int H = ST7735_TFTHEIGHT;

	switch (om)
	{
	case 0:
	{
		O.Off.X = 0;
		O.Off.Y = 0;
		break;
	}
	case 1:
	{
		O.Off.X = W/2;
		O.Off.Y = 0;
		break;
	}
	case 2:
	{
		O.Off.X = W;
		O.Off.Y = 0;
		break;
	}
	case 3:
	{
		O.Off.X = 0;
		O.Off.Y = H/2;
		break;
	}
	case 4:
	{
		O.Off.X = W/2;
		O.Off.Y = H/2;
		break;
	}
	case 5:
	{
		O.Off.X = W;
		O.Off.Y = H/2;
		break;
	}
	case 6:
	{
		O.Off.X = 0;
		O.Off.Y = H;
		break;
	}
	case 7:
	{
		O.Off.X = W/2;
		O.Off.Y = H;
		break;
	}
	case 8:
	{
		O.Off.X = W;
		O.Off.Y = H;
		break;
	}
	default:
	{
		O.Off.X = W/2;
		O.Off.Y = H/2;
		break;
	}
	}

}

struct Light
{
	struct Point Pos;
	float Pow;
};
struct Light Lits;

void SetLightS(struct Point P, float inten)
{
	Lits.Pos.X = P.X;
	Lits.Pos.Y = P.Y;
	Lits.Pos.Z = P.Z;

	Lits.Pow = inten;
}


struct WorldtoScrMat
{
	float R;
	float Ro;
	float CosT;
	float SinT;
	float CosP;
	float SinP;
	float M[4][4];
};

struct WorldtoScrMat Matrix;

void SetMatrixWtS()
{

	float COdis[3];
	COdis[0] = 1.0* (Camera.Pos.X - O.O.X);
	COdis[1] = 1.0* (Camera.Pos.Y - O.O.Y);
	COdis[2] = 1.0* (Camera.Pos.Z - O.O.Z);

	//float SCV[6];

	Matrix.R = sqrt( ( (COdis[0]*COdis[0]) + (COdis[1]*COdis[1]) ) ); // R
	Matrix.Ro = sqrt( ( (COdis[0]*COdis[0]) + (COdis[1]*COdis[1]) + (COdis[2]*COdis[2]) ) ); // Ro
	Matrix.CosT =  COdis[0]/Matrix.R; // cos theta
	Matrix.SinT = COdis[1]/Matrix.R; // sin theta
	Matrix.CosP = COdis[2]/Matrix.Ro; // cos phy
	Matrix.SinP = Matrix.R/Matrix.Ro; // sin phy

	Matrix.M[0][0] = -1.0*Matrix.SinT;
	Matrix.M[0][1] = Matrix.CosT;
	Matrix.M[0][2] = 0.0;
	Matrix.M[0][3] = 0.0;

	Matrix.M[1][0] = -1.0*Matrix.CosT*Matrix.CosP;
	Matrix.M[1][1] = -1.0*Matrix.SinT*Matrix.CosP;
	Matrix.M[1][2] = Matrix.SinP;
	Matrix.M[1][3] = 0.0;

	Matrix.M[2][0] = -1.0*Matrix.SinP*Matrix.CosT;
	Matrix.M[2][1] = -1.0*Matrix.SinP*Matrix.CosT;
	Matrix.M[2][2] = -1.0*Matrix.CosP;
	Matrix.M[2][3] = Matrix.Ro;

	Matrix.M[3][0] = 0.0;
	Matrix.M[3][1] = 0.0;
	Matrix.M[3][2] = 0.0;
	Matrix.M[3][3] = 1.0;
}


struct Pointf WtoV(struct Point P)
{
	struct Pointf retur;
	float v[4];
	float p[4];
	p[0] = 1.0*P.X;
	p[1] = 1.0*P.Y;
	p[2] = 1.0*P.Z;
	p[3] = 1.0;

	for(int i=0; i<4; i=i+1)
	{
		v[i] = 0.0;
		for(int j=0; j<4; j=j+1)
		{
			v[i] = v[i]+(Matrix.M[i][j] * p[j]);
		}
	}

	retur.X = v[0];
	retur.Y = v[1];
	retur.Z = v[2];

	return retur;
}

struct Point VtoS(struct Pointf P)
{
	struct Point Scr;
	Scr.X = (int) (Camera.D * P.X/P.Z);
	Scr.Y = (int) (Camera.D * P.Y/P.Z);
	return Scr;
}

struct Point ScrOffset(struct Point P)
{
	struct Point dum;
	if(O.SCR.XYSwap == T)
	{
		dum.X = (P.Y * O.Dir.Y) + O.Off.X;
		dum.Y = (P.X * O.Dir.X) + O.Off.Y;
	}
	else
	{
		dum.X = (P.X * O.Dir.X) + O.Off.X;
		dum.Y = (P.Y * O.Dir.Y) + O.Off.Y;
	}

	return dum;
}

void CalDisPnts(struct Point PW,struct Point *PD)
{
	*PD = ScrOffset(VtoS(WtoV(PW)));
}

void CalcDispCube(struct Shape *S)
{
	struct Pointf V;
	struct Point Sc;

	for(int i=0;i<8;i=i+1)
	{
		CalDisPnts(S->Pos[i].World,&S->Pos[i].Display);
	}
}

float CalcLam(int zi,int zs)
{
	return (1.0 * -zi/(zs-zi));
}

struct Views ShadowG[4];

void DrawShadow()
{

	float Lamda[4];

	struct Views Shade[4];

	struct Point Pnts[4];	//3D Point

	for (int i=0; i<4; i=i+1)
	{
		Lamda[i] = CalcLam(Cube.Pos[i+4].World.Z,Lits.Pos.Z);
	}

	for (int i=0; i<4; i=i+1)
	{
		Pnts[i].X = Cube.Pos[i+4].World.X + Lamda[i] * (Lits.Pos.X - Cube.Pos[i+4].World.X);
		Pnts[i].Y = Cube.Pos[i+4].World.Y + Lamda[i] * (Lits.Pos.Y - Cube.Pos[i+4].World.Y);
		Pnts[i].Z = Cube.Pos[i+4].World.Z + Lamda[i] * (Lits.Pos.Z - Cube.Pos[i+4].World.Z);
	}

	CalDisPnts(Pnts[0],&Shade[0].Display);
	CalDisPnts(Pnts[1],&Shade[1].Display);
	CalDisPnts(Pnts[2],&Shade[2].Display);
	CalDisPnts(Pnts[3],&Shade[3].Display);

	ShadowG[0] = Shade[0];
	ShadowG[1] = Shade[1];
	ShadowG[2] = Shade[2];
	ShadowG[3] = Shade[3];

	drawLine(Shade[0].Display.X,Shade[0].Display.Y,Shade[1].Display.X,Shade[1].Display.Y,0x808080);
	drawLine(Shade[1].Display.X,Shade[1].Display.Y,Shade[2].Display.X,Shade[2].Display.Y,0x808080);
	drawLine(Shade[2].Display.X,Shade[2].Display.Y,Shade[3].Display.X,Shade[3].Display.Y,0x808080);
	drawLine(Shade[3].Display.X,Shade[3].Display.Y,Shade[0].Display.X,Shade[0].Display.Y,0x808080);

	struct Point Dum,Dum1;

	for(int i=Pnts[0].Y; i<Pnts[2].Y; i++)
	{
		for(int j=Pnts[0].X; j<Pnts[2].X; j++)
		{
			Dum.X = j;
			Dum.Y = i;
			Dum.Z = 0;

					//currPt = Transformation_pipeline(temp.x,temp.y,temp.z);
			CalDisPnts(Dum,&Dum1);
			drawPixel(Dum1.X, Dum1.Y,0x808080);
			//drawLine(Dum1.X,Dum1.Y,RED)
		}
	}
}

void DrawBeam(uint32_t col)
{
	struct Point LS2;
	CalDisPnts(Lits.Pos,&LS2);
	drawLine(ShadowG[1].Display.X,ShadowG[1].Display.Y,LS2.X,LS2.Y,col);
	drawLine(ShadowG[2].Display.X,ShadowG[2].Display.Y,LS2.X,LS2.Y,col);
	drawLine(ShadowG[3].Display.X,ShadowG[3].Display.Y,LS2.X,LS2.Y,col);
}

void FindSurf(SURFACE s, int *P, int *mM)
{
	switch (s)
		{
		case 5:
		{
			*(P+0) = 0;
			*(P+1) = 1;
			*(P+2) = 2;
			*(P+3) = 3;

			*(mM+0) = Cube.Pos[*(P+0)].World.X;
			*(mM+1) = Cube.Pos[*(P+1)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+1)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+0)].World.Z;
			*(mM+5) = Cube.Pos[*(P+2)].World.Z+1;

			break;
		}
		case 0:
		{
			*(P+0) = 4;
			*(P+1) = 5;
			*(P+2) = 6;
			*(P+3) = 7;

			*(mM+0) = Cube.Pos[*(P+0)].World.X;
			*(mM+1) = Cube.Pos[*(P+1)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+1)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+0)].World.Z;
			*(mM+5) = Cube.Pos[*(P+2)].World.Z+1;

			break;
		}
		case 3:
		{
			*(P+0) = 4;
			*(P+1) = 0;
			*(P+2) = 3;
			*(P+3) = 7;

			*(mM+0) = Cube.Pos[*(P+0)].World.X;
			*(mM+1) = Cube.Pos[*(P+2)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+1)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+1)].World.Z;
			*(mM+5) = Cube.Pos[*(P+0)].World.Z+1;

			break;
		}
		case 1:
		{
			*(P+0) = 5;
			*(P+1) = 1;
			*(P+2) = 2;
			*(P+3) = 6;

			*(mM+0) = Cube.Pos[*(P+0)].World.X;
			*(mM+1) = Cube.Pos[*(P+2)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+1)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+1)].World.Z;
			*(mM+5) = Cube.Pos[*(P+0)].World.Z+1;

			break;
		}
		case 4:
		{
			*(P+0) = 5;
			*(P+1) = 1;
			*(P+2) = 0;
			*(P+3) = 4;

			*(mM+0) = Cube.Pos[*(P+2)].World.X;
			*(mM+1) = Cube.Pos[*(P+1)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+0)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+1)].World.Z;
			*(mM+5) = Cube.Pos[*(P+0)].World.Z+1;
			break;
		}
		case 2:
		{
			*(P+0) = 6;
			*(P+1) = 2;
			*(P+2) = 3;
			*(P+3) = 7;

			*(mM+0) = Cube.Pos[*(P+2)].World.X;
			*(mM+1) = Cube.Pos[*(P+1)].World.X+1;

			*(mM+2) = Cube.Pos[*(P+0)].World.Y;
			*(mM+3) = Cube.Pos[*(P+2)].World.Y+1;

			*(mM+4) = Cube.Pos[*(P+1)].World.Z;
			*(mM+5) = Cube.Pos[*(P+0)].World.Z+1;
			break;
		}
		default:
			break;

		}//switch End
}

void DrawSid(SURFACE s,uint32_t Col)
{

	int pnts[4];
	int mM_ind[6];		// Min Max Index

	FindSurf(s,&pnts,&mM_ind);


	struct Views Area[4];
	struct Point Pnts[4];	//3D Point

	CalDisPnts(Cube.Pos[pnts[0]].World,&Area[0].Display);
	CalDisPnts(Cube.Pos[pnts[1]].World,&Area[1].Display);
	CalDisPnts(Cube.Pos[pnts[2]].World,&Area[2].Display);
	CalDisPnts(Cube.Pos[pnts[3]].World,&Area[3].Display);

	struct Point Dum,Dum1;

	for (int i=mM_ind[0]; i<mM_ind[1]; i=i+1)
	{
		for (int j=mM_ind[2]; j<mM_ind[3]; j=j+1)
		{
			for (int k=mM_ind[4]; k<mM_ind[5]; k=k+1)
			{
				Dum.X = i;
				Dum.Y = j;
				Dum.Z = k;

				Dum1.X = i;
				Dum1.Y = j;
				Dum1.Z = k;

						//currPt = Transformation_pipeline(temp.x,temp.y,temp.z);
				CalDisPnts(Dum,&Dum1);
				drawPixel(Dum1.X, Dum1.Y,Col);
			}
		}
	}
}


void DrawCubeW(struct Shape *S)
{
	drawLine(Cube.Pos[0].Display.X , Cube.Pos[0].Display.Y , Cube.Pos[1].Display.X , Cube.Pos[1].Display.Y , BLACK);
	drawLine(Cube.Pos[1].Display.X , Cube.Pos[1].Display.Y , Cube.Pos[2].Display.X , Cube.Pos[2].Display.Y , BLACK);
	drawLine(Cube.Pos[2].Display.X , Cube.Pos[2].Display.Y , Cube.Pos[3].Display.X , Cube.Pos[3].Display.Y , BLACK);
	drawLine(Cube.Pos[3].Display.X , Cube.Pos[3].Display.Y , Cube.Pos[0].Display.X , Cube.Pos[0].Display.Y , BLACK);

	drawLine(Cube.Pos[4].Display.X , Cube.Pos[4].Display.Y , Cube.Pos[5].Display.X , Cube.Pos[5].Display.Y , BLACK);
	drawLine(Cube.Pos[5].Display.X , Cube.Pos[5].Display.Y , Cube.Pos[6].Display.X , Cube.Pos[6].Display.Y , BLACK);
	drawLine(Cube.Pos[6].Display.X , Cube.Pos[6].Display.Y , Cube.Pos[7].Display.X , Cube.Pos[7].Display.Y , BLACK);
	drawLine(Cube.Pos[7].Display.X , Cube.Pos[7].Display.Y , Cube.Pos[4].Display.X , Cube.Pos[4].Display.Y , BLACK);

	drawLine(Cube.Pos[0].Display.X , Cube.Pos[0].Display.Y , Cube.Pos[4].Display.X , Cube.Pos[4].Display.Y , BLACK);
	drawLine(Cube.Pos[1].Display.X , Cube.Pos[1].Display.Y , Cube.Pos[5].Display.X , Cube.Pos[5].Display.Y , BLACK);
	drawLine(Cube.Pos[2].Display.X , Cube.Pos[2].Display.Y , Cube.Pos[6].Display.X , Cube.Pos[6].Display.Y , BLACK);
	drawLine(Cube.Pos[3].Display.X , Cube.Pos[3].Display.Y , Cube.Pos[7].Display.X , Cube.Pos[7].Display.Y , BLACK);
}




float diff_reflection(struct Point P, float Reflec)
{

		//struct  coordinate Ps = {0,40,120};
		float red;
		float scaling = Lits.Pow, shift = 0.2;

		int difxyz[3];
		difxyz[0] = Lits.Pos.X - P.X;
		difxyz[1] = Lits.Pos.Y - P.Y;
		difxyz[2] = Lits.Pos.Z - P.Z;

	    float dis = sqrt( (difxyz[0]*difxyz[0]) + (difxyz[1]*difxyz[1]) + (difxyz[2]*difxyz[2]));
	    float Co = (1.0 * difxyz[2] / dis);
	    float temp1  =  Co / (dis*dis);
	    float temp =  (scaling * temp1);
	    temp = (temp < shift) ? shift : temp;
	    red = (255* Reflec * temp);
	    //printf("\n %f ",dis);

	    return red;
}

uint32_t findcolor(float x, float y,float Idiff1,float Idiff2,struct Point p1,struct Point p2)
{
	float newx,newy,ncolor;
	uint32_t color;
	float green,blue;

	newx = Idiff1 + (Idiff2 - Idiff1)*(x - p1.X)/(p2.X-p1.X);
	newy = Idiff1 + (Idiff2 - Idiff1)*(y - p1.Y)/(p2.Y-p1.Y);
	ncolor = (newx +  newy)/2.0;
	green = 0;
	blue =  0;
	color =(((uint32_t)((int)(ncolor))) << 16) +(((uint32_t)green) << 8) + ((uint32_t)blue);

	return color;

}

void DDA_line(struct Point p1, struct Point p2, float Idiff1,float Idiff2)
{
	uint32_t color;

    int dx = p2.X - p1.X;
    int dy = p2.Y - p1.Y;
    int dz = p2.Z - p1.Z;

    // calculate steps required for generating pixels
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    // calculate increment in x & y for each steps
    float Xinc = dx / (float) steps;
    float Yinc = dy / (float) steps;

    // Put pixel for each step
    float X = p1.X;
    float Y = p1.Y;
    for (int i = 0; i < steps; i++)
    {
    	color = findcolor(X,Y,Idiff1,Idiff2,p1,p2);
        drawPixel(X,Y,color);
        X += Xinc;
        Y += Yinc;
    }

}

void FillDiff(struct Point startPt,struct Point endPt,int size)//)
{
	struct Point temp,currPt,currpt;


	//struct Point ;

	{
		uint32_t color;
			float r,g=0,b=0;
				for(int i=startPt.Y; i<endPt.Y; i++)
				{
					for(int j=startPt.X; j<endPt.X; j++)
					{
						temp.X = j;
						temp.Y = i;
						temp.Z = startPt.Z;
						//r = diff_reflection(temp,L,Cube.Ref[0].R));
						r= diff_reflection(temp,0.8);
						g = 0;
					    b = 0;
					    color =(((uint32_t)((r))) << 16) +(((uint32_t)g) << 8) + ((uint32_t)b);
					    //color = (256*256*r)+(256*g)+(b);

					    CalDisPnts(temp, &currpt);
					    //printf("\n%f",r);
						drawPixel(currpt.X, currpt.Y,color);
					}
				}
	}

}

void DrawDiffSide()
{
	float Idiff[4];

	for(int i=0; i<4; i=i+1)
	{
		Idiff[i] = diff_reflection(Cube.Pos[i].World,Cube.Ref[0].R);
	}

	DDA_line(Cube.Pos[4].Display,Cube.Pos[5].Display,Idiff[0],Idiff[1]);
	DDA_line(Cube.Pos[5].Display,Cube.Pos[6].Display,Idiff[1],Idiff[2]);
	DDA_line(Cube.Pos[6].Display,Cube.Pos[7].Display,Idiff[2],Idiff[3]);
	DDA_line(Cube.Pos[7].Display,Cube.Pos[4].Display,Idiff[3],Idiff[0]);

	FillDiff(Cube.Pos[4].World,Cube.Pos[6].World,Cube.Pos[5].World.X-Cube.Pos[4].World.X);//top surface

}



typedef struct PNT
{
	float x;
	float y;
}PNT;

//Rotate point p with respect to o and angle <angle>
/*PNT rotate_PNT(PNT p, PNT o, float angle)
{
	PNT rt, t, new;

	// Convert to radians
	angle = angle * (3.14285/180);
	float s = sin(angle);
	float c = cos(angle);

	//translate PNT to origin
	t.x = p.x - o.x;
	t.y = p.y - o.y;

	rt.x = t.x * c - t.y * s;
	rt.y = t.x * s + t.y * c;

	//translate PNT back
	new.x = rt.x + o.x;
	new.y = rt.y + o.y;

	return new;
}*/

struct Point Rotate_PNT(struct Point p, struct Point o, float angle)
{
	struct Point rt, t, new;

	// Convert to radians
	angle = angle * (3.14285/180);
	float s = sin(angle);
	float c = cos(angle);

	//translate PNT to origin
	t.X = p.X - o.X;
	t.Y = p.Y - o.Y;
	t.Z = p.Z - o.Z;

	rt.X = t.X * c - t.Z * s;
	rt.Y = t.Y;
	rt.Z = t.X * s + t.Z * c;

	//translate PNT back
	new.X = rt.X + o.X;
	new.Y = rt.Y + o.Y;
	new.Z = rt.Z + o.Z;

	return new;
}

void tree(struct Point S, struct Point E, int level)
{
	PNT c, rtl, rtr, dum;
	if(level == 0)
		return;
	//int color[] = {GREEN2, GREEN2, GREEN2, GREEN3, GREEN3, GREEN3, GREEN3, GREEN4, GREEN4, GREEN4};

	PNT start, end;
	struct Point Dum, C, RTL, RTR;

	CalDisPnts(S,&Dum);
	start.x = 1.0*Dum.X;//rand() % 70;
 	start.y = 1.0*Dum.Y;//rand() % 150;*/

 	CalDisPnts(E,&Dum);
 	end.x = 1.0*Dum.X;
 	end.y = 1.0*Dum.Y;

	c.x = end.x + 0.8 * (end.x - start.x);
	c.y = end.y + 0.8 * (end.y - start.y);

	C.X = E.X + 0.8 * (E.X - S.X);
	C.Y = E.Y + 0.8 * (E.Y - S.Y);
	C.Z = E.Z + 0.8 * (E.Z - S.Z);

	drawLine(c.x, c.y, end.x, end.y, GREEN);
	tree(E, C, level - 1);

	//rtl = rotate_PNT(c, end, 30);
	RTL = Rotate_PNT(C,E,30);
	CalDisPnts(RTL,&Dum);
	dum.x = 1.0*Dum.X;
	dum.y = 1.0*Dum.Y;

	drawLine(dum.x, dum.y, end.x, end.y, GREEN);
	tree(E, RTL, level - 1);

	//rtr = rotate_PNT(c, end, 330);

	RTR = Rotate_PNT(C,E,330);
	CalDisPnts(RTR,&Dum);
	dum.x = 1.0*Dum.X;
	dum.y = 1.0*Dum.Y;
	drawLine(dum.x, dum.y, end.x, end.y, RED);
	tree(E, RTR, level - 1);
}


void drawTreetwig(PNT start, PNT end, uint16_t color, uint8_t thickness)
{
	int i;
	for(i = 0; i < thickness; i++)
		drawLine(start.x, start.y + i, end.x, end.y + i, color);
}


void TreeBT(SURFACE s,int *P, int *mM)
{
	//
}
void TreeD(SURFACE s, int n,int Siz)
{
	int pnts[4],minmax[6];
	FindSurf(s,&pnts,&minmax);

	//int pnts[4];
	struct Views Area[4];
	struct Point Pnts[4];	//3D Point

	CalDisPnts(Cube.Pos[pnts[0]].World,&Area[0].Display);
	CalDisPnts(Cube.Pos[pnts[1]].World,&Area[1].Display);
	CalDisPnts(Cube.Pos[pnts[2]].World,&Area[2].Display);
	CalDisPnts(Cube.Pos[pnts[3]].World,&Area[3].Display);


	struct Point Base,Top,Dum;
	/*Base.X = (Cube.Pos[6].World.X - Cube.Pos[0].World.X)/2;
	Base.Y = Cube.Pos[6].World.Y;
	Base.Z = 0;

	Top.X = (Cube.Pos[6].World.X - Cube.Pos[0].World.X)/2;
	Top.Y = Cube.Pos[6].World.Y;
	Top.Z = Cube.Pos[6].World.Z / (n+1);*/

	Base.X = (Cube.Pos[pnts[0]].World.X - Cube.Pos[0].World.X)/2;
	Base.Y = Cube.Pos[pnts[0]].World.Y;
	Base.Z = Cube.Pos[0].World.Z;

	Top.X = (Cube.Pos[pnts[0]].World.X - Cube.Pos[0].World.X)/2;
	Top.Y = Cube.Pos[pnts[0]].World.Y;
	Top.Z = Cube.Pos[0].World.Z+Siz;// / (n+1);

	PNT start, end;
	CalDisPnts(Base,&Dum);
	start.x = 1.0*Dum.X;//rand() % 70;
 	start.y = 1.0*Dum.Y;//rand() % 150;*/

 	CalDisPnts(Top,&Dum);
 	end.x = 1.0*Dum.X;
 	end.y = 1.0*Dum.Y;

 	drawTreetwig(start, end, RED, 2);
 	tree(Base, Top, n+1);

}



void square(SURFACE s, int l)
{

	int pnts[4];
	int mM_ind[6];		// Min Max Index

	FindSurf(s,&pnts,&mM_ind);


	struct Views Area[4];
	struct Point Pnts[4];	//3D Point

	CalDisPnts(Cube.Pos[pnts[0]].World,&Area[0].Display);
	CalDisPnts(Cube.Pos[pnts[1]].World,&Area[1].Display);
	CalDisPnts(Cube.Pos[pnts[2]].World,&Area[2].Display);
	CalDisPnts(Cube.Pos[pnts[3]].World,&Area[3].Display);



	uint32_t color = RED;
	int x = Area[1].Display.X;
	int y = Area[1].Display.Y;
	int siz = 40;//((Area[0].Display.X-Area[1].Display.X)/2);
	float lamda;
	if(l == 1)
	{
		lamda = 0.2;
	}
	else
	{
		lamda = 0.8;
	}
	//int square[10] = {x+siz,y,x,y,x,y+siz,x+siz,y+siz,x+siz,y};
	int square[10] = {Area[0].Display.X,Area[0].Display.Y,x,y,Area[2].Display.X,Area[2].Display.Y,Area[3].Display.X,Area[3].Display.Y,Area[0].Display.X,Area[0].Display.Y};
	for (int i=0; i<7; i+=2)
	{
		drawLine(square[i],square[(i+1)%8],square[(i+2)%8],square[(i+3)%8], color);
	}
	//printf("square point: (%d,%d), (%d,%d), (%d,%d), (%d,%d) \n",square[0],square[1],square[2],square[3],square[4],square[5],square[6],square[7]);
	for (int i=0; i<11; i++)
	{
		for(int j=0; j<7; j+=2)
		{
			square[(j)] += lamda * (square[((j+2))] - square[(j)]);
			//printf("x%d: %d \t", (j),square[(j)]);
			square[((j+1))] += lamda * (square[((j+3))] - square[((j+1))]);
			//printf("y%d: %d \n", (j),square[(j+1)]);

			/*
			 * 0 - > sq[0%8]  = sq[0%8] + l * (sq[2%8] - sq[0%8]) = 77 + 0.8 * (17 -  77) = 77 - 48 = 29
			 * 		 sq[1%8]  = sq[1%8] + l * (sq[3%8] - sq[1%8]) =
			 * */

		}
		for(int k=0; k<7; k+=2)
		{
			drawLine(square[k],square[(k+1)%8],square[(k+2)%8],square[(k+3)%8], color);
			//lcddelay(100);

		}
		square[8]=square[0];
		square[9]=square[1];

		//printf("square point: (%d,%d), (%d,%d), (%d,%d), (%d,%d) \n",square[0],square[1],square[2],square[3],square[4],square[5],square[6],square[7]);
	}

}


void DrawInitial(SURFACE s, int n[],uint32_t COL)
{

	int pnts[4];
	int mM_ind[6];		// Min Max Index

	FindSurf(s,&pnts,&mM_ind);

	struct Views matrix[5][5];

	matrix[0][0].World = Cube.Pos[pnts[0]].World;
	matrix[0][4].World = Cube.Pos[pnts[1]].World;
	matrix[4][4].World = Cube.Pos[pnts[2]].World;
	matrix[4][0].World = Cube.Pos[pnts[3]].World;

	CalDisPnts(matrix[0][0].World,&matrix[0][0].Display);
	CalDisPnts(matrix[0][4].World,&matrix[0][4].Display);
	CalDisPnts(matrix[4][0].World,&matrix[4][0].Display);
	CalDisPnts(matrix[4][4].World,&matrix[4][4].Display);

	int w = matrix[4][0].Display.X - matrix[0][0].Display.X;
	int h = matrix[0][4].Display.Y - matrix[0][0].Display.Y;
	printf("%i,%i\n",w,h);
	for (int i =1;i<4;i++)
	{
		for(int j=0;j<5;j++)
		{
			matrix[i][j].Display.X = matrix[0][0].Display.X+(i*w/4);
			matrix[i][j].Display.Y = matrix[0][0].Display.Y+(j*h/4);
		}
	}

	for (int i=1;i<4;i++)
	{
		matrix[0][i].Display.X = matrix[0][0].Display.X;
		matrix[0][i].Display.Y = matrix[0][0].Display.Y+(i*h/4);
		matrix[4][i].Display.X = matrix[4][0].Display.X;
		matrix[4][i].Display.Y = matrix[0][0].Display.Y+(i*h/4);
	}

	/*for (int i=0;i<5;i++){
		for (int j=0;j<5;j++){
			CalDisPnts(matrix[i][j].World,&matrix[i][j].Display);
		}
	}*/
	int x= n[0];
	int y = n[1];
	printf("%i,%i\n",x,y);
	printf("%i,%i\n",matrix[x][y].Display.X,matrix[x][y].Display.Y);
	int len = (sizeof n / sizeof n[0])-1;
	for (int i=2;i<len;i++){
		switch(n[i]){

			case 0:
			{
				drawLine(matrix[x][y].Display.X,matrix[x][y].Display.Y,matrix[x][y-1].Display.X,matrix[x][y-1].Display.Y,COL);
				y-=1;
				break;
			}
			case 1:
			{
				drawLine(matrix[x][y].Display.X,matrix[x][y].Display.Y,matrix[x+1][y].Display.X,matrix[x+1][y].Display.Y,COL);
				x+=1;
				break;
			}
			case 2:
			{
				drawLine(matrix[x][y].Display.X,matrix[x][y].Display.Y,matrix[x][y+1].Display.X,matrix[x][y+1].Display.Y,COL);
				y+=1;
				break;
			}
			case 3:
			{
				drawLine(matrix[x][y].Display.X,matrix[x][y].Display.Y,matrix[x-1][y].Display.X,matrix[x-1][y].Display.Y,COL);
				x-=1;
				break;
			}
			default:
			{
				break;
			}
			}
		printf("%i,%i\n",x,y);
	}

}

void displayLetterC(){
	struct Views p1,p2,p3,p4;
	//struct view Pwl1 = {p1.};
	//pworld Pwl1 = {60,20,110};



	//pworld Pwl2 = {60,55,110};
	//pworld Pwl3 = {15,55,110};
	//pworld Pwl4 = {15,20,110};
	uint32_t color = 0x00FF00;


	for(float i=0;i<5.0;i=i+0.5){
		p1.World.X = 60+i;
		p1.World.Y = 20;
		p1.World.Z = 110;

		p2.World.X = 60+i;
		p2.World.Y = 55;
		p2.World.Z = 110;

		p3.World.X = 15;
		p3.World.Y = 55+i;
		p3.World.Z = 110;

		p4.World.X = 15;
		p4.World.Y = 20+i;
		p4.World.Z = 110;

		/*pworld Pwl1 = {60+i,20,110};
		pworld Pwl2 = {60+i,55,110};
		pworld Pwl3 = {15,55+i,110};
		pworld Pwl4 = {15,20+i,110};*/
		CalDisPnts(p1.World,&p1.Display);
		CalDisPnts(p2.World,&p2.Display);
		CalDisPnts(p3.World,&p3.Display);
		CalDisPnts(p4.World,&p4.Display);
		/*pperspective Ppl1 = Transform3D(Pwl1);
		pperspective Ppl2 = Transform3D(Pwl2);
		pperspective Ppl3 = Transform3D(Pwl3);
		pperspective Ppl4 = Transform3D(Pwl4);

		drawLineBetweenPoints(Ppl1,Ppl2,color);
		drawLineBetweenPoints(Ppl2,Ppl3,color);
		drawLineBetweenPoints(Ppl1,Ppl4,color);*/

		drawLine(p1.Display.X,p1.Display.Y,p2.Display.X,p2.Display.Y,color);
		drawLine(p2.Display.X,p2.Display.Y,p3.Display.X,p3.Display.Y,color);
		drawLine(p1.Display.X,p1.Display.Y,p4.Display.X,p4.Display.Y,color);




	}



}

int main (void)
{
uint32_t pnum = PORT_NUM;
	pnum = 0 ;

	if ( pnum == 0 )
		SSP0Init();
	else
		puts("Port number is not correct");

	lcd_init();

	fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, SUNSET);	// clear screen

	struct Point arbP;

	SetOrigin(&O);
	SetDisplay(Landscape_Flipped,MM);
	SetAxes(200);	//axis height

	arbP.X = 40;
	arbP.Y = 60;
	arbP.Z = 220;
	SetLightS(arbP,8000.0);	// set light source

	arbP.X = 200;
	arbP.Y = 200;
	arbP.Z = 200;
	SetCamP(arbP);			// set Camera
	SetCamF(75.0);

	arbP.X = 0;
	arbP.Y = 0;
	arbP.Z = 10;
	SetCube(arbP,100);		// set Cube
	Cube.Solid[0] = T;
	Cube.Solid[1] = T;
	Cube.Solid[2] = T;

	SetMatrixWtS();	// set martix 3d
//----------------------------------------------------------------------------------------------------------------


	DrawShadow();	 //Draws Shadow on XY Plane

	DrawOrigin();

	CalcDispCube(&Cube);
	DrawCubeW(&Cube);
	DrawSid(Surf1,GREEN);
	DrawSid(Surf2,BLUE);
	//DrawSid(Surf0,MAGENTA);

	DrawDiffSide();




	//DrawSides(&Cube,&Matrix,&Lits,&Camera,&O);

	TreeD(Surf2, 5, 25);
	square(Surf1,2);

	DrawBeam(YELLOW);
	//int Initials = {2,4,3,3,3,3};
	//DrawInitial(Surf1,Initials,YELLOW);
	displayLetterC();

	// P
	/*
		 drawLine(10,10,10,25,BLACK);
		 drawLine(10,10,18,10,BLACK);
		 drawLine(10,18,18,18,BLACK);
		 drawPixel(19, 10, BLACK);
		 drawPixel(19, 11, BLACK);
		 drawPixel(20, 12, BLACK);
		 drawPixel(21, 13, BLACK);
		 drawPixel(21, 14, BLACK);
		 drawPixel(20, 15, BLACK);
		 drawPixel(19, 16, BLACK);
		 drawPixel(19, 17, BLACK);
		 // .
		 drawPixel(18, 25, BLACK);
		 drawPixel(19, 25, BLACK);

		 // K
		 drawLine(26,10,26,25,BLACK);
		 drawLine(35,10,26,18,BLACK);
		 drawLine(35,25,29,16,BLACK);
*/
		printf("Done");

		return 0;
}


//-----------------------------------------------------------------------------
// includes
//

#include "sine.h"


//-----------------------------------------------------------------------------
// defines
//

#define SINE_CHANNELS 3
#define magic   0x00012000
#define limithi 0x01800000
#define limitlo 0x0000C000 
#define initial 0x00C06000


//-----------------------------------------------------------------------------
// prototypes
//

void sine_DoTheta (long *theta, long *dtheta);


//-----------------------------------------------------------------------------
// globals
//

extern unsigned char red, green, blue;
static unsigned short LFSR16;
static long theta[SINE_CHANNELS];
static long dtheta[SINE_CHANNELS];


//-----------------------------------------------------------------------------
// constants
//

static const unsigned char sinlookup[256] = {
	0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x99, 0x9c, 0x9f, 0xa2, 
	0xa5, 0xa8, 0xab, 0xae, 0xb1, 0xb4, 0xb6, 0xb9, 0xbc, 0xbf, 0xc2, 0xc4, 
	0xc7, 0xc9, 0xcc, 0xce, 0xd1, 0xd3, 0xd6, 0xd8, 0xda, 0xdc, 0xdf, 0xe1, 
	0xe3, 0xe5, 0xe7, 0xe8, 0xea, 0xec, 0xee, 0xef, 0xf1, 0xf2, 0xf3, 0xf5, 
	0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfd, 0xfe, 0xfe, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfd, 
	0xfd, 0xfc, 0xfb, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf5, 0xf4, 0xf3, 0xf1, 
	0xf0, 0xee, 0xed, 0xeb, 0xe9, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdb, 
	0xd9, 0xd7, 0xd5, 0xd2, 0xd0, 0xcd, 0xcb, 0xc8, 0xc6, 0xc3, 0xc0, 0xbe, 
	0xbb, 0xb8, 0xb5, 0xb2, 0xaf, 0xac, 0xa9, 0xa6, 0xa3, 0xa0, 0x9d, 0x9a, 
	0x97, 0x94, 0x91, 0x8e, 0x8b, 0x88, 0x84, 0x81, 0x7e, 0x7b, 0x78, 0x75, 
	0x72, 0x6e, 0x6b, 0x68, 0x65, 0x62, 0x5f, 0x5c, 0x59, 0x56, 0x53, 0x50, 
	0x4d, 0x4a, 0x47, 0x45, 0x42, 0x3f, 0x3c, 0x3a, 0x37, 0x34, 0x32, 0x2f, 
	0x2d, 0x2b, 0x28, 0x26, 0x24, 0x22, 0x20, 0x1d, 0x1b, 0x1a, 0x18, 0x16, 
	0x14, 0x12, 0x11, 0x0f, 0x0e, 0x0c, 0x0b, 0x0a, 0x09, 0x07, 0x06, 0x05, 
	0x04, 0x04, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04, 0x05, 
	0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x10, 0x11, 0x13, 0x15, 
	0x16, 0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22, 0x24, 0x27, 0x29, 0x2b, 0x2e, 
	0x30, 0x33, 0x35, 0x38, 0x3a, 0x3d, 0x40, 0x43, 0x45, 0x48, 0x4b, 0x4e, 
	0x51, 0x54, 0x57, 0x5a, 0x5d, 0x60, 0x63, 0x66, 0x69, 0x6c, 0x6f, 0x73, 
	0x76, 0x79, 0x7c, 0x7f
};


//-----------------------------------------------------------------------------
// Init this mode
//

void sine_Init (void)
{
    int i;

    for (i = 0; i < SINE_CHANNELS; i++) {
        theta[i] = 0;
        dtheta[i] = initial;
    }

	red = 0;
	green = 0;
	blue = 0;
	
	LFSR16 = 0xFFFF;
}


//-----------------------------------------------------------------------------
// Compute next step of this mode
//

void sine_Tick (void)
{
    int i;

    for (i = 0; i < SINE_CHANNELS; i++) {
        sine_DoTheta (&theta[i],&dtheta[i]);
    }

	red   = sinlookup[ ((unsigned long)theta[0]) >> 24 ];
	green = sinlookup[ ((unsigned long)theta[1]) >> 24 ];
	blue  = sinlookup[ ((unsigned long)theta[2]) >> 24 ];
}


void sine_DoTheta (long *theta, long *dtheta)
{
	if (LFSR16 & 1) {
		LFSR16 = (LFSR16 >> 1) ^ 0xB400;
	} else {
		LFSR16 = (LFSR16 >> 1);
	}

   *theta = *theta + *dtheta;
   
	if (LFSR16 & 1) {
		*dtheta = *dtheta + magic;
        if (*dtheta > limithi) {
            *dtheta = limithi;
        }
	} else {
		*dtheta = *dtheta - magic;
        if (*dtheta < limitlo) {
            *dtheta = limitlo;
        }
    }
}

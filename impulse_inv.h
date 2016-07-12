/* =================================================================================
File name:        IMPULSE_INV.H
===================================================================================*/

#ifndef __IMPULSE_INV_H__
#define __IMPULSEI_INV_H__

typedef struct { Uint32  Period;		// Input: Period of output impulse in a number of sampling cycles (Q0)			
				 Uint32  Out;      		// Output: Impulse generator output - Q0 (0x00000000 or 0x00007FFF)
		  	  	 Uint32  Counter;   	// Variable: Impulse generator counter (Q0)
			   } IMPULSEINV;

/*-----------------------------------------------------------------------------
Default initalizer for the IMPULSE object.
-----------------------------------------------------------------------------*/                     
#define IMPULSEINV_DEFAULTS { 5,0,0 }

/*------------------------------------------------------------------------------
	IMPULSE Macro Definition
------------------------------------------------------------------------------*/

#define IMPULSEINV_MACRO(v)										\
  																\
  v.Out = 0;      /* Always clear impulse output at entry*/		\
  v.Counter++;    /* Increment the skip counter*/				\
																\
  if (v.Counter >= v.Period)									\
  {																\
     v.Out = 0x00007FFF;										\
     v.Counter = 0;        /* Reset counter*/					\
  } 

#endif // __IMPULSE_H__

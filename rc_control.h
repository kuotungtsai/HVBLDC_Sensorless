/*===============================================
File name:    RC_CONTROL.H
=================================================*/
#ifndef __RC_CNT_H__
#define __RC_CNT_H__
/*-------------------------------------------------
		  ___________               ______
		 |           |             |      |
	_____|		     |_____________|      |_____

	-----> PWM width  <------

	the Radio controller will send signal to receiver
	and the receiver will output PWM signal to MCU
	and use GPIO port to detect the width of the high
	level
					update 2016/8/2 by Kuotung Tsai
----------------------------------------------------*/


typedef struct{Uint16 PWM_Status;
			   Uint16 PWM_width;
			   Uint16 Counter;
			   _iq   duty;
			} RCCtrl;

/*-----------------------------------------------------------------------------
Default initalizer for the RCCtrl object.
-----------------------------------------------------------------------------*/
#define RCCtrl_DEFAULTS {0,40,0,0}

/*---------------------------------------------------
	RC_CONTROL Macro Definition
----------------------------------------------------*/

#define RCONTROL_MACRO(pin,v)												\
	v.PWM_Status=0x03&((v.PWM_Status<<1)|GpioDataRegs.GPADAT.bit.pin);		\
	if(v.PWM_Status==0x0)		/*if gpio is high*/							\
		v.Counter=0;														\
	else if(v.PWM_Status==0x03)		/*if gpio is high*/							\
		v.Counter+=1;															\
	else if(v.PWM_Status==0x02)		            /*if gpio is low*/    		\
		{v.PWM_width=v.Counter;												\
		v.duty=_IQ((v.PWM_width-43)*0.028);      /*43-78*/     \
		}\

#endif // __RC_CNT_H__

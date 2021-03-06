/* ==============================================================================
System Name:      HVBLDC_Sensorless

File Name:	  	HVBLDC_Sensorless.C

Description:	Primary system file for the Real Implementation of Sensorless
          		Trapeziodal Control of Brushless DC Motors (BLDC)

=================================================================================  */

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "IQmathLib.h"
#include "HVBLDC_Sensorless.h"
#include "HVBLDC_Sensorless-Settings.h"
#include <math.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
//#pragma CODE_SECTION(i2c_int1a_isr,"ramfuncs");
void MemCopy();
void InitFlash();
#endif

// Prototype statements for functions found within this file.
interrupt void MainISR(void);
void DeviceInit();
void ramp_initial(void);
void Servo_epwm_initial(void);


//interrupt void DebounceISR(void);

// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;


int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;

// Global variables used in this system

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint16 BackTickerTrue = 0;
Uint16 BackTickerFalse = 0;
Uint32 DebounceIsrTicker = 0;
Uint32 CtrlSwitchRemainTime=0;
Uint32 BLDC_decelerateTicker=0;//this ticker is for decelerate
Uint32 TestProbe=0;

Uint32 VirtualTimer = 0;
Uint16 SpeedLoopFlag1 = FALSE;
Uint16 SpeedLoopFlag2 = FALSE;
int16  DFuncDesired1=0x01000;//0x3FFF;      	// Desired duty cycle (Q15), 0x04F0 FOR 2200KV, 0x0CCD FOR 1100KV, 0x1300 FOR 1100KV w F
int16  DFuncDesired2 =0x1000;
int16  DfuncTesting =0x1666;		//0x1300; 1.6%   1F0

Uint16 RotDireChangFlag=0;
Uint16 BLDC_CtrlMod=0;//1:clockwise  0:counter_clockwise
Uint16 BLDC_RotDirec=0;
Uint16 NewBLDCmodStamp=0;//sampling the physical switch state
Uint16 OldBLDCmodStamp=0;
Uint16 Old_CtrlSwitchState=0;//save the physical switch state
Uint16 New_CtrlSwitchState=0;
Uint16 BLDC_decelDoneFlag=0;


Uint16 AlignFlag = 0x000F;
Uint16 LoopCount = 0;

//Uint16 little_driver_enable=0;//1: enable 0:diable

#if (BUILDLEVEL<= LEVEL3)
Uint32 CmtnPeriodTarget = 0x00000320;//150****460
Uint32 CmtnPeriodSetpt = 0x00001194;//200
Uint32 RampDelay = 10;//10
#else
Uint32 CmtnPeriodTarget = 0x0000012c;//12C;//0x0250 FOR 2200KV 1CC//1f4
Uint32 CmtnPeriodSetpt = 0x00000400;//0x000000400
Uint32 RampDelay = 25;
#endif

_iq SpeedRef1=_IQ(0.4);
_iq SpeedRef2=_IQ(0.4);
_iq Rolling=0;
_iq thrust=0;
//_iq SpeedRef=_IQ(0.3);

//int16 DlogCh1 = 0;
//int16 DlogCh2 = 0;
//int16 DlogCh3 = 0;
//int16 DlogCh4 = 0;


// Used for ADC Configuration
int 	ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};//must be tuned**
int     ACQPS[16]   = {6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};

/*------variable for enable the program 20160321-------*/
//volatile Uint16 EnableSwitch = FALSE;//FALSE, This will read Gpio value later
volatile Uint16 EnableFlag =TRUE;//FALSE


// Instance PID regulator to regulate the DC-bus current and speed
PI_CONTROLLER pid1_spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pid2_spd = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;
PWMGEN pwm2 = PWMGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS;

// Instance a RAMP2 Module*******************//change rmp2 id to ramp#
RMP2 rmp2_1 = RMP2_DEFAULTS;
RMP2 rmp2_2 = RMP2_DEFAULTS;

// Instance a RAMP3 Module******************
RMP3 rmp3_1 = RMP3_DEFAULTS;
RMP3 rmp3_2 = RMP3_DEFAULTS;

// Instance a MOD6 Module
MOD6CNT mod1 = MOD6CNT_DEFAULTS;

// Instance a MOD6INV Module
MOD6CNTINV modinv1=MOD6CNTINV_DEFAULTS;

// Instance a IMPULSE Module
IMPULSE impl1 = IMPULSE_DEFAULTS;
IMPULSEINV impl2 = IMPULSEINV_DEFAULTS;

// Instance a COMTRIG Module
CMTN cmtn1 = CMTN_DEFAULTS;

CMTN_INV cmtninv1 = CMTN_INV_DEFAULTS;

// Instance a SPEED_PR Module
SPEED_MEAS_CAP speed1 = SPEED_MEAS_CAP_DEFAULTS;
SPEED_MEAS_CAP speed2 = SPEED_MEAS_CAP_DEFAULTS;

// Create an instance of DATALOG Module
//DLOG_4CH dlog = DLOG_4CH_DEFAULTS;

//Radio control pwm macro
RCCtrl ThrustStick=RCCtrl_DEFAULTS;
RCCtrl RollStick=RCCtrl_DEFAULTS;
RCCtrl YawStick=RCCtrl_DEFAULTS;
RCCtrl PitchStick=RCCtrl_DEFAULTS;
RCCtrl AutoStick=RCCtrl_DEFAULTS;
//
//void I2CA_Init(void);
//void InitI2CGpio(void);
//Uint16 I2CA_WriteData(struct I2CMSG *msg);
//Uint16 I2CA_ReadData(struct I2CMSG *msg);
void PieCntlInit(void);
void PieVectTableInit(void);
//interrupt void i2c_int1a_isr(void);
//
//#define I2C_SLAVE_ADDR   0x53 //i2c device address
//#define I2C_NUMBYTES    2    //the byte of read datasheet
//#define X_HIGH_ADDR 0x32
//#define X_LOW_ADDR 0x33
//#define Y_HIGH_ADDR 0x34
//#define Y_LOW_ADDR 0x35
//#define Z_HIGH_ADDR 0x36
//#define Z_LOW_ADDR 0x37
//
///*declare the channel for select*/
//Uint16 I2C_read_channel=0x00;
//#define X 0X00
//#define Y 0X01
//#define Z 0X02
////Uint16 I2C_HIGH_ADDR;
////Uint16 I2C_LOW_ADDR;
//// Global variables
////this is copy the arduino example, for enable the measurement or something
//struct I2CMSG I2cMsgOut1 = { I2C_MSGSTAT_SEND_WITHSTOP,
//								I2C_SLAVE_ADDR,
//								1,// # of byte
//								0X2D, //I2C_EEPROM_HIGH_ADDR,
//								8,                   // Msg Byte 1
//							};// Msg Byte 1
////this is read the x axis date address
//struct I2CMSG I2cMsgIn1 = { I2C_MSGSTAT_SEND_NOSTOP,
//							I2C_SLAVE_ADDR,
//							I2C_NUMBYTES,
//							X_HIGH_ADDR,
//							X_LOW_ADDR};
//
//struct I2CMSG *CurrentMsgPtr;				// Used in interrupts
//
//double g[3];//the x axis gravity data
//Uint16 virtualtimer = 0;
//Uint16 readcounter = 0x00;//for lower the repetition rate for read
//Uint16 device_intial_done_flag=0xFF;
//Uint16 I2CA_Read_fail=0x00;//for check whether device is connected of not
//Uint16 i2c_read_return=0xff;
//Uint16 i2c_isr_ticker=0;
void main(void){


//	Uint16 Error;
//	Uint16 i;
//
//	CurrentMsgPtr = &I2cMsgOut1;
	DeviceInit();	// Device Life support & GPIO
// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler

#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

//	InitI2CGpio();


//// Tasks State-machine init
//	Alpha_State_Ptr = &A0;
//	A_Task_Ptr = &A1;
//	B_Task_Ptr = &B1;

	//EnableFlag =GpioDataRegs.GPADAT.bit.GPIO15;// Get value from Gpio, and Gpio value is control by physical switch

   // Waiting for enable flag set
	/*Prevent some one forget to turn off the enable switch,
	 * this program need to start at enable switch off state*/
//	while(EnableFlag==TRUE)
//	{
//		BackTickerTrue++;
//	}
	GpioDataRegs.GPADAT.bit.GPIO21=1;
	while (EnableFlag==FALSE)
    {
      BackTickerFalse++;
    }

// Initialize all the Device Peripherals:
// This function is found in DSP280x_CpuTimers.c
   InitCpuTimers();
// Configure CPU-Timer 0 to interrupt every ISR Period:
// 60MHz CPU Freq, ISR Period (in uSeconds)
// This function is found in DSP280x_CpuTimers.c
   ConfigCpuTimer(&CpuTimer0, 60, 1000/ISR_FREQUENCY);
   StartCpuTimer0();
// Configure CPU-Timer 1,2 for background loops
   ConfigCpuTimer(&CpuTimer1, 60, 1000);
   ConfigCpuTimer(&CpuTimer2, 60, 50000);
   StartCpuTimer1();
   StartCpuTimer2();





//   PieCntlInit();
// Disable CPU interrupts and clear all CPU interrupt flags:
//   PieVectTableInit();

// Reassign ISRs.
        // Reassign the PIE vector for TINT0 to point to a different
        // ISR then the shell routine found in DSP280x_DefaultIsr.c.
        // This is done if the user does not want to use the shell ISR routine
        // but instead wants to use their own ISR.

	EALLOW;	// This is needed to write to EALLOW protected registers
//	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	PieVectTable.TINT0 = &MainISR;
//	PieVectTable.TINT1 = &DebounceISR;

	EDIS;   // This is needed to disable write to EALLOW protected registers


	// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
//		PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
	// Enable PIE group 1 interrupt 7 for TINT0
    PieCtrlRegs.PIEIER1.all = M_INT7;


// Enable CPU INT8 which is connected to PIE group 8
//	IER |= M_INT8;
	// Enable CPU INT1 for TINT0:
	IER |= M_INT1;
// Enable Global realtime interrupt DBGM

// Enable CPU INT13 for TINT1:
//	IER |= M_INT13;
//	IER=0x1081;
// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

// Initialize PWM module
    pwm1.PeriodMax = (SYSTEM_FREQUENCY/PWM_FREQUENCY)*1000;  // Asymmetric PWM 1
    pwm1.DutyFunc  = ALIGN_DUTY;            				 // DutyFunc = Q15
	BLDCPWM_INIT_MACRO(1,2,3,pwm1)

    pwm2.PeriodMax = (SYSTEM_FREQUENCY/PWM_FREQUENCY)*1000;  // Asymmetric PWM 2
    pwm2.DutyFunc  = ALIGN_DUTY;            				 // DutyFunc = Q15
	BLDCPWM_INIT_MACRO(4,5,6,pwm2)

// Initialize DATALOG module

//    dlog.iptr1 = &DlogCh1;
//    dlog.iptr2 = &DlogCh2;
//    dlog.iptr3 = &DlogCh3;
//    dlog.iptr4 = &DlogCh4;
//    dlog.trig_value = 0x1;
//    dlog.size = 0x0C8;
//    dlog.prescalar = 25;
//    dlog.init(&dlog);

// Initialize ADC module

// For the kits < Rev 1.1 -------------------------------------------------
//FOR BLDC1 FEEDBACK ADC
     ChSel[0]=10;	// Dummy meas. avoid 1st sample issue Rev0 Picollo*/
	 ChSel[1]=10;	// ChSelect: ADC B2-> Motor1 Phase A Voltage
	 ChSel[2]=9;	// ChSelect: ADC B1-> Motor1 Phase B Voltage
	 ChSel[3]=8;	// ChSelect: ADC B0-> Motor1 Phase C Voltage
//FOR BLDC2 FEEDBACK ADC
//
	 ChSel[5]=2;	// ChSelect: ADC A2-> Motor1 Phase A Voltage
	 ChSel[6]=1;	// ChSelect: ADC A1-> Motor1 Phase B Voltage
	 ChSel[7]=0;	// ChSelect: ADC A0-> Motor1 Phase C Voltage

	 ChSel[10]=5;  // ChSelect: ADC A5-> SPEEDREF
//-------------------------------------------------------------------------

	 ADC_MACRO_INIT(ChSel,TrigSel,ACQPS)

 // Initialize the SPEED_PR module
 	speed1.InputSelect = 1;
 	speed1.BaseRpm = 120*(BASE_FREQ/POLES);
 	speed1.SpeedScaler = (Uint32)(ISR_FREQUENCY/(1*BASE_FREQ*0.001));

 	speed2.InputSelect = 1;
 	speed2.BaseRpm = 120*(BASE_FREQ/POLES);
 	speed2.SpeedScaler = (Uint32)(ISR_FREQUENCY/(1*BASE_FREQ*0.001));

 	/*---initiate the parameter of ramp----*/
 	 ramp_initial();
 	/*end of ramp initiating*/

// Initialize the PI module for speed
	pid1_spd.Kp   = _IQ(0.8);//0.2
	pid1_spd.Ki   = _IQ(T/0.9);///T/0.5);//T/0.1   T/0.8 FOR 1100KV
	pid1_spd.Umax = _IQ(0.99);//0.99
	pid1_spd.Umin = _IQ(0);//0

	pid2_spd.Kp   = _IQ(0.8);//0.25
	pid2_spd.Ki   = _IQ(T/0.9);//0.1
	pid2_spd.Umax = _IQ(0.99);
	pid2_spd.Umin = _IQ(0);
	Servo_epwm_initial();
	RollStick.duty=_IQ(0.5);

	//get BLDC_CtrlMod value from Gpio
// IDLE loop. Just sit and loop forever:
	for(;;)  //infinite loop
	{
		BackTicker++;
		/*------------------------------------------------------------------------------------
				The following code is using for control the bldc motor by Futaba T6K controller
				since the bldc motor control is referenced by the spd.ref, so Speedref has the thrustStick term
				and the rolling is controlled by the motor as well, so the rolling term needed to be considered
				and the
				 --------------------------------------------------------------------------------------*/
		if(AutoStick.PWM_width<45)
		{
			GpioDataRegs.GPADAT.bit.GPIO21=1;
		}

				Rolling=_IQmpy(RollStick.duty-_IQ(0.5),_IQ(0.5));
				thrust=ThrustStick.duty+_IQ(0.1);
				SpeedRef1=_IQsat(thrust+Rolling,_IQ(0.95),_IQ(0.1));
				SpeedRef2=_IQsat(thrust-Rolling,_IQ(0.95),_IQ(0.1));
				EPwm7Regs.CMPA.half.CMPA=47000+4000*_IQ24toF(PitchStick.duty-_IQ(0.5))+2000*_IQ24toF(YawStick.duty-_IQ(0.5));
				EPwm7Regs.CMPB=47000+4000*_IQ24toF(PitchStick.duty-_IQ(0.5))-2000*_IQ24toF(YawStick.duty-_IQ(0.5));
	}
} //END MAIN CODE


//
//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;		// Allow C state tasks
}
/*
void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;		// Back to state A0 tasks
}
*/
//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A3
	A_Task_Ptr = &A3;
	//-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}



//=================================================================================
//	B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{/*
	if(EPwm1Regs.TZFLG.bit.OST==0x1)			// TripZ for PWMs is low (fault trip)
	  { TripFlagDMC=1;
	  GpioDataRegs.GPBTOGGLE.bit.GPIO42 = 1;
	  }

	if(GpioDataRegs.GPADAT.bit.GPIO15 == 1)		// Over Current Prot. for Integrated Power Module is high (fault trip)
	  { TripFlagDMC=1;
	  GpioDataRegs.GPBTOGGLE.bit.GPIO44 = 1;
	  }

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	   // Turn on/off LD3 on the controlCARD

*/
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;
	//-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

	//-----------------

	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;
	//-----------------
}



// ==========================================================================
// =============================  MainISR ===================================
// ==========================================================================

interrupt void MainISR(void)
{

// Verifying the ISR
    IsrTicker++;
//    BLDC_CtrlMod=GpioDataRegs.GPADAT.bit.GPIO16;

 /*-------------decide the rotate direction of bldc---------*/
		if(BLDC_RotDirec==1)
		{
			mod1.Direction=1;
			modinv1.Direction=0;
		}
		else
		{
			mod1.Direction=0;
			modinv1.Direction=1;
		}
/*-----------------end of rotate direction decide-----------*/

	// Initial Rotor Alignment Process
    if (AlignFlag != 0)
    {
    	GpioDataRegs.GPADAT.bit.GPIO21=0;
    	rmp3_1.Ramp3DoneFlag=0;
    	rmp3_2.Ramp3DoneFlag=0;
    	CmtnPeriodSetpt = 0x00000400;

		SpeedRef1=_IQ(0.3);
		SpeedRef2=_IQ(0.3);
//		SpeedRef=_IQ(0.3);
		SpeedLoopFlag1=FALSE;
		SpeedLoopFlag2=FALSE;
		mod1.Counter = 0;
		pwm1.CmtnPointer = 0;

		modinv1.Counter = 0;//inv
		pwm2.CmtnPointer = 0;

      BLDCPWM_MACRO(1,2,3,pwm1)
      BLDCPWM_MACRO(4,5,6,pwm2)

      if (VirtualTimer > 0x7FFE)//0x5997)//0x7FFE
      {
    	  if (LoopCount != LOOP_CNT_MAX)
           LoopCount++;
        else
         {
           AlignFlag = 0;
    	   LoopCount=0;
           VirtualTimer = 0;
      	   VirtualTimer++;
	       VirtualTimer &= 0x00007FFF;
         }
      }
      else
      {
      	 VirtualTimer++;
	     VirtualTimer &= 0x00007FFF;
      }
    }
   else
    {


//	   SpeedRef1=_IQsat(_IQ12toIQ(AdcResult.ADCRESULT10),_IQ(0.99),_IQ(0.1));//_IQ(0.1)+_IQ12toIQ(AdcResult.ADCRESULT10)+_IQ(0.1);
//	   SpeedRef2=_IQsat(_IQ12toIQ(AdcResult.ADCRESULT10),_IQ(0.99),_IQ(0.1));//_IQ(0.1)+_IQ12toIQ(AdcResult.ADCRESULT10)+_IQ(0.1);
	   //DFuncDesired1=_IQ15(0.1)+(AdcResult.ADCRESULT10<<3);//use external variable resistor control BLDC1
	   //DFuncDesired2=AdcResult.ADCRESULT11<<3;//use external variable resistor control BLDC2

// =============================== LEVEL 1 ======================================
//	This Level describes the steps for a minimum system check-out which confirms
//	operation of system interrupts, some peripheral & target independent modules
//	and one peripheral dependent module.
// ==============================================================================

#if (BUILDLEVEL==LEVEL1)


// ------------------------------------------------------------------------------
//    Connect inputs of the COM_TRIG module and call the Commutation trigger macro.
// ------------------------------------------------------------------------------
	  cmtn1.Va = _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb = _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc = _IQ12toIQ(AdcResult.ADCRESULT3);
	  cmtn1.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtn1)

	  cmtninv1.Va = _IQ12toIQ(AdcResult.ADCRESULT5);
	  cmtninv1.Vb = _IQ12toIQ(AdcResult.ADCRESULT6);
	  cmtninv1.Vc = _IQ12toIQ(AdcResult.ADCRESULT7);

      cmtninv1.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtninv1)
// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the RAMP Control 3 macro.
// ------------------------------------------------------------------------------
      rmp3_1.DesiredInput = CmtnPeriodTarget;
      rmp3_1.Ramp3Delay = RampDelay;
	  RC3_MACRO(rmp3_1)

      rmp3_2.DesiredInput = CmtnPeriodTarget;
      rmp3_2.Ramp3Delay = RampDelay;
	  RC3_MACRO(rmp3_2)

// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3_1.Out;
	  IMPULSE_MACRO(impl1)

      impl2.Period = rmp3_2.Out;
	  IMPULSE_MACRO(impl2)

// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Mod 6 counter macro.
// ------------------------------------------------------------------------------
      mod1.TrigInput = impl1.Out;
	  MOD6CNT_MACRO(mod1)

//      mod2.TrigInput = impl2.Out;
//	  MOD6CNT_MACRO(mod2)

      modinv1.TrigInput = impl2.Out;
	  MOD6CNTINV_MACRO(modinv1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//	  update macro.
// ------------------------------------------------------------------------------
      pwm1.CmtnPointer = (int16)mod1.Counter;
	  pwm1.DutyFunc = DfuncTesting;
	  BLDCPWM_MACRO(1,2,3,pwm1)
/*
      pwm2.CmtnPointer = (int16)mod2.Counter;
	  pwm2.DutyFunc = DfuncTesting;
	  BLDCPWM_MACRO(4,5,6,pwm2)
*/
      pwm2.CmtnPointer = (int16)modinv1.Counter;
	  pwm2.DutyFunc = DfuncTesting;
	  BLDCPWM_MACRO(4,5,6,pwm2)

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_REV_PR module and call the speed calculation macro.****
// ------------------------------------------------------------------------------
	  speed1.EventPeriod = cmtn1.RevPeriod;
	  SPEED_PR_MACRO(speed1)

	  speed2.EventPeriod = cmtninv1.RevPeriod;
	  SPEED_PR_MACRO(speed2)
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
//
//      DlogCh1 = (int16)mod1.Counter;
//      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
//      DlogCh3 = (int16)_IQtoIQ15(cmtn1.DebugBemf);
//	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Neutral);

#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
//	  Level 2 verifies the analog-to-digital conversion, offset compensation,
//    open loop motor operation.
// ==============================================================================

#if (BUILDLEVEL==LEVEL2)

// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
	  cmtn1.Va =  	  _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb =  	  _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc =  	  _IQ12toIQ(AdcResult.ADCRESULT3);
	  DCbus_current = _IQ12toIQ(AdcResult.ADCRESULT4)-_IQ(0.500); //1.65V offset added on HVDMC board.

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the Ramp control 3 macro.
// ------------------------------------------------------------------------------
      rmp3.DesiredInput = CmtnPeriodTarget;
      rmp3.Ramp3Delay = RampDelay;
	  RC3_MACRO(rmp3)

// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3.Out;
	  IMPULSE_MACRO(impl1)

// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Modulo 6 counter macro.
// ------------------------------------------------------------------------------
      mod1.TrigInput = impl1.Out;
  	  MOD6CNT_MACRO(mod1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
      pwm1.CmtnPointer = (int16)mod1.Counter;
	  pwm1.DutyFunc = DfuncTesting;
	  BLDCPWM_MACRO(1,2,3,pwm1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
      pwmdac1.MfuncC1 = cmtn1.Va;
      pwmdac1.MfuncC2 = cmtn1.Vc;
      PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B

      pwmdac1.MfuncC1 = cmtn1.Vb;
      pwmdac1.MfuncC2 = mod1.Counter<<20;
	  PWMDAC_MACRO(7,pwmdac1)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
      DlogCh1 = (int16)mod1.Counter;
      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
      DlogCh3 = (int16)_IQtoIQ15(cmtn1.Vb);
	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Vc);

#endif // (BUILDLEVEL==LEVEL2)


// =============================== LEVEL 3 ======================================
//	  Level 3 verifies the peripheral independent CMTN_TRIG_MACRO
// ==============================================================================

#if (BUILDLEVEL==LEVEL3)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the Ramp control 3 macro.
// ------------------------------------------------------------------------------
      rmp3.DesiredInput = CmtnPeriodTarget;
      rmp3.Ramp3Delay = RampDelay;
      RC3_MACRO(rmp3)

// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3.Out;
	  IMPULSE_MACRO(impl1)

// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Modulo 6 counter macro.
// ------------------------------------------------------------------------------
      mod1.TrigInput = impl1.Out;
	  MOD6CNT_MACRO(mod1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
      pwm1.CmtnPointer = (int16)mod1.Counter;
	  pwm1.DutyFunc = DfuncTesting;
	  BLDCPWM_MACRO(1,2,3,pwm1)

// ------------------------------------------------------------------------------
//    Connect inputs of the COM_TRIG module and call the Commutation trigger macro.
// ------------------------------------------------------------------------------
	  cmtn1.Va = _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb = _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc = _IQ12toIQ(AdcResult.ADCRESULT3);
      cmtn1.CmtnPointer = mod1.Counter;
      cmtn1.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtn1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
      pwmdac1.MfuncC1 = cmtn1.Va;
      pwmdac1.MfuncC2 = cmtn1.Vb;
      PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B

      pwmdac1.MfuncC1 = cmtn1.Neutral;
      pwmdac1.MfuncC2 = cmtn1.DebugBemf;
	  PWMDAC_MACRO(7,pwmdac1)							// PWMDAC 7A, 7B

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
      DlogCh1 = (int16)_IQtoIQ15(cmtn1.DebugBemf);
      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
      DlogCh3 = (int16)_IQtoIQ15(cmtn1.Vb);
	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Vc);

#endif // (BUILDLEVEL==LEVEL3)


// =============================== LEVEL 4 ======================================
//	  Level 4 verifies verifies the closed loop motor operation based on the
//	  computed Bemf zero crossings and the resulting commutation trigger points.
// ==============================================================================

#if (BUILDLEVEL==LEVEL4)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the Ramp control 3 macro.
// ------------------------------------------------------------------------------
      rmp3_1.DesiredInput = CmtnPeriodTarget;
      rmp3_1.Ramp3Delay = RampDelay;
   	  RC3_MACRO(rmp3_1)

      rmp3_2.DesiredInput = CmtnPeriodTarget;
      rmp3_2.Ramp3Delay = RampDelay;
   	  RC3_MACRO(rmp3_2)
// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3_1.Out;
	  IMPULSE_MACRO(impl1)

      impl2.Period = rmp3_2.Out;
	  IMPULSE_MACRO(impl2)
// ------------------------------------------------------------------------------
//    Connect inputs of the RMP2 module and call the Ramp control 2 macro.
// ------------------------------------------------------------------------------
      rmp2_1.DesiredInput = (int32)DFuncDesired;
	  RC2_MACRO(rmp2_1)

      rmp2_2.DesiredInput = (int32)DFuncDesired;
	  RC2_MACRO(rmp2_2)
// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Modulo 6 counter macro.
// ------------------------------------------------------------------------------
// Switch from open-loop to closed-loop operation by Ramp3DoneFlag variable

     if (rmp3_1.Ramp3DoneFlag == FALSE)
         mod1.TrigInput = impl1.Out;        // open-loop operation
     else
         mod1.TrigInput = cmtn1.CmtnTrig;   // closed-loop operation
	  MOD6CNT_MACRO(mod1)

	 if (rmp3_2.Ramp3DoneFlag == FALSE)
	     mod2.TrigInput = impl2.Out;        // open-loop operation
	 else
	     mod2.TrigInput = cmtn2.CmtnTrig;   // closed-loop operation
	  MOD6CNT_MACRO(mod2)
// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation update macro.
// ------------------------------------------------------------------------------
      pwm1.CmtnPointer = (int16)mod1.Counter;
      pwm1.DutyFunc = (int16)rmp2_1.Out;
	  BLDCPWM_MACRO(1,2,3,pwm1)

      pwm2.CmtnPointer = (int16)mod2.Counter;
      pwm2.DutyFunc = (int16)rmp2_2.Out;
	  BLDCPWM_MACRO(4,5,6,pwm2)
// ------------------------------------------------------------------------------
//    Connect inputs of the COM_TRIG module and call the Commutation trigger macro.
// ------------------------------------------------------------------------------
	  cmtn1.Va = _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb = _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc = _IQ12toIQ(AdcResult.ADCRESULT3);
      cmtn1.CmtnPointer = mod1.Counter;
      cmtn1.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtn1)

	  cmtn2.Va = _IQ12toIQ(AdcResult.ADCRESULT5);
	  cmtn2.Vb = _IQ12toIQ(AdcResult.ADCRESULT6);
	  cmtn2.Vc = _IQ12toIQ(AdcResult.ADCRESULT7);
      cmtn2.CmtnPointer = mod2.Counter;
      cmtn2.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtn2)
// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
/*      pwmdac1.MfuncC1 = cmtn1.Va;
      pwmdac1.MfuncC2 = cmtn1.Vb;
      PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B

      pwmdac1.MfuncC1 = cmtn1.Vc;
      pwmdac1.MfuncC2 = cmtn1.Neutral;
	  PWMDAC_MACRO(7,pwmdac1)							// PWMDAC 7A, 7B
*/
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
      DlogCh1 = (int16)mod1.Counter;
      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
      DlogCh3 = (int16)_IQtoIQ15(cmtn1.Vb);
	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Vc);


#endif // (BUILDLEVEL==LEVEL4)


// =============================== LEVEL 5 ======================================
//	  Level 5 verifies the closed current loop & current PI regulator
// ==============================================================================

#if (BUILDLEVEL==LEVEL5)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the Ramp control 3 macro.
// ------------------------------------------------------------------------------
      rmp3.DesiredInput = CmtnPeriodTarget;
      rmp3.Ramp3Delay = RampDelay;
	  RC3_MACRO(rmp3)

// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3.Out;
	  IMPULSE_MACRO(impl1)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP2 module and call the Ramp control 2 macro.
// ------------------------------------------------------------------------------
      rmp2.DesiredInput = (int32)DFuncDesired;
	  RC2_MACRO(rmp2)

// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Modulo 6 counter macro.
// ------------------------------------------------------------------------------
// Switch from open-loop to closed-loop operation by Ramp3DoneFlag variable
      if (rmp3.Ramp3DoneFlag == FALSE)
         mod1.TrigInput = impl1.Out;        // open-loop operation
      else
         mod1.TrigInput = cmtn1.CmtnTrig;   // closed-loop operation

	  MOD6CNT_MACRO(mod1)
// ------------------------------------------------------------------------------
//    Connect inputs of the PID_REG3 module and call the PID controller macro.
// ------------------------------------------------------------------------------
      tempIdc = pid1_idc.Fbk;
      pid1_idc.Ref = CurrentSet;
      pid1_idc.Fbk = _IQ12toIQ(AdcResult.ADCRESULT4)-_IQ(0.5);

	  if(pid1_idc.Fbk<0) pid1_idc.Fbk=tempIdc; // Eliminate negative values
	  PI_MACRO(pid1_idc)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
// Switch from fixed duty-cycle or controlled Idc duty-cycle by ILoopFlag variable

      if (ILoopFlag == FALSE)
        {pwm1.DutyFunc = (int16)rmp2.Out;                  // fixed duty-cycle
		 pid1_idc.ui=0;
		 pid1_idc.i1=0; }
      else
         pwm1.DutyFunc = (int16)_IQtoIQ15(pid1_idc.Out);   // controlled Idc duty-cycle

      pwm1.CmtnPointer = (int16)mod1.Counter;
	  BLDCPWM_MACRO(1,2,3,pwm1)

// ------------------------------------------------------------------------------
//    Connect inputs of the COM_TRIG module and call the Commutation trigger macro.
// ------------------------------------------------------------------------------
	  cmtn1.Va = _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb = _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc = _IQ12toIQ(AdcResult.ADCRESULT3);
      cmtn1.CmtnPointer = mod1.Counter;
      cmtn1.VirtualTimer = VirtualTimer;
	  CMTN_TRIG_MACRO(cmtn1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_REV_PR module and call the speed calculation macro.
// ------------------------------------------------------------------------------
      speed1.EventPeriod = cmtn1.RevPeriod;
	  SPEED_PR_MACRO(speed1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
      pwmdac1.MfuncC1 = cmtn1.Va;
      pwmdac1.MfuncC2 = cmtn1.Vb;
      PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B

      pwmdac1.MfuncC1 = cmtn1.Vc;
      pwmdac1.MfuncC2 = pid1_idc.Fbk ;
	  PWMDAC_MACRO(7,pwmdac1)							// PWMDAC 7A, 7B

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
	  DlogCh3 = (int16)_IQtoIQ15(cmtn1.Vb);
	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Vc);
      DlogCh1 = (int16)_IQtoIQ15(pid1_idc.Fbk);


#endif // (BUILDLEVEL==LEVEL5)

// =============================== LEVEL 6 ======================================
//	  Level6 verifies the speed regulator performed by PID_REG 3 module.
//	  The speed loop is closed by using the BEMF zero crossings.
// ==============================================================================

#if (BUILDLEVEL==LEVEL6)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP module and call the Ramp control macro.
// ------------------------------------------------------------------------------
      rc1.TargetValue = SpeedRef1;
      RC_MACRO(rc1)

      rc2.TargetValue = SpeedRef2;
      RC_MACRO(rc2)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP3 module and call the Ramp control 3 macro.
// ------------------------------------------------------------------------------
      rmp3_1.DesiredInput = CmtnPeriodTarget;
      rmp3_1.Ramp3Delay = RampDelay;
	  RC3_MACRO(rmp3_1)

      rmp3_2.DesiredInput = CmtnPeriodTarget;
      rmp3_2.Ramp3Delay = RampDelay;
      RC3_MACRO(rmp3_2)

// ------------------------------------------------------------------------------
//    Connect inputs of the IMPULSE module and call the Impulse macro.
// ------------------------------------------------------------------------------
      impl1.Period = rmp3_1.Out;
	  IMPULSE_MACRO(impl1)

      impl2.Period = rmp3_2.Out;
      IMPULSEINV_MACRO(impl2)

// ------------------------------------------------------------------------------
//    Connect inputs of the RMP2 module and call the Ramp control 2 macro.
// ------------------------------------------------------------------------------
      rmp2_1.DesiredInput = (int32)DFuncDesired1;
	  RC2_MACRO(rmp2_1)

      rmp2_2.DesiredInput = (int32)DFuncDesired2;
	  RC2_MACRO(rmp2_2)
// ------------------------------------------------------------------------------
//    Connect inputs of the MOD6 module and call the Mod  --------------------------
   // Switch from open-loop to closed-loop operation by Ramp3DoneFlag variable
	  if (rmp3_1.Ramp3DoneFlag == FALSE)
	          mod1.TrigInput = impl1.Out;        // open-loop operation
	       else
	          mod1.TrigInput = cmtn1.CmtnTrig;   // closed-loop operation
	  MOD6CNT_MACRO(mod1)

		if (rmp3_2.Ramp3DoneFlag == FALSE)
			modinv1.TrigInput = impl2.Out;        // open-loop operation
			else
				modinv1.TrigInput = cmtninv1.CmtnTrig;   // closed-loop operation


	  MOD6CNTINV_MACRO(modinv1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_REG3 module and call the PID speed controller macro.
// ------------------------------------------------------------------------------
      pid1_spd.Ref = rc1.SetpointValue;
      pid1_spd.Fbk = speed1.Speed;
	  PI_MACRO(pid1_spd)

      pid2_spd.Ref = rc2.SetpointValue;
      pid2_spd.Fbk = speed2.Speed;
	  PI_MACRO(pid2_spd)
// ------------------------------------------------------------------------------
//    Set the speed closed loop flag once the speed is built up to a desired value.
// ------------------------------------------------------------------------------

      if (rc1.EqualFlag == 0x7FFFFFFF)
      {
         SpeedLoopFlag1 = TRUE;
         rc1.RampDelayMax = 30;
      }

      if (rc2.EqualFlag == 0x7FFFFFFF)
      {
         SpeedLoopFlag2 = TRUE;
         rc2.RampDelayMax = 30;//30
      }

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
// Switch from fixed duty-cycle or controlled Speed duty-cycle by SpeedLoopFlag variable

      if(SpeedLoopFlag1 == FALSE)
        {pwm1.DutyFunc = (int16)rmp2_1.Out;          		   // fixed duty-cycle
		 pid1_spd.ui=0;}
      else
         pwm1.DutyFunc = (int16)_IQtoIQ15(pid1_spd.Out);   // controlled Speed duty-cycle

      pwm1.CmtnPointer = (int16)mod1.Counter;
	  BLDCPWM_MACRO(1,2,3,pwm1)

      if(SpeedLoopFlag2 == FALSE)
        {pwm2.DutyFunc = (int16)rmp2_2.Out;          		   // fixed duty-cycle
		 pid2_spd.ui=0;}
      else
         pwm2.DutyFunc = (int16)_IQtoIQ15(pid2_spd.Out);   // controlled Speed duty-cycle

	  pwm2.CmtnPointer = (int16)modinv1.Counter;
	  BLDCPWM_MACRO(4,5,6,pwm2)
// ------------------------------------------------------------------------------
//    Connect inputs of the COM_TRIG module and call the Commutation trigger macro.
// ------------------------------------------------------------------------------
	  cmtn1.Va = _IQ12toIQ(AdcResult.ADCRESULT1);
	  cmtn1.Vb = _IQ12toIQ(AdcResult.ADCRESULT2);
	  cmtn1.Vc = _IQ12toIQ(AdcResult.ADCRESULT3);
      cmtn1.CmtnPointer = mod1.Counter;
      cmtn1.VirtualTimer = VirtualTimer;

	  cmtninv1.Va = _IQ12toIQ(AdcResult.ADCRESULT5);
	  cmtninv1.Vb = _IQ12toIQ(AdcResult.ADCRESULT6);
	  cmtninv1.Vc = _IQ12toIQ(AdcResult.ADCRESULT7);

      cmtninv1.CmtnPointer = modinv1.Counter;
      cmtninv1.VirtualTimer = VirtualTimer;

      if(BLDC_RotDirec){//clockwise
          	  CMTN_TRIG_MACRO(cmtn1);
      	  	  CMTN_TRIG_INV_MACRO(cmtninv1);
           }
      else{
      	      CMTN_TRIG_INV_MACRO(cmtn1);
      		  CMTN_TRIG_MACRO(cmtninv1);
            }

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_REV_PR module and call the speed calculation macro.
// ------------------------------------------------------------------------------
      speed1.EventPeriod = cmtn1.RevPeriod;
	  SPEED_PR_MACRO(speed1)

      speed2.EventPeriod = cmtninv1.RevPeriod;
	  SPEED_PR_MACRO(speed2)

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
//      DlogCh2 = (int16)_IQtoIQ15(cmtn1.Va);
//	  DlogCh3 = (int16)_IQtoIQ15(cmtn1.Vb);
//	  DlogCh4 = (int16)_IQtoIQ15(cmtn1.Vc);
//      DlogCh1 = (int16)mod1.Counter;



#endif // (BUILDLEVEL==LEVEL6)



// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
//    dlog.update(&dlog);

// ------------------------------------------------------------------------------
//    Increase virtual timer and force 15 bit wrap around
// ------------------------------------------------------------------------------
	VirtualTimer++;
	VirtualTimer &= 0x00007FFF;
   }

    	RCONTROL_MACRO(GPIO18,ThrustStick)
    	RCONTROL_MACRO(GPIO19,RollStick)
    	RCONTROL_MACRO(GPIO16,YawStick)
    	RCONTROL_MACRO(GPIO17,PitchStick)
    	RCONTROL_MACRO(GPIO20,AutoStick)

// Acknowledge interrupt to recieve more interrupts from PIE group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


}// ISR Ends Here

/*------the debounceISR is for switch judge, period 1ms-----*/
interrupt void DebounceISR(void)
{
	/*======================================================================
	 Purpose of DebounceISR

	 there is a control switch determine the rotatrion direction
	 but to aviod the miss judgement of the status of the switch
	 this interrupt is to make sure the switch act properly
	 20160623 updated by kuotung tsai
	==========================================================================*/
//	 DebounceIsrTicker++;//check the DebounceISR is working
		if(RotDireChangFlag==0)//the control switch state only need to be checked when the RotDireChangFlag=0
		{
			 /*--sampling the control switch state--*/
			 OldBLDCmodStamp=NewBLDCmodStamp;
			 NewBLDCmodStamp=BLDC_CtrlMod;//update the control mode

			/*--examing the switch state remain time in ms--*/
			 if(OldBLDCmodStamp==NewBLDCmodStamp)
			 {
				 CtrlSwitchRemainTime++;
					if(CtrlSwitchRemainTime>30)//the switch remain for 20ms without being changed
					{
						Old_CtrlSwitchState=New_CtrlSwitchState;
						New_CtrlSwitchState=BLDC_CtrlMod;
							if(Old_CtrlSwitchState!=New_CtrlSwitchState)
							{
//								TestProbe++;
								RotDireChangFlag=1;// set the flag that control switch has been changed
							}
						CtrlSwitchRemainTime=0;//reset the switch remain time
					}

			 }
			 else
					 CtrlSwitchRemainTime=0;//ms
			}

		else
		{
			/*this mean RotDireChangFlag=1, and the motor rotata direction needed to be reversed,
			however, before reverse, the motor should decellarate*/
			BLDC_decelerateTicker++;
			if(BLDC_decelerateTicker>1000)//the SpeedRef will decendance every 1 sec
			{
				if(SpeedRef1>0.01)
				{
				SpeedRef1=SpeedRef1-_IQ(0.02);
				SpeedRef2=SpeedRef2-_IQ(0.02);
				BLDC_decelerateTicker=0;
				}
				else
				{
//					SpeedRef1=0.001;
//					SpeedRef2=0.001;
					RotDireChangFlag=0;//resume the RotDireChangFlag for speed control
					BLDC_RotDirec=BLDC_CtrlMod;

					GpioDataRegs.GPADAT.bit.GPIO21=1;//off the litter driver
					BLDC_decelDoneFlag=1;
				}
			}

		}//end of decelerate

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}//end of debounceISR

void ramp_initial(void)
{
	  	  	  CmtnPeriodSetpt = 0x00000400;
		  // Initialize RMPCNTL module
		      rc1.RampDelayMax = 2;
		      rc1.RampLowLimit = _IQ(0);//0
		      rc1.RampHighLimit = _IQ(1);
		      rc1.RampDelayCount=0;
		      rc1.EqualFlag=0;
		      rc1.Tmp=0;
		      rc1.SetpointValue=0;
		      rc1.TargetValue=0;

		      rc2.RampDelayMax = 2;
		      rc2.RampLowLimit = _IQ(0);//0
		      rc2.RampHighLimit = _IQ(1);
		      rc2.RampDelayCount=0;
		      rc2.EqualFlag=0;
		      rc2.Tmp=0;
		      rc2.SetpointValue=0;
		      rc2.TargetValue=0;

		      impl1.Counter=1000;
		      impl1.Out=0;
		      impl1.Period=0;
		      impl2.Counter=1000;
		      impl2.Out=0;
		      impl2.Period=0;
		  // Initialize RMP2 module
		  	rmp2_1.Out = (int32)ALIGN_DUTY;
		  	rmp2_1.Ramp2Delay =0x00000050;
		    rmp2_1.Ramp2Max = 0x00007FFF;
		    rmp2_1.Ramp2Min = 0x0000000F;
		    rmp2_1.Ramp2DelayCount=0;

		  	rmp2_2.Out = (int32)ALIGN_DUTY;
		  	rmp2_2.Ramp2Delay =0x00000050;
		    rmp2_2.Ramp2Max = 0x00007FFF;
		    rmp2_2.Ramp2Min = 0x0000000F;
		    rmp2_2.Ramp2DelayCount=0;

		  // Initialize RMP3 module
		  	rmp3_1.DesiredInput = CmtnPeriodTarget;
		  	rmp3_1.Ramp3Delay = RampDelay;
		    rmp3_1.Out = CmtnPeriodSetpt;
		    rmp3_1.Ramp3Min = 0x00000010;
		    rmp3_1.Ramp3DelayCount=0;
		    rmp3_1.Ramp3DoneFlag=0;

		  	rmp3_2.DesiredInput = CmtnPeriodTarget;
		  	rmp3_2.Ramp3Delay = RampDelay;
			rmp3_2.Out = CmtnPeriodSetpt;
			rmp3_2.Ramp3Min = 0x00000010;
			rmp3_2.Ramp3DelayCount=0;
			rmp3_2.Ramp3DoneFlag=0;

			mod1.Counter=0;
			mod1.Direction=0;
			mod1.TrigInput=1;
			modinv1.Counter=0;
			modinv1.Direction=0;
			modinv1.TrigInput=1;

			pid1_spd.Out=0;
			pid1_spd.v1=0;
			pid1_spd.ui=0;
			pid1_spd.i1=0;
			pid1_spd.up=0;
			pid2_spd.Out=0;
			pid2_spd.v1=0;
			pid2_spd.ui=0;
			pid2_spd.i1=0;
			pid2_spd.up=0;

		  // Initialize CMTN module*************************************************
			cmtn1.NWDelayThres = 20;
			cmtn1.NWDelta = 2;
			cmtn1.NoiseWindowMax = cmtn1.NWDelayThres - cmtn1.NWDelta;
			cmtn1.CmtnDelay=0;
			cmtn1.CmtnDelayCounter=0;
			cmtn1.Delay30DoneFlag=0;
			cmtn1.VirtualTimer=0;


			cmtninv1.NWDelayThres = 20;
			cmtninv1.NWDelta = 2;
			cmtninv1.NoiseWindowMax = cmtninv1.NWDelayThres - cmtninv1.NWDelta;
}


//interrupt void i2c_int1a_isr(void)     // I2C-A
//{
//	i2c_isr_ticker++;
//	Uint16 IntSource, i;
//
//	// Read interrupt source
//	IntSource = I2caRegs.I2CISRC.all;
//
//	// Interrupt source = stop condition detected
//	if (IntSource == I2C_SCD_ISRC) {
//		// If completed message was writing data, reset msg to inactive state
//		if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY) {
//			CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
//		}
//		else {
//			// If a message receives a NACK during the address setup portion of the
//			// EEPROM read, the code further below included in the register access ready
//			// interrupt source code will generate a stop condition. After the stop
//			// condition is received (here), set the message status to try again.
//			// User may want to limit the number of retries before generating an error.
//			if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY) {
//				CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
//			}
//			// If completed message was reading EEPROM data, reset msg to inactive state
//			// and read data from FIFO.
//			else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY) {
//				CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
//				for (i = 0; i < I2C_NUMBYTES; i++) {
//					CurrentMsgPtr->MsgBuffer[i] = I2caRegs.I2CDRR;
//				}
//				//transfer the buffer's date into gravity force
//
//			}
//		}
//	}  // end of stop condition detected
//
//	// Interrupt source = Register Access Ready
//	// This interrupt is used to determine when the EEPROM address setup portion of the
//	// read data communication is complete. Since no stop bit is commanded, this flag
//	// tells us when the message has been sent instead of the SCD flag. If a NACK is
//	// received, clear the NACK bit and command a stop. Otherwise, move on to the read
//	// data portion of the communication.
//	else if (IntSource == I2C_ARDY_ISRC) {
//		if (I2caRegs.I2CSTR.bit.NACK == 1) {
//			I2caRegs.I2CMDR.bit.STP = 1;
//			I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
//		} else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY) {
//			CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
//		}
//	}  // end of register access ready
//
//	else {
//		// Generate some error due to invalid interrupt source
//		__asm("   ESTOP0");
//	}
//
//	// Enable future I2C (PIE Group 8) interrupts
//	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
//}

void Servo_epwm_initial(void)
{
	// ePWM7 register configuration with HRPWM
		// ePWM7A toggle low/high with MEP control on Rising edge
	EALLOW;
	EPwm7Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	EPwm7Regs.TBPRD = 50000-1;      //period=50000 PWM frequency = 1 / period
	EPwm7Regs.CMPA.half.CMPA =49600;  //Compare to the TBCLK, SET MOTOR A T CENTER POSITION
	EPwm7Regs.CMPB = 49600;	              //Compare to the TBCLK
	EPwm7Regs.TBPHS.all = 0;
	EPwm7Regs.TBCTR = 0;
	EDIS;
	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; //THE CBCLK is up-count mode
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;		       // EPwm1 is the Master
	EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV5;//divide /6  計時和SYSCLKOUT的比率
	EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV4;//divide /4
	//Since the TBCLK=SYSCLK/( HSPCLKDIV* CLKDIV), so TBCLK=(6*4)/60MHz
	EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm7Regs.AQCTLA.bit.ZRO =AQ_CLEAR;          // PWM toggle low/high
	EPwm7Regs.AQCTLA.bit.CAU =AQ_SET;
	EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
	EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;

}
//===========================================================================
// No more.
//===========================================================================

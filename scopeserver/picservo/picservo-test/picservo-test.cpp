//---------------------------------------------------------------------------
//                 NMC Library Example Program
//This simple program illustrates how to use the NMC shared library (libnmc)
//with a PIC-SERVO under Linux.
//
//The sample program itself simply executes the following sequence of operations:
//	1) Initialize the network of NMC controllers (PIC-SERVO or PIC-IO modules)
//	2) Set servo parameters for module 1 (assumed to be a PIC-SERVO)
//	3) Enable the servo at the current motor position (and clear position to 0)
//	4) Move to a fixed position
//	5) Read the position of the motor
//	6) Exit the program
//
//The header files below should be included in your source.  These
//files contain function prototypes as well as assorted data types
//and definitions.  This set of functions is not complete, but using the
//provided source code as an example, you can create your own functions
//to access any of the functionality available from any of the NMC
//modules.
//
//This program is not intended to be useful or even usable (depending on your
//hardware setup).  It is simply an example to get you going.
//---------------------------------------------------------------------------

//Include these header file when using libnmc:
#include "sio_util.h"
#include "nmccom.h"
#include "picio.h"
#include "picservo.h"

//---------------------------------------------------------------------------
//Globals:
int nummod;
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//When the mainform is closed, this function will be called to shut down
//the NMC network.
void NMCClose()
{
  if (nummod == 0) return;
  NmcShutdown();
}
//---------------------------------------------------------------------------
//This routine calls the function NmcInit() to initialize all of the
//controllers found on the specified serial port.  Controller addresses
//are dynamically set, with the end controller starting at address = 1;
//All module group addresses are set to 0xFF.  Addresses may be changed
//later if required, but most applications may leave the addresses as
//set by NmcInit().  The global variable mod[] is an array of type NMCMOD
//which contains basic information about each module found on the network.
//The data type NMCMOD is defined in nmccom.h.
void Init()
{
int i;

//nummod = NmcInit("/dev/ttyS0", 19200);  //Controllers on COM1, use 19200 baud
				        //Returns the number of modules found
nummod = NmcInit("/dev/ttyUSB0", 19200);  //Controllers on COM1, use 19200 baud
				        //Returns the number of modules found
printf("Enumerating devices\n");
if (nummod==0)  NMCClose();
else
  {
  for (i=0; i<nummod; i++)
    {
    if (NmcGetModType((byte)(i+1)) == SERVOMODTYPE)  //Determine the type for module 1
      {
      printf("Module %d: PIC-SERVO Controller\n", i+1);
      }
    if (NmcGetModType((byte)(i+1)) == IOMODTYPE)
      {
      printf("Module %d: PIC-IO Controller\n", i+1);
      }
    }
  if (NmcGetModType(1) != SERVOMODTYPE) 	//If not a PIC-SERVO module, punt on the demo
    {
    printf("Module 1 not a PIC-SERVO - Cannot continue demo.\n");
    NMCClose();
    }
  }
}
//---------------------------------------------------------------------------
//This routine simply sets the servo parameters for motor controller 1
void SetGain()
{
ServoSetGain(1,		//axis = 1
 	     100,	//Kp = 100
             1000,	//Kd = 1000
             0,		//Ki = 0
             0,		//IL = 0
             255,	//OL = 255
             0,		//CL = 0
             4000,	//EL = 4000
             1,		//SR = 1
             0		//DC = 0
             );
             
}
//---------------------------------------------------------------------------
//This routine enables the amplifier and starts the motor servoing at its
//current position.
void ServoOn()
{
ServoStopMotor(1, AMP_ENABLE | MOTOR_OFF);  //enable amp
ServoStopMotor(1, AMP_ENABLE | STOP_ABRUPT);  //stop at current pos.
ServoResetPos(1);                             //reset the posiiton counter to 0

}
//---------------------------------------------------------------------------
//This routine loads a trapeziodal profile trajectory (position, velocity
//and acceleration) and starts the motion immediately.
void Go()
{
byte statbyte;

ServoLoadTraj(1, 		//addr = 0
	      LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW, 
              2000,		//pos = 2000
              10000, 		//vel = 10,000
              100, 		//acc = 100
              0			//pwm = 0
              );


do
  {
  NmcNoOp(1);	//poll controller to get current status data
  statbyte = NmcGetStat(1);
  }
while ( !(statbyte & MOVE_DONE) );  //wait for MOVE_DONE bit to go HIGH

}
//---------------------------------------------------------------------------
void ReadPos()
{
long position;

//Cause PIC-SERVO controller to send back position data with each command
//Position data will also be sent back with this command
NmcDefineStatus(1,    		//addr = 1
				SEND_POS    //Send back position data only
				);

//Retrieve the position data from the local data structure
position = ServoGetPos(1);
printf("Current Position: %ld\n",position);
}
//---------------------------------------------------------------------------

// Do it all
int main () 
{
  printf("Initializing...\n");
  Init();
  sleep(1);
  
  printf("Setting the gain...\n");
  SetGain();
  sleep(1);

  printf("Turning the servo on...\n");
  ServoOn();
  sleep(1);

  printf("Moving...\n");
  Go();
  sleep(1);

  printf("Reading position...\n");
  ReadPos();
  sleep(1);

  printf("Shutting down...\n");
  NMCClose();

  return 0;
}

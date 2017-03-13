//---------------------------------------------------------------------------
#ifndef picservoH
#define picservoH
//---------------------------------------------------------------------------
#endif
//--------------------- Servo Module specific stuff ---------------------------
typedef	struct _GAINVECT {
    short int	kp;				//gain values
    short int	kd;
    short int	ki;
    short int	il;
    byte		ol;
    byte		cl;
    short int	el;
    byte		sr;
    byte		dc;
	} GAINVECT;

typedef struct _SERVOMOD {
    long		pos;     		//current position
    byte		ad;				//a/d value
    short int	vel;            //current velocity
    byte		aux;            //auxilliary status byte
    long		home;           //home position
    short int	perror;			//position error
    byte		npoints;		//number of points in path buffer

    //The following data is stored locally for reference
    long		cmdpos;			//last commanded position
    long		cmdvel;			//last commanded velocity
    long		cmdacc;			//last commanded acceleration
    byte		cmdpwm;			//last commanded PWM value
    GAINVECT    gain;
    long		stoppos;		//motor stop position (used by stop command)
    byte		stopctrl;		//stop control byte
    byte		movectrl;		//load_traj control byte
    byte		ioctrl;			//I/O control byte
    byte		homectrl;		//homing control byte
    byte		ph_adv;			//phase advance (for ss-drive modules)
    byte		ph_off;			//phase offset (for ss-drive modules)
    long		last_ppoint;	//last path point specified
    } SERVOMOD;


//Servo Module Command set:
#define	RESET_POS	  0x00	//Reset encoder counter to 0 (0 bytes)
#define	SET_ADDR	  0x01	//Set address and group address (2 bytes)
#define	DEF_STAT	  0x02	//Define status items to return (1 byte)
#define	READ_STAT	  0x03	//Read value of current status items
#define	LOAD_TRAJ  	  0x04	//Load trahectory date (1 - 14 bytes)
#define START_MOVE	  0x05	//Start pre-loaded trajectory (0 bytes)
#define SET_GAIN	  0x06  //Set servo gain and control parameters (13 or 14)
#define	STOP_MOTOR 	  0x07	//Stop motor (1 byte)
#define	IO_CTRL		  0x08	//Define bit directions and set output (1 byte)
#define SET_HOMING	  0x09  //Define homing mode (1 byte)
#define	SET_BAUD	  0x0A 	//Set the baud rate (1 byte)
#define CLEAR_BITS	  0x0B  //Save current pos. in home pos. register (0 bytes)
#define SAVE_AS_HOME  0x0C	//Store the input bytes and timer val (0 bytes)
#define ADD_PATHPOINT 0x0D  //Adds path points for path mode
#define	NOP			  0x0E	//No operation - returns prev. defined status (0 bytes)
#define HARD_RESET	  0x0F	//RESET - no status is returned

//Servo Module STATUSITEMS bit definitions:
#define	SEND_POS	  0x01	//4 bytes data
#define	SEND_AD		  0x02	//1 byte
#define	SEND_VEL	  0x04	//2 bytes
#define SEND_AUX	  0x08	//1 byte
#define SEND_HOME	  0x10	//4 bytes
#define SEND_ID		  0x20	//2 bytes
#define SEND_PERROR	  0x40  //2 bytes
#define SEND_NPOINTS  0x80  //1 byte

//Servo Module LOAD_TRAJ control byte bit definitions:
#define	LOAD_POS	  0x01	//+4 bytes
#define LOAD_VEL	  0x02	//+4 bytes
#define	LOAD_ACC	  0x04	//+4 bytes
#define LOAD_PWM	  0x08	//+1 byte
#define ENABLE_SERVO  0x10  //1 = servo mode, 0 = PWM mode
#define VEL_MODE	  0x20	//1 = velocity mode, 0 = trap. position mode
#define REVERSE		  0x40  //1 = command neg. PWM or vel, 0 = positive
#define MOVE_REL	  0x40  //1 = move relative, 0 = move absolute
#define START_NOW	  0x80  //1 = start now, 0 = wait for START_MOVE command

//Servo Module STOP_MOTOR control byte bit definitions:
#define	AMP_ENABLE	  0x01	//1 = raise amp enable output, 0 = lower amp enable
#define MOTOR_OFF	  0x02	//set to turn motor off
#define STOP_ABRUPT   0x04	//set to stop motor immediately
#define STOP_SMOOTH	  0x08  //set to decellerate motor smoothly
#define STOP_HERE	  0x10	//set to stop at position (4 add'l data bytes required)
#define ADV_FEATURE   0x20  //enable features in ver. CMC

//Servo Module IO_CTRL control byte bit definitions:
#define SET_OUT1	  0x01	//1 = set limit 1 output, 0 = clear limit 1 output
#define SET_OUT2	  0x02	//1 = set limit 2 output, 0 = clear limit 2 output
#define IO1_IN		  0x04	//1 = limit 1 is an input, 0 = limit 1 is an output
#define IO2_IN		  0x08	//1 = limit 2 is an input, 0 = limit 2 is an output
#define	FAST_PATH	  0x40  //0 = 30 or 60 Hz path execution, 1 = 60 or 120 Hz

//Servo Module SET_HOMING control byte bit definitions:
#define ON_LIMIT1	  0x01	//home on change in limit 1
#define ON_LIMIT2	  0x02	//home on change in limit 2
#define HOME_MOTOR_OFF  0x04  //turn motor off when homed
#define ON_INDEX	  0x08	//home on change in index
#define HOME_STOP_ABRUPT 0x10 //stop abruptly when homed
#define HOME_STOP_SMOOTH 0x20 //stop smoothly when homed
#define ON_POS_ERR	  0x40	//home on excessive position error
#define	ON_CUR_ERR	  0x80  //home on overcurrent error

//Servo Module RESET_POS control byte bit definitions:
//(if no control byte is used, reset is absolute)
#define REL_HOME 	  0x01	//Reset position relative to current home position

//Servo Module ADD_PATHPOINT frequency definitions
#define P_30HZ		  30	//30 hz path resolution
#define P_60HZ		  60    //60 hs path resolution
#define P_120HZ		  120   //120 hs path resolution

//Servo Module Status byte bit definitions:
#define MOVE_DONE	  0x01	//set when move done (trap. pos mode), when goal
							//vel. has been reached (vel mode) or when not servoing
#define CKSUM_ERROR	  0x02	//checksum error in received command
#define OVERCURRENT	  0x04	//set on overcurrent condition (sticky bit)
#define POWER_ON	  0x08	//set when motor power is on
#define POS_ERR		  0x10	//set on excess pos. error (sticky bit)
#define LIMIT1		  0x20	//value of limit 1 input
#define LIMIT2		  0x40	//value of limit 2 input
#define HOME_IN_PROG  0x80  //set while searching for home, cleared when home found

//Servo Module Auxilliary status byte bit definitions:
#define INDEX		  0x01	//value of the encoder index signal
#define POS_WRAP	  0x02	//set when 32 bit position counter wraps around
							//  (sticky bit)
#define SERVO_ON	  0x04	//set when position servo is operating
#define ACCEL_DONE	  0x08	//set when acceleration portion of a move is done
#define SLEW_DONE	  0x10  //set when slew portion of a move is done
#define SERVO_OVERRUN 0x20  //set if servo takes longer than the specified
							//servo period to execute
#define PATH_MODE	  0x40  //path mode is enabled (v.5)

//Servo module function prototypes:
extern "C" SERVOMOD * ServoNewMod();
extern "C" BOOL ServoGetStat(byte addr);
extern "C" long ServoGetPos(byte addr);
extern "C" byte ServoGetAD(byte addr);
extern "C" short int ServoGetVel(byte addr);
extern "C" byte ServoGetAux(byte addr);
extern "C" long ServoGetHome(byte addr);
extern "C" short int ServoGetPError(byte addr);
extern "C" byte ServoGetNPoints(byte addr);
extern "C" long ServoGetCmdPos(byte addr);
extern "C" long ServoGetCmdVel(byte addr);
extern "C" long ServoGetCmdAcc(byte addr);
extern "C" long ServoGetStopPos(byte addr);
extern "C" byte ServoGetCmdPwm(byte addr);
extern "C" byte ServoGetMoveCtrl(byte addr);
extern "C" byte ServoGetStopCtrl(byte addr);
extern "C" byte ServoGetHomeCtrl(byte addr);
extern "C" byte ServoGetIoCtrl(byte addr);
extern "C" byte ServoGetPhAdv(byte addr);
extern "C" byte ServoGetPhOff(byte addr);
extern "C" void ServoGetGain(byte addr, short int * kp, short int * kd, short int * ki,
				  short int * il, byte * ol, byte * cl, short int * el,
                  byte * sr, byte * dc);
extern "C" BOOL ServoSetGain(byte addr, short int kp, short int kd, short int ki,
				  short int il, byte ol, byte cl, short int el,
                  byte sr, byte dc);
extern "C" BOOL ServoResetPos(byte addr);
extern "C" BOOL ServoResetRelHome(byte addr);
extern "C" BOOL ServoClearBits(byte addr);
extern "C" BOOL ServoStopMotor(byte addr, byte mode);
extern "C" BOOL ServoSetIoCtrl(byte addr, byte mode);
extern "C" BOOL ServoLoadTraj(byte addr, byte mode, long pos, long vel, long acc, byte pwm);
extern "C" void ServoInitPath(byte addr);
extern "C" BOOL ServoAddPathpoints(byte addr, int npoints, long *path, int freq);
extern "C" BOOL ServoStartPathMode(byte groupaddr, byte groupleader);
extern "C" BOOL ServoSetHoming(byte addr, byte mode);
extern "C" BOOL ServoSetPhase(byte addr, int padvance, int poffset, int maxpwm);


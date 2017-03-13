//---------------------------------------------------------------------------
#ifndef picioH
#define picioH
//---------------------------------------------------------------------------
#endif
//--------------------- IO Module specific stuff ---------------------------
typedef struct _IOMOD {
    short int	inbits;			//input bits
    byte		ad1;			//A/D input bytes
    byte		ad2;
    byte		ad3;
    unsigned long timer; 		//timer value
    short int	inbits_s;		//synchronized input bytes
    unsigned long timer_s;		//synchronized timer value
    //The following data is stored locally for reference
    byte		pwm1;			//current PWM output values
    byte		pwm2;
    byte		timermode;		//current timer mode
    short int	bitdir;			//current bit direction values
    short int	outbits;		//current output byte values
    } IOMOD;

//IO Module Command set:
#define	SET_IO_DIR	  0x00	//Set direction of IO bits (2 data bytes)
#define	SET_ADDR	  0x01	//Set address and group address (2 bytes)
#define	DEF_STAT	  0x02	//Define status items to return (1 byte)
#define	READ_STAT	  0x03	//Read value of current status items
#define	SET_PWM   	  0x04	//Immediatley set PWM1 and PWM2 (2 bytes)
#define SYNCH_OUT	  0x05	//Output prev. stored PWM & output bytes (0 bytes)
#define SET_OUTPUT	  0x06  //Immediately set output bytes
#define	SET_SYNCH_OUT 0x07	//Store PWM & outputs for synch'd output (4 bytes)
#define	SET_TMR_MODE  0x08	//Set the counter/timer mode (1 byte)
//Not used			  0x09
#define	SET_BAUD	  0x0A 	//Set the baud rate (1 byte)
//Not used			  0x0B
#define SYNCH_INPUT	  0x0C	//Store the input bytes and timer val (0 bytes)
//Not used			  0x0D
#define	NOP			  0x0E	//No operation - returns prev. defined status (0 bytes)
#define HARD_RESET	  0x0F	//RESET - no status is returned

//IO Module STATUSITEMS bit definitions
#define	SEND_INPUTS	  0x01	//2 bytes data
#define	SEND_AD1	  0x02	//1 byte
#define	SEND_AD2	  0x04	//1 byte
#define SEND_AD3	  0x08	//1 byte
#define SEND_TIMER	  0x10	//4 bytes
#define SEND_ID		  0x20	//2 bytes
#define	SEND_SYNC_IN  0x40	//2 bytes
#define	SEND_SYNC_TMR 0x80	//4 bytes

//IO Module Timer mode definitions
//Timer mode and resolution may be OR'd together
#define	OFFMODE		  0x00
#define	COUNTERMODE	  0x03
#define	TIMERMODE	  0x01
#define	RESx1		  0x00
#define RESx2		  0x10
#define RESx4		  0x20
#define RESx8		  0x30
//--------------------- END IO Module specific stuff ------------------------

extern "C" IOMOD * IoNewMod();
extern "C" BOOL IoGetStat(byte addr);
extern "C" BOOL IoInBitVal(byte addr, int bitnum);
extern "C" BOOL IoInBitSVal(byte addr, int bitnum);
extern "C" BOOL IoOutBitVal(byte addr, int bitnum);
extern "C" BOOL IoGetBitDir(byte addr, int bitnum);
extern "C" byte IoGetADCVal(byte addr, int channel);
extern "C" byte IoGetPWMVal(byte addr, int channel);
extern "C" unsigned long IoGetTimerVal(byte addr);
extern "C" unsigned long IoGetTimerSVal(byte addr);
extern "C" byte IoGetTimerMode(byte addr);

extern "C" BOOL IoSetOutBit(byte addr, int bitnum);
extern "C" BOOL IoClrOutBit(byte addr, int bitnum);
extern "C" BOOL IoBitDirIn(byte addr, int bitnum);
extern "C" BOOL IoBitDirOut(byte addr, int bitnum);
extern "C" BOOL IoSetPWMVal(byte addr, byte pwm1, byte pwm2);
extern "C" BOOL IoSetTimerMode(byte addr, byte tmrmode);
extern "C" BOOL IoSetSynchOutput(byte addr, short int outbits, byte pwm1, byte pwm2);


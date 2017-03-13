//---------------------------------------------------------------------------

#include "nmccom.h"
#include "picio.h"
#include "picservo.h"
#include "picstep.h"
//#include "mainform.h"
#include "sio_util.h"
//---------------------------------------------------------------------------
//Global data

NMCMOD mod[MAXNUMMOD]; 	//Array of modules
int nummod = 0;				//start off with no modules
char cmdstr[20];			//use global command string
HANDLE ComPort;
int SioError = 0;
int IOBusy = false;
unsigned int BaudRate;

//---------------------------------------------------------------------------
extern "C" ADCMOD * AdcNewMod()
{
return(NULL);
}

//---------------------------------------------------------------------------
//Reset all controllers with group address 'addr' (should include all modules)
extern "C" BOOL NmcHardReset(byte addr)
{
int i;
char cstr[5];

//Send string of 0's to flush input buffers
cstr[0] = 0;
for (i=0; i<20; i++) SioPutChars(ComPort,cstr, 1);

//Send out reset command string
cstr[0] = 0xAA;			//Header
cstr[1] = addr;
cstr[2] = HARD_RESET;
cstr[3] = (byte)(cstr[1] + cstr[2]);  //checksum
SioPutChars(ComPort, cstr, 4);//send reset

usleep(100);		//wait for reset to execute

for (i=0; i<nummod; i++) delete mod[i].p;  //delete and prev. declared modules
nummod = 0;
SioError = 0;	//reset the number of errors to 0

SioChangeBaud(ComPort, 19200);	//Reset the baud rate to the default

SioClrInbuf(ComPort);   //clear out any random crap left in buffer

return(true);
}

//---------------------------------------------------------------------------
//Change the baud rate of all controllers with group address 'groupaddr'
//(should include all modules) and also changes host's baud rate.
//There should be no group leader for 'addr'
extern "C" BOOL NmcChangeBaud(byte groupaddr, unsigned int baudrate)
{
char cstr[6];

cstr[0] = 0xAA;			//Header
cstr[1] = groupaddr;
cstr[2] = 0x10 | SET_BAUD;
switch (baudrate) {
	case 19200:		cstr[3] = PB19200; break;
	case 57600:		cstr[3] = PB57600; break;
	case 115200:	cstr[3] = PB115200; break;
    default:	printf("Baud rate not supported - using default of 19200\n");
    			cstr[3] = PB19200;
                baudrate = 19200;
    }
cstr[4] = (byte)(cstr[1] + cstr[2] + cstr[3]);  //checksum
SioPutChars(ComPort, cstr, 5);//send command

usleep(100);		//Wait for command to execute

SioChangeBaud(ComPort, baudrate);	//Reset the baud rate to the default

SioClrInbuf(ComPort);   //clear out any random crap left in buffer
usleep(100);

BaudRate = baudrate;
return true;
}

//---------------------------------------------------------------------------
//Initialize misc network variables
extern "C" void InitVars(void)
{
int i;

mod[0].modtype = 0;   //Default to a known module type for module 0
mod[0].modver = 0;
mod[0].stat = 0;
mod[0].statusitems = 0;
mod[0].groupaddr = 0xFF;
mod[0].groupleader = false;

for (i=1; i<MAXNUMMOD; i++)
	{
	mod[i].modtype = 0xFF;
	mod[i].modver = 0;
    mod[i].stat = 0;
    mod[i].statusitems = 0;
    mod[i].groupaddr = 0xFF;
    mod[i].groupleader = false;
    }
}
//---------------------------------------------------------------------------
//Initialize the network of controllers with sequential addresses starting
//at 0.  Retunrs the number of controllers found.
extern "C" int NmcInit(const char *portname, unsigned int baudrate)
{
DWORD numread;
byte addr;
char cstr[20];
//char mess[40];

BaudRate = baudrate;

InitVars();

ComPort = SioOpen(portname, 19200);    //Open with default rate of 19200
if (ComPort < 0) return 0;

//printf("Resetting NMC devices\n");
NmcHardReset(0xFF);
NmcHardReset(0xFF);
//printf("NMC reset complete\n");

addr = 1;
while (1)
	{
    //First set the address to a unique value:
    cstr[0] = 0xAA;		//Header
    cstr[1] = 0;  		//Send to default address of 0
    cstr[2] = 0x20 | SET_ADDR;
    cstr[3] = addr;  	//Set new address sequentially
    cstr[4] = 0xFF;		//Set group address to 0xFF
    cstr[5] = (byte)(cstr[1] + cstr[2] + cstr[3] + cstr[4]);  //checksum
    SioPutChars(ComPort, cstr, 6);  //Send command

    numread = SioGetChars(ComPort, cstr, 2);  //get back status, cksum
    if (numread!=2) break;	//if no response, punt out of loop
    if (cstr[0] != cstr[1])
    	{
        printf("Status checksum error - please reset the Network\n");
        SioClose(ComPort);
        return(0);
        }

    //Read the device type
    cstr[0] = 0xAA;		//Header
    cstr[1] = addr;  		//Send to addr
    cstr[2] = 0x10 | READ_STAT;
    cstr[3] = SEND_ID;  //send ID only
    cstr[4] = (byte)(cstr[1] + cstr[2] + cstr[3]);  //checksum
    SioPutChars(ComPort, cstr, 5);  //Send command

    numread = SioGetChars(ComPort, cstr, 4);  //get back stat, type, ver, cksum
    if (numread!=4)
    	{
        printf("Could not read device type\n");
        SioClose(ComPort);
        return(0);
        }
    mod[addr].stat = cstr[0];
    mod[addr].modtype = cstr[1];
    mod[addr].modver = cstr[2];
    mod[addr].statusitems = 0;
    mod[addr].groupaddr = 0xFF;
    mod[addr].groupleader = false;
    switch (mod[addr].modtype)
    	{
        case SERVOMODTYPE: 	mod[addr].p = ServoNewMod(); break;
        case ADCMODTYPE: 	mod[addr].p = AdcNewMod(); break;
        case IOMODTYPE: 	mod[addr].p = IoNewMod(); break;
        case STEPMODTYPE: 	mod[addr].p = StepNewMod(); break;
        }

    addr++;		//increment the address
    }

nummod = addr-1;

//printf("Found %d devices\n", nummod);

if (nummod > 0)
	{
    NmcChangeBaud(0xFF, baudrate);
	if (nummod>1) printf("%d Modules found\n", nummod);
    else printf("%d Module found\n", nummod);
    }
else
	{
    printf("No modules found on the network.\nPlease check power and connections.\n");
    SioClose(ComPort);
    }

return(nummod);	//Return number of modules
}
//---------------------------------------------------------------------------
extern "C" BOOL AdcGetStat(byte addr)
{
return false;
}
//---------------------------------------------------------------------------
//Sends a command to addr and shoves the returned status data in the
//structure for module address stataddr.  If stataddr = 0, the command
//is taken as a group command with no group leader (no status returned).
//n is the number of auxilliary data bytes.
extern "C" BOOL NmcSendCmd(byte addr, byte cmd, char *datastr, byte n, byte stataddr)
{
byte cksum;
//char mess[40];
byte outstr[20];
int i;
BOOL iostat;

//If too many SIO errors, punt
if (SioError > MAXSIOERROR) return(false);

IOBusy = true;

//Check if data is to be returned from to a known module type
if ( !( ( mod[stataddr].modtype==SERVOMODTYPE) ||
        ( mod[stataddr].modtype==ADCMODTYPE) ||
        ( mod[stataddr].modtype==IOMODTYPE)  ||
        ( mod[stataddr].modtype==STEPMODTYPE) ) )
	{
    printf("Module type %d not supported\n",mod[stataddr].modtype);
    return (false);
    }

//Calculate the adjust command byte, calculate checksum and send the command
cksum = 0;
outstr[0] = 0xAA;	//start with header byte
outstr[1] = addr;	//add on address byte
cksum += outstr[1];
outstr[2] = (byte)(((n<<4) & 0xF0) | cmd);
cksum += outstr[2];
for (i=0; i<n; i++)
	{
    outstr[i+3] = datastr[i];
	cksum += outstr[i+3];
    }
outstr[n+3] = cksum;

SioClrInbuf(ComPort);  //Get rid of any old input chars
SioPutChars(ComPort, (char *)outstr, n+4); //Send the command string

if (stataddr == 0)		//If a group command w/ no leader, add delay then exit
	{
    usleep(60);
    IOBusy = false;
	return true;
    }

iostat = false;
switch (mod[stataddr].modtype) {
	case SERVOMODTYPE:	iostat = ServoGetStat(stataddr); break;
	case ADCMODTYPE:	iostat = AdcGetStat(stataddr); break;
	case IOMODTYPE:		iostat = IoGetStat(stataddr); break;
	case STEPMODTYPE:	iostat = StepGetStat(stataddr); break;
    }

if (iostat == false)
	{
    SioError++;
    FixSioError(stataddr);
    }
else SioError = 0;

IOBusy = false;
return iostat;
}

//---------------------------------------------------------------------------
//Attempt to re-synch communications
extern "C" void FixSioError(byte addr)
{
int i;
char teststr[2];

//If too many errors - prompt to reset
if (SioError >= MAXSIOERROR)
	{
    printf("Multiple communication errors - please reset the Network\n");
    return;
    }

//Otherwise, send out a null string to re-synch
teststr[0] = 0;
for (i=0; i<30; i++) SioPutChars(ComPort, teststr, 1);
usleep(100);
SioClrInbuf(ComPort);
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcSetGroupAddr(byte addr, byte groupaddr, BOOL leader)
{
char cmdstr[2];

if (!(groupaddr & 0x80)) return false;  //punt if not valid group address

cmdstr[0] = addr;
cmdstr[1] = groupaddr;
if (leader) cmdstr[1] &= 0x7F;  //clear upper bit if a group leader
mod[addr].groupaddr = groupaddr;
mod[addr].groupleader = leader;
return NmcSendCmd(addr, SET_ADDR, cmdstr, 2, addr);
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcSynchOutput(byte groupaddr, byte leaderaddr)
//Synchronous output command issued to groupaddr.  Status data goes to
//the leaderaddr.  If no leader, use leaderaddr = 0.
{
return NmcSendCmd(groupaddr, SYNCH_OUT, NULL, 0, leaderaddr);
}
//---------------------------------------------------------------------------
//Synchronous input command issued to groupaddr.  Status data goes to
//the leaderaddr.  If no leader, use leaderaddr = 0.
extern "C" BOOL NmcSynchInput(byte groupaddr, byte leaderaddr)
{
return NmcSendCmd(groupaddr, SYNCH_INPUT, NULL, 0, leaderaddr);
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcNoOp(byte addr)
{
return NmcSendCmd(addr, NOP, NULL, 0, addr);
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcReadStatus(byte addr, byte statusitems)
{
char cmdstr[2];
byte oldstat;
BOOL retval;

cmdstr[0] = statusitems;
oldstat = mod[addr].statusitems;
mod[addr].statusitems = statusitems;
retval = NmcSendCmd(addr, READ_STAT, cmdstr, 1, addr);
mod[addr].statusitems = oldstat;

return retval;
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcDefineStatus(byte addr, byte statusitems)
{
char cmdstr[2];

cmdstr[0] = statusitems;
mod[addr].statusitems = statusitems;
return NmcSendCmd(addr, DEF_STAT, cmdstr, 1, addr);
}
//---------------------------------------------------------------------------
extern "C" byte NmcGetStat(byte addr)
{
return mod[addr].stat;
}
//---------------------------------------------------------------------------
extern "C" byte NmcGetStatItems(byte addr)
{
return mod[addr].statusitems;
}
//---------------------------------------------------------------------------
extern "C" byte NmcGetModType(byte addr)
{
return mod[addr].modtype;
}
//---------------------------------------------------------------------------
extern "C" byte NmcGetModVer(byte addr)
{
return mod[addr].modver;
}
//---------------------------------------------------------------------------
extern "C" byte NmcGetGroupAddr(byte addr)
{
return mod[addr].groupaddr;
}
//---------------------------------------------------------------------------
extern "C" BOOL NmcGroupLeader(byte addr)
{
return mod[addr].groupleader;
}
//---------------------------------------------------------------------------
extern "C" void NmcShutdown(void)
{
//int retval;

//retval = MessageBox(NULL, "Do you wish to reset the NMC network?",
//                                 "",MB_YESNO | MB_DEFBUTTON1 | MB_TASKMODAL);
//
//if (retval == IDYES)
//	{
//	if (ComPort!=INVALID_HANDLE_VALUE && ComPort!=NULL) NmcHardReset(0xFF);
//    }
//else
//	{
//    if (BaudRate != 19200)
//      	{
//      	ErrorMsgBox("Returning NMC Baud rate to 19200 (default)");
//      	NmcChangeBaud(0xFF, 19200);
//      	}
//    }
if (ComPort!=INVALID_HANDLE_VALUE && ComPort!=NULL) NmcHardReset(0xFF);

nummod = 0;    
SioClose(ComPort);
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

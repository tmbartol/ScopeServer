//---------------------------------------------------------------------------

#include "sio_util.h"
//---------------------------------------------------------------------------
// Global variables
int printerrors = 1;
struct termios oldtio,newtio;
//---------------------------------------------------------------------------
//extern "C" WINAPI __declspec(dllexport) int SimpleMsgBox(char *msgstr)
//{
//return MessageBox(NULL, msgstr,"",MB_TASKMODAL | MB_SETFOREGROUND);
//}
//---------------------------------------------------------------------------
extern "C" void ErrorPrinting(int f)
{
printerrors = f;
}
//---------------------------------------------------------------------------
//extern "C" WINAPI __declspec(dllexport) int ErrorMsgBox(char *msgstr)
//{
//if (printerrors)
//  return MessageBox(NULL, msgstr,"",MB_TASKMODAL | MB_SETFOREGROUND);
//else return(0);
//}
//---------------------------------------------------------------------------
//Opens "COM1:" thru "COM4:", returns a handle to be used by other
//SIO operations.  Sets up read and write timeouts.
//*** Add parameter for baud rate ***
extern "C" HANDLE SioOpen(char *name, unsigned int baudrate)
{
BOOL RetStat;
HANDLE ComHandle;
DWORD winrate;
//char msgstr[50];

//Open COM port as a file
printf("Opening %s\n",name);
ComHandle = open(name, O_RDWR | O_NOCTTY);

//while (true)
//	{
	if (ComHandle < 0)
		{
        printf("%s failed to open\n",name);
        perror(name); 
        //exit(-1);
    	}

    switch (baudrate) {
		case 9600: 	winrate = B9600; break;
		case 19200: 	winrate = B19200; break;
		case 38400: 	winrate = B38400; break;
		case 57600: 	winrate = B57600; break;
		case 115200: 	winrate = B115200; break;
    	default:		printf("Baud rate not supported - using default of 19200\n");
    					winrate = B19200;
    	}
   // printf("tcgetattr(ComHandle,&oldtio)\n");
    tcgetattr(ComHandle,&oldtio); // save current port settings

    bzero(&newtio, sizeof(newtio));

    //newtio.c_cflag = winrate | CS8 | CREAD;
    newtio.c_cflag = winrate | CS8 | CLOCAL | CREAD;
    //newtio.c_iflag = 0;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;

    // Block indefinitely until one character is received
    //newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    //newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

    // Wait block for 100ms, then timeout
    newtio.c_cc[VTIME]    = 1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 1 chars received */

    //tcflush(ComHandle, TCIFLUSH);
    tcflush(ComHandle, TCIOFLUSH);
    //printf("tcsetattr(ComHandle,TCSANOW,&newtio)\n");
    RetStat = tcsetattr(ComHandle,TCSANOW,&newtio);

	if (RetStat < 0)
    	{
    	printf("Failed to set COMM configuration\n");
        //break;
        }

   // if (RetStat == 0)
   // 	{
   //     printf("Failed to set Comm timeouts\n");
   //     break;
   //     }

//    break;
//	}
//printf("Setup complete, returning file handle.\n");
return(ComHandle);
}
//---------------------------------------------------------------------------
//Change the baud rate to the specified values.  Valid rates are:
//9600, 19200, 38400, 57600, 115200.  Returns TRUE on success.
extern "C" BOOL SioChangeBaud(HANDLE ComPort, unsigned int baudrate)
{
BOOL RetStat;
DWORD winrate;

RetStat = tcgetattr(ComPort,&newtio);
if (RetStat == false) return RetStat;
switch (baudrate) {
	case 9600: 	winrate = B9600; break;
	case 19200: 	winrate = B19200; break;
	case 38400: 	winrate = B38400; break;
	case 57600: 	winrate = B57600; break;
	case 115200: 	winrate = B115200; break;
    default:		printf("Baud rate not supported\n");
    				return false;
    }
newtio.c_cflag = winrate | CS8 | CREAD;
RetStat = tcsetattr(ComPort,TCSANOW,&newtio);
if (RetStat == false) return RetStat;
return(true);
}

//---------------------------------------------------------------------------
//Write out N chars to the comport, returns only after chas have been sent
//return 0 on failure, non-zero on success
extern "C" BOOL SioPutChars(HANDLE ComPort, char *stuff, int n)
{
BOOL RetStat;
int j;

for(j=0;j<n;j++) {
  //printf("Writing %d/%d\n", j, n-1);
  RetStat = write(ComPort,&stuff[j],1);
  if (RetStat != 1) {
    printf("SioPutChars failed\n");
    printf("write returned %d errno = %d\n",RetStat,errno);
  }
}
tcdrain (ComPort);
return RetStat;
}
//---------------------------------------------------------------------------
//Read n chars into the array stuff (not null terminated)
//Function returns the number of chars actually read.
extern "C" DWORD SioGetChars(HANDLE ComPort, char *stuff, int n)
{
BOOL RetStat;
int j;
DWORD numread = 0;

for(j=0;j<n;j++) {
  RetStat = read(ComPort,&stuff[j],1);
  //printf("Read:%d/%d\n", j, n-1);
  if (RetStat != 1) {
    printf("SioReadChars failed\n");
    printf("read returned %d errno = %d\n",RetStat,errno);
  }
  else {
    numread++;
  }
}

return numread;
}
//---------------------------------------------------------------------------
//Returns the number of chars in a port's input buffer
//extern "C" DWORD SioTest(HANDLE ComPort)
//{
//COMSTAT cs;
//DWORD Errors;
//BOOL RetStat;
//
//RetStat = ClearCommError(ComPort, &Errors, &cs);
//if (RetStat == 0) printf("SioTest failed\n");
//return cs.cbInQue;
//}
//---------------------------------------------------------------------------
//Purge all chars from the input buffer
extern "C" BOOL SioClrInbuf(HANDLE ComPort)
{
BOOL RetStat;

RetStat = tcflush(ComPort,TCIFLUSH);
if (RetStat < 0) printf("SioClrInbuf failed\n");

return RetStat;
}
//---------------------------------------------------------------------------
//Close a previously opened COM port
extern "C" BOOL SioClose(HANDLE ComPort)
{
tcsetattr(ComPort,TCSANOW,&oldtio);
return(close(ComPort));
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
#ifndef sio_utilH
#define sio_utilH
//---------------------------------------------------------------------------
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

//#include "nmccom.h"

// Non-windows platforms need these too -BR
typedef int             BOOL;  //0=false, <>0 true
typedef int             HANDLE;
typedef unsigned long   DWORD;
#define INVALID_HANDLE_VALUE -1


extern "C" void ErrorPrinting(int f);
//extern "C" int ErrorMsgBox(char *msgstr);
//extern "C" int SimpleMsgBox(char *msgstr);
extern "C" HANDLE SioOpen(const char *name, unsigned int baudrate);
extern "C" BOOL SioPutChars(HANDLE ComPort, char *stuff, int n);
extern "C" DWORD SioGetChars(HANDLE ComPort, char *stuff, int n);
//extern "C" DWORD SioTest(HANDLE ComPort);
extern "C" BOOL SioClrInbuf(HANDLE ComPort);
extern "C" BOOL SioChangeBaud(HANDLE ComPort, unsigned int baudrate);
extern "C" BOOL SioClose(HANDLE ComPort);








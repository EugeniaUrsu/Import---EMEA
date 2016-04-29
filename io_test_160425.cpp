/**************************************************************************
版权声明：1999-2100，艾默生网络能源有限公司
设备介绍：IDU虚拟设备通讯协议-虚拟IO设备
文 件 名：io.cpp
开发工具：VC6.0
作    者：韦宏伟
版 本 号：V3.O0
日    期：2006.06.12
开发地点：深圳
应用地点：

  修改记录：修改人、日期、修改原因、版本变动情况。
    1. (请按时间、修改人、地点、修改原因、版本变动填写)
    2. 2006.12.26 whw 增加IO控制脉冲方式。
    3. 2007.08.03 lyg 调整so库接口 V4.01->V5.00
	4. eStoneV2专用修改V5.00


  说    明：描述该程序文件完成的主要功能等
  其    他：

    函数清单：
    1. 版本信息函数：
        DLLExport char* DLLInfo( void )
    2. V4.0/TSR/OCE 数据采集函数：
        DLLExport BOOL Read( HANDLE hComm, int nUnitNo, void *pData )
    3. V4.0/PowerStar 控制函数（参数有区别）：
        DLLExport BOOL Write( HANDLE hComm, int nUnitNo, char *pCmdStr )
    4. V4.1/V5.0 PowerStar 数据采集函数：
        DLLExport BOOL Query( HANDLE hComm, int nUnitNo, ENUMSIGNALPROC EnumProc, LPVOID lpvoid )
    5. V4.1/V5.0/TSR/OCE 控制函数：
        DLLExport BOOL Control( HANDLE hComm, int nUnitNo, char *pCmdStr )
    6. 跟踪测试采集函数
        DLLExport BOOL Test( HANDLE hComm, int nUnitNo, ENUMSIGNALPROC EnumProc, LPVOID lpvoid )

*****************************************************************************/


// 共用头文件的定义，适用于 VC++ 和 BC++部分
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include <math.h>
//#include "time.h"
//#include <sys/timeb.h>

// 本程序是智能设备协议解析的主程序，本段宏定义主要是用于DLL、TSR、OCE
// 及协议编程调测时使用
// 使用方法：
//   驱动程序类型                                应包括的宏定义
//   动态库(支持直连、VP6000、OCI-5)             _DLL_
//   现场调试用应用程序　                        _DLL_,_DEBUG_
//   AMS-1的TSR                                  _TSR_
//   OCE的DRV                                    _OCE_,_TSR_


#define _LINUX_
#ifdef _LINUX_
	#undef  _OCE_
	#undef  _WINDOWS_
	#undef  _TSR_
	#undef _DEBUG_
	#include "local_linux.h"
    extern bool bTestFlag;
	#include <sys/stat.h>
	// 跟踪测试用的数据记录文件名，请根据具体协议更改！
	char HEX_FILE[]={"eStoneII-IO.hex"};
	char ASC_FILE[]={"eStoneII-IO.asc"};
	char ErrLog_FILE[]={"eStoneII-IO.log"};

#else
	#include <stdio.h>
	#include <conio.h>
	#include <string.h>
	#include <stdlib.h>
	#include <math.h>
	#include <assert.h>

	#ifdef _WINDOWS
		#define _DLL_
	#endif


    #ifdef _CONSOLE
      #define _DLL_
      #define _DEBUG_
    #endif

    #define _OCE_        // OCE驱动程序用

    #ifdef  _DLL_
        #undef  _OCE_    // DLL驱动程序用
    #endif
    #ifdef  _TSR_
        #undef  _OCE_    // TSR驱动程序用
    #endif
    #ifdef  _OCE_
        #define  _TSR_   // TSR驱动程序用
    #endif

// 动态库使用部分
#ifdef _DLL_
    #include "local.h"
    #include "snuoci5.h"

    // 跟踪测试标志：TURE－跟踪测试运行，要调试信息；FALSE－普通运行。
    BOOL bTestFlag = FALSE;
    extern int nExtOci5ID;

    // 跟踪测试用的数据记录文件名，请根据具体协议更改！
    char HEX_FILE[]={"eStoneII-IO.HEX"};
    char ASC_FILE[]={"eStoneII-IO.ASC"};
    char ErrLog_FILE[]={"eStoneII-IO.Log"};

    // 如果记录告警发生时的采集原始数据请将bWriteErrLog置位。
    BOOL bWriteErrLog=FALSE;
    //BOOL bWriteErrLog=TRUE;
    #define HEXLOG_FILE  "eStoneII-IOLog.HEX"    // 告警记录文件名称

#endif  //end #ifdef _DLL_

#ifndef _OCE_
    #ifdef _TSR_
    #include "struct.h"
    #endif
#endif

#ifdef _OCE_
    #include "Comset.h"
    #ifndef _TSR_
        #define _TSR_
    #endif
#endif
#endif

//以下的常量需根据实际情况修改
const BYTE bySOI = 0x7E;         // 头字符
#ifdef _LINUX_
int nMaxChanelNo = 140;     // 最大采集信号个数
#else
#define nMaxChanelNo 140
#endif

const int nMaxDllBufNum = 512;   // DLL最大接收缓冲区大小，大于512时请谨慎考虑。
const int BUFFER_SIZE  = 1024 * 4;

// 当生成TSR或OCE时如果所需的接收缓冲区较大时请尽量用全局变量，因为它们的堆栈
// 较小。为支持多线程重入，动态库必须用局部变量，如确实需要用于写操作的全局变
// 量可用线程局部存储，在Local.H结构CGlobalInfo中增加一个元素。
#ifndef _DLL_
    BYTE Frame[nMaxDllBufNum]={0};    // 接收数据缓冲区：大小请根据情况调整。
#endif

typedef union
{
	BYTE byValue[4];
	float fValue;
}UNDATA;

// 数据结构表定义：
//     要求尽量按每个回收数据包的内容定义多个表；
//     对于结构相同的应避免重复定义，而放入程序循环，如电源模块参数。

// TODO: 请将数据处理部分的表结构、内容定义在此
//       要求结构、内容后均有注释
// 数据包解析结构
typedef struct
{
    int nType;          // 数据类型:0-结束标志,10-单字节开关量,11-单字节模拟量...
    int nOffset;        // 数据起始地址：在本数据包中的起始位置。
    int nScaleBit;      // 对于开关量,即位号；模拟量即比例关系,如10倍等。
    int nChannel;       // 系统分配的通道号，对于模块的参数，请用偏移表示，也可直接指定。
    // 为了增减信号时，简化模板库的维护，
    // 包括单数据包的情况也请保留本成员。
}DATASTRUCT;

// 请根据协议中各数据包的顺序、算法等，填写下表。
// 对于多数据包的情况，请根据每个数据包分别定义其包中的数据。在作数据处理时，
// 传输结构即可。
// 例如：

int n3130Start = 0; // 获取IO板模拟量数值
DATASTRUCT str3130Data[] =
{
    { 41,   0,   1,  0  },  // 模拟输入01值
    { 41,   4,   1,  1  },  // 模拟输入02值
    { 41,   8,   1,  2  },  // 模拟输入03值
    { 41,  12,   1,  3  },  // 模拟输入04值
    { 41,  16,   1,  4  },  // 模拟输入05值
    { 41,  20,   1,  5  },  // 模拟输入06值
    { 41,  24,   1,  6  },  // 模拟输入07值
    { 41,  28,   1,  7  },  // 模拟输入08值
    { 41,  32,   1,  8  },  // 电池电压1
    { 41,  36,   1,  9  },  // 电池电压2
    { 41,  40,   1, 10  },  // 电池电压3
    { 41,  44,   1, 11  },  // 电池电压4

    { 41,  48,   1, 45  },  // I2C sensor 13
    { 41,  52,   1, 46  },  // I2C sensor 14
	{ 41,  56,   1, 47  },  // I2C sensor 15
	{ 41,  60,   1, 48  },  // I2C sensor 16
	{ 41,  64,   1, 49  },  // I2C sensor 17
	{ 41,  68,   1, 50  },  // I2C sensor 18
	{ 41,  72,   1, 51  },  // I2C sensor 19
	{ 41,  76,   1, 52  },  // I2C sensor 20

	{ 41,  80,   1, 12  },  // I2C temperature
	{ 41,  84,   1, 13  },  // I2C humidity

    {  0,   0,   0,  0  }   // 结束
};

int n3131Start = 16; // 获取IO板开关量数值
DATASTRUCT str3131Data[] =
{
    { 10,   0,   0,  44  },  // DI1状态
    { 10,   0,   1,  45  },  // DI2状态
    { 10,   0,   2,  46  },  // DI3状态
    { 10,   0,   3,  43  },  // 水浸状态

	{ 10,   1,   0,  8  },  // 继电器1状态
    { 10,   1,   1,  9  },  // 继电器2状态
    { 10,   1,   2,  10  },  // 继电器3状态
    { 10,   1,   3,  11  },  // 继电器4状态
    { 10,   1,   4,  12  },  // 烟感状态

	{ 11,   2,   0,  0 },  // AI->DI 1
	{ 11,   2,   2,  1 },  // AI->DI 2
	{ 11,   2,   4,  2 },  // AI->DI 3
	{ 11,   2,   6,  3 },  // AI->DI 4
	{ 11,   3,   0,  4 },  // AI->DI 5
	{ 11,   3,   2,  5 },  // AI->DI 6
    { 11,   3,   4,  6 },  // AI->DI 7
    { 11,   3,   6,  7 },  // AI->DI 8


    {  0,   0,   0,  0  }   // 结束
};
/*
int n3132Start = 39; // 获取IO板继电器状态
DATASTRUCT str3132Data[] =
{
    { 10,   0,   0,  0  },  // 继电器1状态
    { 10,   0,   1,  1  },  // 继电器2状态
    { 10,   0,   2,  2  },  // 继电器3状态
    { 10,   0,   3,  3  },  // 继电器4状态
    { 10,   0,   4,  4  },  // 烟感状态


    {  0,   0,   0,  0  }   // 结束
};
*/



// 版本信息，在作修改后应补充到此栏中，以便在应用程序可查阅。
// 供 DLL/TSR 和 OCE 使用。
char Info[] = {
    "   eStoneII-IO设备通讯协议\n"
    " \n"
    " 版本号：V2.01\n"   // 修改后，请一定修改版本号。

    "  程序设计者：韦宏伟\n"
    "  开发日期：2006.06.12\n"
    "  开发地点：深圳\n"
    "  修改者：姜楠\n"
    "  修改说明：eStoneII专用\n"
	"  修改说明： 增加sample异常退出判断处理机制\n"

    " \n"
    " 注意事项: \n"
    " 1、跟踪测试生成的记录文件文件名为eStoneII-IO.asc和eStoneII-IO.hex\n" //注意修改！！
    " 2、测试时，如果发现数据有问题，请作跟踪测试。\n"
    " 本程序最后编译时间：\n"               // 请勿在此后增加信息!
    "                                \n"    // 请保留此行(分配内存用)
};

#ifndef _TSR_
//*****************************************************************
// 函数名称：DLLInfo();
// 功能描述：动态库版本中将信息包 Info 输出，以作版本信息等标志。
// 输入参数：Info--版本信息数组
// 输出参数：
// 返回：    版本信息数组
// 其他：
//*****************************************************************
DLLExport char* DLLInfo( )
{
    int nStrLen = strlen( Info );
    sprintf( Info+nStrLen-30, "%s ", __DATE__ );
    strcat( Info, __TIME__ );
    strcat( Info, " \n" );

    return Info;
}
#endif    // end of #ifdef _DLL_

//*****************************************************************
// 函数名称：ASSERT
// 功能描述：此函数是断言函数，调试使用。
// 输入参数：布尔型的条件值
// 输出参数：满足条件时，不做任何事，不满足条件，且处理于调试状态时，提
//           示出错信息，并选择继续运行，还是退出。
// 返回：    版本信息数组
// 其他：
//*****************************************************************
void ASSERT(BOOL bFlag)
{
#ifdef _DEBUG_
    if (!bFlag)
    {
        printf("Assert Error!\n");
        printf("Press Enter to ignore, other keys to exit.\n");

        //int nRet = getch();

        //if (nRet == 13)
        //{
        //    return;
       // }
        exit(1);
    }
#endif

    return;
}

//*****************************************************************
// 函数名称：StrToK
// 功能描述：将字符串S以cSep为分隔符分段
// 输入参数：S-源字符串，D-第一段字符串，cSep-分隔符
// 输出参数：
// 返回：    第一段字符串的个数
// 其他：
//*****************************************************************
int StrToK( char *S, char *D, char cSep )
{
    int i=0;

    ASSERT( S!=NULL && D!=NULL );

    while( S[i] && S[i] != cSep )
    {
        D[i] = S[i++];
    }

    D[i] = 0;

    //return S[i] ? i+1 : 0;    // 因BC的兼容性问题，禁止使用。
    if( S[i] )
    {
        return i + 1;
    }
    return 0;
}


// 功能：根据给定的以\0结尾的字符串，计算其校验码，并将结果放在最末尾。
void CheckSum( BYTE *Frame, int nLen )
{
    BYTE R=0;

    for( int i=1; i<nLen; i++ )        // 计算累加和，不包含头字符
    {
        R ^= Frame[i];
    }

    // 将校验码放在字符串尾。
    Frame[nLen] = R;
}


//*****************************************************************
// 函数名称：SendString
// 功能描述：向通讯口句柄hComm 发送字符串sSendStr的前 nStrLen个字符
// 输入参数：hComm - 通讯口句柄,sSendStr - 需要发送的字符串,
//           nStrLen - 需要发送的字符个数
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其他：
//*****************************************************************
BOOL SendString(
                HANDLE hComm,       // 通讯口句柄
                BYTE* sSendStr,     // 要发送的字符串指针
                int nStrLen )       // 要发送字符个数
{
    DWORD lWritten=0;           // 实际向设备发送的字符个数

    ASSERT( hComm!=0 && sSendStr!=NULL && nStrLen>0 );

    // 为了减少干扰，发送前清发送/接收缓冲区
    PurgeComm( hComm, PURGE_TXCLEAR );
    PurgeComm( hComm, PURGE_RXCLEAR );

    // 对于部分设备，需在发送数据包前暂停片刻，请打开下面的注释，并确认时间（单位：ms）。
    //Sleep( 500 );

    // 向设备 hComm 发送字符串 sSendStr 的前nStrLen个字符，并将实际写入个数返回到 lWritten 中。
    if( nStrLen )
    {
        WriteFile( hComm, (char*)sSendStr, nStrLen, &lWritten, NULL );
    }
    else
    {
        return TRUE;
    }

    // 只在动态库中作如下处理。
#ifndef _TSR_
    // 如果是跟踪测试，且非OCI－5方案(nExtOci5ID缺省为-1)，则记录调试信息。
    //if( bTestFlag && nExtOci5ID<0 )
    //{
        //MessageBox( NULL, (char*)sSendStr, "发送信息", MB_OK );    // 显示发送信息。
    //}   // end of if( bTestFlag )
//    bTestFlag = TRUE;
    WriteAsc( ASC_FILE, "\r\nSend:%d char\r\n", nStrLen );
    WriteAsc( HEX_FILE, "\r\nSend:%d char\r\n", nStrLen );
    WriteAsc( HEX_FILE, "\r\nSend Succeed:%d char\r\n", lWritten );
    WriteAsc( ASC_FILE, (char*)sSendStr, nStrLen );
    WriteHex( HEX_FILE, (char*)sSendStr, nStrLen );
//	bTestFlag = FALSE;
#endif    // end of #ifdef _DLL_

    return TRUE;
}

//*****************************************************************
// 函数名称：ReceiveString
// 功能描述：从设备 hComm 中向字符串 sRecStr 读入字符
// 输入参数：hComm - 通讯口句柄,sRecStr - 接收字符串指针,
//           nStrLen - 需要接收的字符个数
// 输出参数：实际接收的字符个数
// 返    回：成功采集的字符个数
// 其他：
//*****************************************************************
int ReceiveString(
                  HANDLE hComm,     // 通讯口句柄
                  int nUnitNo,      // 采集器单元地址
                  BYTE* sRecStr,    // 接收字符串指针
                  int nStrLen       // 需要接收的字符个数
                  )
{
    DWORD lRead=0;  //实际从设备读取的字符个数

    // OCI5传输有延时、波特率较低的设备等，请打开此注释
    //Sleep( 500 );

    ASSERT( hComm!=0 && sRecStr!=NULL );

    // 情况1：
    // 无反馈信息的情况，直接返回，通常是在执行控制命令时。
    if( nStrLen==0 )
    {
        Sleep( 1000 );  // 等待设备执行控制命令时间。
        return 0;
    }

   ReadFile( hComm, (char*)sRecStr, BUFFER_SIZE, &lRead, NULL );



    ////////////////////////////////////////////////////////////////////////
    //
    // 请注意 TSR.CPP文件 ReadFile函数中的超时时间 lTimeOut和结束字符 byEOI
    //
    ////////////////////////////////////////////////////////////////////////


    // 跟踪调试的接收信息处理：回应信息记录。
#ifndef _TSR_
    //if( bTestFlag )
    //{
        //MessageBox( NULL, (char*)sRecStr, "接收信息", MB_OK );  // 显示接收信息。
    //}   // end of if( bTestFlag )
//    bTestFlag = TRUE;
    WriteAsc( ASC_FILE, "\r\nRecv:%d char\r\n", (int)lRead );
    WriteAsc( HEX_FILE, "\r\nRecv:%d char\r\n", (int)lRead );
    WriteAsc( ASC_FILE, (char*)sRecStr );
    WriteHex( HEX_FILE, (char*)sRecStr, (int)lRead );
//	bTestFlag = FALSE;
#endif    // end of #ifdef _DLL_


    return (int)lRead;
}



//*****************************************************************
// 函数名称：Fix_Data
// 功能描述：对已存放在全局变量数组Frame[]中的数据根据数据结构所
//           定义的对应关系进行解析
// 输入参数：Frame - 原始数据数组, fData - 解析后的数据
//           strData - 用于进行数据解析的数据结构
// 其    他：
//*****************************************************************
void Fix_Data(
              float* fData,             // 处理好的数据缓冲区
              DATASTRUCT strData[],     // 要处理的数据结构
              BYTE *Frame )             // 要处理的字符串
{
    int nLoop = 0;
	UNDATA fV;

    ASSERT( fData!=NULL );

    while( strData[nLoop].nType )       // 循环直到结束：类型=0
    {
        switch( strData[nLoop].nType )  // 根据数据类型作相应处理。
        {
        case 41:
			memcpy( fV.byValue, Frame+strData[nLoop].nOffset, 4 );
            fData[strData[nLoop].nChannel] = fV.fValue / strData[nLoop].nScaleBit;
            break;
        case 10:
            fData[strData[nLoop].nChannel] = (float)
                (
                ( Frame[strData[nLoop].nOffset] >> strData[nLoop].nScaleBit )
                & 0x01
                );
			break;
		case 11:
            fData[strData[nLoop].nChannel] = (float)
                (
                ( Frame[strData[nLoop].nOffset] >> strData[nLoop].nScaleBit )
                & 0x03
                );
			break;
		default:
            break;
        }   //End Switch
        nLoop++;
    }   //End While

    // 记录告警数据
    //if( Frame[3]!=0x30 || Frame[4]!=0x30 )    // 要记录告警的条件
    //{
    //    WriteHexLog( "欠压状态", HEXLOG_FILE, Frame, nRecv );
    //}

}
#ifdef _LINUX_
char* FiltSpace( char* pKey )
{
    int i = 0;

    while( *pKey==' ' && *pKey!=0 )
    {
        pKey ++;
    }

    return pKey;
}
DWORD GetPrivateProfileString(char* pAppName, char* pKeyName, char* pDefStr, char* pRetStr, DWORD nSize, char* sCfgName)
{
    char    FileName[128] = { 0 };
    char    strTxtLine[1024] = { 0 };       // 定义一行字符串
    int     nStep = 0;
    BOOL    blExit = FALSE;
    char*   pKey;

	strcat( FileName, sCfgName );
    // 打开配置文件
    FILE *fp;
    fp = fopen( FileName, "rb" );
    //fp = fopen( "/home/idu_script/iolan.cfg", "rb" );

    if( fp == NULL)
    {
    //    AppLogOut( "%s:打开配置文件<%s>失败!\x0D\x0A", __FUNCTION__, FileName );
        blExit = TRUE;
    }

    sprintf( FileName, "[%s]", pAppName );  // 应用名

    memset( pRetStr, 0, nSize+1);
    strncpy( pRetStr, pDefStr, nSize );     // 默认返回值

    while( !blExit )
	{
        memset( strTxtLine, 0, 1024 );
        if( fgets( strTxtLine, 1024-1, fp ) == NULL )
        {
            break;
        }

        pKey = strTxtLine;

        switch( nStep )
        {
        case 0: // 查找应用名
            //if( strstr( strTxtLine, FileName ) != NULL )
            if( strncmp( (char*)strTxtLine, (char*)FileName, strlen((char*)FileName) ) == 0 )
            {
                nStep ++;
            }
            break;
        case 1: // 查找关键字
            if( strncmp((char*)strTxtLine, (char*)pKeyName, strlen((char*)pKeyName) )==0 )
            {
                pKey = strTxtLine;
            }
            else
            {
                pKey = NULL;
            }
            //pKey = strstr( strTxtLine, pKeyName );
            if( pKey == NULL )
            {   // 是否下一字段开始
                if( strstr( strTxtLine, "[" ) )
                {
                    blExit = TRUE;
                }
                break;
            }

            pKey += strlen(pKeyName);
            pKey = FiltSpace( pKey );
            if( pKey[0] != '=' )
            {
                break;
            }

            pKey ++;
            pKey = FiltSpace( pKey );
            if( pKey[0] != 0 )
            {
                strncpy( pRetStr, pKey, nSize );
            }

            blExit = TRUE;

            break;
        default:
            break;
        }
    };

    if( fp )
        fclose( fp );

    // 过滤掉后面的空格和回车换行
    int nLen = strlen(pRetStr);
    int i = 0;
    int j = 0;
    for( i=0; i<nLen; i++ )
    {
        if( pRetStr[i]==' ' )
        {
            j ++;
        }
        else if( pRetStr[i]==0x0d || pRetStr[i]==0x0a )
        {
            pRetStr[i-j] = 0;
            break;
        }
        else
        {
            j = 0;
        }
    }

    //AppLogOut("[%s]-----%s---------\n", __FUNCTION__, pRetStr);
    pRetStr[nSize] = 0;
    //AppLogOut("[%s]======%s=========\n", __FUNCTION__, pRetStr);

    return (DWORD)strlen(pRetStr);
}

BOOL WritePrivateProfileString(char* pAppName, char* pKeyName, char* pWriteStr, char* sCfgName)
{
    char    FileName[128] = { 0 };
    char    strTxtLine[1024] = { 0 };       // 定义一行字符串
	char	szAppName[100]={0};
    int     nStep = 0;
    BOOL    blExit = FALSE;
    char*   pKey;

	//char	szTemp[2048]={0x00};
	char	szTemp[5120]={0x00};

	// 获取应用程序所在的目录
    strcat( FileName, sCfgName );

    // 打开配置文件
    FILE *fp;
    fp = fopen( FileName, "rb+" );

	//printf("[Logic-%s]FileName<%s>%s\n", __FUNCTION__, FileName, pWriteStr);

    if( fp == NULL)
    {
        return FALSE;
    }

    sprintf( szAppName, "[%s]", pAppName );  // 应用名

	//printf("[Logic-%s]AppName<%s>\n", __FUNCTION__, FileName);

    while( !blExit )
	{
        memset( strTxtLine, 0, 1024 );
        if( fgets( strTxtLine, 1024-1, fp ) == NULL )
        {
            break;
        }

        pKey = strTxtLine;

        switch( nStep )
        {
        case 0: // 查找应用名
            if( strncmp( (char*)strTxtLine, (char*)szAppName, strlen((char*)szAppName) ) == 0 )
            {
				//printf("[Logic-%s]Find AppName<%s>\n", __FUNCTION__, FileName);
                nStep ++;
            }
            break;
        case 1: // 查找关键字
            if( strncmp((char*)strTxtLine, (char*)pKeyName, strlen((char*)pKeyName) )==0 )
            {
				//printf("[Logic-%s]Find KeyName<%s>\n", __FUNCTION__, pKeyName);
                pKey = strTxtLine;
            }
            else
            {
                pKey = NULL;
            }

			if( strTxtLine[strlen(pKeyName)] != ' ' && strTxtLine[strlen(pKeyName)] != '=')
			{
				pKey = NULL;
			}
            if( pKey != NULL )
            {
				long ptr=ftell(fp);
				int iOffset = strlen(strTxtLine);
                sprintf(strTxtLine, "%s=%s\r\n", pKeyName, pWriteStr);

				struct stat buf;
				int nRet = stat(FileName, &buf);
				if(nRet)
					return FALSE;
				int nTotalSize = buf.st_size;
				int nSizeLeft = fread(szTemp,1,nTotalSize-ptr,fp);
				if(feof(fp))
					return FALSE;

				ptr -= iOffset;
				fseek(fp, ptr, SEEK_SET);
                fwrite(strTxtLine,1,strlen(strTxtLine),fp);
                fwrite(szTemp,1,nSizeLeft,fp);
				fclose(fp);
				return TRUE;
            }

            // 是否下一应用开始
            if( strstr( strTxtLine, "[" ) )
            {
                blExit = TRUE;
            }

            break;

        default:

            blExit = TRUE;

            break;
        }
    };

    return FALSE;
}

#endif
//*****************************************************************
// 函数名称：GetResponeData
// 功能描述：查询数据包数据
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           sSendStr - 要发送的命令串, Frame - 接收数据缓冲区
//           nSend - 发送命令串的长度, nRecv - 接收的数据长度
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
BOOL GetResponeData(
                   HANDLE hComm,    // 通讯口句柄
                   int nUnitNo,     // 采集器单元地址
                   BYTE*sSendStr,   // 要发送字符串指针
                   BYTE*Frame,      // 要接收字符串指针
                   int nSend,       // 要发送字符个数
                   int nRecv )      // 要接收字符个数
{
    int nNum=0; // 重试次数计数器：提高通讯成功率。
    int nRet=0; // 接收函数返回值：实际接收到的字符个数。

    ASSERT( hComm!=0 && sSendStr!=NULL && Frame!=NULL );


    if( !SendString( hComm, sSendStr, nSend ) )     // 发送命令串
    {
        return FALSE;
    }

    // 为了部分返回特别慢的设备增加延时用。
    //Sleep( 1000 * (nNum+2) );

    nRet = ReceiveString( hComm, nUnitNo, Frame, nRecv );     // 接收数据
	if(nRet <= 0)
	{
		return FALSE;
	}



    return TRUE;
}
typedef struct
{
    float LCurrent;       //最小值
    float HCurrnet;       //最高值
    float VHCurrent;      //超高值
}IO_Config;

IO_Config m_IO_Config[9]; //Sebastian: add one more config for system voltage
bool bConfigFlag = TRUE;
//*****************************************************************
// 函数名称：GetData
// 功能描述：根据协议组织读取设备数据。
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           pData - 上报数据缓冲区指针
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
BOOL GetData( HANDLE hComm,     // 通讯口句柄
             int nUnitNo,       // 采集器单元地址
             float* pData )     // 上报数据缓冲区指针
{
    // 保证重入，动态库采用局部变量
#ifndef _TSR_
    BYTE Frame[nMaxDllBufNum]={ 0 };    // 接收数据缓冲区
#endif
    BYTE sSendStr[32]={0x00}; // 发送命令缓冲区

	int j = 0;
	if (bConfigFlag)
	{
		char szFileName[260] ={0};
		char szTemp[20]={0};
		char AppName[20];
#ifdef _LINUX_
		sprintf(szFileName,"%s","/home/idu/SO/IOcfg.ini");
#else
		char ProFileName[MAX_PATH];
		//GetCurrentDirectory(100, ProFileName);
		//sprintf(szFileName,"%s%s%d%s",ProFileName,"\\YD2050_",nUnitNo,".ini");
#endif
		for(j = 0; j < 10; j++)
    	{
    		sprintf(AppName, "IO%d%s", j+1,"_Cfg");

    		GetPrivateProfileString(AppName,"LowCurrent","0",szTemp,10,szFileName);
    		m_IO_Config[j].LCurrent = atof(szTemp);
    		GetPrivateProfileString(AppName,"HighCurrent","0",szTemp,10,szFileName);
    		m_IO_Config[j].HCurrnet = atof(szTemp);
    		GetPrivateProfileString(AppName,"VeryHighC","0",szTemp,10,szFileName);
    		m_IO_Config[j].VHCurrent = atof(szTemp);
    	}
		bConfigFlag = FALSE;
	}

    ASSERT( hComm!=0 && pData!=NULL );

    int nSend = 8;
    int nRecv = 8;

    // IO板模拟量数值、IO板开关量数值，IO板继电器状态
    int nCID2[] = { 0x30,0x31,0x00 };
    int i = 0;

    do
    {
        //CheckSum( sSendStr, nSend-1 );   // 校验码计算

		sSendStr[0] = nCID2[i];
        if( !GetResponeData( hComm, nUnitNo, sSendStr, Frame, nSend, nRecv ) )
        {
            return FALSE;
        }

        if( i == 0 )
            Fix_Data( pData+n3130Start, str3130Data, Frame); // IO板模拟量数值
        else if( i == 1 )
            Fix_Data( pData+n3131Start, str3131Data, Frame); // IO板开关量数值
//         else if( i == 2 )
//             Fix_Data( pData+n3132Start, str3132Data, Frame); // IO板继电器状态

    }while( nCID2[++i] != 0x00 );

	//8通道虚拟信号
	pData[63] = pData[8] - pData[9];
	pData[64] = pData[9] - pData[10];
	pData[65] = pData[10] - pData[11];
	pData[66] = pData[11];
	pData[67] = pData[20];
	pData[68] = pData[21];
	pData[69] = pData[22];
	pData[70] = pData[23];

	//add 20151103

	pData[100] = m_IO_Config[0].LCurrent;
	pData[101] = m_IO_Config[0].HCurrnet;
	pData[102] = m_IO_Config[0].VHCurrent;
	pData[103] = m_IO_Config[1].LCurrent;
	pData[104] = m_IO_Config[1].HCurrnet;
	pData[105] = m_IO_Config[1].VHCurrent;
	pData[106] = m_IO_Config[2].LCurrent;
	pData[107] = m_IO_Config[2].HCurrnet;
	pData[108] = m_IO_Config[2].VHCurrent;
	pData[109] = m_IO_Config[3].LCurrent;
	pData[110] = m_IO_Config[3].HCurrnet;
	pData[111] = m_IO_Config[3].VHCurrent;
	pData[112] = m_IO_Config[4].LCurrent;
	pData[113] = m_IO_Config[4].HCurrnet;
	pData[114] = m_IO_Config[4].VHCurrent;
	pData[115] = m_IO_Config[5].LCurrent;
	pData[116] = m_IO_Config[5].HCurrnet;
	pData[117] = m_IO_Config[5].VHCurrent;
	pData[118] = m_IO_Config[6].LCurrent;
	pData[119] = m_IO_Config[6].HCurrnet;
	pData[120] = m_IO_Config[6].VHCurrent;
	pData[121] = m_IO_Config[7].LCurrent;
	pData[122] = m_IO_Config[7].HCurrnet;
	pData[123] = m_IO_Config[7].VHCurrent;
    pData[124] = m_IO_Config[8].LCurrent; //Sebastian: added for voltage channel #8
	pData[125] = m_IO_Config[8].HCurrnet;
	pData[126] = m_IO_Config[8].VHCurrent;
    pData[127] = m_IO_Config[9].LCurrent; //Sebastian: added for voltage channel #9
	pData[128] = m_IO_Config[9].HCurrnet;
	pData[129] = m_IO_Config[9].VHCurrent;

	for(i=0;i<10;i++) // Sebastian: modify to include the battery voltage channel #9
	{
		if(pData[i] < m_IO_Config[i].LCurrent)
		{
			pData[130 + i] = 1;
		}
		else if (pData[i] > m_IO_Config[i].VHCurrent)
		{
			pData[130 + i] = 3;
		}
		else if (pData[i] > m_IO_Config[i].HCurrnet)
		{
			pData[130 + i] = 2;
		}
		else
		{
			pData[130 + i] = 0;
		}
	}

    return TRUE;
}



//*****************************************************************
// 函数名称：Read
// 功能描述：输出函数即PSMS与动态库/TSR等间的采集接口，采集并上报采
//           集的数据。采用本接口的有如下版本：PSMS4.0、TSR、
//           PowerStar、OCE等。
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           pData - 上报数据缓冲区指针
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
DLLExport BOOL Read( HANDLE hComm,  // 通讯口句柄
                    int nUnitNo,    // 采集器单元地址
                    void* pData )   // 上报数据缓冲区指针
{
    ASSERT( hComm!=0 && pData!=NULL );


    // 用于TSR跟踪测试：显示程序开始运行信息。
#ifdef _TSR_
    WriteString( "IDUIO Start!", 1, 23, 0x1f );   // 请修改文件名IDUIO
#endif


    int nRet = GetData( hComm, nUnitNo, (float*)pData );


    // 用于TSR跟踪测试：显示程序结束运行信息。
#ifdef _TSR_
    if( nRet )
    {
        WriteString( "T                 ", 1, 23, 0x1f );
    }
    else
    {
        WriteString( "F                 ", 1, 23, 0x1f );
    }
    // 采集周期控制：一般均需要，但时间应根据具体情况考虑增减。
    Sleep( 2000 );
#endif

    return nRet;
}

#ifdef _DLL_
//*****************************************************************
// 函数名称：Query
// 功能描述：输出函数即PSMS与动态库的采集接口，采集并上报采集的数据。
//           采用本接口的有如下版本：V4.1以上4.x版/V5.0版
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           pData - 上报数据缓冲区指针
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
DLLExport BOOL Query(
                     HANDLE hComm,              // 通讯口句柄
                     int nUnitNo,               // 采集器单元地址
                     ENUMSIGNALPROC EnumProc,   // 枚举函数
                     LPVOID lpvoid              // 空类型指针
                     )
{
    try // 用于捕捉动态库的错误.
    {
        float p[nMaxChanelNo]={0.0f};

        ASSERT( hComm!=0 );

        // 数据采集
        if( !Read( hComm, nUnitNo, p ) )
        {
            return FALSE;
        }


        // 数据上报
        for( int i=0; i<nMaxChanelNo; i++ )
        {
            EnumProc( i, p[i], lpvoid );
        }
    }
    catch( ... )
    {
        bTestFlag = TRUE;
        WriteAsc( ErrLog_FILE, "\r\nIDUIO.DLL Error\r\n" );
        bTestFlag = FALSE;
    }

    return TRUE;
}

// 跟踪测试入口函数
// 功能：在测试前，将全局标志 bTestFlag 置位；测试后，复位标志。
//*****************************************************************
// 函数名称：Test
// 功能描述：跟踪测试入口函数，在测试前，将全局标志 bTestFlag 置位；
//           测试后，复位标志。
// 输入参数：hComm - 通信句柄,
//           nUnitNo - 采集器单元地址,
//           pData - 上报数据缓冲区指针
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
DLLExport BOOL Test(HANDLE hComm,               // 通讯口句柄
                    int nUnitNo,                // 采集器单元地址
                    ENUMSIGNALPROC EnumProc,    // 枚举函数
                    LPVOID lpvoid)              // 空类型指针
{
    // 将调试标志置位
    bTestFlag = TRUE;

    char sBuildTime[28]={0};

    // 显示记录驱动程序编译时间用，用于确定现场版本。
    sprintf( sBuildTime, "//：%s ", __DATE__ );
    strcat( sBuildTime, __TIME__ );
    strcat( sBuildTime, "\r\n" );
    int nLen = strlen( sBuildTime );

    WriteAsc( ASC_FILE, sBuildTime, nLen );
    //WriteAsc( HEX_FILE, sBuildTime, nLen );

    // 调用采集函数采集数据，其中会因调试标志的置位而显示调试信息。
    BOOL bFlag = Query( hComm, nUnitNo, EnumProc, lpvoid );

    WriteAsc( ASC_FILE, "\r\n本次采集结束\r\n", 16 );
    WriteAsc( HEX_FILE, "\r\n本次采集结束\r\n", 16 );

    // 将调试标志复位
    bTestFlag = FALSE;

    return bFlag;
}
#endif



////////////////////////////////////////////////////////////////////////////////
//
// 设备控制命令部分：遥调、遥信、遥控
//
////////////////////////////////////////////////////////////////////////////////

// 控制命令
// 对于控制命令规范的设备，可在程序中作简单的说明即可。
// 对于控制命令无规律的设备，建议用下面的控制结构：

const int nMinControlNo = 10;   // 最小控制信号通道
const int nMaxControlNo = 50;   // 最大控制信号通道

//*****************************************************************
// 函数名称：SetDev
// 功能描述：控制命令的组织函数：根据协议的规定，组织命令
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           nCmdNo - 命令号, fValue - 控制值/状态
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
BOOL SetDev( HANDLE hComm,  // 通讯口句柄
            int nUnitNo,    // 采集器单元地址
            int nCmdNo,     // 控制命令号
            float fValue1,  // 控制参数1
            float fValue2 ) // 控制参数2
{
    // 保证重入，动态库采用局部变量
#ifndef _TSR_
    BYTE Frame[nMaxDllBufNum]={0};    // 接收数据缓冲区
#endif
    BYTE sSendStr[32]={0x00 }; // 获取IO板继电器状态

    char mTarget[128]={0};
    char ProFileName[100]={0x00};

    ASSERT( hComm!=0 );


    if( nCmdNo<nMinControlNo || nCmdNo>nMaxControlNo )
    {
        return FALSE;   // 控制命令非法：超出指定范围。
    }

    nCmdNo = nCmdNo-10;
    if(nCmdNo>10)
	{
		sprintf(ProFileName,"%s", "/home/idu/SO/IOcfg.ini");
		sprintf(mTarget,"%f",fValue1);

		if (nCmdNo==11)
		{
			WritePrivateProfileString("IO1_Cfg","LowCurrent", mTarget, ProFileName);
    	}
		else if (nCmdNo==12)
		{
			WritePrivateProfileString("IO1_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==13)
		{
			WritePrivateProfileString("IO1_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==14)
		{
			WritePrivateProfileString("IO2_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==15)
		{
			WritePrivateProfileString("IO2_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==16)
		{
			WritePrivateProfileString("IO2_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==17)
		{
			WritePrivateProfileString("IO3_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==18)
		{
			WritePrivateProfileString("IO3_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==19)
		{
			WritePrivateProfileString("IO3_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==20)
		{
			WritePrivateProfileString("IO4_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==21)
		{
			WritePrivateProfileString("IO4_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==22)
		{
			WritePrivateProfileString("IO4_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==23)
		{
			WritePrivateProfileString("IO5_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==24)
		{
			WritePrivateProfileString("IO5_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==25)
		{
			WritePrivateProfileString("IO5_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==26)
		{
			WritePrivateProfileString("IO6_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==27)
		{
			WritePrivateProfileString("IO6_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==28)
		{
			WritePrivateProfileString("IO6_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==29)
		{
			WritePrivateProfileString("IO7_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==30)
		{
			WritePrivateProfileString("IO7_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==31)
		{
			WritePrivateProfileString("IO7_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==32)
		{
			WritePrivateProfileString("IO8_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==33)
		{
			WritePrivateProfileString("IO8_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==34)
		{
			WritePrivateProfileString("IO8_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else if (nCmdNo==35)
		{
			WritePrivateProfileString("IO9_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==36)
		{
			WritePrivateProfileString("IO9_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==37)
		{
			WritePrivateProfileString("IO9_Cfg","VeryHighC", mTarget, ProFileName);
		}
        else if (nCmdNo==38)
		{
			WritePrivateProfileString("IO10_Cfg","LowCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==39)
		{
			WritePrivateProfileString("IO10_Cfg","HighCurrent", mTarget, ProFileName);
		}
		else if (nCmdNo==40)
		{
			WritePrivateProfileString("IO10_Cfg","VeryHighC", mTarget, ProFileName);
		}
		else
		{
			return FALSE;
		}

		bConfigFlag = TRUE;
		Sleep(500);
		return TRUE;
	}
	else
	{
		int nSend = 8;
    int nRecv = 8;

	sSendStr[0] = 0x33;         // CID2 = 0x33;
    sSendStr[1] = 0x04;         // Length = 4;
    sSendStr[3] = nCmdNo + 1;   // 继电器号1=RELAY1，2= RELAY 2，3= RELAY 3，4= RELAY 4
    sSendStr[4] = (BYTE)fValue2;// 0代表失电，1代表得电
    sSendStr[5] = ((WORD)fValue1) & 0xFF;// 得电/失电的延时时间，单位10ms
	sSendStr[6] = ((WORD)fValue1 >> 8 ) & 0xFF;

    int nRet = GetResponeData( hComm, nUnitNo, sSendStr, Frame, nSend, nRecv );

	}

    return TRUE;

}

//*****************************************************************
// 函数名称：Control
// 功能描述：PSMS V4.1及以上4.x版/V5.0版的控制函数。
// 输入参数：hComm - 通信句柄, nUnitNo - 采集器单元地址,
//           pCmdStr - 命令串，命令号在前，参数在后，用“,”分割。
// 输出参数：
// 返    回：TRUE－成功；FALSE－失败。
// 其    他：
//*****************************************************************
DLLExport BOOL Control(
                       HANDLE hComm,    // 通讯口句柄
                       int nUnitNo,     // 采集器单元地址
                       char *pCmdStr )  // 控制命令串
{
    char sTarget[128]={0};
    float fValue1 = 0.0f;
    float fValue2 = 0.0f;
//	bTestFlag = TRUE;

    //Get nCmdNo
    int nPoint = StrToK( pCmdStr, sTarget, ',' );

    if( pCmdStr==NULL )
    {
        return FALSE;
    }
    int nCmdNo = (int)atoi( pCmdStr );
    //Get Value
    if( nPoint>0 )
    {
        nPoint += StrToK( pCmdStr+nPoint, sTarget, ',' );

        if( sTarget==NULL )
        {
            return FALSE;
        }
        fValue1 = (float)atof( sTarget );

		WriteAsc( HEX_FILE, "\r\nfValue1 = %4.2f\r\n", fValue1 );

        nPoint = StrToK( pCmdStr+nPoint, sTarget, ',' );

        if( sTarget != NULL )
        {
            fValue2 = (float)atof( sTarget );
        }
		WriteAsc( HEX_FILE, "\r\nfValue2 = %4.2f\r\n", fValue2 );

    }

    // 对于超过一个参数的情况，请按取值的方式分解。
//    sSendStr[5] = (BYTE)fValue2;// 0代表失电，1代表得电
//    sSendStr[6] = (BYTE)fValue1;// 得电/失电的延时时间，单位10ms

    WriteAsc(HEX_FILE,"\r\n 得电:%d , 延时:%d \r\n",fValue2,fValue1);
    return SetDev( hComm, nUnitNo, nCmdNo, fValue1, fValue2 );
}





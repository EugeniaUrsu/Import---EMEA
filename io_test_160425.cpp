/**************************************************************************
��Ȩ������1999-2100����Ĭ��������Դ���޹�˾
�豸���ܣ�IDU�����豸ͨѶЭ��-����IO�豸
�� �� ����io.cpp
�������ߣ�VC6.0
��    �ߣ�Τ��ΰ
�� �� �ţ�V3.O0
��    �ڣ�2006.06.12
�����ص㣺����
Ӧ�õص㣺

  �޸ļ�¼���޸��ˡ����ڡ��޸�ԭ�򡢰汾�䶯�����
    1. (�밴ʱ�䡢�޸��ˡ��ص㡢�޸�ԭ�򡢰汾�䶯��д)
    2. 2006.12.26 whw ����IO�������巽ʽ��
    3. 2007.08.03 lyg ����so��ӿ� V4.01->V5.00
	4. eStoneV2ר���޸�V5.00


  ˵    ���������ó����ļ���ɵ���Ҫ���ܵ�
  ��    ����

    �����嵥��
    1. �汾��Ϣ������
        DLLExport char* DLLInfo( void )
    2. V4.0/TSR/OCE ���ݲɼ�������
        DLLExport BOOL Read( HANDLE hComm, int nUnitNo, void *pData )
    3. V4.0/PowerStar ���ƺ��������������𣩣�
        DLLExport BOOL Write( HANDLE hComm, int nUnitNo, char *pCmdStr )
    4. V4.1/V5.0 PowerStar ���ݲɼ�������
        DLLExport BOOL Query( HANDLE hComm, int nUnitNo, ENUMSIGNALPROC EnumProc, LPVOID lpvoid )
    5. V4.1/V5.0/TSR/OCE ���ƺ�����
        DLLExport BOOL Control( HANDLE hComm, int nUnitNo, char *pCmdStr )
    6. ���ٲ��Բɼ�����
        DLLExport BOOL Test( HANDLE hComm, int nUnitNo, ENUMSIGNALPROC EnumProc, LPVOID lpvoid )

*****************************************************************************/


// ����ͷ�ļ��Ķ��壬������ VC++ �� BC++����
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include <math.h>
//#include "time.h"
//#include <sys/timeb.h>

// �������������豸Э������������򣬱��κ궨����Ҫ������DLL��TSR��OCE
// ��Э���̵���ʱʹ��
// ʹ�÷�����
//   ������������                                Ӧ�����ĺ궨��
//   ��̬��(֧��ֱ����VP6000��OCI-5)             _DLL_
//   �ֳ�������Ӧ�ó���                        _DLL_,_DEBUG_
//   AMS-1��TSR                                  _TSR_
//   OCE��DRV                                    _OCE_,_TSR_


#define _LINUX_
#ifdef _LINUX_
	#undef  _OCE_
	#undef  _WINDOWS_
	#undef  _TSR_
	#undef _DEBUG_
	#include "local_linux.h"
    extern bool bTestFlag;
	#include <sys/stat.h>
	// ���ٲ����õ����ݼ�¼�ļ���������ݾ���Э����ģ�
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

    #define _OCE_        // OCE����������

    #ifdef  _DLL_
        #undef  _OCE_    // DLL����������
    #endif
    #ifdef  _TSR_
        #undef  _OCE_    // TSR����������
    #endif
    #ifdef  _OCE_
        #define  _TSR_   // TSR����������
    #endif

// ��̬��ʹ�ò���
#ifdef _DLL_
    #include "local.h"
    #include "snuoci5.h"

    // ���ٲ��Ա�־��TURE�����ٲ������У�Ҫ������Ϣ��FALSE����ͨ���С�
    BOOL bTestFlag = FALSE;
    extern int nExtOci5ID;

    // ���ٲ����õ����ݼ�¼�ļ���������ݾ���Э����ģ�
    char HEX_FILE[]={"eStoneII-IO.HEX"};
    char ASC_FILE[]={"eStoneII-IO.ASC"};
    char ErrLog_FILE[]={"eStoneII-IO.Log"};

    // �����¼�澯����ʱ�Ĳɼ�ԭʼ�����뽫bWriteErrLog��λ��
    BOOL bWriteErrLog=FALSE;
    //BOOL bWriteErrLog=TRUE;
    #define HEXLOG_FILE  "eStoneII-IOLog.HEX"    // �澯��¼�ļ�����

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

//���µĳ��������ʵ������޸�
const BYTE bySOI = 0x7E;         // ͷ�ַ�
#ifdef _LINUX_
int nMaxChanelNo = 140;     // ���ɼ��źŸ���
#else
#define nMaxChanelNo 140
#endif

const int nMaxDllBufNum = 512;   // DLL�����ջ�������С������512ʱ��������ǡ�
const int BUFFER_SIZE  = 1024 * 4;

// ������TSR��OCEʱ�������Ľ��ջ������ϴ�ʱ�뾡����ȫ�ֱ�������Ϊ���ǵĶ�ջ
// ��С��Ϊ֧�ֶ��߳����룬��̬������þֲ���������ȷʵ��Ҫ����д������ȫ�ֱ�
// �������ֲ߳̾��洢����Local.H�ṹCGlobalInfo������һ��Ԫ�ء�
#ifndef _DLL_
    BYTE Frame[nMaxDllBufNum]={0};    // �������ݻ���������С��������������
#endif

typedef union
{
	BYTE byValue[4];
	float fValue;
}UNDATA;

// ���ݽṹ���壺
//     Ҫ������ÿ���������ݰ������ݶ�������
//     ���ڽṹ��ͬ��Ӧ�����ظ����壬���������ѭ�������Դģ�������

// TODO: �뽫���ݴ����ֵı�ṹ�����ݶ����ڴ�
//       Ҫ��ṹ�����ݺ����ע��
// ���ݰ������ṹ
typedef struct
{
    int nType;          // ��������:0-������־,10-���ֽڿ�����,11-���ֽ�ģ����...
    int nOffset;        // ������ʼ��ַ���ڱ����ݰ��е���ʼλ�á�
    int nScaleBit;      // ���ڿ�����,��λ�ţ�ģ������������ϵ,��10���ȡ�
    int nChannel;       // ϵͳ�����ͨ���ţ�����ģ��Ĳ���������ƫ�Ʊ�ʾ��Ҳ��ֱ��ָ����
    // Ϊ�������ź�ʱ����ģ����ά����
    // ���������ݰ������Ҳ�뱣������Ա��
}DATASTRUCT;

// �����Э���и����ݰ���˳���㷨�ȣ���д�±�
// ���ڶ����ݰ�������������ÿ�����ݰ��ֱ�������е����ݡ��������ݴ���ʱ��
// ����ṹ���ɡ�
// ���磺

int n3130Start = 0; // ��ȡIO��ģ������ֵ
DATASTRUCT str3130Data[] =
{
    { 41,   0,   1,  0  },  // ģ������01ֵ
    { 41,   4,   1,  1  },  // ģ������02ֵ
    { 41,   8,   1,  2  },  // ģ������03ֵ
    { 41,  12,   1,  3  },  // ģ������04ֵ
    { 41,  16,   1,  4  },  // ģ������05ֵ
    { 41,  20,   1,  5  },  // ģ������06ֵ
    { 41,  24,   1,  6  },  // ģ������07ֵ
    { 41,  28,   1,  7  },  // ģ������08ֵ
    { 41,  32,   1,  8  },  // ��ص�ѹ1
    { 41,  36,   1,  9  },  // ��ص�ѹ2
    { 41,  40,   1, 10  },  // ��ص�ѹ3
    { 41,  44,   1, 11  },  // ��ص�ѹ4

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

    {  0,   0,   0,  0  }   // ����
};

int n3131Start = 16; // ��ȡIO�忪������ֵ
DATASTRUCT str3131Data[] =
{
    { 10,   0,   0,  44  },  // DI1״̬
    { 10,   0,   1,  45  },  // DI2״̬
    { 10,   0,   2,  46  },  // DI3״̬
    { 10,   0,   3,  43  },  // ˮ��״̬

	{ 10,   1,   0,  8  },  // �̵���1״̬
    { 10,   1,   1,  9  },  // �̵���2״̬
    { 10,   1,   2,  10  },  // �̵���3״̬
    { 10,   1,   3,  11  },  // �̵���4״̬
    { 10,   1,   4,  12  },  // �̸�״̬

	{ 11,   2,   0,  0 },  // AI->DI 1
	{ 11,   2,   2,  1 },  // AI->DI 2
	{ 11,   2,   4,  2 },  // AI->DI 3
	{ 11,   2,   6,  3 },  // AI->DI 4
	{ 11,   3,   0,  4 },  // AI->DI 5
	{ 11,   3,   2,  5 },  // AI->DI 6
    { 11,   3,   4,  6 },  // AI->DI 7
    { 11,   3,   6,  7 },  // AI->DI 8


    {  0,   0,   0,  0  }   // ����
};
/*
int n3132Start = 39; // ��ȡIO��̵���״̬
DATASTRUCT str3132Data[] =
{
    { 10,   0,   0,  0  },  // �̵���1״̬
    { 10,   0,   1,  1  },  // �̵���2״̬
    { 10,   0,   2,  2  },  // �̵���3״̬
    { 10,   0,   3,  3  },  // �̵���4״̬
    { 10,   0,   4,  4  },  // �̸�״̬


    {  0,   0,   0,  0  }   // ����
};
*/



// �汾��Ϣ�������޸ĺ�Ӧ���䵽�����У��Ա���Ӧ�ó���ɲ��ġ�
// �� DLL/TSR �� OCE ʹ�á�
char Info[] = {
    "   eStoneII-IO�豸ͨѶЭ��\n"
    " \n"
    " �汾�ţ�V2.01\n"   // �޸ĺ���һ���޸İ汾�š�

    "  ��������ߣ�Τ��ΰ\n"
    "  �������ڣ�2006.06.12\n"
    "  �����ص㣺����\n"
    "  �޸��ߣ����\n"
    "  �޸�˵����eStoneIIר��\n"
	"  �޸�˵���� ����sample�쳣�˳��жϴ������\n"

    " \n"
    " ע������: \n"
    " 1�����ٲ������ɵļ�¼�ļ��ļ���ΪeStoneII-IO.asc��eStoneII-IO.hex\n" //ע���޸ģ���
    " 2������ʱ������������������⣬�������ٲ��ԡ�\n"
    " ������������ʱ�䣺\n"               // �����ڴ˺�������Ϣ!
    "                                \n"    // �뱣������(�����ڴ���)
};

#ifndef _TSR_
//*****************************************************************
// �������ƣ�DLLInfo();
// ������������̬��汾�н���Ϣ�� Info ����������汾��Ϣ�ȱ�־��
// ���������Info--�汾��Ϣ����
// ���������
// ���أ�    �汾��Ϣ����
// ������
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
// �������ƣ�ASSERT
// �����������˺����Ƕ��Ժ���������ʹ�á�
// ��������������͵�����ֵ
// �����������������ʱ�������κ��£��������������Ҵ����ڵ���״̬ʱ����
//           ʾ������Ϣ����ѡ��������У������˳���
// ���أ�    �汾��Ϣ����
// ������
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
// �������ƣ�StrToK
// �������������ַ���S��cSepΪ�ָ����ֶ�
// ���������S-Դ�ַ�����D-��һ���ַ�����cSep-�ָ���
// ���������
// ���أ�    ��һ���ַ����ĸ���
// ������
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

    //return S[i] ? i+1 : 0;    // ��BC�ļ��������⣬��ֹʹ�á�
    if( S[i] )
    {
        return i + 1;
    }
    return 0;
}


// ���ܣ����ݸ�������\0��β���ַ�����������У���룬�������������ĩβ��
void CheckSum( BYTE *Frame, int nLen )
{
    BYTE R=0;

    for( int i=1; i<nLen; i++ )        // �����ۼӺͣ�������ͷ�ַ�
    {
        R ^= Frame[i];
    }

    // ��У��������ַ���β��
    Frame[nLen] = R;
}


//*****************************************************************
// �������ƣ�SendString
// ������������ͨѶ�ھ��hComm �����ַ���sSendStr��ǰ nStrLen���ַ�
// ���������hComm - ͨѶ�ھ��,sSendStr - ��Ҫ���͵��ַ���,
//           nStrLen - ��Ҫ���͵��ַ�����
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ������
//*****************************************************************
BOOL SendString(
                HANDLE hComm,       // ͨѶ�ھ��
                BYTE* sSendStr,     // Ҫ���͵��ַ���ָ��
                int nStrLen )       // Ҫ�����ַ�����
{
    DWORD lWritten=0;           // ʵ�����豸���͵��ַ�����

    ASSERT( hComm!=0 && sSendStr!=NULL && nStrLen>0 );

    // Ϊ�˼��ٸ��ţ�����ǰ�巢��/���ջ�����
    PurgeComm( hComm, PURGE_TXCLEAR );
    PurgeComm( hComm, PURGE_RXCLEAR );

    // ���ڲ����豸�����ڷ������ݰ�ǰ��ͣƬ�̣���������ע�ͣ���ȷ��ʱ�䣨��λ��ms����
    //Sleep( 500 );

    // ���豸 hComm �����ַ��� sSendStr ��ǰnStrLen���ַ�������ʵ��д��������ص� lWritten �С�
    if( nStrLen )
    {
        WriteFile( hComm, (char*)sSendStr, nStrLen, &lWritten, NULL );
    }
    else
    {
        return TRUE;
    }

    // ֻ�ڶ�̬���������´���
#ifndef _TSR_
    // ����Ǹ��ٲ��ԣ��ҷ�OCI��5����(nExtOci5IDȱʡΪ-1)�����¼������Ϣ��
    //if( bTestFlag && nExtOci5ID<0 )
    //{
        //MessageBox( NULL, (char*)sSendStr, "������Ϣ", MB_OK );    // ��ʾ������Ϣ��
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
// �������ƣ�ReceiveString
// �������������豸 hComm �����ַ��� sRecStr �����ַ�
// ���������hComm - ͨѶ�ھ��,sRecStr - �����ַ���ָ��,
//           nStrLen - ��Ҫ���յ��ַ�����
// ���������ʵ�ʽ��յ��ַ�����
// ��    �أ��ɹ��ɼ����ַ�����
// ������
//*****************************************************************
int ReceiveString(
                  HANDLE hComm,     // ͨѶ�ھ��
                  int nUnitNo,      // �ɼ�����Ԫ��ַ
                  BYTE* sRecStr,    // �����ַ���ָ��
                  int nStrLen       // ��Ҫ���յ��ַ�����
                  )
{
    DWORD lRead=0;  //ʵ�ʴ��豸��ȡ���ַ�����

    // OCI5��������ʱ�������ʽϵ͵��豸�ȣ���򿪴�ע��
    //Sleep( 500 );

    ASSERT( hComm!=0 && sRecStr!=NULL );

    // ���1��
    // �޷�����Ϣ�������ֱ�ӷ��أ�ͨ������ִ�п�������ʱ��
    if( nStrLen==0 )
    {
        Sleep( 1000 );  // �ȴ��豸ִ�п�������ʱ�䡣
        return 0;
    }

   ReadFile( hComm, (char*)sRecStr, BUFFER_SIZE, &lRead, NULL );



    ////////////////////////////////////////////////////////////////////////
    //
    // ��ע�� TSR.CPP�ļ� ReadFile�����еĳ�ʱʱ�� lTimeOut�ͽ����ַ� byEOI
    //
    ////////////////////////////////////////////////////////////////////////


    // ���ٵ��ԵĽ�����Ϣ������Ӧ��Ϣ��¼��
#ifndef _TSR_
    //if( bTestFlag )
    //{
        //MessageBox( NULL, (char*)sRecStr, "������Ϣ", MB_OK );  // ��ʾ������Ϣ��
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
// �������ƣ�Fix_Data
// �������������Ѵ����ȫ�ֱ�������Frame[]�е����ݸ������ݽṹ��
//           ����Ķ�Ӧ��ϵ���н���
// ���������Frame - ԭʼ��������, fData - �����������
//           strData - ���ڽ������ݽ��������ݽṹ
// ��    ����
//*****************************************************************
void Fix_Data(
              float* fData,             // ����õ����ݻ�����
              DATASTRUCT strData[],     // Ҫ��������ݽṹ
              BYTE *Frame )             // Ҫ������ַ���
{
    int nLoop = 0;
	UNDATA fV;

    ASSERT( fData!=NULL );

    while( strData[nLoop].nType )       // ѭ��ֱ������������=0
    {
        switch( strData[nLoop].nType )  // ����������������Ӧ����
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

    // ��¼�澯����
    //if( Frame[3]!=0x30 || Frame[4]!=0x30 )    // Ҫ��¼�澯������
    //{
    //    WriteHexLog( "Ƿѹ״̬", HEXLOG_FILE, Frame, nRecv );
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
    char    strTxtLine[1024] = { 0 };       // ����һ���ַ���
    int     nStep = 0;
    BOOL    blExit = FALSE;
    char*   pKey;

	strcat( FileName, sCfgName );
    // �������ļ�
    FILE *fp;
    fp = fopen( FileName, "rb" );
    //fp = fopen( "/home/idu_script/iolan.cfg", "rb" );

    if( fp == NULL)
    {
    //    AppLogOut( "%s:�������ļ�<%s>ʧ��!\x0D\x0A", __FUNCTION__, FileName );
        blExit = TRUE;
    }

    sprintf( FileName, "[%s]", pAppName );  // Ӧ����

    memset( pRetStr, 0, nSize+1);
    strncpy( pRetStr, pDefStr, nSize );     // Ĭ�Ϸ���ֵ

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
        case 0: // ����Ӧ����
            //if( strstr( strTxtLine, FileName ) != NULL )
            if( strncmp( (char*)strTxtLine, (char*)FileName, strlen((char*)FileName) ) == 0 )
            {
                nStep ++;
            }
            break;
        case 1: // ���ҹؼ���
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
            {   // �Ƿ���һ�ֶο�ʼ
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

    // ���˵�����Ŀո�ͻس�����
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
    char    strTxtLine[1024] = { 0 };       // ����һ���ַ���
	char	szAppName[100]={0};
    int     nStep = 0;
    BOOL    blExit = FALSE;
    char*   pKey;

	//char	szTemp[2048]={0x00};
	char	szTemp[5120]={0x00};

	// ��ȡӦ�ó������ڵ�Ŀ¼
    strcat( FileName, sCfgName );

    // �������ļ�
    FILE *fp;
    fp = fopen( FileName, "rb+" );

	//printf("[Logic-%s]FileName<%s>%s\n", __FUNCTION__, FileName, pWriteStr);

    if( fp == NULL)
    {
        return FALSE;
    }

    sprintf( szAppName, "[%s]", pAppName );  // Ӧ����

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
        case 0: // ����Ӧ����
            if( strncmp( (char*)strTxtLine, (char*)szAppName, strlen((char*)szAppName) ) == 0 )
            {
				//printf("[Logic-%s]Find AppName<%s>\n", __FUNCTION__, FileName);
                nStep ++;
            }
            break;
        case 1: // ���ҹؼ���
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

            // �Ƿ���һӦ�ÿ�ʼ
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
// �������ƣ�GetResponeData
// ������������ѯ���ݰ�����
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           sSendStr - Ҫ���͵����, Frame - �������ݻ�����
//           nSend - ��������ĳ���, nRecv - ���յ����ݳ���
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
BOOL GetResponeData(
                   HANDLE hComm,    // ͨѶ�ھ��
                   int nUnitNo,     // �ɼ�����Ԫ��ַ
                   BYTE*sSendStr,   // Ҫ�����ַ���ָ��
                   BYTE*Frame,      // Ҫ�����ַ���ָ��
                   int nSend,       // Ҫ�����ַ�����
                   int nRecv )      // Ҫ�����ַ�����
{
    int nNum=0; // ���Դ��������������ͨѶ�ɹ��ʡ�
    int nRet=0; // ���պ�������ֵ��ʵ�ʽ��յ����ַ�������

    ASSERT( hComm!=0 && sSendStr!=NULL && Frame!=NULL );


    if( !SendString( hComm, sSendStr, nSend ) )     // �������
    {
        return FALSE;
    }

    // Ϊ�˲��ַ����ر������豸������ʱ�á�
    //Sleep( 1000 * (nNum+2) );

    nRet = ReceiveString( hComm, nUnitNo, Frame, nRecv );     // ��������
	if(nRet <= 0)
	{
		return FALSE;
	}



    return TRUE;
}
typedef struct
{
    float LCurrent;       //��Сֵ
    float HCurrnet;       //���ֵ
    float VHCurrent;      //����ֵ
}IO_Config;

IO_Config m_IO_Config[9]; //Sebastian: add one more config for system voltage
bool bConfigFlag = TRUE;
//*****************************************************************
// �������ƣ�GetData
// ��������������Э����֯��ȡ�豸���ݡ�
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           pData - �ϱ����ݻ�����ָ��
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
BOOL GetData( HANDLE hComm,     // ͨѶ�ھ��
             int nUnitNo,       // �ɼ�����Ԫ��ַ
             float* pData )     // �ϱ����ݻ�����ָ��
{
    // ��֤���룬��̬����þֲ�����
#ifndef _TSR_
    BYTE Frame[nMaxDllBufNum]={ 0 };    // �������ݻ�����
#endif
    BYTE sSendStr[32]={0x00}; // �����������

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

    // IO��ģ������ֵ��IO�忪������ֵ��IO��̵���״̬
    int nCID2[] = { 0x30,0x31,0x00 };
    int i = 0;

    do
    {
        //CheckSum( sSendStr, nSend-1 );   // У�������

		sSendStr[0] = nCID2[i];
        if( !GetResponeData( hComm, nUnitNo, sSendStr, Frame, nSend, nRecv ) )
        {
            return FALSE;
        }

        if( i == 0 )
            Fix_Data( pData+n3130Start, str3130Data, Frame); // IO��ģ������ֵ
        else if( i == 1 )
            Fix_Data( pData+n3131Start, str3131Data, Frame); // IO�忪������ֵ
//         else if( i == 2 )
//             Fix_Data( pData+n3132Start, str3132Data, Frame); // IO��̵���״̬

    }while( nCID2[++i] != 0x00 );

	//8ͨ�������ź�
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
// �������ƣ�Read
// �������������������PSMS�붯̬��/TSR�ȼ�Ĳɼ��ӿڣ��ɼ����ϱ���
//           �������ݡ����ñ��ӿڵ������°汾��PSMS4.0��TSR��
//           PowerStar��OCE�ȡ�
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           pData - �ϱ����ݻ�����ָ��
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
DLLExport BOOL Read( HANDLE hComm,  // ͨѶ�ھ��
                    int nUnitNo,    // �ɼ�����Ԫ��ַ
                    void* pData )   // �ϱ����ݻ�����ָ��
{
    ASSERT( hComm!=0 && pData!=NULL );


    // ����TSR���ٲ��ԣ���ʾ����ʼ������Ϣ��
#ifdef _TSR_
    WriteString( "IDUIO Start!", 1, 23, 0x1f );   // ���޸��ļ���IDUIO
#endif


    int nRet = GetData( hComm, nUnitNo, (float*)pData );


    // ����TSR���ٲ��ԣ���ʾ�������������Ϣ��
#ifdef _TSR_
    if( nRet )
    {
        WriteString( "T                 ", 1, 23, 0x1f );
    }
    else
    {
        WriteString( "F                 ", 1, 23, 0x1f );
    }
    // �ɼ����ڿ��ƣ�һ�����Ҫ����ʱ��Ӧ���ݾ����������������
    Sleep( 2000 );
#endif

    return nRet;
}

#ifdef _DLL_
//*****************************************************************
// �������ƣ�Query
// �������������������PSMS�붯̬��Ĳɼ��ӿڣ��ɼ����ϱ��ɼ������ݡ�
//           ���ñ��ӿڵ������°汾��V4.1����4.x��/V5.0��
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           pData - �ϱ����ݻ�����ָ��
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
DLLExport BOOL Query(
                     HANDLE hComm,              // ͨѶ�ھ��
                     int nUnitNo,               // �ɼ�����Ԫ��ַ
                     ENUMSIGNALPROC EnumProc,   // ö�ٺ���
                     LPVOID lpvoid              // ������ָ��
                     )
{
    try // ���ڲ�׽��̬��Ĵ���.
    {
        float p[nMaxChanelNo]={0.0f};

        ASSERT( hComm!=0 );

        // ���ݲɼ�
        if( !Read( hComm, nUnitNo, p ) )
        {
            return FALSE;
        }


        // �����ϱ�
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

// ���ٲ�����ں���
// ���ܣ��ڲ���ǰ����ȫ�ֱ�־ bTestFlag ��λ�����Ժ󣬸�λ��־��
//*****************************************************************
// �������ƣ�Test
// �������������ٲ�����ں������ڲ���ǰ����ȫ�ֱ�־ bTestFlag ��λ��
//           ���Ժ󣬸�λ��־��
// ���������hComm - ͨ�ž��,
//           nUnitNo - �ɼ�����Ԫ��ַ,
//           pData - �ϱ����ݻ�����ָ��
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
DLLExport BOOL Test(HANDLE hComm,               // ͨѶ�ھ��
                    int nUnitNo,                // �ɼ�����Ԫ��ַ
                    ENUMSIGNALPROC EnumProc,    // ö�ٺ���
                    LPVOID lpvoid)              // ������ָ��
{
    // �����Ա�־��λ
    bTestFlag = TRUE;

    char sBuildTime[28]={0};

    // ��ʾ��¼�����������ʱ���ã�����ȷ���ֳ��汾��
    sprintf( sBuildTime, "//��%s ", __DATE__ );
    strcat( sBuildTime, __TIME__ );
    strcat( sBuildTime, "\r\n" );
    int nLen = strlen( sBuildTime );

    WriteAsc( ASC_FILE, sBuildTime, nLen );
    //WriteAsc( HEX_FILE, sBuildTime, nLen );

    // ���òɼ������ɼ����ݣ����л�����Ա�־����λ����ʾ������Ϣ��
    BOOL bFlag = Query( hComm, nUnitNo, EnumProc, lpvoid );

    WriteAsc( ASC_FILE, "\r\n���βɼ�����\r\n", 16 );
    WriteAsc( HEX_FILE, "\r\n���βɼ�����\r\n", 16 );

    // �����Ա�־��λ
    bTestFlag = FALSE;

    return bFlag;
}
#endif



////////////////////////////////////////////////////////////////////////////////
//
// �豸��������֣�ң����ң�š�ң��
//
////////////////////////////////////////////////////////////////////////////////

// ��������
// ���ڿ�������淶���豸�����ڳ��������򵥵�˵�����ɡ�
// ���ڿ��������޹��ɵ��豸������������Ŀ��ƽṹ��

const int nMinControlNo = 10;   // ��С�����ź�ͨ��
const int nMaxControlNo = 50;   // �������ź�ͨ��

//*****************************************************************
// �������ƣ�SetDev
// ���������������������֯����������Э��Ĺ涨����֯����
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           nCmdNo - �����, fValue - ����ֵ/״̬
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
BOOL SetDev( HANDLE hComm,  // ͨѶ�ھ��
            int nUnitNo,    // �ɼ�����Ԫ��ַ
            int nCmdNo,     // ���������
            float fValue1,  // ���Ʋ���1
            float fValue2 ) // ���Ʋ���2
{
    // ��֤���룬��̬����þֲ�����
#ifndef _TSR_
    BYTE Frame[nMaxDllBufNum]={0};    // �������ݻ�����
#endif
    BYTE sSendStr[32]={0x00 }; // ��ȡIO��̵���״̬

    char mTarget[128]={0};
    char ProFileName[100]={0x00};

    ASSERT( hComm!=0 );


    if( nCmdNo<nMinControlNo || nCmdNo>nMaxControlNo )
    {
        return FALSE;   // ��������Ƿ�������ָ����Χ��
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
    sSendStr[3] = nCmdNo + 1;   // �̵�����1=RELAY1��2= RELAY 2��3= RELAY 3��4= RELAY 4
    sSendStr[4] = (BYTE)fValue2;// 0����ʧ�磬1����õ�
    sSendStr[5] = ((WORD)fValue1) & 0xFF;// �õ�/ʧ�����ʱʱ�䣬��λ10ms
	sSendStr[6] = ((WORD)fValue1 >> 8 ) & 0xFF;

    int nRet = GetResponeData( hComm, nUnitNo, sSendStr, Frame, nSend, nRecv );

	}

    return TRUE;

}

//*****************************************************************
// �������ƣ�Control
// ����������PSMS V4.1������4.x��/V5.0��Ŀ��ƺ�����
// ���������hComm - ͨ�ž��, nUnitNo - �ɼ�����Ԫ��ַ,
//           pCmdStr - ������������ǰ�������ں��á�,���ָ
// ���������
// ��    �أ�TRUE���ɹ���FALSE��ʧ�ܡ�
// ��    ����
//*****************************************************************
DLLExport BOOL Control(
                       HANDLE hComm,    // ͨѶ�ھ��
                       int nUnitNo,     // �ɼ�����Ԫ��ַ
                       char *pCmdStr )  // �������
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

    // ���ڳ���һ��������������밴ȡֵ�ķ�ʽ�ֽ⡣
//    sSendStr[5] = (BYTE)fValue2;// 0����ʧ�磬1����õ�
//    sSendStr[6] = (BYTE)fValue1;// �õ�/ʧ�����ʱʱ�䣬��λ10ms

    WriteAsc(HEX_FILE,"\r\n �õ�:%d , ��ʱ:%d \r\n",fValue2,fValue1);
    return SetDev( hComm, nUnitNo, nCmdNo, fValue1, fValue2 );
}





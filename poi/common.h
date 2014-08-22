//--------------------------------------------------------------------------
//	共通ヘッダファイル														
//--------------------------------------------------------------------------

#ifndef __COMMON_H__
#define __COMMON_H__

#include <limits.h>

typedef	unsigned char	U1;
typedef	unsigned short	U2;
typedef unsigned long	U4;
typedef signed	char	S1;
typedef	signed	short	S2;
typedef	signed	long	S4;
typedef int Bool;

// 1byte ビットフィールﾄﾞ共用体
typedef	union{
	U1	byte;
	struct{
		U1	b0:1;
		U1	b1:1;
		U1	b2:1;
		U1	b3:1;
		U1	b4:1;
		U1	b5:1;
		U1	b6:1;
		U1	b7:1;
	}bit;
	struct{
		U1	lo:4;
		U1	hi:4;
	}nbl;
}F1;

// 4byte ビットフィールﾄﾞ共用体
typedef union{
	U4  dword;
	U4	word;
	struct{
		U2	lo;
		U2	hi;
	}hword;
	U1	byte[4];
	struct{
		U4	bit00:1;
		U4	bit01:1;
		U4	bit02:1;
		U4	bit03:1;
		U4	bit04:1;
		U4	bit05:1;
		U4	bit06:1;
		U4	bit07:1;

		U4	bit08:1;
		U4	bit09:1;
		U4	bit10:1;
		U4	bit11:1;
		U4	bit12:1;
		U4	bit13:1;
		U4	bit14:1;
		U4	bit15:1;

		U4	bit16:1;
		U4	bit17:1;
		U4	bit18:1;
		U4	bit19:1;
		U4	bit20:1;
		U4	bit21:1;
		U4	bit22:1;
		U4	bit23:1;

		U4	bit24:1;
		U4	bit25:1;
		U4	bit26:1;
		U4	bit27:1;
		U4	bit28:1;
		U4	bit29:1;
		U4	bit30:1;
		U4	bit31:1;
	}bit;
}F4;

#define	NG		0
#define	OK		1

#define	FALSE	0
#define	TRUE	1

#define	OFF		0
#define	ON		1

#define	SMALL	0
#define	LARGE	1

#define	LO		0
#define	HI		1

#define	U1_MAX	UCHAR_MAX
#define	U2_MAX	USHRT_MAX
#define	U4_MAX	ULONG_MAX

#define	S1_MAX	SCHAR_MAX
#define	S1_MIN	SCHAR_MIN
#define	S2_MAX	SHRT_MAX
#define	S2_MIN	SHRT_MIN
#define	S4_MAX	LONG_MAX
#define	S4_MIN	LONG_MIN

#ifndef NULL
#define	NULL	0
#endif

// 絶対値差分算出マクロ
#define	ABS_SUB(a,b)		((a >= b) ? (a-b) : (b-a))

#endif


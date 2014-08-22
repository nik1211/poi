//--------------------------------------------------------------------------//
//	���L�f�[�^�w�b�_�t�@�C��												//
//--------------------------------------------------------------------------//

#ifndef __COMDAT_H__
#define __COMDAT_H__

#include <limits.h>

typedef	unsigned char	U1;
typedef	unsigned short	U2;
typedef unsigned long	U4;
typedef signed	char	S1;
typedef	signed	short	S2;
typedef	signed	long	S4;
typedef int Bool;

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

// ��Βl�����Z�o�}�N��
#define	ABS_SUB(a,b)		((a >= b) ? (a-b) : (b-a))

// ���
typedef enum{
	TGT_DUMMY = 0,							// �_�~�[									0
	TGT_RD_ORBIS,							// RD���I�[�r�X								1
	TGT_HSYS_ORBIS,							// H�V�X�e�����I�[�r�X						2
	TGT_LHSYS_ORBIS,						// LH�V�X�e�����I�[�r�X						3
	TGT_LOOP_ORBIS,							// ���[�v�R�C�����I�[�r�X					4
	TGT_KODEN_ORBIS,						// ���d���I�[�r�X							5
	
	TGT_TRAP_ZONE,							// �g���b�v�]�[��							6
	TGT_CHKPNT_ZONE,						// �`�F�b�N�|�C���g�]�[��					7
	
	TGT_NSYS,								// N�V�X�e��								8
	TGT_TRFCHK,								// ��ʊĎ��V�X�e��							9

	TGT_MYAREA,								// �}�C�G���A								10
	TGT_MYCANCEL,							// �}�C�L�����Z���G���A						11
	TGT_ICANCEL,							// I�L�����Z��(�����o�^)					12
	
	TGT_POLICE,								// �x�@��									13
	TGT_CROSSING,							// �����_�Ď��V�X�e��						14
	TGT_ACCIDENT,							// ���̑����G���A							15
	TGT_SIGNAL,								// �M�������}�~�V�X�e��						16
	TGT_MICHINOEKI,							// ���̉w									17
	TGT_HWOASYS,							// �n�C�E�F�C�I�A�V�X						18
	TGT_SA,									// �T�[�r�X�G���A							19
	TGT_PA,									// �p�[�L���O�G���A							20
	TGT_HWRADIO,							// �n�C�E�F�C���W�I							21
#ifdef	__TGT_HW__
	TGT_HWCHGLMTSPD_40KMH,					// �������������x�ؑ֒n�_ 40km/h
	TGT_HWCHGLMTSPD_50KMH,					// �������������x�ؑ֒n�_ 50km/h
	TGT_HWCHGLMTSPD_60KMH,					// �������������x�ؑ֒n�_ 60km/h
	TGT_HWCHGLMTSPD_70KMH,					// �������������x�ؑ֒n�_ 70km/h
	TGT_HWCHGLMTSPD_80KMH,					// �������������x�ؑ֒n�_ 80km/h
	TGT_HWCHGLMTSPD_90KMH,					// �������������x�ؑ֒n�_ 90km/h
	TGT_HWCHGLMTSPD_100KMH,					// �������������x�ؑ֒n�_ 100km/h

	TGT_HWORBIS_LMTSPD_40KMH,				// �������I�[�r�X�n�_�������x 40km/h
	TGT_HWORBIS_LMTSPD_50KMH,				// �������I�[�r�X�n�_�������x 50km/h
	TGT_HWORBIS_LMTSPD_60KMH,				// �������I�[�r�X�n�_�������x 60km/h
	TGT_HWORBIS_LMTSPD_70KMH,				// �������I�[�r�X�n�_�������x 70km/h
	TGT_HWORBIS_LMTSPD_80KMH,				// �������I�[�r�X�n�_�������x 80km/h
	TGT_HWORBIS_LMTSPD_90KMH,				// �������I�[�r�X�n�_�������x 90km/h
	TGT_HWORBIS_LMTSPD_100KMH,				// �������I�[�r�X�n�_�������x 100km/h
	TGT_PARKCHK_AREA,
#endif
	TGT_SCP_ICI,							// �������������x�ؑ֒n�_ �{������			22
	TGT_SCP_ICO,							// �������������x�ؑ֒n�_ �{���o��			23
	TGT_SCP_JC,								// �������������x�ؑ֒n�_ �W�����N�V����	24
	TGT_SCP_SPD,							// �������������x�ؑ֒n�_ �{��				25
	TGT_SCP_PO,								// �������������x�ؑ֒n�_ �p�[�L���O�o��	26

	TGT_PARKCHK_AREA_SZ,					// ���ԋ֎~�����ŏd�_�G���A				27
	TGT_PARKCHK_AREA_Z,						// ���ԋ֎~�����d�_�G���A					28
	TGT_PARKING,							// ���ԏ�									29

	TGT_FCANCEL,							// �Œ�L�����Z���G���A						30
	TGT_CURVE,								// �}�J�[�u									31
	TGT_BRAJCT,								// ����E�����|�C���g						32
	TGT_KENKYO,								// ����										33
	TGT_TUNNEL,								// �����g���l��								34
	TGT_ETC,								// ETC���[��								35
	TGT_STEEP_SLOPE,						// �}���z									36
	TGT_RAILROAD_CROSSING,					// ����										37

	TGT_PIN,								// �s��										38
	TGT_LNKUNT,								// �����N���j�b�g							39
	TGT_RT_TRAP_ZONE,						// ���A���^�C������G���A					40
	TGT_TORUPA,								// �Ƃ��									41
	TGT_TMPSTOP,							// �ꎞ��~									42
	TGT_SHAJYOU_AREA,						// �ԏ�_�������G���A						43

	TGT_NOFIX_YUDO,							// �񑪈ʗU���f�[�^							44
	TGT_NOFIX_RD_ORBIS,						// �񑪈�RD���I�[�r�X						45
	TGT_NOFIX_HSYS_ORBIS,					// �񑪈�H�V�X�e�����I�[�r�X				46
	TGT_NOFIX_LHSYS_ORBIS,					// �񑪈�LH�V�X�e�����I�[�r�X				47
	TGT_NOFIX_LOOP_ORBIS,					// �񑪈ʃ��[�v�R�C�����I�[�r�X				48
	TGT_NOFIX_KODEN_ORBIS,					// �񑪈ʌ��d�ǎ��I�[�r�X					49

	TGT_NOFIX_TRAP_ZONE,					// �񑪈ʎ���G���A							50
	TGT_NOFIX_CHKPNT_ZONE,					// �񑪈ʌ���G���A							51

	TGT_TOILET,								// ���O�g�C��								52
	TGT_NOFIX_SENSOR_YUDO,					// �񑪈ʃZ���T�U���f�[�^					53
	TGT_KOBAN,								// ���										54

	TGT_FIRE,								// ���h��									55
	TGT_HOIKU,								// �ۈ牀�E�c�t��							56

	TGT_HWBUS_STOP,							// �����o�X��								57
	TGT_ZONE30,								// �]�[��30�G���A							58

	TGT_GUARD_MAX,							// �K�[�h�l

}EM_TGTCODE;

// ���ʏ��
#define	DEG_INVALID		0xFF				// ���ʖ���

// ���H���ʏ��
typedef enum{
	ROADJDG_STS_NORM = 0,					// ��ʓ����s��
	ROADJDG_STS_CHKHIGH,					// ���������s���蒆
	ROADJDG_STS_HIGH,						// ���������s��
	ROADJDG_STS_CHKNORM,					// ��ʓ����s���蒆
	ROADJDG_STS_NORMHIGH,					// �����H���s��
}EM_ROADJDG_STS;

// ���ԋ֎~�G���A�����
typedef enum{
	PCHK_AREA_OUT = 0,						// ���ԋ֎~�G���A�O
	PCHK_AREA_SZ,							// �ŏd�_�Ď��G���A��
	PCHK_AREA_Z,							// �d�_�Ď��G���A��
}EM_PCHK_AREA_STS;

// �ԏ�_�������G���A�����
typedef enum{
	SHAJYO_AREA_OUT = 0,					// �ԏ�_�������G���A�O
	SHAJYO_AREA_IN,							// �ԏ�_�������G���A��
}EM_SHAJYO_AREA_STS;

// �]�[��30�G���A�����
typedef enum{
	ZONE30_AREA_OUT = 0,					// �]�[��30�G���A�O
	ZONE30_AREA_IN,							// �]�[��30�G���A��
}EM_ZONE30_AREA_STS;

// ��Ԋ�{���
// ���H����
typedef enum{
	ROAD_NORM = 0,							// ��ʓ�
	ROAD_HIGH,								// ������
	ROAD_NORM_TOLL,							// ��ʗL����
}EM_ATTRIB_ROAD;

// �g���l������
typedef enum{
	NOT_TUNNEL = 0,							// ��g���l��
	TUNNEL_OUT,								// �g���l���o��
	TUNNEL_IN,								// �g���l����
}EM_ATTRIB_TUNNEL;

// �f�[�^�G���A����
typedef enum{
	MAKER_DATA_AREA = 0,					// ���[�J�[�f�[�^�G���A
	CUSTOM_DATA_AREA,						// �J�X�^���f�[�^�G���A
}EM_ATTRIB_DATA_AREA;

// ��Ԋg�����
// �J�����ʒu
typedef enum{
	CAMERA_NONE = 0,						// �J�����Ȃ�
	CAMERA_UP,								// �J�����ʒu��
	CAMERA_LEFT,							// �J�����ʒu��
	CAMERA_RIGHT,							// �J�����ʒu�E
}EM_EXTRA_CAMERA;

// �����E��ʓ��H���ʉ�
typedef enum{
	ROADJDG_INVALID = 0,					// �����E��ʂ̋�ʂ��ł��Ȃ�
	ROADJDG_HIGHWAY,						// �������Ƃ��Ĕ��ʉ�
	ROADJDG_NORMWAY,						// ��ʓ��Ƃ��Ĕ��ʉ�
}EM_EXTRA_ROADJDG;

// ���ցE�ԏ�_���G���A���a
typedef enum{
	AREA_ROUND_500M = 0,					// ���a500m
	AREA_ROUND_1000M,						// ���a1000m
	AREA_ROUND_1500M,						// ���a1500m
	AREA_ROUND_2000M,						// ���a2000m
	AREA_ROUND_2500M,						// ���a2500m
	AREA_ROUND_50M,							// ���a50m
	AREA_ROUND_100M,						// ���a100m
	AREA_ROUND_300M,						// ���a300m
}EM_EXTRA_AREA_ROUND;

// �������x
typedef enum{
	LMTSPD_NONE = 0,						// �������x���Ȃ�
	LMTSPD_10KMH,							// �������x10km/h
	LMTSPD_20KMH,							// �������x20km/h
	LMTSPD_30KMH,							// �������x30km/h
	LMTSPD_40KMH,							// �������x40km/h
	LMTSPD_50KMH,							// �������x50km/h
	LMTSPD_60KMH,							// �������x60km/h
	LMTSPD_70KMH,							// �������x70km/h
	LMTSPD_80KMH,							// �������x80km/h
	LMTSPD_90KMH,							// �������x90km/h
	LMTSPD_100KMH,							// �������x100km/h
	LMTSPD_110KMH,							// �������x110km/h
	LMTSPD_120KMH,							// �������x120km/h
}EM_EXTRA_LMTSPD;

typedef enum{
	TRAPCHK_LEVEL_INVALID = 0,				// ���x������
	TRAPCHK_LEVEL1,							// ���x��1
	TRAPCHK_LEVEL2,							// ���x��2
	TRAPCHK_LEVEL3,							// ���x��3
	TRAPCHK_LEVEL4,							// ���x��4
	TRAPCHK_LEVEL5,							// ���x��5
}EM_EXTRA_TRAPCHK_LEVEL;

// ����E�����@
typedef enum{
	METHOD_UNKNOWN = 0,						// ����s�\
	METHOD_MOVABLE_ORBIS_RADAR,				// �ړ��I�[�r�X(���[�_�[)
	METHOD_MOVABLE_ORBIS_STEALTH,			// �ړ��I�[�r�X(�X�e���X)
	METHOD_MOVABLE_ORBIS_KODEN,				// �ړ��I�[�r�X(���d��)
	METHOD_MOUSE_TRAP_RADAR,				// �l�Y�~�߂�(���[�_�[)
	METHOD_MOUSE_TRAP_STEALTH,				// �l�Y�~�߂�(�X�e���X)
	METHOD_MOUSE_TRAP_KODEN,				// �l�Y�~�߂�(���d��)
	METHOD_MOTORCYCLE_CHASE,				// �ǔ�(���o�C)
	METHOD_MASKED_POLICECAR_CHASE,			// �ǔ�(���ʃp�g�J�[)
	METHOD_POLICECAR_CHASE,					// �ǔ�(�p�g�J�[)
	METHOD_TEMPORARY_STOP,					// �ꎞ��~
	METHOD_SIGNAL,							// �M������
	METHOD_SEATBELT,						// �V�[�g�x���g
	METHOD_PHONE_DRIVING,					// �g�ѓd�b
	METHOD_DRUNKEN_DRIVING,					// �����^�]
}EM_EXTRA_TRAPCHK_METHOD;

// ���ԁE�ԏ�_���֎~�G���A�̌`
typedef enum{
	AREA_SHAPE_CIRCLE = 0,					// �~�`
	AREA_SHAPE_OVAL250M,					// 250m���~
	AREA_SHAPE_OVAL500M,					// 500m���~
	AREA_SHAPE_OVAL750M,					// 750m���~
	AREA_SHAPE_OVAL1000M,					// 1000m���~
	AREA_SHAPE_OVAL1500M,					// 1500m���~
	AREA_SHAPE_OVAL2000M,					// 2000m���~
	AREA_SHAPE_OVAL2500M,					// 2500m���~
}EM_EXTRA_AREA_SHAPE;

// �x�@���
typedef enum{
	POLICE_TYPE_STATION = 0,				// �x�@��
	POLICE_TYPE_HIGHWAY,					// �����x�@��
}EM_EXTRA_POLICE_TYPE;

// �X�}�[�gIC���
typedef enum{
	SMART_IC_NOINFO = 0,					// �X�}�[�gIC���Ȃ�
	SMART_IC_NOEXIST,						// �X�}�[�gIC�Ȃ�
	SMART_IC_EXIST,							// �X�}�[�gIC����
}EM_EXTRA_SMART_IC;

// �K�\�����X�^���h���
typedef enum{
	GS_NONE = 0,							// �K�\�����X�^���h�Ȃ�
	GS_JOMO,								// JOMO
	GS_USAMI,								// �F����
	GS_ESSO,								// ESSO
	GS_CARENEX,								// �J�[�G�l�N�X
	GS_KYGNUS,								// �L�O�i�X
	GS_COSMO,								// �R�X��
	GS_KYUSHU,								// ��B�Ζ�
	GS_IDEMITSU,							// �o��
	GS_SHELL,								// ���a�V�F��
	GS_ENEOS,								// ENEOS
	GS_TAIYO,								// ���z�Ζ�
	GS_GENERAL,								// �[�l����
	GS_MOBIL,								// ���[�r��
	GS_KOUNAN,								// �R�[�i��
	GS_TAKARA,								// �^�J���t���[�g
	GS_SORATO,								// �\���g
	GS_MAX,
}EM_EXTRA_GAS_STATION;

// �J�[�u���
typedef enum{
	LEFT_CURVE = 0,							// ���J�[�u
	RIGHT_CURVE,							// �E�J�[�u
	LEFT_TO_RIGHT_CURVE,					// �����E�J�[�u
	RIGHT_TO_LEFT_CURVE,					// �E�����J�[�u
	BOTH_LEFT_AND_RIGHT_CURVE,				// ���E����J�[�u
}EM_EXTRA_CURVE_TYPE;

// �������H���
typedef enum{
	NORMAL_HIGHWAY = 0,						// �ʏ퍂��
	URBAN_HIGHWAY,							// �s�s����
}EM_EXTRA_HIGHWAY_TYPE;

// ����E�������
typedef enum{
	BRAJCT_BRA_LEFT = 0,					// ������
	BRAJCT_BRA_RIGHT,						// �E����
	BRAJCT_BRA_LEFT_RIGHT,					// ���E����
	BRAJCT_JCT_LEFT,						// ������
	BRAJCT_JCT_RIGHT,						// �E����
	BRAJCT_JCT_LEFT_RIGHT,					// ���E����
}EM_EXTRA_BRAJCT_TYPE;

// �g���l�����
typedef enum{
	TUNNEL_TYPE_LONG = 0,					// �����g���l��
	TUNNEL_TYPE_CONT,						// �A���g���l��
}EM_TUNNEL_TYPE;

// ETC�K�C�h
typedef enum{
	ETC_GUIDE_LANE_LEFT = 0,				// ETC���[������
	ETC_GUIDE_LANE_CENTER,					// ETC���[������
	ETC_GUIDE_LANE_RIGHT,					// ETC���[���E��
	ETC_GUIDE_LANE_BOTH_SIDE,				// ETC���[�����[
	ETC_GUIDE_LANE_NONE,					// ETC���[���Ȃ�
}EM_ETC_GUIDE;

// ETC�|�C���g���
typedef enum{
	ETC_POINT_IN_START = 0,					// ���������ē��J�n�|�C���g
	ETC_POINT_OUT_START,					// �����o���ē��J�n�|�C���g
	ETC_POINT_STOP,							// �ē��I���|�C���g
}EM_ETC_POINT_TYPE;

// �ŗD��^�[�Q�b�g�t�H�[�J�X��`
typedef enum{
	PRI_FOCUS_TGT_NONE = 0,							// �ŗD��^�[�Q�b�g�t�H�[�J�X�Ȃ�
	PRI_FOCUS_TGT_ICON,								// �ŗD��^�[�Q�b�g�A�C�R���t�H�[�J�X
	PRI_FOCUS_TGT_POLIGON,							// �ŗD��^�[�Q�b�g�|���S���t�H�[�J�X
	PRI_FOCUS_TGT_POLIGON_SOUNDING,					// �ŗD��^�[�Q�b�g�|���S���t�H�[�J�X(�����o�͒�)
}EM_PRI_FOCUS_TGT;

// �U���^�C�v
typedef enum{
	YUDO_TYPE_COMMON = 0,					// ���ʗU��
	YUDO_TYPE_VPS_ONLY,						// VPS��p�U��
}EM_EXTRA_YUDO_TYPE;

// �U���ԍ�����
#define	u1_YUDO_NUM_INVALID	0

typedef enum{
	DIMMER_PHASE_NIGHT_OR_DAYLIGHT,
	DIMMER_PHASE_NIGHT_TO_MORNING,
	DIMMER_PHASE_MORNING_TO_DAYLIGHT,
	DIMMER_PHASE_DAYLIGHT_TO_EVENING,
	DIMMER_PHASE_EVENING_TO_NIGHT,
}EM_DIMMER_PHASE;

//--------------------------------------------------------------------------//
//	�^��`																	//
//--------------------------------------------------------------------------//

#define	GPSMAP_MAX			500				// �ő�^�[�Q�b�g�f�[�^��
#define GPS_VISIBLE_TGT_MAX	150				// ���^�[�Q�b�g�ő吔
#define	EXTRA_DATA_SIZE		8				// �g�����f�[�^��

// ��ԋ��p��
typedef union{
	struct{
		U2						:3;			// �\��
		U2		b_road			:2;			// ���H����(EM_ATTRIB_ROAD)
		U2		b_tunnel		:2;			// �g���l������(EM_ATTRIB_TUNNEL)
		U2		b_dataArea		:1;			// �f�[�^�G���A����(EM_ATTRIB_DATA_AREA)
		U2		b_code			:8;			// �^�[�Q�b�g(EM_TGTCODE)
	}bit;
	U2	word;
}UN_TYPE;

#pragma pack(1)
// ��ԕʊg���^
// �I�[�r�X
typedef struct{
	// extra1
	U1	b_camera		:3;					// �J����(EM_EXTRA_CAMERA)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_photoNum;						// �ʐ^�ԍ�
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_voiceNum;						// �{�C�X�ԍ�
}ST_EXTRA_ORBIS;
// ����E����G���A
typedef union{
	U2	hword;
	struct{
		U2		day		:5;
		U2		month	:4;
		U2		year	:7;
	}mbr;
}UN_REGDAY;

typedef struct{
	// extra1
	U1	b_level			:3;					// ���x��(EM_EXTRA_TRAPCHK_LEVEL)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_method;							// ���@(EM_EXTRA_TRAPCHK_METHOD)
	// extra4-5
	UN_REGDAY	un_regday;					// �o�^��
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_TRAPCHK;

// �x�@���֘A
typedef struct{
	// extra1
	U1	b_policeType	:3;					// �x�@�敪(EM_EXTRA_POLICE_TYPE)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_POLICE;

// SA�EPA�EOA�֘A
typedef struct{
	// extra1
	U1	b_smartIC		:3;					// �X�}�[�gIC���(EM_EXTRA_SMART_IC)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_gasStation;						// �K�\�����X�^���h���(EM_EXTRA_GAS_STATION)
	// extra4-5
	U2	u2_idNumber;						// SA�EPA�EOA�ԍ�
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_SAPAOA;

// �������x�ؑ�
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_SCP;

// ���փG���A
typedef struct{
	// extra1
	U1	b_areaRound		:3;					// �G���A���a(EM_EXTRA_AREA_ROUND)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_shape			:3;					// �G���A�̌`(EM_EXTRA_AREA_SHAPE)
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_NOPARKING;

// ���ԏ�
typedef struct{
	// extra1
	U1	b_charge		:3;					// �������
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_openTime;						// �c�ƊJ�n����(�P��:0.25h)
	// extra4
	U1	u1_closeTime;						// �c�ƏI������(�P��:0.25h)
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_PARKING;

// �}�J�[�u
typedef struct{
	// extra1
	U1	b_curveType		:3;					// �J�[�u���(EM_EXTRA_CURVE_TYPE)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_highwayType	:3;					// �������(EM_EXTRA_HIGHWAY_TYPE)
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_CURVE;

// ����E�����|�C���g
typedef struct{
	// extra1
	U1	b_braJctType	:3;					// ����E�������(EM_EXTRA_BRAJCT_TYPE)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_BRAJCT;

// ����
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_kenkyoNum;						// �s���{���ԍ�(0-46)
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_KENKYO;

// �g���l��
typedef struct{
	// extra1
	U1	b_tunnelType	:3;					// �g���l�����(EM_TUNNEL_TYPE)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_length;							// �g���l���S��(�P�ʁFm)
	// extra5-6
	U2	u2_nameNumber;						// �g���l�����̔ԍ�
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_TUNNEL;

// ETC���[��
typedef struct{
	// extra1
	U1	b_ETCGuide		:3;					// ETC�K�C�h(EM_ETC_GUIDE)
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_pointType		:3;					// ETC�|�C���g���(EM_ETC_POINT_TYPE)
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;

	// extra3-6
	U4	u4_laneDetail;						// ETC���[���ڍ�
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_ETC;

// ���̉w�E�r���[�|�C���g�p�[�L���O
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4-5
	U2	u2_idNumber;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_MICHI_TORUPA;

// �ԏ�_�������G���A
typedef struct{
	// extra1
	U1	b_areaRound		:3;					// �G���A���a
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4-5
	U2	u2_idNumber;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_SHAJYOU;

// ���I
typedef struct{
	// extra1
	U1	u1_mark;							// �L���E�����}�[�N
	// extra2-3
	UN_REGDAY	un_regday;					// ����o�^�N����
	// extra4
	U1	u1_sum;								// �`�F�b�N�T��
	// extra5-6
	U1	u1_oldFlags[2];						// ���t���O
	// extra7-8
	U1	u1_newFlags[2];						// �V�t���O
}ST_EXTRA_DYNAMIC;

// ���̑����ꍀ�ڂȂ��̔ėp
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// �g���l���c����
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�

}ST_EXTRA_COMMON;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_yudo_type;						// �U���^�C�v(EM_EXTRA_YUDO_TYPE)
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7
	U1					:8;
	// extra8
	U1					:8;
}ST_EXTRA_YUDO;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_syudo_number;					// �U���ԍ�
	// extra4
	U1	u1_syudo_rad;						// �U�����a(LSB 100m)
	// extra5
	U1	u1_syudo_deg;						// �U���p�x(LSB 1��)
	// extra6
	U1					:8;
	// extra7
	U1					:8;
	// extra8
	U1					:8;
}ST_EXTRA_SYUDO;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// ���H����(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// �������x(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_radius;							// �U���^�C�v(EM_EXTRA_YUDO_TYPE)
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// �n�}�E�摜�ԍ�
}ST_EXTRA_ZONE30;


// EXTRA�A�N�Z�X���p��
typedef union{
	ST_EXTRA_ORBIS			orbis;			// �I�[�r�X�n��ԗp
	ST_EXTRA_TRAPCHK		trapchk;		// ����E�����ԗp
	ST_EXTRA_POLICE			police;			// �x�@����ԗp
	ST_EXTRA_MICHI_TORUPA	michiTorupa;	// ���̉w�E�Ƃ�ώ�ԗp
	ST_EXTRA_SAPAOA			sapaoa;			// �T�[�r�X�G���A�E�p�[�L���O�G���A�E�n�C�E�F�C�I�A�V�X��ԗp
	ST_EXTRA_SCP			scp;			// �������x�ؑ֎�ԗp
	ST_EXTRA_NOPARKING		no_parking;		// ���փG���A��ԗp
	ST_EXTRA_PARKING		parking;		// ���ԏ��ԗp
	ST_EXTRA_CURVE			curve;			// �}�J�[�u��ԗp
	ST_EXTRA_BRAJCT			brajct;			// ����E������ԗp
	ST_EXTRA_KENKYO			kenkyo;			// ������ԗp
	ST_EXTRA_TUNNEL			tunnel;			// �g���l����ԗp
	ST_EXTRA_ETC			etc;			// ETC��ԗp
	ST_EXTRA_SHAJYOU		shajyou;		// �ԏ�_���G���A��ԗp
	ST_EXTRA_DYNAMIC		dynamic;		// ���I�p
	ST_EXTRA_COMMON			common;			// ���̑�����
	ST_EXTRA_YUDO			yudo;			// �U��
	ST_EXTRA_SYUDO			syudo;			// �Z���T�U��
	ST_EXTRA_ZONE30			zone30;			// �]�[��30�p
	U1						byte[EXTRA_DATA_SIZE];
											// byte�A�N�Z�X
}UN_EXTRA;

// �^�[�Q�b�g�}�b�v�^
typedef struct{
	U4			u4_tgtLat;					// �^�[�Q�b�g�ܓx(�P�� 10^-3��)
	U4			u4_tgtLon;					// �^�[�Q�b�g�o�x(�P�� 10^-3��)
	U4			u4_dataAddr;				// �f�[�^���A�h���X
	U2			u2_dst;						// ����-�^�[�Q�b�g�ԋ���(�P�� m)
	U2			u2_tgtDeg;					// �^�[�Q�b�g����(�P�� 0.1��)
	U2			u2_degA;					// �^�[�Q�b�g���猩�����ԕ���(�P�� 0.1��)
	S2			s2_degB;					// ���Ԃ̃^�[�Q�b�g�Ό��p(�P�� 0.1�� -180��< deg �� +180���F�E������)
	U2			u2_countdown_dist;
	U2			u2_dst_old;					// �O��̋���
	U1			hys_ahead;					// �O�����
	U1			ahead_hys_count;			// �O���q�X�J�E���g
	U1			u1_areaStsRd;				// RD�p�G���A���
	U1			u1_areaStsSc;				// �����p�G���A���
	U1			u1_areaScSnd;				// �G���A�������L��
	UN_TYPE		un_type;					// �^�[�Q�b�g���
	UN_EXTRA	un_extra;					// �g�����
	U1			u1_wrnSts;					// �x����(��0:��x�� ��0:�x�񒆂������͗����҂�)
}ST_GPSMAP;
#pragma pack()

// GPS�^�[�Q�b�g���^
typedef struct{
	ST_GPSMAP	*pst_gpsMap;				// �^�[�Q�b�g�}�b�v�|�C���^(ST_GPSMAP�^ �z��̐擪�|�C���^)
	U2			u2_tgtNum;					// �^�[�Q�b�g����
}ST_GPSMAP_INF;

// �w�b�_�\��
typedef struct{
	U4	u4_headCode;								// �w�b�_�R�[�h
	U4	u4_staAdr;									// �J�n�A�h���X
	U4	u4_endAdr;									// �I���A�h���X
	U4	u4_allChksum;								// �S�f�[�^�`�F�b�N�T��

	union{
		struct{
			U2	b_orbisDataArea		:1;				// �I�[�r�X�f�[�^�G���A
			U2	u2_orbisDataSpec	:15;			// �I�[�r�X�f�[�^�X�y�b�N
		}orbis;
		U2	u2_mapMainVer;							// �}�b�v���C���o�[�W����
	}mainVer;
	U2	u2_subVer;									// �T�u�o�[�W����

	U1	u1_hour;									// ��
	U1	u1_day;										// ��
	U1	u1_month;									// ��
	U1	u1_year;									// �N

	U1	u1_min;										// ��
	U1	u1_sec;										// �b
	U1	u1_reserved[2];								// �\��
	U4	u4_headerChksum;							// �w�b�_�`�F�b�N�T��
}ST_HEADER;

// INDEX1���\��
#pragma pack(2)
typedef struct{
	U2	u2_elenum;									// �u���b�N���v�f��
	U4	u4_adr;										// �A�h���X
}ST_INDEX1;
#pragma pack()

#pragma pack(1)
typedef struct{
	U1	u1_lat3		:5;
	U1	road		:2;
	U1	area		:1;
	U2	u2_lat12;

	U1	u1_lon3		:5;
	U1	tunnel		:2;
	U1	rsv			:1;
	U2	u2_lon12;

	U1	u1_type;
	U1	u1_deg;

	UN_EXTRA	un_extra;
}ST_ROMIMAGE;
#pragma pack()

// �q�����^
#pragma pack(1)
typedef struct{
	U1		id;										// 0�̎��ȉ��̏��͖����AGPS: 1(G1)-32(G32), GLONASS 65(R1)-96(R32), SBAS 120(S120)-151(S151), QZSS 193(Q1)-197(Q5)
	S1		ev;										// [deg]
	S2		az;										// [deg]
	U1		cn;										// [dbHz]
	U1		flg;									// bSvFlg
#define	bSvFlgUsed				bit0					// SV is used for navigation
#define	bSvFlgCorrct			bit1					// Differential correction data is available for this SV
#define	bSvFlgEpAl				bit2					// Orbit information is available for this SV (Ephemeris or Almanach)
#define	bSvFlgEp				bit3					// Orbit information is Ephemeris
#define	bSvFlgUnhelth			bit4					// SV is unhealthy / shall not be used
//								bit5
//								bit6
//								bit7
#define	kSvFlgInvalid			(0xFF)					// flg�𖳌��ɂ���Abit0��bit4�������ɗL���ɂȂ邱�Ƃ͂Ȃ����낤
}ST_SAT_INF;

// GPS��Ԍ^
typedef	struct{
	U4			u4_carLat;					// ���Ԉܓx (�P�ʁF10^-3��)
	U4			u4_carLon;					// ���Ԍo�x (�P�ʁF10^-3��)
	U2			u2_carDeg;					// ���ԕ��� (�P�ʁF0.1��)	
	S2			s2_carHight;				// ���ԍ��x (�P�ʁF1m)
	U2			u2_carSpd;					// ���Ԏԑ� (�P�ʁF0.1km/h)

	U1			b_curSokui		:1;			// ���݂̑��ʏ��(���W���[���f�[�^���̂܂�)
	U1			b_sysSokui		:1;			// �V�X�e�����ʏ��(�\���≹�Ɏg�p������)
	U1			b_degValid		:1;			// ���ʗL���t���O
	U1			b_hightValid	:1;			// ���x�L���t���O
	U1			b_timValid		:1;			// �����L���t���O
	U1			b_timDim		:2;			// �^�C���f�B�}�[���(EM_DIMMER)
	U1			b_spdValid		:1;			// ���x�m��t���O

	U1			b_gpsModSts		:3;			// GPS���W���[��������(EM_GPSMOD_STS)
	U1			b_rsv1			:4;			// ���F���ʎg�p�q����(0�`12)
	U1			b_sys1stFix		:1;			// �V�X�e��1stFix�t���O

	U1			u1_latarea;					// �ܓx�G���A
	U2			u2_lonarea;					// �o�x�G���A

	U1			u1_year;					// �N(2000�N����̃I�t�Z�b�g)
	U1			u1_month;					// ��(1�`12)
	U1			u1_day;						// ��(1�`31)
	U1			u1_hour;					// ��(0�`23)
	U1			u1_min;						// ��(0�`59)
	U1			u1_sec;						// �b(0�`59)

	U1 			b_carSpdValid   :1;     	// ���ԑ��x�L��(�\����"u2_carSpd" & "b_carSpdValid"�Ŕ��f����)
	U1			b_CarAveMaxSpdValid	:1;
	U1			b_forceGyroDegUse	:1;
	U1 			b_rsv           :5;     	// 

	EM_DIMMER_PHASE	dim_phase;
	U2			dim_sec_offs;
	
	U2			u2_carAveSpd;				// ���ϑ��x
	U2			u2_carMaxSpd;				// �ō����x

#define	SAT_INF_NUM				40			// �q�����
	U1			u1_viewSatNum;				// ������q����
	ST_SAT_INF	st_satInf[SAT_INF_NUM];		// �q�����

}ST_GPS_STS;
#pragma pack()
#define	GPSMAP_MAX			500				// �ő�^�[�Q�b�g�f�[�^��
#define GPS_VISIBLE_TGT_MAX	150				// ���^�[�Q�b�g�ő吔
#define	EXTRA_DATA_SIZE		8				// �g�����f�[�^��

#endif


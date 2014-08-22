//--------------------------------------------------------------------------
//�y���W���[���zGPS�R���g���[��												
//�y�@�\�z      GPS�ʒu������͂Ƃ��Ċe��GPS�x����o�͂���				
//�y���l�z																	
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//  �C���N���[�h�t�@�C��                                                    
//--------------------------------------------------------------------------

#include "stdafx.h"
#include "gpsctrl.h"
#include "gpsctrl.prm"
#include "crypt.h"
#include "math.h"
#include "setmgr.h"

//--------------------------------------------------------------------------
//  �}�N����`                                                              
//--------------------------------------------------------------------------

#define ZMATH_PI			3.14159265358979323846f
#define ZMATH_PI_2			1.57079632679489661923f // pi / 2

// �I�[�r�X�x���Ԓ�`
enum{
	STS_ORBIS_NOALARM = 0,							// ��x����
	STS_ORBIS_2100M,								// 2100m�x����
	STS_ORBIS_1100M,								// 1100m�x����
	STS_ORBIS_600M,									// 600m�x����
	STS_ORBIS_300M,									// 300m�x����
	STS_ORBIS_50M,									// 50m�x����
};

// �}�C�G���A
enum{
	STS_MYAREA_NOALARM = 0,							// ��x����
	STS_MYAREA_1100M,								// 1100m�x����
	STS_MYAREA_600M,								// 600m�x����
	STS_MYAREA_50M,									// 50m�x����
};
// �}�C�L�����Z��
enum{
	STS_MYCANCEL_RNGOUT = 0,						// ���O���
	STS_MYCANCEL_200M,								// 200m����
	STS_MYCANCEL_200M_RD,							// 200m����(RD��M����)
};
// �����L�����Z��
enum{
	STS_ATCANCEL_RNGOUT = 0,						// ���O���
	STS_ATCANCEL_REG1ST,							// �o�^1��ڒʉ�
	STS_ATCANCEL_200M_NORD,							// 200m����(RD��M�Ȃ�)
	STS_ATCANCEL_100M_NORD,							// 100m����(RD��M�Ȃ�)
	STS_ATCANCEL_RDRX,								// ����RD��M����
	STS_ATCANCEL_AACDIS,							// ����AAC�֎~
};
// �����L�����Z���o�^
typedef enum{
	REGSTS_IDLE = 0,								// �o�^�E�폜�҂����
	REGSTS_SRCHPOS,									// 30�`150m�ʒu�T�[�`���
	REGSTS_CHK300M,									// 300m�����`�F�b�N���
	REGSTS_CHK500M,									// 500m�����`�F�b�N���
}EM_ATCANCEL_REGSTS;

// SCP�R���e���c��Ԓ�`
enum{
	STS_SCP_RNGOUT = 0,								// ���O
	STS_SCP_100M,									// 100m�������
	STS_SCP_50M,									// 50m�������
};
// �����V���b�g�R���e���c��Ԓ�`
enum{
	STS_ONESHOTCONT_NOALARM = 0,
	STS_ONESHOTCONT_ALARM,
};

// 1km�R���e���c��Ԓ�`
enum{
	STS_1KMCONT_NOALARM = 0,						// ��x����
	STS_1KMCONT_1100M,								// 1100m�x����
	STS_1KMCONT_600M,								// 600m�x����
};
// �G���A�^�C�v�R���e���c��Ԓ�`
enum{
	STS_AREACONT_OUTAREA = 0,						// �G���A�O���
	STS_AREACONT_INAREA,							// �G���A�����
};

// ETC�Q�[�g��Ԓ�`
enum{
	STS_ETC_NOPASS = 0,								// ���ʉ�
	STS_ETC_PASS,									// �ʉ�
};

// ���֌x����
enum{
	PCHK_WRN_OUT = 0,								// ���O
	PCHK_WRN_INSZ_NOSNDOUT,							// �ŏd�_�G���A�������o��
	PCHK_WRN_INSZ_SNDOUT,							// �ŏd�_�G���A�����o�͍ς�
	PCHK_WRN_INZ_NOSNDOUT,							// �d�_�G���A�������o��
	PCHK_WRN_INZ_SNDOUT,							// �d�_�G���A�����o�͍ς�
};

// �]�[��30���
enum{
	ZONE30_WRN_OUT = 0,								// ���O
	ZONE30_WRN_IN_NOSNDOUT,							// �����������o��
	ZONE30_WRN_IN_SNDOUT,							// ���������o�͍ς�
};

// �ԏ�_�����
enum{
	SHAJYO_WRN_OUT = 0,								// ���O
	SHAJYO_WRN_IN_NOSNDOUT,							// �����������o��
	SHAJYO_WRN_IN_SNDOUT,							// ���������o�͍ς�
};

// �g���l�����I�[�r�X�x���Ԓ�`
enum{
	STS_TUNNEL_ORBIS_NOALARM = 0,
	STS_TUNNEL_ORBIS_2100M,
	STS_TUNNEL_ORBIS_1100M,
	STS_TUNNEL_ORBIS_600M,
};

// MAP�����t�[�Y���̃T�u�t�F�[�Y
enum{
	MAP_SPHASE_INBLK,								// �u���b�N���o�C�i���T�[�`
	MAP_SPHASE_GET_SMALL,							// �����T�[�`�_����̈ܓx���T�[�`
	MAP_SPHASE_GET_LARGE,							// �����T�[�`�_����̈ܓx��T�[�`
};

#define	u4_LON_SPLIT_BASE		((U4)0x00740000)	// ������o�x
#define	u4_LON_SPLIT_WIDTH		((U4)0x00000800)	// �o�x������
#define	u2_DATA_SPLIT_NUM		((U2)576)			// �f�[�^������
#define	u1_MAPDATA_SIZE			((U1)16)			// �I�[�r�XROM�f�[�^�T�C�Y

enum{
	TYPE_LAT = 0,
	TYPE_LON,
};

typedef enum{
	TYPE_TGTDEG_EXIST = 0,							// �p�x����i�ʏ�j
	TYPE_TGTDEG_NOEXIST,							// �p�x�Ȃ�
	TYPE_TGTDEG_EXIST_VERY_FAR,						// �p�x����(degB�ŉ���)
	TYPE_TGTDEG_EXIST_DEGA_WIDE,					// �p�x����(degA�L���͈�)
	TYPE_TGTDEG_NOEXIST_BACK,						// �p�x�Ȃ�����`�F�b�N
	TYPE_TGTDEG_DEGB_FRONT,							// �O���`�F�b�N(degB�L���͈�)
	TYPE_TGTDEG_EXIST_HOKAN,						// �p�x����(�ʒu�⊮)
}EM_TGTDEG;

// �T�[�`���ʒ�`
enum{
	u1_SRCH_OK = 0,									// �T�[�`����
	u1_SRCH_NG_SMALL,								// �T�[�`�����ł�����
	u1_SRCH_NG_LARGE,								// �T�[�`�����ł�����
	u1_SRCH_NG_NONE,								// �T�[�`�����ł����|�C���g�Ȃ�
	u1_SRCH_NG_DEV_BUSY,							// �f�o�C�X�r�W�[
	u1_SRCH_END,									// �T�[�`�I��
	u1_SRCH_MAX_ERR									// �T�[�`���ő�G���[
};

// �}�b�v�o�^���ʒ�`
enum{
	REGOK = 0,
	REGNG_LAT_SMALL,
	REGNG_LAT_LARGE,
	REGNG_LON_SMALL,
	REGNG_LON_LARGE,
	REGNG_MAP_MAX,
	REGNG_EEP_BUSY
};

enum{
	RUNSTOP_INIT = 0,
	RUNSTOP_RUN,
	RUNSTOP_STOP,
};

#define	VIRTUAL_INDEX_TYPES		7
enum{
	SCP_VIRTUAL_INDEX = 0,
	SZZ_VIRTUAL_INDEX,
	TZ_VIRTUAL_INDEX,
	KENKYO_VIRTUAL_INDEX,
	ETC_VIRTUAL_INDEX,
	CURVE_VIRTUAL_INDEX,
	ZN30_VIRTUAL_INDEX,
};

enum{
	NON_TARGET_ADDRESS = 0,
	SCP_VIRTUAL_ADDRESS,
	SZZ_VIRTUAL_ADDRESS,
	TZ_VIRTUAL_ADDRESS,
	KENKYO_VIRTUAL_ADDRESS,
	ETC_VIRTUAL_ADDRESS,
	CV_VIRTUAL_ADDRESS,
	ZN30_VIRTUAL_ADDRESS,
};

// �G���A���
enum{
	AREA_STATUS_OUT = 0,
	AREA_STATUS_IN_NOFIRE,
	AREA_STATUS_IN_FIRING,
	AREA_STATUS_IN_FIRED,
};

typedef enum{
	EM_TGT_DATA_MAKER = 0,
	EM_TGT_DATA_CUSTOM,
	EM_TGT_DATA_DYNAMIC,
#if __FREE_DATA_DOWNLOAD__
	EM_TGT_DATA_PAY,		//�L����
#endif
}EM_TGT_DATA_TYPE;

//--------------------------------------------------------------------------
//  �����^��`                                                              
//--------------------------------------------------------------------------
// GPS�^�[�Q�b�gROM�f�[�^
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

	U4	u4_addr;
}ST_GPSROM;

// �D��t�H�[�J�X�^�[�Q�b�g
typedef struct{
	U2			u2_dst;								// ����
	U2			u2_absDegB;
	U1			u1_priTunnelIn;						// ���D��g���l��������
	U1			u1_sts;								// �t�H�[�J�X���
	U2			u2_num;								// �^�[�Q�b�g�ԍ�
}ST_FOCUS_TGT;

//--------------------------------------------------------------------------
//  �O�����J�ϐ�                                                            
//--------------------------------------------------------------------------
ST_GPS_STS	stg_gpsSts;

//--------------------------------------------------------------------------
//  �ϐ���`
//--------------------------------------------------------------------------
static ST_GPSINF	sts_gpsinf;						// GPS���͏��
static U1			u1s_poiSetSel;					// POI�ݒ�Z���N�g
F1					f1g_gpsdat_flg;

static F4			f4s_gpsctl_flg;					// �t���O(�x��t�F�[�Y�J�n��������)
#define	F_RORBIS_CAN	(f4s_gpsctl_flg.bit.bit00)	// ���ΎԐ��I�[�r�X�L�����Z���t���O
#define	F_RORBIS_NOCAN	(f4s_gpsctl_flg.bit.bit01)	// ���ΎԐ��I�[�r�X�L�����Z���ے�t���O
#define	F_MYAREA_IN		(f4s_gpsctl_flg.bit.bit02)	// �}�C�G���A�����t���O
#define	F_MYCANCEL_IN	(f4s_gpsctl_flg.bit.bit03)	// �}�C�L�����Z�������t���O
#define	F_ATCANCEL_IN	(f4s_gpsctl_flg.bit.bit04)	// �I�[�g�L�����Z�������t���O
#define	F_CANCEL_NG		(f4s_gpsctl_flg.bit.bit05)	// �L�����Z��NG�t���O
#define	F_RDAREA_IN		(f4s_gpsctl_flg.bit.bit06)	// ���[�_�[�G���A���t���O
#define	F_TRAPZONE_IN	(f4s_gpsctl_flg.bit.bit07)	// �g���b�v�]�[�������t���O
#define	F_PCHK_SZ_IN	(f4s_gpsctl_flg.bit.bit08)	// ���֊Ď��ŏd�_�G���A�t���O
#define	F_PCHK_Z_IN		(f4s_gpsctl_flg.bit.bit09)	// ���֊Ď��d�_�G���A�t���O
#define	F_SCP_H_PASS	(f4s_gpsctl_flg.bit.bit10)	// ���H����H�ʉ߃t���O
#define	F_SCP_N_PASS	(f4s_gpsctl_flg.bit.bit11)	// ���H����N�ʉ߃t���O
#define	F_SCP_HCHK_PASS	(f4s_gpsctl_flg.bit.bit12)	// ���H����H����ʉ߃t���O
#define	F_SCP_NCHK_PASS	(f4s_gpsctl_flg.bit.bit13)	// ���H����N����ʉ߃t���O
#define	F_SCP_HCHK_NEAR_PASS	\
						(f4s_gpsctl_flg.bit.bit14)	// ���H����H����ߐڒʉ߃t���O
#define	F_SCP_ANYPASS	(f4s_gpsctl_flg.bit.bit15)	// SCP�ʉ߃t���O
#define	F_ATCANCEL_NEAR	(f4s_gpsctl_flg.bit.bit16)	// �I�[�g�L�����Z���ߐڃt���O
#define	F_ETC_START		(f4s_gpsctl_flg.bit.bit17)	// ETC�Q�[�g�ē�START�t���O
#define	F_ETC_STOP		(f4s_gpsctl_flg.bit.bit18)	// ETC�Q�[�g�ē�STOP�t���O
#define	F_SHAJYO_IN		(f4s_gpsctl_flg.bit.bit19)	// �ԏ�_���G���A�t���O
#define	F_RDSCOPE_IN	(f4s_gpsctl_flg.bit.bit20)	// ���[�_�[�X�R�[�v�t���O
#define	F_CHKPNT_IN		(f4s_gpsctl_flg.bit.bit21)	// �`�F�b�N�|�C���g�]�[�������t���O
#define F_ZONE30_IN		(f4s_gpsctl_flg.bit.bit22)	// �]�[��30�G���A�����t���O

static F1			f1s_gpsctl_flg2;				// �t���O(�t�F�[�Y�J�n���ۑ�)
#define	F_MAPUPD		(f1s_gpsctl_flg2.bit.b2)	// �}�b�v�X�V�ʒm

static F1			f1s_gpsctl_flg4;
#define	F_TRAP_INCOMING	(f1s_gpsctl_flg4.bit.b0)	// ��������ԋ�
#define	F_CHKPNT_INCOMING	(f1s_gpsctl_flg4.bit.b1)	// ���⌗���ԋ�
#define	F_TRAPOUT_REQ	(f1s_gpsctl_flg4.bit.b2)	// ������O�o�͗v��
#define	F_CHKOUT_REQ	(f1s_gpsctl_flg4.bit.b3)	// ���⌗�O�o�͗v��

static U2			u2s_sub_phase;					// GPS�����T�u�t�F�[�Y

static U2			u2s_mkmap_hiPri_num;			// ���D��}�b�v�쐬��
static U2			u2s_mkmap_loPri_num;			// ��D��}�b�v�쐬��

static U2			u2s_mkmap_num;					// �쐬�}�b�v�v�f��
static U2			u2s_chkmap_num;					// ����}�b�v�v�f��
static U2			u2s_oldmap_num;
static U1			u1s_parkChkAreaSts;				// ���֊Ď��G���A���
static U1			u1s_zone30AreaSts;				// �]�[��30�G���A���
static U1			u1s_roadJdgSts;					// ���H������
static U2			u2s_inisrch_pos;				// �����T�[�`�|�C���g
static U2			u2s_lonarea_old;				// �o�x�G���A�O��l
static U2			u2s_srch_pos;					// �T�[�`���Ă���|�C���g
static U2			u2s_high_pos;					// �T�[�`��[
static U2			u2s_low_pos;					// �T�[�`���[
static U2			u2s_srch_center_grp;			// �T�[�`�����O���[�v

// �G���A�֘A
static U1			u1s_runStopSts;

ST_GPSMAP			sts_gpsmap[GPSMAP_MAX];			// GPS�}�b�v
static ST_GPSMAP	*psts_chkmap;					// ����p�}�b�v�|�C���^
static ST_GPSMAP	sts_oldmap[GPSMAP_MAX];			// GPS�}�b�v����ւ��p�O��f�[�^
static U4			u4s_latDiffPrmMinus;
static U4			u4s_latDiffPrmPlus;
static U4			u4s_lonDiffPrmMinus;
static U4			u4s_lonDiffPrmPlus;
static ST_GPSROM	sts_gpsrom;						// GPS ROM�f�[�^���[�N

static ST_FOCUS_TGT	sts_focusTgt;					// �t�H�[�J�X�^�[�Q�b�g
static ST_FOCUS_TGT	sts_secondary_focusTgt;			// �Z�J���_���t�H�[�J�X�^�[�Q�b�g
static ST_FOCUS_TGT	sts_warning_focusTgt_Primary;	// �v���C�}���x��t�H�[�J�X�^�[�Q�b�g
static ST_FOCUS_TGT	sts_warning_focusTgt_Secondary;	// �Z�J���_���x��t�H�[�J�X�^�[�Q�b�g

static ST_INDEX1	sts_index1[3];					// ����INDEX1���
static ST_INDEX1 	*psts_index1;					// INDEX1�|�C���^

// GPS�x�񃌃x��
static U1			u1s_gpsWarningLvl_wrk;			// ���[�N
U1					u1g_gpsWarningLvl;				// ���J�f�[�^
static U2			u2s_gpsWarningLvl_dist_wrk;		// ���[�N
U2					u2g_gpsWarningLvl_dist;			// ���J�f�[�^

static UN_REGDAY	uns_today;						// ���t��r�p

static U2	s_SrcRefIdxList[GPS_VISIBLE_TGT_MAX];	// ���C���f�b�N�X���X�g
static U2	srcRefIdxListSize;						// ���C���f�b�N�X���X�g��
static float k_lon2_meter;
static Bool is_orbisCountDown;

ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS�ݒ�l
ST_SET_MISC				stg_setMisc;					// ���̑��ݒ�l
U1						u1g_setAcsSel;					// �ݒ�l�I��
static FILE	*gpsDataFile;
const char	*gpspoi_filename;
static U2	u2g_dataSpec;

//--------------------------------------------------------------------------
//  �����֐��v���g�^�C�v�錾												
//--------------------------------------------------------------------------
static void	PhaseChkMov(void);						// �ړ�����t�F�[�Y����
static void	PhaseMakeMap(void);						// MAP�����t�F�[�Y����
static void	PhaseChgMap(void);						// MAP�ύX����t�F�[�Y����
static void	PhaseChkWrn(void);						// �x�񔻒�t�F�[�Y����

static void	UpdLatLonArea(void);					// �ܓx�E�o�x�͈͍X�V����
static void	UpdIndex1(void);						// INDEX1�X�V����
static U1	u1_BlkSrch(EM_TGT_DATA_TYPE);			// �u���b�N����
static U1	u1_PntSrch(U1, EM_TGT_DATA_TYPE);		// �|�C���g�T�[�`
static U1	u1_ReadGpsData(U4, EM_TGT_DATA_TYPE);	// GPS�f�[�^�ǂݏo������
static U1	u1_RegMap(void);						// MAP�o�^����
static Bool Can_data_pre_expire(ST_GPSROM *);		// ���O�f�[�^���p���菈��
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type);	// �X�N�����u����������
static	U2	Shahen(U2, U2);							// �ΕӎZ�o����
static U2	u2_CalDstAxis(U4, U1);					// �����������Z�o����
static U2	u2_CalDegA(U2,U2, ST_GPSMAP *);			// ��A�Z�o����
static S2	s2_CalDegB(U2);							// ��B�Z�o����
static void TransitDummy(EM_TGTDEG);				// �_�~�[
static void	TransitOrbis(EM_TGTDEG);				// �I�[�r�X�x��J��
static void	TransitZone(EM_TGTDEG);					// �]�[���x��J��
static void TransitMyArea(EM_TGTDEG);				// �}�C�G���A����
static void	TransitMyCancel(EM_TGTDEG);				// �}�C�L�����Z������
static void	TransitAtCancel(EM_TGTDEG);				// �I�[�g�L�����Z������
static void	TransitContScp(EM_TGTDEG);				// ���x�ؑփR���e���c����
static void TransitPin(EM_TGTDEG);					// �s����ԑJ��

static void	TransitCont1kmType(EM_TGTDEG);			// 1km�^�C�v�R���e���c����
static void	TransitContOneShotType(EM_TGTDEG);		// �����V���b�g�^�C�v�R���e���c����
static void	TransitContAreaType(EM_TGTDEG);			// �G���A�^�C�v�R���e���c����
static void	TransitETCGateType(EM_TGTDEG);			// ETC�R���e���c����
static void TransitZone30Type(EM_TGTDEG emh_tgtDeg);// �]�[��30�^�C�v�R���e���c����

static void TransitTunnelOrbis(EM_TGTDEG);			// �g���l���I�[�r�X�x��J��
static void TransitYudo(EM_TGTDEG);					// �U���J�ڏ���
static void TransitSyudo(EM_TGTDEG);				// �Z���T�U���J�ڏ���
static void	TransitTunnelZone(EM_TGTDEG);			// �g���l���]�[���x��J�ڏ���

static void	TransitPchkSts(void);					// ���֊Ď���ԑJ��
static void TransitZone30Sts(void);					// �]�[��30��ԑJ��
static void	TransitShajyoSts(void);					// �ԏ�_����ԑJ��
static void	TransitETCGuide(void);					// ETC�K�C�h�Ǘ�

static U1	u1_ChkDegRng(EM_TGTDEG, U2);			// �p�x�͈͔��菈��

static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus);

S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2);
static U1	u1_DegAWideIsOk(ST_GPSMAP *psth_map);

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt);

static U1	IsJudgePOISet(U1);						// POI�ݒ蔻�菈��
static U1	u1_JudgePOISet(ST_GPSMAP *);			// 
static U1	u1_JudgePOISetTunnel(ST_GPSMAP *);		// 

static void update_ahead_hys(ST_GPSMAP *psth_map);
static void update_distance(ST_GPSMAP *psth_map);

//--------------------------------------------------------------------------
//  ����const data��`
//--------------------------------------------------------------------------
static const EM_TGTDEG	TBL_TGTTRANS_PRM[] = {

	TYPE_TGTDEG_NOEXIST,					// dummy					0
	TYPE_TGTDEG_EXIST,						// RD���I�[�r�X				1
	TYPE_TGTDEG_EXIST,						// H�V�X�e�����I�[�r�X		2
	TYPE_TGTDEG_EXIST,						// LH�V�X�e�����I�[�r�X		3
	TYPE_TGTDEG_EXIST,						// ���[�v�R�C�����I�[�r�X	4
	TYPE_TGTDEG_EXIST,						// ���d���I�[�r�X			5

	TYPE_TGTDEG_EXIST,						// ����G���A				6
	TYPE_TGTDEG_EXIST,						// ����G���A				7

	TYPE_TGTDEG_EXIST,						// N�V�X�e��				8
	TYPE_TGTDEG_EXIST,						// ��ʊĎ��V�X�e��			9

	TYPE_TGTDEG_NOEXIST,					// �}�C�G���A				10
	TYPE_TGTDEG_NOEXIST,					// �}�C�L�����Z���G���A		11
	TYPE_TGTDEG_NOEXIST,					// I�L�����Z���G���A		12

	TYPE_TGTDEG_NOEXIST,					// �x�@��					13
	TYPE_TGTDEG_NOEXIST,					// �����_�Ď��|�C���g		14
	TYPE_TGTDEG_NOEXIST,					// ���̑����G���A			15
	TYPE_TGTDEG_NOEXIST,					// �M�������}�~�V�X�e��		16
	
	TYPE_TGTDEG_NOEXIST,					// ���̉w					17
	TYPE_TGTDEG_NOEXIST,					// �n�C�E�F�C�I�A�V�X		18
	TYPE_TGTDEG_EXIST,						// �T�[�r�X�G���A			19
	TYPE_TGTDEG_EXIST,						// �p�[�L���O�G���A			20
	TYPE_TGTDEG_EXIST,						// �n�C�E�F�C���W�I			21

	TYPE_TGTDEG_EXIST,						// SCP�{������				22
	TYPE_TGTDEG_EXIST,						// SCP�{���o��				23
	TYPE_TGTDEG_EXIST,						// SCP�W�����N�V����		24
	TYPE_TGTDEG_EXIST,						// SCP�{��					25
	TYPE_TGTDEG_EXIST,						// SCP�p�[�L���O�o��		26

	TYPE_TGTDEG_NOEXIST,					// ���֍ŏd�_�G���A			27
	TYPE_TGTDEG_NOEXIST,					// ���֏d�_�G���A			28
	TYPE_TGTDEG_NOEXIST,					// ���ԏ�					29

	TYPE_TGTDEG_NOEXIST,					// �Œ�L�����Z���G���A		30
	TYPE_TGTDEG_EXIST,						// �}�J�[�u					31
	TYPE_TGTDEG_EXIST,						// ����E�����|�C���g		32
	TYPE_TGTDEG_EXIST,						// ����						33
	TYPE_TGTDEG_EXIST,						// �����g���l��				34
	TYPE_TGTDEG_EXIST,						// ETC���[��				35
	TYPE_TGTDEG_EXIST,						// �}���z					36
	TYPE_TGTDEG_NOEXIST,					// ����						37
	TYPE_TGTDEG_EXIST,						// ���[�U�[�s��				38
	TYPE_TGTDEG_NOEXIST,					// �����N���j�b�g			39
	TYPE_TGTDEG_EXIST,						// ���A���^�C������G���A	40
	TYPE_TGTDEG_NOEXIST,					// �Ƃ��					41
	TYPE_TGTDEG_EXIST,						// �ꎞ��~					42
	TYPE_TGTDEG_NOEXIST,					// �ԏ�_�������G���A		43

	TYPE_TGTDEG_EXIST,						// �񑪈ʗU���f�[�^				44
	TYPE_TGTDEG_EXIST,						// �񑪈�RD���I�[�r�X			45
	TYPE_TGTDEG_EXIST,						// �񑪈�H�V�X�e�����I�[�r�X	46
	TYPE_TGTDEG_EXIST,						// �񑪈�LH�V�X�e�����I�[�r�X	47
	TYPE_TGTDEG_EXIST,						// �񑪈ʃ��[�v�R�C�����I�[�r�X	48
	TYPE_TGTDEG_EXIST,						// �񑪈ʌ��d�ǎ��I�[�r�X		49

	TYPE_TGTDEG_EXIST,						// �񑪈ʎ���G���A			50
	TYPE_TGTDEG_EXIST,						// �񑪈ʌ���G���A			51

	TYPE_TGTDEG_NOEXIST,					// ���O�g�C��				52
	TYPE_TGTDEG_EXIST,						// �񑪈ʃZ���T�U���f�[�^	53
	TYPE_TGTDEG_NOEXIST,					// ���						54

	TYPE_TGTDEG_NOEXIST,					// ���h��					55
	TYPE_TGTDEG_NOEXIST,					// �ۈ牀�E�c�t��			56
	
	TYPE_TGTDEG_NOEXIST,					// �����o�X��				57
	TYPE_TGTDEG_NOEXIST,					// �]�[��30					58
};

static void (*const fnc_TBL_TGTTRANS[])(EM_TGTDEG) = {
	
	TransitDummy,							// dummy					0
/*	TransitOrbis,							// RD���I�[�r�X				1
	TransitOrbis,							// H�V�X�e�����I�[�r�X		2
	TransitOrbis,							// LH�V�X�e�����I�[�r�X		3
	TransitOrbis,							// ���[�v�R�C�����I�[�r�X	4
	TransitOrbis,							// ���d���I�[�r�X			5
	
	TransitZone,							// ����G���A				6
	TransitZone,							// ����G���A				7

	TransitContOneShotType,					// N�V�X�e��				8
	TransitContOneShotType,					// ��ʊĎ��V�X�e��			9

	TransitMyArea,							// �}�C�G���A				10
	TransitMyCancel,						// �}�C�L�����Z���G���A		11
	TransitAtCancel,						// I�L�����Z���G���A		12

	TransitContOneShotType,					// �x�@��					13
	TransitContOneShotType,					// �����_�Ď��|�C���g		14
	TransitContOneShotType,					// ���̑����G���A			15
	TransitContOneShotType,					// �M�������}�~�V�X�e��		16

	TransitCont1kmType,						// ���̉w					17
	TransitContOneShotType,					// �n�C�E�F�C�I�A�V�X		18
	TransitContOneShotType,					// �T�[�r�X�G���A			19
	TransitContOneShotType,					// �p�[�L���O�G���A			20
	TransitContOneShotType,					// �n�C�E�F�C���W�I			21

	TransitContScp,							// SCP�{������				22
	TransitContScp,							// SCP�{���o��				23
	TransitContScp,							// SCP�W�����N�V����		24
	TransitContScp,							// SCP�{��					25
	TransitContScp,							// SCP�p�[�L���O�o��		26

	TransitContAreaType,					// ���֍ŏd�_�G���A			27
	TransitContAreaType,					// ���֏d�_�G���A			28
	TransitContOneShotType,					// ���ԏ�					29

	TransitDummy,							// �Œ�L�����Z���G���A		30
	TransitContOneShotType,					// �}�J�[�u					31
	TransitContOneShotType,					// ����E�����|�C���g		32
	TransitContOneShotType,					// ����						33
	TransitCont1kmType,						// �����g���l��				34
	TransitETCGateType,						// ETC���[��				35
	TransitDummy,							// �}���z					36
	TransitContOneShotType,					// ����						37
	TransitPin,								// ���[�U�[�s��				38
	TransitDummy,							// �����N���j�b�g			39
	TransitZone,							// ���A���^�C������G���A	40
	TransitCont1kmType,						// �Ƃ��					41
	TransitContOneShotType,					// �ꎞ��~					42
	TransitContAreaType,					// �ԏ�_�������G���A		43

	TransitYudo,							// �񑪈ʗU���f�[�^				44
	TransitTunnelOrbis,						// �񑪈�RD���I�[�r�X			45
	TransitTunnelOrbis,						// �񑪈�H�V�X�e�����I�[�r�X	46
	TransitTunnelOrbis,						// �񑪈�LH�V�X�e�����I�[�r�X	47
	TransitTunnelOrbis,						// �񑪈ʃ��[�v�R�C�����I�[�r�X	48
	TransitTunnelOrbis,						// �񑪈ʌ��d�ǎ��I�[�r�X		49

	TransitTunnelZone,						// �񑪈ʎ���G���A			50
	TransitTunnelZone,						// �񑪈ʌ���G���A			51
	
	TransitContOneShotType,					// ���O�g�C��				52
	TransitSyudo,							// �񑪈ʃZ���T�U���f�[�^	53
	TransitContOneShotType,					// ���						54

	TransitContOneShotType,					// ���h��					55
	TransitContOneShotType,					// �ۈ牀�E�c�t��			56

	TransitDummy,							// �����o�X��				57
	TransitZone30Type,						// �]�[��30					58*/
};

//--------------------------------------------------------------------------
//  �O���֐���`															
//--------------------------------------------------------------------------
void PoiSample(const char* fname, U2 dataSpec){

	gpspoi_filename = fname;
	u2g_dataSpec = dataSpec;

	sts_gpsinf.lat = 2107021;
	sts_gpsinf.lon = 8220672;
	u2s_lonarea_old = U2_MAX;						// �O��o�x�G���A�͈͊O��

	PhaseChkMov();
	PhaseMakeMap();
	PhaseChgMap();
}

//--------------------------------------------------------------------------
//�y�֐��z�ړ�����t�F�[�Y													
//�y�@�\�z���Ԉړ��������v�����āA�ړ����Ă����INDEX1���X�V����			
//--------------------------------------------------------------------------
static void PhaseChkMov(void){

	ST_GPSMAP	*pstt_oldmap;
	ST_GPSMAP	*pstt_curmap;

	UpdLatLonArea();								// �ܓx�E�o�x�͈͂̌���

	UpdIndex1();								// INDEX1�X�V
	// �}�b�v�쐬�O�ɋ��f�[�^�̑ޔ�
	pstt_oldmap = &sts_oldmap[0];
	pstt_curmap = &sts_gpsmap[0];

	u2s_oldmap_num = u2s_chkmap_num;
	if(u2s_oldmap_num != 0){
		memcpy(pstt_oldmap, pstt_curmap, sizeof(ST_GPSMAP)*u2s_oldmap_num);
	}
	// �ܓx�E�o�x�̋��e�͈̓p�����[�^����
	u4s_latDiffPrmMinus = sts_gpsinf.lat - u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_latDiffPrmPlus = sts_gpsinf.lat + u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmMinus = sts_gpsinf.lon - u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmPlus = sts_gpsinf.lon + u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];

	u2s_mkmap_hiPri_num = VIRTUAL_INDEX_TYPES;	// ���D��}�b�v�쐬��������(���z�^�[�Q�b�g���͊m��)
	u2s_mkmap_loPri_num = 0;					// ��D��}�b�v�쐬��������
	u2s_mkmap_num = VIRTUAL_INDEX_TYPES;		// �쐬MAP�v�f���N���A(���z�^�[�Q�b�g���͊m��)
}
//--------------------------------------------------------------------------
//�y�֐��zMAP�����t�F�[�Y����												
//�y�@�\�z���Ԉʒu���ROM�f�[�^�����o���AMAP�𐶐�����					
//�y���l�z�u���b�N�ԍ����傫�����A�ܓx���傫��								
//--------------------------------------------------------------------------
static void PhaseMakeMap(void){

	U2	u2t_pos_bak;									// ���_�ۑ��l
	U1	u1t_groupEnd;
	U1	u1t_srchStep;
	U4	u4t_lon_center;
	ST_INDEX1	*pstt_index1;
	EM_TGT_DATA_TYPE type;

	pstt_index1 = &sts_index1[0];
	type = EM_TGT_DATA_MAKER;

	// ������
	u2s_srch_pos = 0;									// �T�[�`�ʒu������
	for(u1t_srchStep = 0; u1t_srchStep <=2 ; u1t_srchStep++){
		switch(u1t_srchStep){
		case 0:
			psts_index1 = &pstt_index1[1];				// ���߂͒�������
			break;
		case 1:
			// ���݈ʒu�������Ȃ獶�D��A�E���Ȃ�E�D��
			u4t_lon_center = u4_LON_SPLIT_BASE + (U4)u2s_srch_center_grp*u4_LON_SPLIT_WIDTH + (u4_LON_SPLIT_WIDTH / 2);
			if(sts_gpsinf.lon < u4t_lon_center){		// �����H
				psts_index1 = &pstt_index1[0];			// ���͍�
			}
			else{
				psts_index1 = &pstt_index1[2];			// ���͉E
			}
			break;
		case 2:
			if(psts_index1 == &pstt_index1[0]){			// �܂��ǂ�łȂ����ɂ���
				psts_index1 = &pstt_index1[2];
			}
			else{
				psts_index1 = &pstt_index1[0];
			}
			break;
		}
		// 1�̃O���[�v������
		// �o�C�i���T�[�`����
		// 1�̃O���[�v������
		if(psts_index1->u2_elenum == 0){
			continue;
		}
		u1t_groupEnd = OFF;
		u2s_low_pos = 0;								// ���_
		u2s_high_pos = psts_index1->u2_elenum;			// ��_
		u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1);
														// ���_�Z�o
		u2s_sub_phase = MAP_SPHASE_INBLK;				// �o�C�i���T�[�`�J�n

		for(;;){
			switch(u2s_sub_phase){
		//-----------------�u���b�N���_����̃T�[�`-------------
			case MAP_SPHASE_INBLK:
				u2t_pos_bak = u2s_srch_pos;
				switch(u1_BlkSrch(type)){				// �u���b�N�����_�T�[�`
				
				case u1_SRCH_OK:						// �����T�[�`�_����
					u2s_sub_phase = MAP_SPHASE_GET_SMALL;
														// ����ɒ��o�J�n
					break;
					
				case u1_SRCH_NG_LARGE:					// �ܓx�呤�ɔ͈͊O���ܓx������T��
					u2s_high_pos = u2s_srch_pos;		// ���_���̂܂� ��_�͍��̒��_
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// ���_�Z�o
					if(u2s_srch_pos == u2t_pos_bak){	// ���_�ɕύX�Ȃ�(�|�C���g�Ȃ�)�H
						u1t_groupEnd = ON;				// �O���[�v�I��
					}
					break;
					
				case u1_SRCH_NG_SMALL:					// �ܓx�����ɔ͈͊O���ܓx�呤��T��
					u2s_low_pos = u2s_srch_pos;			// ���_�͍��̒��_ ��_�͂��̂܂�
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// ���_�Z�o
					if(u2s_srch_pos == u2t_pos_bak){	// ���_�ɕύX�Ȃ�(�|�C���g�Ȃ�)�H
						u1t_groupEnd = ON;				// �O���[�v�I��
					}
					break;
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
//				case u1_SRCH_NG_NONE:					// �Ȃ�
				default:
					goto srch_end;
				}
				break;

		//----------�����T�[�`�_����̈ܓx�������T�[�`----------------------------
			case MAP_SPHASE_GET_SMALL:
				switch(u1_PntSrch(SMALL, type)){		// �ܓx������Xkm�T��
				case u1_SRCH_END:						// �ܓx�����I���Ȃ�
					u2s_sub_phase = MAP_SPHASE_GET_LARGE;
														// ���͈ܓx�呤��T��
					u2s_srch_pos = u2s_inisrch_pos;		// �����T�[�`�ʒu�����[�h
					break;
				case u1_SRCH_OK:						// �o�^OK
														// ���������T��
					break;
//				case u1_SRCH_MAX_ERR:					// �o�^�ő�NG
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
				default:
					goto srch_end;						// �t�F�[�Y�I��

				}
				break;
		//----------�����T�[�`�_����̈ܓx������T�[�`----------------------------
			case MAP_SPHASE_GET_LARGE:
				switch(u1_PntSrch(LARGE, type)){		// �ܓx�呤��Xkm�T��
				case u1_SRCH_END:
					u1t_groupEnd = ON;					// �O���[�v�I��
					break;

				case u1_SRCH_OK:						// �o�^OK -> ���������T��
					break;

//				case u1_SRCH_MAX_ERR:
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
				default:
					goto srch_end;						// �T�[�`�I��

				}
				break;
			}
			
			if(u1t_groupEnd){							// �O���[�v�I�� ?
				break;
			}
		}
	}
srch_end:
	return;
}

//--------------------------------------------------------------------------
//�y�֐��zMAP�|�C���g�ύX�`�F�b�N�t�F�[�Y����								
//�y�@�\�zMAP����ւ��ɂ��x���Ԃ̏������A�x��I�����u���s��				
//--------------------------------------------------------------------------
static void	PhaseChgMap(void){

	U2	i;											// �쐬MAP���[�v�J�E���^
	U2	j;											// ����MAP���[�v�J�E���^
	ST_GPSMAP	*pstt_oldmap;
	ST_GPSMAP	*pstt_newmap;

	// �}�b�v���l�߂�
	if((u2s_mkmap_loPri_num != 0)					// ��D��f�[�^���� and
			&& (u2s_mkmap_num < GPSMAP_MAX)){				// �}�b�v�t���ȊO�̂Ƃ�
		// ��D��f�[�^�����D��f�[�^�̂������Ɉړ�����
		memcpy(&sts_gpsmap[u2s_mkmap_hiPri_num], &sts_gpsmap[GPSMAP_MAX - u2s_mkmap_loPri_num], sizeof(ST_GPSMAP)*(int)u2s_mkmap_loPri_num);
	}

	if((u2s_oldmap_num != 0)						// ���}�b�v��0�łȂ��H
			&& (u2s_mkmap_num != 0)){						// �V�}�b�v��0�łȂ��H
		// ���}�b�v��Ŕ�r����
		pstt_oldmap = &sts_oldmap[0];				// ���苌MAP�|�C���^�ݒ�

		for(i=0;i<u2s_oldmap_num; i++, pstt_oldmap++){
			pstt_newmap = &sts_gpsmap[0];		// �VMAP�|�C���^�ݒ�
			// �쐬�}�b�v�ƃA�h���X��r����
			for(j=0;j<u2s_mkmap_num; j++, pstt_newmap++){
				if(pstt_newmap->u4_dataAddr == pstt_oldmap->u4_dataAddr){
					// ����f�[�^�A�h���X����
					*pstt_newmap = *pstt_oldmap;
					break;						// ���������̂Ŏ��̔���}�b�v
				}
			}
		}
	}
	u2s_chkmap_num = u2s_mkmap_num;
	F_MAPUPD = ON;
}

//--------------------------------------------------------------------------
//�y�֐��z�x�񔻒�t�F�[�Y����												
//�y�@�\�zGPS�x��𔻒肷��													
//--------------------------------------------------------------------------
static void PhaseChkWrn(void){

	U2	u2t_deltaX;
	U2	u2t_deltaY;

	// �t�F�[�Y������
	u2s_sub_phase = 0;								// �T�u�t�F�[�Y������
	psts_chkmap = &sts_gpsmap[VIRTUAL_INDEX_TYPES];	// ����}�b�v�|�C���^������
	f4s_gpsctl_flg.word = 0;
	f1s_gpsctl_flg4.byte = 0;
	u1s_gpsWarningLvl_wrk = SYS_NO_WARNING;			// �x��Ȃ��ŏ�����
	u2s_gpsWarningLvl_dist_wrk = 10000;				// 10km�����ŏ�����

	// �t�H�[�J�X�^�[�Q�b�g���[�N������
	initFocusTgt(&sts_focusTgt);
	initFocusTgt(&sts_secondary_focusTgt);
	initFocusTgt(&sts_warning_focusTgt_Primary);
	initFocusTgt(&sts_warning_focusTgt_Secondary);

	// ���C���f�b�N�X���X�g������
	srcRefIdxListSize = 0;

	// �I�[�r�X�J�E���g�_�E��������
	is_orbisCountDown = FALSE;

	if(u2s_chkmap_num > VIRTUAL_INDEX_TYPES){		// �}�b�v�|�C���g����Ȃ�
		for(u2s_sub_phase = VIRTUAL_INDEX_TYPES; u2s_sub_phase < u2s_chkmap_num ; u2s_sub_phase++){
			// ����<->�^�[�Q�b�g�ԋ����̍X�V
			// �����������̍������Z�o
			u2t_deltaX = u2_CalDstAxis(psts_chkmap->u4_tgtLat, TYPE_LAT);
			u2t_deltaY = u2_CalDstAxis(psts_chkmap->u4_tgtLon, TYPE_LON);

			// �����ŐV�l�Z�o
			psts_chkmap->u2_dst = Shahen(u2t_deltaX, u2t_deltaY);

			// �^�[�Q�b�g�Ƃ̎��Ԃ����񂾒����̊p�x�i�ڂ`�j���X�V
			psts_chkmap->u2_degA = u2_CalDegA(u2t_deltaX, u2t_deltaY, psts_chkmap);
			
			// �ڂ`�Ǝ��ԕ��ʂƂ̍����i�ڂa�j���X�V
			psts_chkmap->s2_degB = s2_CalDegB(psts_chkmap->u2_degA);

			// �������X�V
			update_distance(psts_chkmap);

			// �O�����o�q�X���X�V
			update_ahead_hys(psts_chkmap);

			// ��ޕʌx���ԑJ��
			if(IsJudgePOISet(u1s_poiSetSel) == OK){
				fnc_TBL_TGTTRANS[psts_chkmap->un_type.bit.b_code](TBL_TGTTRANS_PRM[psts_chkmap->un_type.bit.b_code]);
			}
			else{
				psts_chkmap->u1_wrnSts = 0;						// �x���ԏ�����
			}
			psts_chkmap++;										// MAP�|�C���^��i�߂�
		}
	}

	// ���H���菈��
	// RoadJudge();

	// ���փG���A�̑J�ڏ���
	TransitPchkSts();

	// �]�[��30�̑J�ڏ���
	TransitZone30Sts();
	
}

//-----------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------
//�y�֐��z�ܓx�E�o�x�͈͍X�V����											
//�y�@�\�z���݂̈ܓx��5���������G���A�̂ǂ̈ʒu�ɂ��邩�����肷��			
//		  ���݂̌o�x��576���������G���A�̂ǂ̈ʒu�ɂ��邩�����肷��			
//�y���l�z�G���A�ԍ���1�`574�܂�(�[��1�����ɓ����						
//--------------------------------------------------------------------------
static void	UpdLatLonArea(void){

	U1	i;											// ���[�v�J�E���^
	U4	u4t_lonwrk;

	// �ܓx�G���A�̌���
	for(i = 0; i< LATAREA_MAX; i++){
		if(((sts_gpsinf.lat & 0x00FFFF00) >> 8) < u2_TBL_LATAREA[i]){
													// ���2byte�T�C�Y�Ō���
													// �������l��菬�Ȃ�
			break;									// �G���A����
		}
	}
	sts_gpsinf.u1_latarea = i;

	u4t_lonwrk = sts_gpsinf.lon;					// �o�x
	if(u4t_lonwrk >= 0x860000){
		u4t_lonwrk = 0x860000 - 1;
	}
	else if(u4t_lonwrk <= u4_LON_SPLIT_BASE){
		u4t_lonwrk = u4_LON_SPLIT_BASE;
	}

	u4t_lonwrk -= u4_LON_SPLIT_BASE;
	sts_gpsinf.u2_lonarea =  (U2)(u4t_lonwrk / u4_LON_SPLIT_WIDTH);
	if(sts_gpsinf.u2_lonarea == 0){
		sts_gpsinf.u2_lonarea = 1;
	}
	else if(sts_gpsinf.u2_lonarea >= (u2_DATA_SPLIT_NUM - 1)){
		sts_gpsinf.u2_lonarea = u2_DATA_SPLIT_NUM - 2;
	}

    stg_gpsSts.u1_latarea = sts_gpsinf.u1_latarea;
    stg_gpsSts.u2_lonarea = sts_gpsinf.u2_lonarea;
}

//--------------------------------------------------------------------------
//�y�֐��zINDEX1�X�V����													
//�y�@�\�z�G���A���Z�o���A�G���A�ύX������V����INDEX1�ɍX�V����			
//�y���l�z																	
//--------------------------------------------------------------------------
static void UpdIndex1(void){

	U4	u4t_adr;									// �Ǐo���A�h���X

	// �G���A�ύX�Ȃ�V����INDEX1���\�z����
	if(u2s_lonarea_old != sts_gpsinf.u2_lonarea){	// �ω��L��H

		u2s_srch_center_grp = sts_gpsinf.u2_lonarea;

		// YP�f�[�^ INDEX1
		u4t_adr = sizeof(ST_HEADER) + ((U4)(u2s_srch_center_grp-1) * (U4)sizeof(ST_INDEX1));
		if((fopen_s(&gpsDataFile, gpspoi_filename, "rb") == 0)
			&& (fseek(gpsDataFile, u4t_adr, SEEK_CUR) == 0)
			&& (fread(&sts_index1[0], sizeof(unsigned char), sizeof(ST_INDEX1) * 3, gpsDataFile) == sizeof(ST_INDEX1)*3))
		{
			// �Í�����
			DataCryptPoi(sizeof(ST_INDEX1)*3, u4t_adr, (U1 *)&sts_index1[0], u2g_dataSpec);
		}
		fclose(gpsDataFile);

		u2s_lonarea_old = sts_gpsinf.u2_lonarea;	// �O��o�x�G���A��ۑ�
	}
	// �o�x�G���A�ύX�Ȃ��Ȃ�INDEX1�͕ς��Ȃ�

}

//--------------------------------------------------------------------------//
//�y�֐��zGPS�f�[�^�ǂݏo������												//
//�y�@�\�z�w��u���b�N�A�w��ʒu�̃f�[�^��RAM�ɓW�J����						//
//�y�ߒl�zOK�F�ǂݏo������		NG�F�ǂݏo�����s							//
//�y���l�zdynamic��static�̐ؑ֎��͕K��cache disable�ɂ��邱��				//
//        ���ǂݏo�����������ɂȂ�A�����A�h���X���d�����邽��				//
//--------------------------------------------------------------------------//

enum{
	LOWER_CACHE = 0,
	UPPER_CACHE,
};
static U1	u1_ReadGpsData(U4 u4h_dataAddr, EM_TGT_DATA_TYPE type){

	if((fopen_s(&gpsDataFile, gpspoi_filename, "rb") != 0)
	|| (fseek(gpsDataFile, u4h_dataAddr, SEEK_CUR) != 0)
	|| (fread(&sts_gpsrom, sizeof(unsigned char), u1_MAPDATA_SIZE, gpsDataFile) != u1_MAPDATA_SIZE)){
		fclose(gpsDataFile);
		return NG;
	}
	fclose(gpsDataFile);
	DataCryptPoi(u1_MAPDATA_SIZE, u4h_dataAddr, (U1 *)&sts_gpsrom, u2g_dataSpec);

	sts_gpsrom.u4_addr = u4h_dataAddr;	// �f�[�^�̕����A�h���X��ۑ�

	return	OK;
}

//--------------------------------------------------------------------------
//�y�֐��z�u���b�N���T�[�`����												
//�y�@�\�z�u���b�N�̎w��ʒu�f�[�^���͈͓����`�F�b�N����					
//�y�����zu1h_romType �F�Ώ�ROM�^�C�v										
//�y�ߒl�zu1_SRCH_NG_NONE	: �u���b�N���Ƀ|�C���g�Ȃ��̂��ߎ��s			
//		  u1_SRCH_NG_DEV_BUSY: �f�o�C�X�A�N�Z�X���s							
//		  u1_SRCH_NG_SMALL	: �ܓx���̂��ߎ��s								
//		  u1_SRCH_NG_LARGE	: �ܓx��̂��ߎ��s								
//		  u1_SRCH_OK		: �T�[�`����									
//--------------------------------------------------------------------------
static U1 u1_BlkSrch(EM_TGT_DATA_TYPE type){

	U1	u1t_ret;									// �ߒl
	U4	u4t_dataAddr;

	// INDEX1�̊Y���u���b�N�v�f�����`�F�b�N
	if(psts_index1->u2_elenum == 0){				// �|�C���g�Ȃ�
		u1t_ret = u1_SRCH_NG_NONE;					// �|�C���g�Ȃ��ɂ�莸�s
	}
	else{
		// INDEX1�̃O���[�v�擪�ʒu�ƈʒu����f�[�^�A�h���X���o��
		u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
		// �ǂݏo��
		if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
													// �ǂݏo�����s�Ȃ�
			u1t_ret = u1_SRCH_NG_DEV_BUSY;			// DEVICE BUSY��Ԃ�
		}
		// MAP�o�^�����݂�
		else{
			switch(u1_RegMap()){					// �o�^���s����
			case REGNG_LAT_SMALL:					// ���݈ܓx��菬
				u1t_ret = u1_SRCH_NG_SMALL;
				break;
			
			case REGNG_LAT_LARGE:					// ���݈ܓx����
				u1t_ret = u1_SRCH_NG_LARGE;
				break;
			
			default:								// �ܓx�͈͓��Ȃ�
				u1t_ret = u1_SRCH_OK;				// �����T�[�`�Ƃ��Ă�OK
				// �����T�[�`�_�����L��
				u2s_inisrch_pos = u2s_srch_pos;		// �u���b�N���ʒu�L��
				break;
			}
		}
	}
	return	u1t_ret;
}

//--------------------------------------------------------------------------
//�y�֐��z�|�C���g�T�[�`����												
//�y�@�\�z���݂̃|�C���g����ܓx�㉺�̎w������Ƀ|�C���g�����o��			
//�y�����zu1h_romType �F�Ώ�ROM�^�C�v										
//		  u1h_dir�F���o������		SMALL / LARGE							
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_PntSrch(U1 u1h_dir, EM_TGT_DATA_TYPE type){
	
	U1	u1t_ret = u1_SRCH_OK;						// �ߒl
	U4	u4t_dataAddr;
	
	// ���̃f�[�^�����o�����߁A�u���b�N���ʒu�E�u���b�N�ԍ����X�V����
	// �ܓx����
	if(u1h_dir == SMALL){							// �ܓx��������
		if(u2s_srch_pos == 0){						// �ʒu����ԏ�H
			return	u1_SRCH_END;					// �I��
		}
		else{										// �ʒu����ԏ�łȂ�
			u2s_srch_pos--;							// �ʒu�ړ�
		}
	}
	else{											// �ܓx�������
		if(u2s_srch_pos >= (psts_index1->u2_elenum - (U2)1)){
													// �u���b�N�ŉ��_�ɓ��B�H
			return u1_SRCH_END;
		}
		else{
			u2s_srch_pos++;							// �ʒu�ړ�
		}
	}

	// �f�[�^��ǂݏo��
	u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
	if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
		return	u1_SRCH_NG_DEV_BUSY;
	}

	// MAP�o�^�����݂�
	switch(u1_RegMap()){

	case REGNG_MAP_MAX:								// �o�^�����I�[�o�[�œo�^���ꂸ
		u1t_ret = u1_SRCH_MAX_ERR;					// �ő吔�G���[��ʒm
		break;

	case REGNG_LAT_SMALL:							// �ܓx���̂��ߓo�^���ꂸ
	case REGNG_LAT_LARGE:							// �ܓx��̂��ߓo�^���ꂸ
		u1t_ret = u1_SRCH_END;						// �ܓx�����ɂ͂����Ȃ����ߏI��
		break;

//	case REGOK:										// �͈͓��̂��ߓo�^���ꂽ
//	case REGNG_LON_SMALL:							// �o�x���̂��ߓo�^���ꂸ
//	case REGNG_LON_LARGE:							// �o�x��̂��ߓo�^���ꂸ
	default:
		u1t_ret = u1_SRCH_OK;						// �ܓx�����͓����Ă���̂Ōp��
		break;
	}
	
	return	u1t_ret;
}
//--------------------------------------------------------------------------
//�y�֐��z�}�b�v�o�^����													
//�y�@�\�z�|�C���g���͈͓��ł���΃}�b�v�ɓo�^����							
//�y�ߒl�zOK�F�o�^�\		NG�F�o�^�s�\									
//--------------------------------------------------------------------------
static U1	u1_RegMap(void){

	U4	u4t_diffLat;								// �ܓx����
	U4	u4t_diffLon;								// �o�x����
	U4	u4t_lat;									// �ܓx
	U4	u4t_lon;									// �o�x
	ST_GPSMAP	*pstt_mkmap;
	U1	u1t_highway;
	Bool in_range = FALSE;

	u4t_lat = u4_RomDeScrmble(&sts_gpsrom, TYPE_LAT);	// �ܓx�l�X�N�����u������

	if(u4t_lat < u4s_latDiffPrmMinus){
		return	REGNG_LAT_SMALL;					// �����l��艺�̂���NG
	}
	if(u4t_lat > u4s_latDiffPrmPlus){
		return	REGNG_LAT_LARGE;					// ����l����̂���NG
	}

	u4t_lon = u4_RomDeScrmble(&sts_gpsrom, TYPE_LON);	// �o�x�l�X�N�����u������

	if(u4t_lon < u4s_lonDiffPrmMinus){
		return	REGNG_LON_SMALL;					// �����l��艺�̂���NG
	}
	if(u4t_lon > u4s_lonDiffPrmPlus){
		return	REGNG_LON_LARGE;					// ����l����̂���NG
	}

	u4t_diffLat = ABS_SUB(sts_gpsinf.lat, u4t_lat);
	u4t_diffLon = ABS_SUB(sts_gpsinf.lon, u4t_lon);

	// POI�f�[�^����ʂɂ���̂ŁA���̑������͎̂��O�Ɏ̂Ă�悤�ɂ���
	if(Can_data_pre_expire(&sts_gpsrom) == TRUE){
		return REGOK;								// ���ɐi��
	}

	if(sts_gpsrom.road == ROAD_HIGH){
		u1t_highway = ON;
	}
	else{
		u1t_highway = OFF;
	}
	
	{
		U1 distRng = em_TBL_DISTRNG[sts_gpsrom.u1_type][u1t_highway];

		if((u4t_diffLat <= u4_TBL_LATDIST[distRng][sts_gpsinf.u1_latarea])
		&& (u4t_diffLon <= u4_TBL_LONDIST[distRng][sts_gpsinf.u1_latarea])){
			in_range = TRUE;
		}
	}

	// �͈͓��m��
	if(in_range){
		// �D��x����
		if(u1_TBL_MAPPRI[sts_gpsrom.u1_type] == LO){
			if(u2s_mkmap_num >= GPSMAP_MAX){		// ���łɒ�D�������]�n�Ȃ��H
				return	REGOK;						// �o�^�͂��Ȃ����A�܂����D�悪���邩������Ȃ��̂�
													// ���̍쐬�ɐi��
			}
			pstt_mkmap = &sts_gpsmap[(GPSMAP_MAX - 1) - u2s_mkmap_loPri_num];
													// ��D��}�b�v�쐬�ʒu
			u2s_mkmap_loPri_num++;					// ��D��}�b�v�쐬���X�V
		}
		else{
			if(u2s_mkmap_hiPri_num >= GPSMAP_MAX){	// ���łɂ��ׂč��D��œ����]�n�Ȃ��H
				return	REGNG_MAP_MAX;				// �ő���̂��ߓo�^�s��
			}
			pstt_mkmap = &sts_gpsmap[u2s_mkmap_hiPri_num];
													// ���D��}�b�v�ʒu
			u2s_mkmap_hiPri_num++;					// ���D��}�b�v�쐬���X�V
			if((u2s_mkmap_loPri_num != 0)			// ��D��}�b�v�P���ȏ゠�� and
			&& ((u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num) > GPSMAP_MAX)){
													// ���D�悪��D��̏㏑�������H
				u2s_mkmap_loPri_num--;				// ��D��}�b�v�P���폜
			}
		}
		u2s_mkmap_num = u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num;
													// �}�b�v�쐬���X�V

		pstt_mkmap->u4_tgtLat = u4t_lat;			// �ܓx�ۑ�
		pstt_mkmap->u4_tgtLon = u4t_lon;			// �o�x�ۑ�
		pstt_mkmap->u4_dataAddr = sts_gpsrom.u4_addr;
													// ���A�h���X�ۑ�
		pstt_mkmap->u2_tgtDeg = (U2)sts_gpsrom.u1_deg * (U2)DEGDAT_LSB * (U2)DEG_LSB;
		pstt_mkmap->un_type.bit.b_code = sts_gpsrom.u1_type;
													// ���
		pstt_mkmap->un_type.bit.b_road = sts_gpsrom.road;
													// ���H����
		pstt_mkmap->un_type.bit.b_tunnel = sts_gpsrom.tunnel;
													// �g���l������
		pstt_mkmap->un_type.bit.b_dataArea = sts_gpsrom.area;
													// �G���A�����ۑ�
		pstt_mkmap->u1_wrnSts = 0;					// �x���ԏ�����
		pstt_mkmap->hys_ahead = FALSE;				// �O�����o������
		pstt_mkmap->ahead_hys_count = 0;			// �O���q�X�J�E���^������
		pstt_mkmap->u2_countdown_dist = U2_MAX;
		pstt_mkmap->u2_dst_old = U2_MAX;
		pstt_mkmap->u1_areaStsRd = AREA_STATUS_OUT;
		pstt_mkmap->u1_areaStsSc = AREA_STATUS_OUT;

		if(sts_gpsrom.u1_type == TGT_ICANCEL){		// I�L�����Z���̂Ƃ�
			if(sts_gpsrom.un_extra.dynamic.un_regday.hword == uns_today.hword){
				pstt_mkmap->u1_wrnSts = STS_ATCANCEL_REG1ST;
													// �L�����Z�������Ȃ�
			}
		}
		memcpy(&pstt_mkmap->un_extra.byte[0], &sts_gpsrom.un_extra.byte[0], EXTRA_DATA_SIZE);
													// EXTRA�ۑ�
		// �o�^����
	}
	// �͈͖͂����ł��o�^��OK�ɂ���(�����T�[�`�̏ꍇ�̓}�b�v���͂Ȃ��Ă��悢)

	return	REGOK;

}

static Bool	Can_data_pre_expire(ST_GPSROM *psth_gpsrom){
	return FALSE;
}

//--------------------------------------------------------------------------
//�y�֐��zGPS�ܓx�E�o�x���X�N�����u����������								
//�y�@�\�z�����g�p�ł��鐔�l�ɕϊ�����									
//�y�����z�Ώۃ^�C�v  TYPE_LAT�F�ܓx  TYPE_LON�F�o�x						
//�y�ߒl�zU4	�ܓx�l or �o�x�l (LSB : 10^-3��)							
//�y���l�z																	
//--------------------------------------------------------------------------
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type){

	U4	u4t_calwork;								// �Z�o���[�N
	U4	u4t_addval;									// ���Z�l
	
	if(u1h_type == TYPE_LAT){
		u4t_calwork = (U4)(((U4)p_rom->u1_lat3 << 16) + (U4)p_rom->u2_lat12);
		u4t_addval = u4_LAT_BASE_MIN;
	}
	else{
		u4t_calwork = (U4)(((U4)p_rom->u1_lon3 << 16) + (U4)p_rom->u2_lon12);
		u4t_addval = u4_LON_BASE_MIN;
	}

	u4t_calwork &= u4_LATLON_MSKPTN;
	u4t_calwork += u4t_addval;
	
	return	u4t_calwork;
}

//--------------------------------------------------------------------------
//�y�֐��z����<->�C�ӓ_�Ԏ����������Z�o										
//�y�@�\�z���ԂƔC�ӓ_�Ƃ̎w�莲�����̋������Z�o����						
//�y�����zU4	u4h_latlon�F�ܓx or �o�x									
//        U1	u1h_type�F�ܓx or �o�x										
//�y�ߒl�zU2	����(LSB 1m)												
//--------------------------------------------------------------------------
static U2	u2_CalDstAxis(U4 u4h_latlon, U1 u1h_type){

	U4	u4t_diff;

	if(u1h_type == TYPE_LAT){						// �ܓx�����w��(X��)
		u4t_diff = ABS_SUB(u4h_latlon, sts_gpsinf.lat);
													// �o�x���Z�o
		u4t_diff *= (U4)u2_TBL_LAT2MTR[sts_gpsinf.u1_latarea];
													// m�����ɕϊ�
		u4t_diff += 500;							// 0.1m�̈ʂŎl�̌ܓ�
		u4t_diff /= 1000;							// LSB 0.001m��1m
	}
	else{											// �o�x�����w��(Y��)
		u4t_diff = ABS_SUB(u4h_latlon, sts_gpsinf.lon);
													// �o�x���Z�o
		u4t_diff = (U4)((float)u4t_diff * k_lon2_meter + 0.5f);
													// 1m�ɕϊ�
	}

	// �ꉞ�K�[�h����
	if(u4t_diff > (U4)U2_MAX){
		u4t_diff = (U4)U2_MAX;
	}

	return	(U2)u4t_diff;
}

//	-----------------------------------------------------------------------------------------------
static	U2	Shahen(U2 inX, U2 inY){

	U2	z;

	if(inX == 0){
		z = (U2)inY;
	}
	else if(inY == 0){
		z = (U2)inX;
	}
	else
	{
		U4 wrk = (U4)sqrtf((float)((S4)inX*(S4)inX + (S4)inY*(S4)inY));
		if(wrk > U2_MAX)
		{
			wrk = U2_MAX;
		}
		z = (U2)wrk;
	}

	return (U2)z;

}

//--------------------------------------------------------------------------
//�y�֐��z����<->�w��_�����p�x�Z�o											
//�y�@�\�z���ԂƎw��_�Ƃ����񂾒����̎w��_��p�x���Z�o����				
//�y�����zu2h_deltaX:�ܓx��		u2h_deltaY�F�o�x��							
//�y�ߒl�zU2 �p�x(LSB 0.1��)												
//--------------------------------------------------------------------------
static U2	u2_CalDegA(U2 u2h_deltaX, U2 u2h_deltaY, ST_GPSMAP *psth_map){

	U1	u1t_deg_area;								// �ی�
	U2	u2t_deg = 0;								// �Z�o�p�x

	// �ی��̓���
	if(psth_map->u4_tgtLat <= sts_gpsinf.lat){
													// �w��_���猩�Ďԗ��͈ܓx�{���H
		// ��P�������͑�S�ی�
		if(psth_map->u4_tgtLon <= sts_gpsinf.lon){
													// �w��_���猩�Ďԗ��͌o�x�{���H
			u1t_deg_area = 1;						// ��P�ی�
		}
		else{										// ��S�ی�
			u1t_deg_area = 4;
		}
	}
	else{											// �w��_���猩�Ďԗ��͈ܓx�|��
		// ��Q�������͑�R�ی�
		if(psth_map->u4_tgtLon <= sts_gpsinf.lon){
													// �w��_���猩�Ďԗ��͌o�x�{���H
			u1t_deg_area = 2;						// ��Q�ی�
		}
		else{
			u1t_deg_area = 3;						// ��R�ی�
		}
	}

	// �p�x�Z�o
	if(u2h_deltaX == 0){							// ����0�Ōv�Z�s�\��
		u2t_deg = (U2)(90*DEG_LSB);					// 90���ɂ���
	}
	else{
		u2t_deg = (U2)(atan2f((float)u2h_deltaY, (float)u2h_deltaX) * (180.0f * (float)DEG_LSB / ZMATH_PI));
	}

	// �ی����̊p�x
	switch(u1t_deg_area){
	case 1:											// ��P�ی�
													// ��A���̂܂�
		break;
	case 2:											// ��Q�ی�
		u2t_deg = (U2)(180*DEG_LSB) - u2t_deg;		// ��A = 180��-��A
		break;
	case 3:											// ��R�ی�
		u2t_deg = (U2)(180*DEG_LSB) + u2t_deg;		// ��A = 180��+��A
		break;
	case 4:											// ��S�ی�
		u2t_deg = (U2)(360*DEG_LSB) - u2t_deg;		// ��A = 360��-��A
		break;
	}

	return	u2t_deg;
}
//--------------------------------------------------------------------------
//�y�֐��z�|�C���g�Ό��p�Z�o												
//�y�@�\�z���Ԃ��|�C���g�ɑ΂��Č����Ă���p�x���Z�o����					
//�y�����zU2 ��A (LSB 0.1��)												
//�y�ߒl�zS2 ��B (LSB 0.1��)												
//--------------------------------------------------------------------------
static S2	s2_CalDegB(U2 u2h_degA){

	U2	u2t_degA;
	S2	s2t_degB;

	// ��A���|�C���g�->���Ԋ�ɂ���i180�����Α��ɂ���j
	if(u2h_degA >= (180*DEG_LSB)){						// 180���ȏ�Ȃ�
		u2t_degA = u2h_degA - (180*DEG_LSB);
	}
	else{												// 180�������Ȃ�
		u2t_degA = u2h_degA + (180*DEG_LSB);
	}
	// ���ԕ��ʊ�Ł�A�̐^�t�����p�Ƃ̍������Z�o
	s2t_degB = s2_CalDegSub(u2t_degA, sts_gpsinf.u2_deg);

	return	s2t_degB;
}

//--------------------------------------------------------------------------
//�y�֐��z�p�x���Z�o����													
//�y�@�\�z�p�x2���猩���p�x1�̍������Z�o����								
//�y�����zu2h_deg1:�p�x1(LSB 0.1��)		u2h_deg2�F�p�x2(LSB 0.1��)			
//�y�ߒl�z�p�x���FS2 (LSB 0.1��)											
//�y���l�z�E��]��+�A����]��-�A�����W��-180���`+180��						
//--------------------------------------------------------------------------
S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2){

	S2 s2t_degdiff = (S2)u2h_deg1 - (S2)u2h_deg2;

	if(s2t_degdiff >= 180*DEG_LSB)
	{
		s2t_degdiff -= 360*DEG_LSB;
	}
	else if(s2t_degdiff <= -180*DEG_LSB)
	{
		s2t_degdiff += 360*DEG_LSB;
	}

	return	s2t_degdiff;
}
//--------------------------------------------------------------------------
//�y�֐��z�I�[�r�X�x��J�ڏ���												
//�y�@�\�z�I�[�r�X�x��𔻒肷��											
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitOrbis(EM_TGTDEG emh_tgtDeg){
	
	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U2	u2t_rngOutDst = u2_DIST_1500M;						// 1500m�ŏ�����
	U2	u2t_passDst = u2_DIST_60M;							// 60m�ʉߔ���ŏ�����

	// ���E����
	if((psts_chkmap->un_type.bit.b_road == ROAD_HIGH)		// ����������
	&& (psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL)){	// and ��g���l��
		u2t_rngOutDst = u2_DIST_2500M;
	}
	// �ʉߋ���
	if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){		// ����������
		u2t_passDst = u2_DIST_100M;							// 100m�Œʉߔ���
	}

	if(u1s_poiSetSel == SEL_POI_NORMAL){					// �g���l���U����ԂłȂ��Ȃ�x�񏈗�

		if(psts_chkmap->u2_dst > u2t_rngOutDst){				// ���O��������Ȃ�
			psts_chkmap->u1_wrnSts = STS_ORBIS_NOALARM;			// ��x���Ԃ�
		}
		else{
			// �x���ԑJ��
			switch(psts_chkmap->u1_wrnSts){
		//-------��x����-------------------------------------------------------//
			case STS_ORBIS_NOALARM:									// ��x����
				// ��͈͓��Ń^�[�Q�b�g�Ɍ������ƌx��J�n
				if(u1t_degChk != OK){								// �p�xNG�H
					break;
				}
				if(psts_chkmap->un_type.bit.b_tunnel == TUNNEL_IN){	// �g���l��������
					break;
				}
				if(psts_chkmap->u2_dst <= u2_DIST_330M){			// 330m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m�x���Ԃ�
																	// 500m�x�񉹗v��
					// �_�C���N�g��300m�ɔ�э��񂾂Ƃ��A�ʉߑ��x�͍��m���Ȃ��i�Ԃɍ���Ȃ��j
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_600M;		// 600m�x���Ԃ�
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M)	{	// 1100m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m�x���Ԃ�
				}
				else{
					if((psts_chkmap->u2_dst <= u2_DIST_2100M)		// 2100m�ȓ�
					&& (u1_ChkDegRng(TYPE_TGTDEG_EXIST_VERY_FAR, psts_chkmap->u2_dst) == OK)
																	// �Ό��p�xOK
					&& (psts_chkmap->un_type.bit.b_road == ROAD_HIGH)
																	// ��������
					&& (psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL)
																	// ��g���l��
					&& ((stg_setMisc.b_roadSel != SET_ROAD_AUTO) || (u1s_roadJdgSts == ROADJDG_STS_HIGH))
																	// ���H�I���I�[�g�ȊO���A�I�[�g�ō������s���
					&& (sts_gpsinf.b_spddeg_ok == OK)					// ���x���ʗL����
					&& (sts_gpsinf.u2_spd >= ((U2)psts_chkmap->un_extra.orbis.b_lmtSpd * (U2)10 * SPD_LSB))){
																	// �������x�ȏ�
						psts_chkmap->u1_wrnSts = STS_ORBIS_2100M;	// 2100m�x���Ԃ�
					}
				}
				break;
		//-------2100m�x����----------------------------------------------------//
			case STS_ORBIS_2100M:									// 2100m�x����
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// �p�xOK and 1100m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m�x���Ԃ�
				}
				break;
		//-------1100m�x����----------------------------------------------------//
			case STS_ORBIS_1100M:									// 1100m�x����
				if(u1t_degChk == OK){								// �p�xOK
					if(psts_chkmap->u2_dst <= u2_DIST_330M){		// 330m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_ORBIS_300M;	// 300m�x���Ԃ�
						// �_�C���N�g��300m�ɔ�э��񂾂Ƃ��A�ʉߑ��x�͍��m���Ȃ��i�Ԃɍ���Ȃ��j
					}
				 	else if(psts_chkmap->u2_dst <= u2_DIST_600M){	// 600m�͈͓��Ȃ�
						psts_chkmap->u1_wrnSts = STS_ORBIS_600M;	// 600m�x���Ԃ�
					}
				}
				break;
		//-------600m�x����----------------------------------------------------//
			case STS_ORBIS_600M:									// 600m�x����
				if(psts_chkmap->u2_dst <= u2t_passDst){				// �ʉߔ��苗�����H
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m�x���Ԃ�
				}
				else if((psts_chkmap->u2_dst <= u2_DIST_330M) && (u1t_degChk == OK)){
																	// 330m�͈͓� and �p�xOK 
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m�x���Ԃ�
				}
				break;
		//-------300m�x����----------------------------------------------------//
			case STS_ORBIS_300M:									// 300m�x����
				if(psts_chkmap->u2_dst <= u2t_passDst){				// �ʉߔ��苗�����H
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m�x���Ԃ�
				}
				break;

			default:
				break;
			}
		}

		// �I�[�r�X�ł̃t�H�[�J�X�^�[�Q�b�g����
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){		// �����I�[�r�X
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, FALSE);
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
			else{													// ��ʃI�[�r�X
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
		}

		// GPS�x�񃌃x���̔���
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			if(u1t_degChk == OK){
				// HI�̔���
				if(psts_chkmap->u2_dst <= u2_DIST_600M){
					// �ʉߋ����ȓ��Ȃ�ԐF����J������
					if(psts_chkmap->u2_dst > u2t_passDst)
					{
						update_gps_warning_lvl(SYS_RED_WARNING_HI, psts_chkmap->u2_dst);
					}
				}
				// MID�̔���
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
					update_gps_warning_lvl(SYS_RED_WARNING_MID, psts_chkmap->u2_dst);
				}
				// LO�̔���
				else if(psts_chkmap->u2_dst <= u2_DIST_2100M){
					update_gps_warning_lvl(SYS_RED_WARNING_LO, psts_chkmap->u2_dst);
				}
			}
		}
	}
}
//--------------------------------------------------------------------------
//�y�֐��z�p�x�͈͔��菈��													
//�y�@�\�z�����Ŏw�肳�ꂽ�^�C�v�Ŋp�x�����𔻒肷��						
//�y�����zemh_tgtdeg : �^�[�Q�b�g���ʗL��									
//		  u2h_dst �F ���ԁ`�^�[�Q�b�g���S�_����								
//�y�ߒl�zU1 OK : �p�x�͈͓�  NG:�p�x�͈͊O									
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_ChkDegRng(EM_TGTDEG emh_tgtdeg, U2 u2h_dst){

	U1	u1t_ret = NG;									// �s�����ŏ�����

	switch(emh_tgtdeg){
	default:
		if((u1_ChkDegValid())							// �p�x�L��
		&& (u1_DegBIsOk(emh_tgtdeg, u2h_dst) == TRUE)){	// �Ό��p�L��
			if((emh_tgtdeg == TYPE_TGTDEG_NOEXIST)		// �^�[�Q�b�g�ɕ��ʂ��Ȃ��Ƃ�
			|| (u1_DegAIsOk(psts_chkmap) == TRUE)){		//  or ����p�x
				u1t_ret = OK;							// ����
			}
		}
		break;
	case TYPE_TGTDEG_EXIST_DEGA_WIDE:
		if((u1_ChkDegValid())							// �p�x�L��
		&& (u1_DegBIsOk(emh_tgtdeg,u2h_dst) == TRUE)	// �Ό��p�L��
		&& (u1_DegAWideIsOk(psts_chkmap) == TRUE)){		// ��L�͈�
			u1t_ret = OK;
		}
		break;
	case TYPE_TGTDEG_NOEXIST_BACK:						// �w��(�^�[�Q�b�g�̒��S�����)
		if((u1_ChkDegValid())							// �p�x�L��
		&& (u1_DegBIsOk(TYPE_TGTDEG_DEGB_FRONT, u2h_dst) == FALSE)){
														// �O���ɑ��݂��Ȃ�
			u1t_ret = OK;
		}
		break;
	}

	return	u1t_ret;

}
static void update_distance(ST_GPSMAP *psth_map)
{

	// �O��l���Ȃ��Ƃ��ŐV�l�̂܂�
	if(psth_map->u2_dst_old == U2_MAX)
	{
		psth_map->u2_dst_old = psth_map->u2_dst;
	}
	else
	{
		S2 diff = (S2)psth_map->u2_dst - (S2)psth_map->u2_dst_old;

		// �������� �������� +20m�ȏ���
		if(diff <= 0 || diff >= 20)
		{
			psth_map->u2_dst_old = psth_map->u2_dst;
		}
		else
		{
			psth_map->u2_dst = psth_map->u2_dst_old;	// �O��l�ŋ�����ۂ�
		}
	}
}

static void update_ahead_hys(ST_GPSMAP *psth_map)
{
	Bool face = FALSE;

	if(TBL_TGTTRANS_PRM[psth_map->un_type.bit.b_code] == TYPE_TGTDEG_EXIST)
	{
		if(u1_DegAWideIsOk(psth_map) == TRUE)
		{
			face = TRUE;
		}
	}
	else
	{
		face = TRUE;
	}

	// ��Ԃ�J�ڂ���
	if(psth_map->hys_ahead)
	{
		// ���ʕt���ꍇ�A���̕��ʂ̕����ɂ��Ȃ��ꍇ�͑�������
		if(!face)
		{
			psth_map->hys_ahead = FALSE;
			psth_map->ahead_hys_count = 0;
		}
		else
		{
			// �O���ɑ��݂��Ȃ�
			if(psth_map->s2_degB < s2_DEG_TGTFWD_MIN || psth_map->s2_degB > s2_DEG_TGTFWD_MAX)
			{
				// �񓪉^���Ȃǂŋɒ[�Ɍ���ɂȂ����ꍇ�͈ꔭ����
				if(psth_map->s2_degB < s2_NOAHEAD_ABS_DEG_MIN || psth_map->s2_degB > s2_NOAHEAD_ABS_DEG_MAX)
				{
					psth_map->hys_ahead = FALSE;
					psth_map->ahead_hys_count = 0;
				}
				else
				{
					// �A����n��p������Δ�O���֑J��
					if(++(psth_map->ahead_hys_count) >= u1_NOAHEAD_HYS_COUNT)
					{
						psth_map->hys_ahead = FALSE;
						psth_map->ahead_hys_count = 0;
					}
				}
			}
			else
			{
				psth_map->ahead_hys_count = 0;
			}
		}
	}
	else
	{
		// �Ό����Ă��āA�O���ɑ��݂���Ȃ瑦���O���m��
		if(face && psth_map->s2_degB >= s2_DEG_TGTFWD_MIN && psth_map->s2_degB <= s2_DEG_TGTFWD_MAX)
		{
			psth_map->hys_ahead = TRUE;
			psth_map->ahead_hys_count = 0;
		}
	}
}

//--------------------------------------------------------------------------
//�y�֐��z��A�͈͔��菈��(�L�͈�)											
//�y�@�\�z���Ԃ��^�[�Q�b�g���ʂ́}90�������ɑ��݂��邩�ǂ������肷��		
//�y�����z�Ȃ�																
//�y�ߒl�zU1 TRUE�F�͈͓�	FALSE�F�͈͊O									
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_DegAWideIsOk(ST_GPSMAP *psth_map){

	U1	u1t_ret = FALSE;							// �͈͊O�ŏ�����
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ��A���^�[�Q�b�g�́}90�������ɂ���Δ͈͓�
	if((s2t_deg > s2_DEG_TGTRNG_WIDE_MIN)
	&& (s2t_deg < s2_DEG_TGTRNG_WIDE_MAX)){
		u1t_ret = TRUE;								// �͈͓�
	}
	
	return	u1t_ret;
}

//--------------------------------------------------------------------------
//�y�֐��zPOI�ݒ蔻�菈��													
//�y�@�\�z�ΏۂƂ��Ă���POI�f�[�^���ݒ��L�����ǂ������肷��				
//�y���l�z																	
//--------------------------------------------------------------------------
enum{
	ROADCHK_OFF,
	ROADCHK_ON,
	ROADCHK_ONLY_MAN,								// �}�j���A�����̂݃`�F�b�N
};
static	U1	IsJudgePOISet(U1 u1h_sel)
{
	if(u1h_sel == SEL_POI_NORMAL){						// �m�[�}��
		return (u1_JudgePOISet(psts_chkmap));			// 
	}
	else{												// �g���l��
		return (u1_JudgePOISetTunnel(psts_chkmap));			// 
	}
}

static	U1	u1_JudgePOISet(ST_GPSMAP *psth_map){

	U1	u1t_ret = NG;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_roadChk = ROADCHK_ON;					// ���H�`�F�b�N�v

	// ��Ԕ���
	switch(psth_map->un_type.bit.b_code){
	default:
		break;

	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
		}
		break;

	case TGT_TRAP_ZONE:
	case TGT_RT_TRAP_ZONE:
	case TGT_NOFIX_TRAP_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_trapWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// �S�֎~�ȊO
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// ���x������ or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_trapWrnLvl)){
																				// �ݒ背�x���ȏ�
				u1t_ret = OK;
			}
		}
		break;

	case TGT_CHKPNT_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_chkWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// �S�֎~�ȊO
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// ���x������ or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_chkWrnLvl)){
																				// �ݒ背�x���ȏ�
				u1t_ret = OK;
			}
		}
		break;

	case TGT_NSYS:
		if(stg_setGps[u1g_setAcsSel].b_nSys){		// N�V�X�e���ݒ�ON
			u1t_ret = OK;
		}
		break;

	case TGT_TRFCHK:
		if(stg_setGps[u1g_setAcsSel].b_trafChkSys){	// ��ʊĎ��V�X�e��ON
			u1t_ret = OK;
		}
		break;

	case TGT_POLICE:
		if(psth_map->un_extra.police.b_policeType == POLICE_TYPE_STATION){
			if(stg_setGps[u1g_setAcsSel].b_police){	// �x�@��ON
				u1t_ret = OK;
			}
		}
		else{
			if(stg_setGps[u1g_setAcsSel].b_hwPolice){
													// �����x�@��ON
				u1t_ret = OK;
			}
		}
		break;

	case TGT_ACCIDENT:
		if(stg_setGps[u1g_setAcsSel].b_accident){	// ���̑���ON
			u1t_ret = OK;
		}
		break;

	case TGT_MICHINOEKI:
		if(stg_setGps[u1g_setAcsSel].b_michinoeki){	// ���̉wON
			u1t_ret = OK;
		}
		break;

	case TGT_CROSSING:
		if(stg_setGps[u1g_setAcsSel].b_crossing){	// �����_�Ď��V�X�e��ON
			u1t_ret = OK;
		}
		break;

	case TGT_SIGNAL:
		if(stg_setGps[u1g_setAcsSel].b_signal){		// �M�������}�~�V�X�e��ON
			u1t_ret = OK;
		}
		break;

	case TGT_SA:
		if(stg_setGps[u1g_setAcsSel].b_sa){			// �T�[�r�X�G���AON
			u1t_ret = OK;
		}
		break;

	case TGT_PA:
		if(stg_setGps[u1g_setAcsSel].b_pa){			// �p�[�L���O�G���AON
			u1t_ret = OK;
		}
		break;

	case TGT_HWOASYS:
		if(stg_setGps[u1g_setAcsSel].b_hwOasys){	// �n�C�E�F�C�I�A�V�XON
			u1t_roadChk = ROADCHK_OFF;
			u1t_ret = OK;
		}
		break;

	case TGT_HWRADIO:
		if(stg_setGps[u1g_setAcsSel].b_hwRadio){	// �n�C�E�F�C���W�ION
			u1t_ret = OK;
		}
		break;

	case TGT_PARKCHK_AREA_SZ:
	case TGT_PARKCHK_AREA_Z:
		if(stg_setGps[u1g_setAcsSel].b_parkChkArea){// ���փG���AON
			u1t_ret = OK;
		}
		break;

	case TGT_SHAJYOU_AREA:
		if(stg_setGps[u1g_setAcsSel].b_shajyoArea){	// �ԏ�_�������G���AON
			u1t_ret = OK;
		}
		break;

	case TGT_PARKING:
		if(stg_setGps[u1g_setAcsSel].b_parking){	// ���ԏ�\��ON
			u1t_ret = OK;
		}
		break;

	case TGT_MYAREA:								// ���H�����Ɉˑ����Ȃ�
	case TGT_MYCANCEL:
	case TGT_PIN:
		u1t_roadChk = ROADCHK_OFF;
		u1t_ret = OK;
		break;

	case TGT_ETC:
		if(stg_setGps[u1g_setAcsSel].b_etcLane){
			if(u1t_roadSel != SET_ROAD_NORM){
				u1t_ret = OK;
				if(u1t_roadSel == SET_ROAD_AUTO){
					if(psth_map->un_extra.etc.b_pointType == ETC_POINT_STOP){
													// STOP�_��
						u1t_roadChk = ROADCHK_OFF;	// ���H�����s��
					}
				}
				else{								// �I�[�� or ����
					u1t_roadChk = ROADCHK_OFF;		// ���H�����s��
				}
			}
		}
		break;

	case TGT_CURVE:
		if(stg_setGps[u1g_setAcsSel].b_curve){		// �}�J�[�uON
			u1t_ret = OK;
		}
		break;

	case TGT_BRAJCT:
		if(stg_setGps[u1g_setAcsSel].b_brajct){		// ����E����ON
			u1t_ret = OK;
		}
		break;

	case TGT_KENKYO:
		if(stg_setGps[u1g_setAcsSel].b_kenkyo){		// ����ON
			u1t_ret = OK;
		}
		break;

	case TGT_TUNNEL:
		if(stg_setGps[u1g_setAcsSel].b_tunnel){		// �g���l��ON
			u1t_ret = OK;
		}
		break;

	case TGT_TORUPA:
		if(stg_setGps[u1g_setAcsSel].b_torupa){		// �Ƃ��ON
			u1t_ret = OK;
		}
		break;

	case TGT_SCP_ICI:
	case TGT_SCP_ICO:
	case TGT_SCP_JC:
	case TGT_SCP_SPD:
	case TGT_SCP_PO:
		u1t_roadChk = ROADCHK_ONLY_MAN;				// �蓮���̂݃`�F�b�N�v
		u1t_ret = OK;
		break;

	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
			if((u1t_roadSel == SET_ROAD_AUTO)		// �I�[�g�I����
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)){
													// ����70km/h�ȉ��̍���������
				u1t_roadChk = ROADCHK_OFF;			// ���H����s�v
			}
		}
		break;

	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_SENSOR_YUDO:
		u1t_ret = OK;
		u1t_roadChk = ROADCHK_OFF;					// ���H����s�v
		break;

	case TGT_RAILROAD_CROSSING:						// ����
	case TGT_TMPSTOP:								// �ꎞ��~���Ӄ|�C���g
	case TGT_TOILET:
	case TGT_KOBAN:
	case TGT_FIRE:
	case TGT_HOIKU:
		u1t_ret = OK;								// �}�b�v�ǂݍ��ݎ��ɐݒ�͊m�F�ς�
		break;
	case TGT_ZONE30:
		if(stg_setGps[u1g_setAcsSel].b_zone30){		// �]�[��30
			u1t_ret = OK;
		}
		break;

	}

	// ���H����
	if(u1t_ret == OK){
		if(u1t_roadChk != ROADCHK_OFF){						// ���H�`�F�b�N�v
			if(psth_map->un_type.bit.b_road != ROAD_HIGH){	// ��ʓ�POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// ���H��������ݒ莞
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_HIGH)){// ���ݍ�����
						u1t_ret = NG;
					}
				}
				else{										// �蓮�ݒ莞
					if(u1t_roadSel == SET_ROAD_HIGH){		// �����̂ݍ��m�ݒ�
						u1t_ret = NG;
					}
				}
			}
			else{											// ������POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// ���H��������ݒ莞
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_NORM)){
															// ��ʓ����m�̂ݗv ?
						u1t_ret = NG;
					}
				}
				else{										// �蓮�ݒ莞
					if(u1t_roadSel == SET_ROAD_NORM){		// ��ʂ̂ݍ��m�ݒ�
						u1t_ret = NG;
					}
				}
			}
		}
	}
	return	u1t_ret;

}

//--------------------------------------------------------------------------
//�y�֐��zPOI�ݒ蔻�菈���i�g���l�����j										
//�y�@�\�z�ΏۂƂ��Ă���POI�f�[�^���ݒ��L�����ǂ������肷��				
//�y���l�z																	
//--------------------------------------------------------------------------
static	U1	u1_JudgePOISetTunnel(ST_GPSMAP *psth_map){

	U1	u1t_ret = NG;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_roadChk = ROADCHK_ON;					// ���H�`�F�b�N�v

	// ��Ԕ���
	switch(psth_map->un_type.bit.b_code){
	default:
		break;
// �I�[�r�X�͊�����(Y)���U���Ɏg��
	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
		}
		break;

	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
			if((u1t_roadSel == SET_ROAD_AUTO)		// �I�[�g�I����
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)){
													// ����70km/h�ȉ��̍���������
				u1t_roadChk = ROADCHK_OFF;			// ���H����s�v
			}
		}
		break;

	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_SENSOR_YUDO:
		u1t_ret = OK;
		u1t_roadChk = ROADCHK_OFF;					// ���H����s�v
		break;

	case TGT_NOFIX_TRAP_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_trapWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// �S�֎~�ȊO
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// ���x������ or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_trapWrnLvl)){
																				// �ݒ背�x���ȏ�
				u1t_ret = OK;
			}
		}
		break;

	case TGT_NOFIX_CHKPNT_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_chkWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// �S�֎~�ȊO
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// ���x������ or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_chkWrnLvl)){
																				// �ݒ背�x���ȏ�
				u1t_ret = OK;
			}
		}
		break;
	}

	// ���H����
	if(u1t_ret == OK){
		if(u1t_roadChk != ROADCHK_OFF){						// ���H�`�F�b�N�v
			if(psth_map->un_type.bit.b_road != ROAD_HIGH){	// ��ʓ�POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// ���H��������ݒ莞
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_HIGH)){// ���ݍ�����
						u1t_ret = NG;
					}
				}
				else{										// �蓮�ݒ莞
					if(u1t_roadSel == SET_ROAD_HIGH){		// �����̂ݍ��m�ݒ�
						u1t_ret = NG;
					}
				}
			}
			else{											// ������POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// ���H��������ݒ莞
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_NORM)){
															// ��ʓ����m�̂ݗv ?
						u1t_ret = NG;
					}
				}
				else{										// �蓮�ݒ莞
					if(u1t_roadSel == SET_ROAD_NORM){		// ��ʂ̂ݍ��m�ݒ�
						u1t_ret = NG;
					}
				}
			}
		}
	}
	return	u1t_ret;
}

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt)
{
	psth_tgt->u1_sts = PRI_FOCUS_TGT_NONE;
	psth_tgt->u2_dst = U2_MAX;
	psth_tgt->u2_num = TGTNUM_NOTGT;
	psth_tgt->u2_absDegB = (U2)(180*DEG_LSB);
	psth_tgt->u1_priTunnelIn = FALSE;
}

//--------------------------------------------------------------------------
//�y�֐��z���֊Ď��G���A��ԑJ�ڏ���										
//�y�@�\�z���֊Ď��G���A��ԑJ�ڂ��s��										
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitPchkSts(void){

	if(stg_setGps[u1g_setAcsSel].b_parkChkArea == OFF){		// �ݒ�OFF ?
		u1s_parkChkAreaSts = PCHK_WRN_OUT;						// ���O�ɂ���
		return;
	}

	switch(u1s_parkChkAreaSts){
	case PCHK_WRN_OUT:											// �G���A�O
		if(F_PCHK_SZ_IN){										// �ŏd�_�G���A�� ?
			if(u1s_runStopSts == RUNSTOP_STOP){					// ���s���� ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// �ŏd�_�����o�͍ς݂�
				
				// ���z�}�b�v�X�V
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// �����L���[�i�[�v��
																// �x�񉹏o�͗v��
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// �ŏd�_�������o�͂�
			}
		}
		else if(F_PCHK_Z_IN){									// �d�_�G���A�� ?
			if(u1s_runStopSts == RUNSTOP_STOP){					// ���s���� ?
				u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;		// �d�_�����o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// �����L���[�i�[�v��
																// �x�񉹏o�͗v��
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INZ_NOSNDOUT;		// �d�_�������o�͂�
			}
		}
		break;

	case PCHK_WRN_INSZ_NOSNDOUT:								// �ŏd�_�ŉ������o��
		if(!F_PCHK_SZ_IN){										// �ŏd�_����͗��E
			if(F_PCHK_Z_IN){									// �d�_�G���A�� ?
				if(u1s_runStopSts == RUNSTOP_STOP){				// ���s���� ?
					u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;	// �d�_�����o�͍ς݂�
					// ���z�}�b�v�X�V
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
					// �����L���[�i�[�v��
																// �x�񉹏o�͗v��
				}
				else{
					u1s_parkChkAreaSts = PCHK_WRN_INZ_NOSNDOUT;	// �d�_�������o�͂�
				}
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// ���O��
			}
		}
		else{													// �ŏd�_�G���A�p��
			if(u1s_runStopSts == RUNSTOP_STOP){					// ���s���� ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// �ŏd�_�����o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// �����L���[�i�[�v��
																// �x�񉹏o�͗v��
			}
		}
		break;

	case PCHK_WRN_INSZ_SNDOUT:									// �ŏd�_�ŉ����o�͍ς�
		if(!F_PCHK_SZ_IN){										// �ŏd�_����͗��E
			if(F_PCHK_Z_IN){									// �d�_�G���A�� ?
				u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;		// �d�_�����o�͍ς݂�
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// ���O��
			}
		}
		break;

	case PCHK_WRN_INZ_NOSNDOUT:									// �d�_�ŉ������o��
		if(F_PCHK_SZ_IN){										// �ŏd�_����
			if(u1s_runStopSts == RUNSTOP_STOP){					// ���s���� ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// �ŏd�_�����o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// �����L���[�i�[�v��
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// �ŏd�_�������o�͂�
			}
		}
		else{
			if(F_PCHK_Z_IN){									// �d�_�p��?
				if(u1s_runStopSts == RUNSTOP_STOP){				// ���s���� ?
					u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;	// �d�_�����o�͍ς݂�
					// ���z�}�b�v�X�V
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
					// �����L���[�i�[�v��
				}
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// ���O��
			}
		}
		break;

	case PCHK_WRN_INZ_SNDOUT:									// �d�_�ŉ����o�͍ς�
		if(F_PCHK_SZ_IN){										// �ŏd�_����
			if(u1s_runStopSts == RUNSTOP_STOP){					// ���s���� ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// �ŏd�_�����o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;

				// �����L���[�i�[�v��
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// �ŏd�_�������o�͂�
			}
		}
		else{
			if(!F_PCHK_Z_IN){									// �d�_���O?
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// ���O��
			}
		}
		break;

	default:
		u1s_parkChkAreaSts = PCHK_WRN_OUT;
		break;
	}
}

//--------------------------------------------------------------------------
//�y�֐��z�]�[��30�G���A��ԑJ�ڏ���										
//�y�@�\�z�]�[��30�G���A��ԑJ�ڂ��s��										
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void TransitZone30Sts(void){

	switch(u1s_zone30AreaSts){
	case ZONE30_WRN_OUT:										// �G���A�O
		if(F_ZONE30_IN){										// �G���A���H
			if(u1s_runStopSts == RUNSTOP_RUN){					// ���s��
				u1s_zone30AreaSts = ZONE30_WRN_IN_SNDOUT;		// ���������o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_type.bit.b_code = TGT_ZONE30; // 58
				sts_gpsmap[ZN30_VIRTUAL_INDEX].u4_dataAddr = ZN30_VIRTUAL_ADDRESS;
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_extra.zone30.u2_mapPctNum = 0;
				// �����L���[�i�[�v�f
			}
			else{
				u1s_zone30AreaSts = ZONE30_WRN_IN_NOSNDOUT;		// �����������o�͂�
			}
		}
		break;
	
	case ZONE30_WRN_IN_NOSNDOUT:								// �����������o��
		if(!F_ZONE30_IN){
			u1s_zone30AreaSts = ZONE30_WRN_OUT;
		}
		else{
			if(u1s_runStopSts == RUNSTOP_RUN){					// ���s��
				u1s_zone30AreaSts = ZONE30_WRN_IN_SNDOUT;		// ���������o�͍ς݂�
				// ���z�}�b�v�X�V
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_type.bit.b_code = TGT_ZONE30; // 58
				sts_gpsmap[ZN30_VIRTUAL_INDEX].u4_dataAddr = ZN30_VIRTUAL_ADDRESS;
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_extra.zone30.u2_mapPctNum = 0;
				// �����L���[�i�[�v�f
			}
		}
		break;
	
	case ZONE30_WRN_IN_SNDOUT:									// ���������o�͍ς�
		if(!F_ZONE30_IN){
			u1s_zone30AreaSts = ZONE30_WRN_OUT;
		}
		break;
	default:
		u1s_zone30AreaSts = ZONE30_WRN_OUT;
		break;
	}
}


static void	TransitDummy(EM_TGTDEG emh_tgtDeg){

}

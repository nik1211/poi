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
#include "mupass.h"

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
// �L���[���
enum{
	VC_QUEUE_EMPTY = 0,								// �L���[��
	VC_QUEUE_DATON,									// �L���[�f�[�^����
	VC_QUEUE_FULL,									// �L���[�t��
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

enum{
	INDX_RDRX_OFF = 0,
	INDX_RDRX_ON,
};

enum{
	GPSTGT_VC_STS_INIT = 0,							// �������
	GPSTGT_VC_STS_FIXOUTWT,							// ���ʉ��o�͑҂����
	GPSTGT_VC_STS_FIXOUT,							// ���ʉ��o�͒����
	GPSTGT_VC_STS_OUTOK,							// �o�͋����
};

typedef enum{
	MYREG_RET_REGNG = 0,							// �o�^�ł��Ȃ�����
	MYREG_RET_REGOK_PTFULL,							// �o�^�ł������}�b�v���t��
	MYREG_RET_REGOK,								// �o�^�ł���
}EM_MYREG_RET;

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
#pragma pack()

// GPS�{�C�X�L���[
typedef struct{
	EM_VC			u1_vcNum;							// �{�C�X�ԍ�
	EM_VCGPS_TYPE	em_vcType;							// �{�C�X�^�C�v
	U4				u4_addr;							// �^�[�Q�b�g�f�[�^�A�h���X
	U4				live_count;							// �����J�E���g�l
}ST_GPSVC_QUEUE;

// GPS�{�C�X�L���[�Ǘ����
typedef struct{
	ST_GPSVC_QUEUE	*queue;
	U1				queueSize;
	U1				sts;
	U1				rdPos;
	U1				wrPos;
}ST_GPSVC_QUEUE_CTRL;
enum{
	VCGPS_HIPRI = 0,
	VCGPS_MIDPRI,
	VCGPS_LOPRI,
};

// �D��t�H�[�J�X�^�[�Q�b�g
typedef struct{
	U2			u2_dst;								// ����
	U2			u2_absDegB;
	U1			u1_priTunnelIn;						// ���D��g���l��������
	U1			u1_sts;								// �t�H�[�J�X���
	U2			u2_num;								// �^�[�Q�b�g�ԍ�
}ST_FOCUS_TGT;

// MAP�I��p���
typedef struct{
	ST_GPSMAP	*pst_map;
	U1			isDegOk;
}ST_MAPSEL;

typedef struct{
	U4	u4_dataAddr;								// �f�[�^�A�h���X
	U2	u2_countdown_dist;							// �J�E���g�_�E������
	U2	u2_dst_old;									// �����O��l
	U1	u1_wrnSts;									// �x����
	U1	hys_ahead;									// �O�����o���
	U1	ahead_hys_count;							// �O���q�X�J�E���^
	U1	u1_areaStsRd;								// ���[�_�[�G���A���
	U1	u1_areaStsSc;								// �����G���A���
	U1	u1_areaScSnd;								// �G���A�������L��
}ST_OLDMAP;

#define		__ENABLE_DATA_CACHE__

#ifdef		__ENABLE_DATA_CACHE__
//#define	__CACHE_DEBUG__
#define	u2_DATA_CACHE_SIZE	256
typedef struct{
	U1	u1_cache[u2_DATA_CACHE_SIZE];
	U1	u1_valid;
	U4	u4_topAddr;
	U4	u4_endAddr;
}ST_DATA_CACHE;
#endif

typedef enum{
	HOKAN_MARK_INVALID = 0,
	HOKAN_MARK_VALID_NOPRI,
	HOKAN_MARK_VALID_PRI,
}EM_HOKAN_MARK;

// �⊮�^�[�Q�b�g
typedef struct{
	U4	u4_data_address;		// �f�[�^�A�h���X
	EM_HOKAN_MARK	mark;		// �}�[�N
	U2	u2_deg;					// ����
	U2	u2_dst;					// ����
	U2	u2_lmtspd;				// �������x
	U1	u1_yudo_number;			// �U���ԍ�
}ST_HOKAN_TARGET;

#if __I2C_NCYC_SEPARATE__ == 1
	#define SEM_I2C_GPSCTL SEM_I2C_2
#else
	#define SEM_I2C_GPSCTL SEM_I2C
#endif

//--------------------------------------------------------------------------
//  �ϐ���`
//--------------------------------------------------------------------------
ST_GPS_STS	stg_gpsSts;

// �O���ϐ�
ST_GPSMAP_INF		stg_gpsMapInf;					// GPS�}�b�v���
ST_GPSWRN_INF		stg_gpsWrnInf;					// GPS�x����
F1					f1g_gpsFlg;						// GPS�t���O
// �S��
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
#define	F_STS_ATCANCEL	(f1s_gpsctl_flg2.bit.b0)	// �����L�����Z���������
#define	F_STS_MYCANCEL	(f1s_gpsctl_flg2.bit.b1)	// �}�C�L�����Z���������
#define	F_MAPUPD		(f1s_gpsctl_flg2.bit.b2)	// �}�b�v�X�V�ʒm
#define	F_ATCANCEL_DEL	(f1s_gpsctl_flg2.bit.b3)	// �I�[�g�L�����Z���폜�v���t���O
#define	F_MYCANCEL_DEL	(f1s_gpsctl_flg2.bit.b4)	// �}�C�L�����Z���폜�v���t���O
#define	F_MYAREA_DEL	(f1s_gpsctl_flg2.bit.b5)	// �}�C�G���A�폜�v���t���O
#define	F_ROADJDG_NOFIX	(f1s_gpsctl_flg2.bit.b6)
#define	F_PRIPLAYSND	(f1s_gpsctl_flg2.bit.b7)	// �D��T�E���h�Đ��t���O

static F1			f1s_gpsctl_flg3;				// �t���O(�d��ON���ۑ�)
#define	F_REQ_DEFRAG	(f1s_gpsctl_flg3.bit.b0)	// �f�t���O�v���L��

static F1			f1s_gpsctl_flg4;
#define	F_TRAP_INCOMING	(f1s_gpsctl_flg4.bit.b0)	// ��������ԋ�
#define	F_CHKPNT_INCOMING	(f1s_gpsctl_flg4.bit.b1)	// ���⌗���ԋ�
#define	F_TRAPOUT_REQ	(f1s_gpsctl_flg4.bit.b2)	// ������O�o�͗v��
#define	F_CHKOUT_REQ	(f1s_gpsctl_flg4.bit.b3)	// ���⌗�O�o�͗v��

static U1			u1s_gpsctl_phase;				// GPS�����t�F�[�Y
static U2			u2s_sub_phase;					// GPS�����T�u�t�F�[�Y
static U2			u2s_mkmap_hiPri_num;			// ���D��}�b�v�쐬��
static U2			u2s_mkmap_loPri_num;			// ��D��}�b�v�쐬��

static U2			u2s_mkmap_num;					// �쐬�}�b�v�v�f��
static U2			u2s_chkmap_num;					// ����}�b�v�v�f��
static U2			u2s_oldmap_num;
static U1			u1s_parkChkAreaSts;				// ���֊Ď��G���A���
static U1			u1s_zone30AreaSts;				// �]�[��30�G���A���
static U1			u1s_shajyoAreaSts;				// �ԏ�_���G���A���
static U1			u1s_roadJdgSts;					// ���H������
static U1			u1s_carStopTmr;					// ��ԃ^�C�}
static U1			u1s_carNosokuiTmr;				// ���s�E��Ԕ񑪈ʃ^�C�}
static U1			u1s_curLmtSpd;					// ���݂̐������x�l
static U1			u1s_memLmtSpd;					// �������x���m�L���l
static U1			u1s_relaxSetOld;				// �����b�N�X�`���C���ݒ�O��l
static U1			u1s_myswEvtTmr;					// �}�CSW�C�x���g�^�C�}
static U1			u1s_dynamicTargetFace;			// ���I�^�[�Q�b�g��
static U1			ems_atCancelRegSts;				// �I�[�g�L�����Z���o�^���(EM_ATCANCEL_REGSTS)
static U1			u1s_wrt_wrk;					// �������݃��[�N
static U2			u2s_inisrch_pos;				// �����T�[�`�|�C���g
static U2			u2s_lonarea_old;				// �o�x�G���A�O��l
static U2			u2s_srch_pos;					// �T�[�`���Ă���|�C���g
static U2			u2s_high_pos;					// �T�[�`��[
static U2			u2s_low_pos;					// �T�[�`���[
static U2			u2s_srch_box;					// �T�[�`��BOX
static U2			u2s_srch_center_grp;			// �T�[�`�����O���[�v
static U2			u2s_srch_inbox_pos;				// �T�[�`BOX���ʒu
static U2			u2s_rdNoRxTmr;					// RD���M�^�C�}
static U2			u2s_scpPassTmr;					// SCP�ʉ߃^�C�}
static U2			u2s_myswTimOut;					// �}�CSW�^�C���A�E�g
static U2			u2s_carRunTmr;					// ���s�^�C�}
static U2			u2s_dynamicBoxUsedNum;			// �g�p�ς�BOX��

// �G���A�֘A
static U1			u1s_runStopSts;

// ETC�֘A
static U1			u1s_ETC_guide_sts;
static U2			u2s_ETC_guide_tmr;
static ST_GPSMAP	sts_ETC_guide_point;

// �I�[�r�X�ETRAP�EN�֘A
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

static U2			u2s_sndTgtNum;					// �����o�̓^�[�Q�b�g�ԍ�
static U4			u4s_sndTgtAddress;

static ST_GPSMAP	sts_scpPoint;
static ST_GPSMAP	*psts_trapOutMap;
static ST_GPSMAP	*psts_chkOutMap;

static U4			u4s_mapupd_lat;					// �}�b�v�X�V���ܓx
static U4			u4s_mapupd_lon;					// �}�b�v�X�V���y�x
static ST_INDEX1	sts_index1[3];					// ����INDEX1���
static ST_INDEX1	sts_index1_dynamic[3];			// ����INDEX1���(���I)
static ST_INDEX1	sts_index1_custom[3];			// ����INDEX1���(�J�X�^��)
static ST_INDEX1 	*psts_index1;					// INDEX1�|�C���^
#if __FREE_DATA_DOWNLOAD__
static ST_INDEX1	sts_index1_pay[3];				// ����INDEX1���(�L����)
#endif
static U4			u4s_srch_address;

static U4			u4s_relaxTmr;					// �����b�N�X�`���C���^�C�}

// SW�֘A
U2					u2g_failTmr;					// �t�F�[���^�C�}

// �L���[�֘A
static ST_GPSVC_QUEUE	sts_gpsVcHiPriQueue[VC_HIPRI_QUEUE_MAX];
													// ���D��{�C�X�L���[
static ST_GPSVC_QUEUE	sts_gpsVcMidPriQueue[VC_MIDPRI_QUEUE_MAX];
													// ���D��{�C�X�L���[
static ST_GPSVC_QUEUE	sts_gpsVcLoPriQueue[VC_LOPRI_QUEUE_MAX];
													// ��D��{�C�X�L���[

static ST_GPSVC_QUEUE_CTRL	sts_gpsVcQueueCtrl[3];

// �L���b�V��
#ifdef __ENABLE_DATA_CACHE__
static ST_DATA_CACHE	sts_dataCache;
#ifdef __CACHE_DEBUG__
static U2			u2s_cacheHitNum;
static U2			u2s_cacheMissHitNum;
#endif
#endif

// GPS�x�񃌃x��
static U1			u1s_gpsWarningLvl_wrk;			// ���[�N
U1					u1g_gpsWarningLvl;				// ���J�f�[�^
static U2			u2s_gpsWarningLvl_dist_wrk;		// ���[�N
U2					u2g_gpsWarningLvl_dist;			// ���J�f�[�^
U1					u1g_gpsRdScopeShow;
static U1			u1s_gpsRdScopeTmr;
// ���t��r�p
static UN_REGDAY	uns_today;

static Bool			g_InMyArea;						// �}�C�G���A����
static Bool			g_InMyCancelArea;				// �}�C�L�����Z���G���A����

static U2	s_SrcRefIdxList[GPS_VISIBLE_TGT_MAX];	// ���C���f�b�N�X���X�g
static U2	srcRefIdxListSize;						// ���C���f�b�N�X���X�g��

static U2	u2s_gpsHoldTmr;							// GPS�ێ��^�C�}

static U1	u1s_max_rxperc_for_icancel;

static U1	u1s_gpsTgtVcSts;
static U1	u1s_kenkyo_mem;

ST_GPSVC_QUEUE		sts_lastRequested_queue;

static S4	gps_distToHighway;

static float k_lon2_meter;

static U4	u4s_live_counter;

static ST_GPSMAP *psts_rd_bp2_pri_map;				// RD�ŗD��
static ST_GPSMAP *psts_sc_bp2_pri_map;				// SC�ŗD��

static Bool is_orbisCountDown;

static U2 u2s_vcsts_failsafe_tmr;

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

static void CustomDataChk(void);					// �J�X�^���f�[�^�`�F�b�N

static U1	u1_ChkGpsMapUpd(void);					// MAP�X�V���菈��
static void	UpdLatLonArea(void);					// �ܓx�E�o�x�͈͍X�V����
static void	UpdIndex1(void);						// INDEX1�X�V����
static U1	u1_BlkSrch(EM_TGT_DATA_TYPE);			// �u���b�N����
static U1	u1_PntSrch(U1, EM_TGT_DATA_TYPE);		// �|�C���g�T�[�`
static U1	u1_ReadGpsData(U4, EM_TGT_DATA_TYPE);	// GPS�f�[�^�ǂݏo������
static U1	u1_RegMap(void);						// MAP�o�^����
static Bool Can_data_pre_expire(ST_GPSROM *);		// ���O�f�[�^���p���菈��
static void	RomEnScrmble(U1, U4, U1 *);				// �X�N�����u������
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
static void	RoadJudge(void);						// ���H���菈��
static U1	u1_TransitRoadJudge(void);
static U1	u1_ChkDegRng(EM_TGTDEG, U2);			// �p�x�͈͔��菈��
static U1	u1_DegAIsOk(ST_GPSMAP *);				// ��A�͈͔��菈��
static U1	u1_DegAIsOk_Variable(ST_GPSMAP *psth_map, U1 u1h_deg);
													// ��A�͈͔��菈��(��)
static U1	u1_DegAWideIsOk(ST_GPSMAP *);			// ��A�͈͔��菈��(�L�͈�)
static U1	u1_DegBIsOk(EM_TGTDEG, U2);				// ��B�͈͔��菈��
static U1	u1_GpsOrbisVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// GPS�I�[�r�X�{�C�X�v��
static U1	u1_GpsZoneVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// �]�[���{�C�X�v��
static U1	u1_Gps1kmContVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// 1km�^�C�v�R���e���c�{�C�X�v��
static U1	u1_Gps500mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 500m�^�C�v�R���e���c�{�C�X�v��
static U1	u1_Gps300mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 300m�^�C�v�R���e���c�{�C�X�v��
static U1	u1_Gps100mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 100m�^�C�v�R���e���c�{�C�X�v��
static U1	u1_GpsScpContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// ���x�ؑփR���e���c�{�C�X�v��
static U1	u1_GpsAreaVcReq(EM_VCGPS_TYPE, ST_VCGPS *);
													// �G���A�{�C�X�v��
static U1	SubVcReqDir(S2 , ST_VCGPS *);			// �T�u�֐��F�{�C�X��������
static U1	SubVcReqUpper1000m(U2, ST_VCGPS *);		// �T�u�֐��F�{�C�X1000m�ȏ㋗������
static U1	SubVcReqUpper500m(U2, ST_VCGPS *);		// �T�u�֐��F�{�C�X500m�ȏ㋗������
static void	SubVcReqUnder500m(U2, ST_VCGPS *);		// �T�u�֐��F�{�C�X500m������������
static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus);
static void	ChkFocusTgt_2nd(ST_FOCUS_TGT *psth_2nd_focus_tgt, ST_GPSMAP *psth_map, U2 u2t_dstSub, U2 u2t_absDegB, U1 u1t_focus);

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt);

static void ChkRevOrbisArea(ST_GPSMAP *);			// ���ΎԐ��I�[�r�X���菈��
static U1	IsJudgePOISet(U1);						// POI�ݒ蔻�菈��
static U1	u1_JudgePOISet(ST_GPSMAP *);			// 
static U1	u1_JudgePOISetTunnel(ST_GPSMAP *);		// 

static void	RoadJdgRoutine(void);					// ���H����������

static void	AddVisibleIdxLst(U2);					// ���C���f�b�N�X���X�g�ǉ�����
static void	SortVisibleIdxLst(U2);					// ���C���f�b�N�X���X�g�\�[�g����
static void	SortVisibleIdxLst_2nd(U2 u2h_secondIdx);

static void sortSrcRefIdxByDistance(void);			// �C���f�b�N�X���X�g�\�[�g�����\�[�g����

//static void	CalTunnelInRemDist(void);				// �g���l�����c�����Z�o����
static void	ChkZoneGpsWrnLvl(U1);					// �]�[��GPS�x�񃌃x�����菈��

static void	GuardPictNumber(ST_GPSMAP *);
static Bool	UpdateSoundTarget(U2 *pu2h_sndTgtNum, U4 *pu4h_orgAddress, const ST_GPSMAP *psth_map, U2 u2h_mapNum); 

static void update_gps_warning_lvl(EM_SYS_WARNING_LVL src_lvl, U2 dist);

static void update_ahead_hys(ST_GPSMAP *psth_map);
static void update_distance(ST_GPSMAP *psth_map);

static void TransitAreaStatusForRd(ST_GPSMAP *psth_map);
static void TransitAreaStatusForSc(ST_GPSMAP *psth_map, U1 degChk);

static Bool isCustomDateValid(ST_HEADER *header);

S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2);

void	GpsVcEnQueue(EM_VC u1h_voice, EM_VCGPS_TYPE emh_vcType, U4 u4h_addr);
void	VcReq(EM_VC u1h_req){
}
U1	u1_ChkDegValid(void){
	return	(U1)TRUE;
}
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
	TransitOrbis,							// RD���I�[�r�X				1
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
	TransitZone30Type,						// �]�[��30					58
};

// ���֏�ԕϊ�
static const U1	u1_TBL_CONV_PCHKSTS[] = {
	PCHK_AREA_OUT,
	PCHK_AREA_SZ,
	PCHK_AREA_SZ,
	PCHK_AREA_Z,
	PCHK_AREA_Z,
};
// �]�[��30��ԕϊ�
static const U1	u1_TBL_CONV_ZONE30STS[] = {
	ZONE30_AREA_OUT,
	ZONE30_AREA_IN,
	ZONE30_AREA_IN,
};

// �L���[�f�t�H���g�l
static const ST_GPSVC_QUEUE_CTRL	st_TBL_gpsVcQueueDef[] = {

	{	&sts_gpsVcHiPriQueue[0],	VC_HIPRI_QUEUE_MAX,		VC_QUEUE_EMPTY,		0,		0,		},
	{	&sts_gpsVcMidPriQueue[0],	VC_MIDPRI_QUEUE_MAX,	VC_QUEUE_EMPTY,		0,		0,		},
	{	&sts_gpsVcLoPriQueue[0],	VC_LOPRI_QUEUE_MAX,		VC_QUEUE_EMPTY,		0,		0,		},
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
//�y�֐��zGPS�}�b�v�X�V���菈��												
//�y�@�\�z�O��}�b�v�X�V�����ʒu����̈ړ�������MAP�X�V�𔻒肷��			
//�y�ߒl�zTRUE�F�X�V�v		FALSE�F�X�V�s�v									
//--------------------------------------------------------------------------
static U1	u1_ChkGpsMapUpd(void){

	U4	u4t_latdiff;								// �ܓx����
	U4	u4t_londiff;								// �o�x����
	U1	u1t_judge = FALSE;

	if(0){//vpol_flg1(EVTFLG_REQ_MAPUPD, TWF_CLR) == E_OK){
		u1t_judge = TRUE;
		u2s_lonarea_old = U2_MAX;					// INDEX�����X�V
	}
	else{
		// �ŏI�}�b�v�X�V�ʒu����̋����𑪒�
		// �ܓx������Βl����
		u4t_latdiff = ABS_SUB(sts_gpsinf.lat, u4s_mapupd_lat);

		// �o�x������Βl����
		u4t_londiff = ABS_SUB(sts_gpsinf.lon, u4s_mapupd_lon);

		// �ړ��ʂ��K�苗�����z���Ă��� or �X�V�v������ �Ȃ�}�b�v���X�V������
		if((u4t_latdiff >= u4_TBL_LATDIST[DISTRNG_200M][sts_gpsinf.u1_latarea])
		|| (u4t_londiff >= u4_TBL_LONDIST[DISTRNG_200M][sts_gpsinf.u1_latarea])){
			u1t_judge = TRUE;
		}
	}

	if(u1t_judge){
		// �X�V�ʒu���L��
		u4s_mapupd_lat = sts_gpsinf.lat;
		u4s_mapupd_lon = sts_gpsinf.lon;
	}

	// �ړ��ʂ����Ȃ���΍X�V�s�v

	return	u1t_judge;
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

		memcpy(&pstt_mkmap->un_extra.byte[0], &sts_gpsrom.un_extra.byte[0], EXTRA_DATA_SIZE);
													// EXTRA�ۑ�
		// �o�^����
	}
	// �͈͖͂����ł��o�^��OK�ɂ���(�����T�[�`�̏ꍇ�̓}�b�v���͂Ȃ��Ă��悢)

	return	REGOK;

}

static Bool	Can_data_pre_expire(ST_GPSROM *psth_gpsrom){

	Bool	ret = FALSE;
	// ���W�x���������̂̓}�b�v�ɍڂ��鎞�_�Őݒ�Ŏ̂Ă�
	// ���؁A�ꎞ��~�A���O�g�C���A���
	
	// ���U���͎̂Ă�
	
	switch(psth_gpsrom->u1_type){
	case TGT_RAILROAD_CROSSING:
		if(!stg_setGps[u1g_setAcsSel].b_fumikiri){
			ret = TRUE;
		}
		break;
	case TGT_TMPSTOP:
		if(!stg_setGps[u1g_setAcsSel].b_tmpstop){
			ret = TRUE;
		}
		break;

	case TGT_NOFIX_YUDO:
		if(psth_gpsrom->un_extra.yudo.u1_yudo_type != YUDO_TYPE_COMMON){
			ret = TRUE;
		}
		break;

	case TGT_TOILET:
		if(!stg_setGps[u1g_setAcsSel].b_toilet){
			ret = TRUE;
		}
		break;
	case TGT_KOBAN:
		if(!stg_setGps[u1g_setAcsSel].b_koban){
			ret = TRUE;
		}
		break;

	case TGT_FIRE:
		if(!stg_setGps[u1g_setAcsSel].b_fire){
			ret = TRUE;
		}
		break;

	case TGT_HOIKU:
		if(!stg_setGps[u1g_setAcsSel].b_hoiku){
			ret = TRUE;
		}
		break;
	}
	
	return ret;
}

//--------------------------------------------------------------------------
//�y�֐��zGPS�ܓx�E�o�x���X�N�����u������									
//�y�@�\�z�I�t�Z�b�g�l�ɕϊ����Ċi�[����									
//�y���l�z																	
//--------------------------------------------------------------------------
static void	RomEnScrmble(U1 u1h_type, U4 u4h_input, U1 *pu1h_output){

	if(u1h_type == TYPE_LAT){						// �ܓx
		u4h_input -= u4_LAT_BASE_MIN;
	}
	else{											// �o�x
		u4h_input -= u4_LON_BASE_MIN;
	}

	// 1��2��3�̏��Ŋi�[
	pu1h_output[0] = (U1)(u4h_input & (U4)0x000000FF);
	pu1h_output[1] = (U1)((u4h_input & (U4)0x0000FF00) >> 8);
	pu1h_output[2] = (U1)((u4h_input & (U4)0x00FF0000) >> 16);
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
			psts_chkmap->u2_countdown_dist = U2_MAX;			// �J�E���g�_�E���N���A
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
																	// 500m�x�񉹗v��
					// �_�C���N�g��300m�ɔ�э��񂾂Ƃ��A�ʉߑ��x�͍��m���Ȃ��i�Ԃɍ���Ȃ��j
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_600M;		// 600m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M)	{	// 1100m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
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
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_2KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------2100m�x����----------------------------------------------------//
			case STS_ORBIS_2100M:									// 2100m�x����
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// �p�xOK and 1100m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------1100m�x����----------------------------------------------------//
			case STS_ORBIS_1100M:									// 1100m�x����
				if(u1t_degChk == OK){								// �p�xOK
					if(psts_chkmap->u2_dst <= u2_DIST_330M){		// 330m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_ORBIS_300M;	// 300m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
						// �_�C���N�g��300m�ɔ�э��񂾂Ƃ��A�ʉߑ��x�͍��m���Ȃ��i�Ԃɍ���Ȃ��j
					}
				 	else if(psts_chkmap->u2_dst <= u2_DIST_600M){	// 600m�͈͓��Ȃ�
						psts_chkmap->u1_wrnSts = STS_ORBIS_600M;	// 600m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------600m�x����----------------------------------------------------//
			case STS_ORBIS_600M:									// 600m�x����
				if(psts_chkmap->u2_dst <= u2t_passDst){				// �ʉߔ��苗�����H
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m�x���Ԃ�
					if((u1_ChkDegRng(TYPE_TGTDEG_EXIST_DEGA_WIDE, psts_chkmap->u2_dst)== OK)
																	// �ʉߊp�xOK
					&& (psts_chkmap->un_type.bit.b_tunnel == OFF)	// �g���l���ȊO
					&& (stg_setGps[u1g_setAcsSel].b_orbisPass)){	// �I�[�r�X�ʉߍ��mON�Ȃ�
						if(u1s_gpsTgtVcSts == GPSTGT_VC_STS_OUTOK){
							VcReq(VC_ORBIS_PASS);					// �ʉߍ��m
							F_PRIPLAYSND = TRUE;
							u2s_sndTgtNum = u2s_sub_phase;
							u4s_sndTgtAddress = psts_chkmap->u4_dataAddr;
						}
					}
				}
				else if((psts_chkmap->u2_dst <= u2_DIST_330M) && (u1t_degChk == OK)){
																	// 330m�͈͓� and �p�xOK 
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m�x���Ԃ�
					if((psts_chkmap->un_type.bit.b_tunnel == OFF)	// �g���l���ȊO
					&& (stg_setGps[u1g_setAcsSel].b_orbisPassSpd)	// �I�[�r�X�ʉߑ��x���mON�Ȃ�
					&& (psts_chkmap->u2_dst >= u2_DIST_100M)){
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_PASSSPD, psts_chkmap->u4_dataAddr);
																	// �ʉߑ��x���m
					}
				}
				break;
		//-------300m�x����----------------------------------------------------//
			case STS_ORBIS_300M:									// 300m�x����
				if(psts_chkmap->u2_dst <= u2t_passDst){				// �ʉߔ��苗�����H
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m�x���Ԃ�
					if((u1_ChkDegRng(TYPE_TGTDEG_EXIST_DEGA_WIDE, psts_chkmap->u2_dst)== OK)
																	// �ʉߊp�xOK
					&& (psts_chkmap->un_type.bit.b_tunnel == OFF)	// �g���l���ȊO
					&& (stg_setGps[u1g_setAcsSel].b_orbisPass)){	// �I�[�r�X�ʉߍ��mON�Ȃ�
						if(u1s_gpsTgtVcSts == GPSTGT_VC_STS_OUTOK){
							VcReq(VC_ORBIS_PASS);				// �ʉߍ��m
							F_PRIPLAYSND = TRUE;
							u2s_sndTgtNum = u2s_sub_phase;
							u4s_sndTgtAddress = psts_chkmap->u4_dataAddr;
						}
					}
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
			//	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
			else{													// ��ʃI�[�r�X
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
			//	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
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
//�y�֐��z���ΎԐ��I�[�r�X�L�����Z���G���A���菈��							
//�y�@�\�z���ΎԐ��I�[�r�X�L�����Z���G���A�����E�ے�𔻒肷��				
//�y�����zpsth_map�F���肷��}�b�v�|�C���g�ւ̃|�C���^						
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	ChkRevOrbisArea(ST_GPSMAP *psth_map){

	S2	s2t_degDiff;										// �p�x��

	// �^�[�Q�b�g�p�x�Ǝ��ԕ��ʊp�Ƃ̍����Z�o
	s2t_degDiff = s2_CalDegSub(psth_map->u2_tgtDeg, sts_gpsinf.u2_deg);
	if(s2t_degDiff < 0){
		s2t_degDiff *= (S2)-1;								// ��Βl�ɂ���
	}
	if(s2t_degDiff <= 90*DEG_LSB){							// 90���ȓ�(�Ό����Ă��Ȃ���)
		if((psth_map->u2_dst <= u2_DIST_600M)				// �^�[�Q�b�g���S����600m�ȓ� or
		|| ((psth_map->u2_dst <= u2_DIST_1100M) && (u1_DegAIsOk(psth_map) == TRUE))){
															// �^�[�Q�b�g���ʂ�1100m��͈͓��ɂ��� ?
			F_RORBIS_CAN = ON;								// �L�����Z���t���O�Z�b�g
		}
	}
	else{													// 90������(�Ό����Ă��鑤)
		if((psth_map->u2_dst <= u2_DIST_1100M) && (u1_DegAIsOk(psth_map) == TRUE)){
															// �^�[�Q�b�g���S����1100m�ȓ� ?
			F_RORBIS_NOCAN = ON;							// �L�����Z���ے�t���O�Z�b�g
		}
	}
}
#if 0
//--------------------------------------------------------------------------//
//�y�֐��z�g���l�����c�����Z�o����											//
//�y�@�\�z�g���l���������f�[�^�̎c�������Z�o����							//
//�y�����z�Ȃ�																//
//�y�ߒl�z�Ȃ�																//
//�y���l�z���b��GPS�ʒu�f�[�^���X�V����邲�ƂɁA����(u2_dst)�͉��z�_�����	//
//        �����������Ă���B												//
//        ���^�[�Q�b�g�̋����\�[�g�I���サ���s���Ă͂����Ȃ�				//
//--------------------------------------------------------------------------//
static void	CalTunnelInRemDist(void){

	int	i;
	ST_GPSMAP	*pstt_map;

	// ���^�[�Q�b�g�̒�����Ώۂ����邩�������A��������v�Z����B
	if(srcRefIdxListSize != 0){
		for(i=0 ; i<srcRefIdxListSize ; i++){
			pstt_map = &sts_gpsmap[s_SrcRefIdxList[i]];
			if(pstt_map->un_type.bit.b_tunnel == TUNNEL_IN){
													// �g���l��������
				pstt_map->u2_dst = GRDADDU2(pstt_map->u2_dst, ((U2)pstt_map->un_extra.common.u1_tunnelRem*(U2)100));
			}
		}
	}
}
#endif
//--------------------------------------------------------------------------
//�y�֐��z�]�[���x��J�ڏ���												
//�y�@�\�z�]�[���x��𔻒肷��												
//�y�����z���ʗL��															
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitZone(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U1	u1t_thisTargetOut = OFF;
	Bool	isTrapZone = TRUE;

	if(psts_chkmap->un_type.bit.b_tunnel != NOT_TUNNEL){// �g���l��������
		return;											// �����Ȃ��E�x�񂵂Ȃ�
	}

	switch(psts_chkmap->un_type.bit.b_code){
	case TGT_CHKPNT_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
		isTrapZone = FALSE;
		break;
	default:
		break;
	}

	if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){
		switch(psts_chkmap->u1_wrnSts){
	//-------��x����--------------------------------------------------------//
		case STS_ZONE_NOALARM:
			if((u1t_degChk == OK)								// �p�xOK�H
			&& (psts_chkmap->u2_dst <= u2_DIST_100M)){			// 100m�͈͓�?
				psts_chkmap->u1_wrnSts = STS_ZONE_100M;			// 100m�x���Ԃ�
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_100M, psts_chkmap->u4_dataAddr);
			}
			break;
		case STS_ZONE_100M:
			if(psts_chkmap->u2_dst > u2_DIST_600M){				// ��������?
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
			}
			break;
		}

		// �]�[���ł̃t�H�[�J�X�^�[�Q�b�g����
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
	}
	else{
		// �x���ԑJ��
		switch(psts_chkmap->u1_wrnSts){
	//-------��x����--------------------------------------------------------//
		case STS_ZONE_NOALARM:
			// ��͈͓��Ń^�[�Q�b�g�Ɍ������ƌx��J�n
			if(u1t_degChk == OK){								// �p�xOK�H
				if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m�͈͓�?
					if(psts_chkmap->u2_dst <= u2_DIST_500M){	// 500m�͈͓��Ȃ�
						psts_chkmap->u1_wrnSts = STS_ZONE_500M;	// 500m�x���Ԃ�
					}
					else{
						psts_chkmap->u1_wrnSts = STS_ZONE_600M;	// 600m�x���Ԃ�
					}
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
				}
				else{
					if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_ZONE_1100M;// 1100m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_1KM, psts_chkmap->u4_dataAddr);
					}
				}
			}
			break;
	//-------1100m�x����-----------------------------------------------------//
		case STS_ZONE_1100M:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O�H
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
			}
			else if((u1t_degChk == OK)							// �p�xOK and
				 && (psts_chkmap->u2_dst <= u2_DIST_600M)){		// 600m�͈͓��Ȃ�
				if(psts_chkmap->u2_dst <= u2_DIST_500M){		// 500m�͈͓��Ȃ�
					psts_chkmap->u1_wrnSts = STS_ZONE_500M;		// 500m�x���Ԃ�
				}
				else{
					psts_chkmap->u1_wrnSts = STS_ZONE_600M;		// 600m�x���Ԃ�
				}
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
			}
			break;
	//-------600m�x����------------------------------------------------------//
		case STS_ZONE_600M:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O�H
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
			}
			// �����500m�����ɓ��������m�F����
			else if((u1t_degChk == OK)							// �p�xOK and
				 && (psts_chkmap->u2_dst <= u2_DIST_500M)){		// 500m�͈͓��Ȃ�
				psts_chkmap->u1_wrnSts = STS_ZONE_500M;			// 500m�x���Ԃ�
			}
			break;
	//-------500m�x����------------------------------------------------------//
		case STS_ZONE_500M:
			if(psts_chkmap->u2_dst > u2_DIST_500M){				// 500m�͈͊O�H
				psts_chkmap->u1_wrnSts = STS_ZONE_OUTWT_FAR;	// ���S���E�҂���Ԃ�
				if(isTrapZone){
					F_TRAPOUT_REQ = ON;
					psts_trapOutMap = psts_chkmap;				// �|�C���^�ۑ�
				}
				else{
					F_CHKOUT_REQ = ON;
					psts_chkOutMap = psts_chkmap;				// �|�C���^�ۑ�
				}
				u1t_thisTargetOut = ON;
			}
			break;
	//-------�G���A�����S���E�҂����-----------------------------------------//
		case STS_ZONE_OUTWT_FAR:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
			}
			break;
		}

		// �]�[���ł̃t�H�[�J�X�^�[�Q�b�g����
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
	}

	// 	�C���f�b�N�X���X�g�ǉ�
	AddVisibleIdxLst(u2s_sub_phase);

	// �g���b�v�]�[�����ł̓��ʏ���
	if(isTrapZone){
		// �ꎞ��~�F�L�����Z��NG�͈͂ɂ��Ȃ��ARD���x�㏸�s�v�A���O�s�v
		// �����_�F�L�����Z��NG�͈͂ɂ��Ȃ��ARD���x�㏸�s�v

		if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_TEMPORARY_STOP){
																// �ꎞ��~�ȊO�̂Ƃ�
			if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_SIGNAL){
																// ����Ɍ����_�ȊO�̂Ƃ�
				// �L�����Z��NG����
			 	if(psts_chkmap->u2_dst <= u2_DIST_600M){
					F_CANCEL_NG = ON;							// �L�����Z��NG����
				}
				// RD���x�㏸�p����
				if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){	// ���a500m���� ?
					F_TRAPZONE_IN = ON;
				}
			}
			// ���̃]�[���ɓ����Ă��Ă��邩�ǂ����̔���
			if((u1t_thisTargetOut == OFF)						// ���̃^�[�Q�b�g�������O�v�����Ă��Ȃ�
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)		// 
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)	// 
			&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// and 1100m����
				F_TRAP_INCOMING = ON;
			}
		}
	}
	else{														// ����G���A
		if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){			// ���a500m���� ?
			F_CHKPNT_IN = ON;
		}

		if((u1t_thisTargetOut == OFF)							// ���̃^�[�Q�b�g�������O�v�����Ă��Ȃ�
		&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)			// 
		&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)		// 
		&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){				// and 1100m����
			F_CHKPNT_INCOMING = ON;
		}
	}

	// �G���A��ԑJ��
	TransitAreaStatusForRd(psts_chkmap);
	TransitAreaStatusForSc(psts_chkmap, u1t_degChk);

	ChkZoneGpsWrnLvl(u1t_degChk);								// �]�[��GPS�x�񃌃x������
}

//--------------------------------------------------------------------------
//�y�֐��z�]�[��GPS�x�񃌃x�����菈��										
//�y�@�\�z�]�[���̌x�񃌃x���𔻒肷��										
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	ChkZoneGpsWrnLvl(U1 u1h_degChk){

	// GPS�x�񃌃x���̔���
	if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){

		if(u1h_degChk == OK){								// ���S�Ɍ������Ă���
			if(psts_chkmap->u2_dst <= u2_DIST_100M)			// 100m�ȓ�
			{
				update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
			}
			else if(psts_chkmap->u2_dst <= u2_DIST_300M)	// 300m�ȓ�
			{
				update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
			}
		}
	}
	else{
		// YELLOW�x�񃌃x���̔���
		if((psts_chkmap->u2_dst <= u2_DIST_600M)			// 600m�ȓ���
		&& (u1h_degChk == OK)){								// ���S�Ɍ������Ă���
			update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
		}
		// MID�̔���
		else if((psts_chkmap->u1_wrnSts == STS_ZONE_500M)	// ���O�҂���
		|| ((psts_chkmap->u2_dst <= u2_DIST_1100M) && (u1h_degChk == OK))){
															// 1100m�ȓ��Œ��S�Ɍ������Ă���
			update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
		}
	}

}

//--------------------------------------------------------------------------
//�y�֐��z�}�C�G���A�x��J�ڏ���											
//�y�@�\�z�}�C�G���A���m�𔻒肷��											
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z1500m�����ōČx��\												
//--------------------------------------------------------------------------
static void TransitMyArea(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//�y�֐��z���[�U�[�s����ԑJ�ڏ���											
//�y�@�\�z�t�H�[�J�X�𔻒肷��												
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//--------------------------------------------------------------------------
static void TransitPin(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtDeg, psts_chkmap->u2_dst);

	// �s���̃t�H�[�J�X�^�[�Q�b�g����
	ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);

	// �C���f�b�N�X���X�g�ǉ�
	AddVisibleIdxLst(u2s_sub_phase);
}

//--------------------------------------------------------------------------
//�y�֐��z�}�C�L�����Z������												
//�y�@�\�z�蓮�o�^��I�L�����Z���x���ԑJ�ڂ��s���A�������肷��				
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�zF_MYCANCEL_IN�͌x�񔻒�t�F�[�Y�J�n���ɏ������ς�					
//--------------------------------------------------------------------------
static void TransitMyCancel(EM_TGTDEG emh_tgtDeg){
}
//--------------------------------------------------------------------------
//�y�֐��z�I�[�g�L�����Z���J�ڏ���											
//�y�@�\�z�����o�^��I�L�����Z���x���ԑJ�ڂ��s���A��������э폜���肷��	
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�zF_ATCANCEL_IN�͌x�񔻒�t�F�[�Y�J�n���ɏ������ς�					
//		  ���M�����[��AAC�֎~�ł���M����H									
//		  �\�[���[��AAC�֎~�Ȃ��M���Ȃ��̂Ŏ�M�Ȃ��ʉߔ��肵�Ȃ�			
//--------------------------------------------------------------------------
static void TransitAtCancel(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//�y�֐��z1km�^�C�v�R���e���c����											
//�y�@�\�z1km�R���e���c���m��ԑJ�ڂ��s��									
//�y�����zemh_tgtdeg : �^�[�Q�b�g���ʗL��									
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitCont1kmType(EM_TGTDEG emh_tgtdeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtdeg, psts_chkmap->u2_dst);

	// ��ԑJ��

	if(psts_chkmap->u2_dst > u2_DIST_1500M){					// 1500m�͈͊O ?
		psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;			// ��x���Ԃ�
	}
	else{
		switch(psts_chkmap->u1_wrnSts){
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_NOALARM:								// ��x����
			if(u1t_degChk == OK){								// �p�xOK
				if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m�͈͓� ?
					psts_chkmap->u1_wrnSts = STS_1KMCONT_600M;	// 600m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_500M, psts_chkmap->u4_dataAddr);
				}
				else{
					if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_1KMCONT_1100M;
																// 1100m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_1KM, psts_chkmap->u4_dataAddr);
					}
				}
			}
			break;
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_1100M:									// 1100m�x����
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;	// ��x���Ԃ�
			}
			else if((u1t_degChk == OK) && (psts_chkmap->u2_dst <= u2_DIST_600M)){
																// 600m�͈͓� and �p�xOK ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_600M;		// 600m�x���Ԃ�
				GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_500M, psts_chkmap->u4_dataAddr);
			}
			break;
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_600M:									// 600m�x����
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;	// ��x���Ԃ�
			}
			break;
		}

		// 1km�R���e���c�̍ŐڋߑΌ��^�[�Q�b�g����
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		}
	}

	// �C���f�b�N�X���X�g�ǉ�
	if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
		AddVisibleIdxLst(u2s_sub_phase);
	}
}

//--------------------------------------------------------------------------
//�y�֐��z�����V���b�g�^�C�v�R���e���c����									
//�y�@�\�z�����V���b�g�R���e���c���m��ԑJ�ڂ��s��							
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitContOneShotType(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtDeg, psts_chkmap->u2_dst);
	U2	u2t_inDst;
	U2	u2t_outDst;
	EM_VCGPS_TYPE	u1t_vcGps = VCGPS_NONE;
	U1	u1t_reqFocus = TRUE;					// �t�H�[�J�X�v�ŏ�����
	U1  u1t_wrnFocus = FALSE;
	U2	u2t_focusNearDst = 0;					// �ߐڃt�H�[�J�X����Ȃ��ŏ�����
	U1	u1t_visible = TRUE;						// ���^�[�Q�b�g���ǂ���

	switch(psts_chkmap->un_type.bit.b_code){
	// 1100m�O���[�v
	case TGT_SA:
	case TGT_PA:
	case TGT_HWOASYS:
		u2t_inDst = u2_DIST_1100M;
		u2t_outDst = u2_DIST_1600M;
		u1t_vcGps = VCGPS_1KMCONT_1KM;
		u2t_focusNearDst = u2_DIST_1100M;		// 1100m���ߐ�
		break;
	// 600m�O���[�v
	case TGT_CURVE:								// �}�J�[�u
		u2t_inDst = u2_DIST_600M;
		u2t_outDst = u2_DIST_1100M;
		u1t_vcGps = VCGPS_500MCONT;
		u1t_reqFocus = FALSE;					// �t�H�[�J�X�s�v
		u1t_visible = FALSE;					// �����Ȃ�
		break;
	case TGT_POLICE:							// �x�@��
		if(psts_chkmap->un_extra.police.b_policeType != POLICE_TYPE_STATION)
		{
			u1t_wrnFocus = TRUE;
		}
		// fall
	case TGT_KOBAN:								// ���
		u2t_inDst = u2_DIST_600M;
		u2t_outDst = u2_DIST_1100M;
		u1t_vcGps = VCGPS_500MCONT;
		u2t_focusNearDst = u2_DIST_600M;		// 600m���ߐ�
		break;
	// 300m�O���[�v
	case TGT_CROSSING:							// �����_�Ď��|�C���g
	case TGT_SIGNAL:							// �M�������}�~�V�X�e��
		u1t_wrnFocus = TRUE;
		// fall 
	case TGT_NSYS:								// N�V�X�e��
	case TGT_TRFCHK:							// ��ʊĎ��V�X�e��
	case TGT_ACCIDENT:							// ���̑����G���A
		u2t_inDst = u2_DIST_300M;
		u2t_outDst = u2_DIST_800M;
		u1t_vcGps = VCGPS_300MCONT;
		u2t_focusNearDst = u2_DIST_300M;		// 300m���ߐ�
		break;
	// 200m�O���[�v
	case TGT_BRAJCT:							// ����E����
		u2t_inDst = u2_DIST_200M;
		u2t_outDst = u2_DIST_700M;
		u1t_reqFocus = FALSE;					// �t�H�[�J�X�s�v
		u1t_visible = FALSE;					// �����Ȃ�
		u1t_vcGps = VCGPS_300MCONT;
		break;
	// 100m�O���[�v
	case TGT_KENKYO:							// ����
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// �t�H�[�J�X�s�v
		u1t_visible = FALSE;					// �����Ȃ�
		u1t_vcGps = VCGPS_100MCONT;
		break;

	case TGT_PARKING:							// ���ԏ�
	case TGT_RAILROAD_CROSSING:					// ����
	case TGT_TOILET:							// ���O�g�C��
	case TGT_FIRE:								// ���h��
	case TGT_HOIKU:								// �ۈ牀
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// �t�H�[�J�X�s�v
		u1t_vcGps = VCGPS_NONE;					// �����s�v
		break;

	case TGT_TMPSTOP:							// �ꎞ��~���Ӄ|�C���g
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// �t�H�[�J�X�s�v
		u1t_vcGps = VCGPS_NONE;					// �����s�v
		// �p�x���茋�ʂ������茋�ʂɂ���
		u1t_visible = u1t_degChk;
		break;

	case TGT_HWRADIO:							// �n�C�E�F�C���W�I
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_vcGps = VCGPS_100MCONT;
		u2t_focusNearDst = u2_DIST_100M;		// 100m���ߐ�
		break;
	}

	// ��ԑJ��
	if(psts_chkmap->u1_wrnSts == STS_ONESHOTCONT_NOALARM){
		if((psts_chkmap->u2_dst <= u2t_inDst)				// �w��͈͓�
		&& (u1t_degChk == OK)){								// �^�[�Q�b�g�Ό�
			psts_chkmap->u1_wrnSts = STS_ONESHOTCONT_ALARM;	// �x���Ԃ�
			if(u1t_vcGps != VCGPS_NONE){					// �����o�͗v
				switch(psts_chkmap->un_type.bit.b_code){
				case TGT_KENKYO:
					sts_gpsmap[KENKYO_VIRTUAL_INDEX] = *psts_chkmap;
															// ��x�S���R�s�[
					sts_gpsmap[KENKYO_VIRTUAL_INDEX].u4_dataAddr = KENKYO_VIRTUAL_ADDRESS;
															// �A�h���X�����z�^�[�Q�b�g�ɂ���
					sts_gpsmap[KENKYO_VIRTUAL_INDEX].un_extra.common.u2_mapPctNum = 0;
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, KENKYO_VIRTUAL_ADDRESS);
					break;
				case TGT_CURVE:
					sts_gpsmap[CURVE_VIRTUAL_INDEX] = *psts_chkmap;
															// ��x�S���R�s�[
					sts_gpsmap[CURVE_VIRTUAL_INDEX].u4_dataAddr = CV_VIRTUAL_ADDRESS;
															// �A�h���X�����z�^�[�Q�b�g�ɂ���
					sts_gpsmap[CURVE_VIRTUAL_INDEX].un_extra.common.u2_mapPctNum = 0;
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, CV_VIRTUAL_ADDRESS);
					break;
				default:
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, psts_chkmap->u4_dataAddr);
					break;
				}
			}
		}
	}
	else{
		if(psts_chkmap->u2_dst > u2t_outDst){				// �͈͊O�ɏo���H
			psts_chkmap->u1_wrnSts = STS_ONESHOTCONT_NOALARM;
															// ��x���Ԃ�
		}
	}
	if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
		// �R���e���c�̃t�H�[�J�X�^�[�Q�b�g����
		if(u1t_reqFocus){
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2t_focusNearDst, u2_DIST_1100M, u1t_degChk, FALSE);
			if(u1t_wrnFocus)
			{
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2t_focusNearDst, u2_DIST_1100M, u1t_degChk, FALSE);
				// �x�񃌃x������
				if(u1t_degChk)
				{
					U2	near_dist;
					U2	far_dist;
					if(psts_chkmap->un_type.bit.b_code == TGT_POLICE)
					{
						// �����x�@
						near_dist = 600;
						far_dist = 1100;
					}
					else
					{
						near_dist = 300;
						far_dist = 600;
					}
					if(psts_chkmap->u2_dst <= near_dist)
					{
						update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
					}
					else if(psts_chkmap->u2_dst <= far_dist)
					{
						update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
					}
				}
			}
		}
		// ���^�[�Q�b�g�̃C���f�b�N�X���X�g�ǉ�
		if(u1t_visible){
			AddVisibleIdxLst(u2s_sub_phase);
		}
	}
}
//--------------------------------------------------------------------------
//�y�֐��z���x�ؑփR���e���c����											
//�y�@�\�z���x�ؑփR���e���c�̒ʉ߂𔻕ʂ���								
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z�s���^�[�Q�b�g(�������̉��^�[�Q�b�g�͉��z�^�[�Q�b�g)			
//--------------------------------------------------------------------------

static void	TransitContScp(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//�y�֐��z�G���A�^�C�v�R���e���c����										
//�y�@�\�z�G���A�^�C�v�R���e���c���m��ԑJ�ڂ��s��							
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z�s���^�[�Q�b�g(�������̉��^�[�Q�b�g�͉��z�^�[�Q�b�g)			
//--------------------------------------------------------------------------
static void	TransitContAreaType(EM_TGTDEG emh_tgtDeg){

	U2	u2t_inDst;

	// ��ԑJ��
	if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_SZ){
															// �ŏd�_�G���A�̂Ƃ�
		u2t_inDst = u2_DIST_500M;							// �͈͓�����
	}
	else{
		u2t_inDst = u2_TBL_AREA_ROUND[psts_chkmap->un_extra.no_parking.b_areaRound];
	}
	
	if(psts_chkmap->u1_wrnSts == STS_AREACONT_OUTAREA){		// �G���A�O���
		if(psts_chkmap->u2_dst <= u2t_inDst){				// �͈͓��N��?
			psts_chkmap->u1_wrnSts = STS_AREACONT_INAREA;	// �G���A����Ԃ�
		}
	}
	else{													// �G���A�����
		if(psts_chkmap->u2_dst > (u2t_inDst + u2_DIST_100M)){
															// �͈͊O���E�H
			psts_chkmap->u1_wrnSts = STS_AREACONT_OUTAREA;	// ��x���Ԃ�
		}
	}

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_INAREA){		// �G���A�����
		if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_SZ){
			F_PCHK_SZ_IN = ON;								// �ŏd�_����ŏ㏑��
		}
		else if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_Z){
			F_PCHK_Z_IN = ON;								// �d�_����ŏ㏑��
		}
		else{												// �ԏ�_��
			F_SHAJYO_IN = ON;
		}
	}

}
//--------------------------------------------------------------------------//
//�y�֐��zETC�Q�[�g�R���e���c����											//
//�y�@�\�zETC�Q�[�gSTART�ESTOP�̒ʉߔ�����s��								//
//�y�����z���ʗL��															//
//�y�ߒl�z�Ȃ�																//
//�y���l�z																	//
//--------------------------------------------------------------------------//
static void	TransitETCGateType(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//�y�֐��z�]�[��30�^�C�v�R���e���c����										
//�y�@�\�z�]�[��30�^�C�v�R���e���c���m��ԑJ�ڂ��s��						
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z�s���^�[�Q�b�g(�������̉��^�[�Q�b�g�͉��z�^�[�Q�b�g)			
//--------------------------------------------------------------------------
static void TransitZone30Type(EM_TGTDEG emh_tgtDeg){

	U2 u2t_inDst;
	
	// ��ԑJ��
	u2t_inDst = psts_chkmap->un_extra.zone30.u2_radius;

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_OUTAREA){		// �G���A�O���
		if(psts_chkmap->u2_dst <= u2t_inDst){				// �͈͓��i���H
			psts_chkmap->u1_wrnSts = STS_AREACONT_INAREA;	// �G���A����Ԃ�
		}
	}
	else{													// �G���A�����
		if(psts_chkmap->u2_dst > (u2t_inDst + u2_DIST_20M)){
															// �͈͊O���E�H
			psts_chkmap->u1_wrnSts = STS_AREACONT_OUTAREA;	// ��x���Ԃ�
		}
	}

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_INAREA){
		F_ZONE30_IN = ON;									// �]�[��30
	}
}

//--------------------------------------------------------------------------
//�y�֐��z�g���l���I�[�r�X�x��J�ڏ���										
//�y�@�\�z�g���l���I�[�r�X�x��𔻒肷��									
//�y�����z���ʗL��															
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void TransitTunnelOrbis(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U2	u2t_rngOutDst = u2_DIST_1500M;						// 1500m�ŏ�����
	U2	u2t_hokanJudgeDist = u2_DIST_1100M;

	if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){
		u2t_hokanJudgeDist = u2_DIST_2100M;
		u2t_rngOutDst = u2_DIST_2500M;
	}

	// SEL_POI_NORMAL�̂Ƃ��͕s������x�񂾂��A�U������͂���

	if(u1s_poiSetSel == SEL_POI_TUNNEL){		// �g���l���U����ԂȂ�x��

		if(psts_chkmap->u2_dst > u2t_rngOutDst){				// ���O��������Ȃ�
			psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_NOALARM;	// ��x���Ԃ�
			psts_chkmap->u2_countdown_dist = U2_MAX;			// �J�E���g�_�E���N���A
		}
		else{
			// �x���ԑJ��
			switch(psts_chkmap->u1_wrnSts){
		//-------��x����-------------------------------------------------------//
			case STS_TUNNEL_ORBIS_NOALARM:							// ��x����
				// ��͈͓��Ń^�[�Q�b�g�Ɍ������ƌx��J�n
				if(u1t_degChk != OK){								// �p�xNG�H
					break;
				}
				if(psts_chkmap->u2_dst <= u2_DIST_600M){			// 600m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; // 600m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_1100M;// 1100m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_2100M){		// 2100M�ȓ�
					if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){
																	// ��������
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_2100M;
																	// 2100m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_2KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------2100m�x����----------------------------------------------------//
			case STS_TUNNEL_ORBIS_2100M:
				if(u1t_degChk == OK){										// �p�x�n�j
					if(psts_chkmap->u2_dst <= u2_DIST_600M){				// 600m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; 	// 600m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
					}
					else if(psts_chkmap->u2_dst <= u2_DIST_1100M){			// 1100m�͈͓�?
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_1100M; 	// 1100m�x���Ԃ�
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------1100m�x����----------------------------------------------------//
			case STS_TUNNEL_ORBIS_1100M:
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_600M)){			// �p�xOK and 600m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; // 600m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------600m�x����------------------------------------------------------//
			case STS_TUNNEL_ORBIS_600M:
				break;
			}
		}

		// �t�H�[�J�X�^�[�Q�b�g����
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2t_hokanJudgeDist, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2t_hokanJudgeDist, u1t_degChk, FALSE);

		// GPS�x�񃌃x���̔���
		if(u1t_degChk == OK){
			// RED HI�̔���
			if(psts_chkmap->u2_dst <= u2_DIST_600M){
				update_gps_warning_lvl(SYS_RED_WARNING_HI, psts_chkmap->u2_dst);
			}
			// RED MID�̔���
			else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
				update_gps_warning_lvl(SYS_RED_WARNING_MID, psts_chkmap->u2_dst);
			}
			// RED LO�̔���
			else if(psts_chkmap->u2_dst <= u2_DIST_2100M){
				update_gps_warning_lvl(SYS_RED_WARNING_LO, psts_chkmap->u2_dst);
			}
		}
		// �C���f�b�N�X���X�g�ǉ�
		AddVisibleIdxLst(u2s_sub_phase);
	}
}
//--------------------------------------------------------------------------
//�y�֐��z�g���l���]�[���x��J�ڏ���										
//�y�@�\�z�g���l������^����x��𔻒肷��									
//�y�����z���ʗL��															
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void TransitTunnelZone(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U1	u1t_thisTargetOut = OFF;
	Bool	isTrapZone = TRUE;

	// SEL_POI_NORMAL�̂Ƃ��͕s������x�񂾂��A�U������͂���

	if(u1s_poiSetSel == SEL_POI_TUNNEL){		// �g���l���U����ԂȂ�x��

		switch(psts_chkmap->un_type.bit.b_code){
		case TGT_CHKPNT_ZONE:
		case TGT_NOFIX_CHKPNT_ZONE:
			isTrapZone = FALSE;
			break;
		default:
			break;
		}

		if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){
			switch(psts_chkmap->u1_wrnSts){
		//-------��x����--------------------------------------------------------//
			case STS_ZONE_NOALARM:
				if((u1t_degChk == OK)								// �p�xOK�H
				&& (psts_chkmap->u2_dst <= u2_DIST_100M)){			// 100m�͈͓�?
					psts_chkmap->u1_wrnSts = STS_ZONE_100M;			// 100m�x���Ԃ�
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_100M, psts_chkmap->u4_dataAddr);
				}
				break;
			case STS_ZONE_100M:
				if(psts_chkmap->u2_dst > u2_DIST_600M){				// ��������?
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
				}
				break;
			}

			// �]�[���ł̃t�H�[�J�X�^�[�Q�b�g����
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
			ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);

		}
		else{
			// �x���ԑJ��
			switch(psts_chkmap->u1_wrnSts){
		//-------��x����--------------------------------------------------------//
			case STS_ZONE_NOALARM:
				// ��͈͓��Ń^�[�Q�b�g�Ɍ������ƌx��J�n
				if(u1t_degChk == OK){								// �p�xOK�H
					if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m�͈͓�?
						if(psts_chkmap->u2_dst <= u2_DIST_500M){	// 500m�͈͓��Ȃ�
							psts_chkmap->u1_wrnSts = STS_ZONE_500M;	// 500m�x���Ԃ�
						}
						else{
							psts_chkmap->u1_wrnSts = STS_ZONE_600M;	// 600m�x���Ԃ�
						}
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
					}
					else{
						if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m�͈͓�?
							psts_chkmap->u1_wrnSts = STS_ZONE_1100M;// 1100m�x���Ԃ�
							GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_1KM, psts_chkmap->u4_dataAddr);
						}
					}
				}
				break;
		//-------1100m�x����-----------------------------------------------------//
			case STS_ZONE_1100M:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O�H
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
				}
				else if((u1t_degChk == OK)							// �p�xOK and
					 && (psts_chkmap->u2_dst <= u2_DIST_600M)){		// 600m�͈͓��Ȃ�
					if(psts_chkmap->u2_dst <= u2_DIST_500M){		// 500m�͈͓��Ȃ�
						psts_chkmap->u1_wrnSts = STS_ZONE_500M;		// 500m�x���Ԃ�
					}
					else{
						psts_chkmap->u1_wrnSts = STS_ZONE_600M;		// 600m�x���Ԃ�
					}
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------600m�x����------------------------------------------------------//
			case STS_ZONE_600M:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O�H
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
				}
				// �����500m�����ɓ��������m�F����
				else if((u1t_degChk == OK)							// �p�xOK and
					 && (psts_chkmap->u2_dst <= u2_DIST_500M)){		// 500m�͈͓��Ȃ�
					psts_chkmap->u1_wrnSts = STS_ZONE_500M;			// 500m�x���Ԃ�
				}
				break;
		//-------500m�x����------------------------------------------------------//
			case STS_ZONE_500M:
				if(psts_chkmap->u2_dst > u2_DIST_500M){				// 500m�͈͊O�H
					psts_chkmap->u1_wrnSts = STS_ZONE_OUTWT_FAR;	// ���S���E�҂���Ԃ�
					if(isTrapZone){
						F_TRAPOUT_REQ = ON;
						psts_trapOutMap = psts_chkmap;				// �|�C���^�ۑ�
					}
					else{
						F_CHKOUT_REQ = ON;
						psts_chkOutMap = psts_chkmap;				// �|�C���^�ۑ�
					}
					u1t_thisTargetOut = ON;
				}
				break;
		//-------�G���A�����S���E�҂����-----------------------------------------//
			case STS_ZONE_OUTWT_FAR:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m�͈͊O
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// ��x���Ԃ�
				}
				break;
			}

			// �]�[���ł̃t�H�[�J�X�^�[�Q�b�g����
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
			ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		}

		// 	�C���f�b�N�X���X�g�ǉ�
		AddVisibleIdxLst(u2s_sub_phase);

		// �g���b�v�]�[�����ł̓��ʏ���
		if(isTrapZone){
			// �ꎞ��~�F�L�����Z��NG�͈͂ɂ��Ȃ��ARD���x�㏸�s�v�A���O�s�v
			// �����_�F�L�����Z��NG�͈͂ɂ��Ȃ��ARD���x�㏸�s�v

			if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_TEMPORARY_STOP){
																	// �ꎞ��~�ȊO�̂Ƃ�
				if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_SIGNAL){
																	// ����Ɍ����_�ȊO�̂Ƃ�
					// �L�����Z��NG����
				 	if(psts_chkmap->u2_dst <= u2_DIST_600M){
						F_CANCEL_NG = ON;							// �L�����Z��NG����
					}
					// RD���x�㏸�p����
					if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){	// ���a500m���� ?
						F_TRAPZONE_IN = ON;
					}
				}
				// ���̃]�[���ɓ����Ă��Ă��邩�ǂ����̔���
				if((u1t_thisTargetOut == OFF)						// ���̃^�[�Q�b�g�������O�v�����Ă��Ȃ�
				&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)		// 
				&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)	// 
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// and 1100m����
					F_TRAP_INCOMING = ON;
				}
			}
		}
		else{														// ����G���A
			if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){			// ���a500m���� ?
				F_CHKPNT_IN = ON;
			}

			if((u1t_thisTargetOut == OFF)							// ���̃^�[�Q�b�g�������O�v�����Ă��Ȃ�
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)			// 
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)		// 
			&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){				// and 1100m����
				F_CHKPNT_INCOMING = ON;
			}
		}

		ChkZoneGpsWrnLvl(u1t_degChk);								// �]�[��GPS�x�񃌃x������

	}
	// �G���A��ԑJ��
	TransitAreaStatusForRd(psts_chkmap);
	TransitAreaStatusForSc(psts_chkmap, u1t_degChk);

}
//--------------------------------------------------------------------------
//�y�֐��z�U���J�ڏ���														
//�y�@�\�z�U���G���A�𔻒肷��												
//�y�����z���ʗL��															
//�y�ߒl�z�Ȃ�																
//�y���l�z�U���̂ݍs���A�x�񂵂Ȃ�											
//--------------------------------------------------------------------------
static void TransitYudo(EM_TGTDEG emh_tgtDeg)
{
}
//--------------------------------------------------------------------------
//�y�֐��z�Z���T�U���J�ڏ���												
//�y�@�\�z�U���G���A�𔻒肷��												
//�y�����z���ʗL��															
//�y�ߒl�z�Ȃ�																
//�y���l�z�U���̂ݍs���A�x�񂵂Ȃ�											
//--------------------------------------------------------------------------
static void TransitSyudo(EM_TGTDEG emh_tgtDeg)
{
}

//--------------------------------------------------------------------------
//�y�֐��z���s����������													
//�y�@�\�z																	
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void RoadJdgRoutine(void){

	U2		u2t_lmtSpd;

	// �񑪈ʃ^�C�}�̌v��
	if(sts_gpsinf.b_spddeg_ok != OK){				// ���x�����Ȃ�
		INCCNTU1(u1s_carNosokuiTmr);			// �A���񑪈ʎ��Ԍv��
		// ���s�^�C�}�̖���������
		if(u1s_carNosokuiTmr >= u1_RUN_NOSOKUI_TIM){
												// ��莞�Ԕ񑪈ʌp��
			u2s_carRunTmr = 0;					// ���s�^�C�}�v����蒼��
			if(u1s_roadJdgSts == ROADJDG_STS_CHKHIGH){
				F_ROADJDG_NOFIX = ON;			// �񑪈ʌ��o��ʒm
			}
		}
#if 0
		// ��ԃ^�C�}�̖���������
		if(u1s_carNosokuiTmr >= u1_STOP_NOSOKUI_TIM){
			u1s_carStopTmr = 0;
		}
#endif
	}
	else{										// ���x�L��
		u1s_carNosokuiTmr = 0;					// �񑪈ʃ^�C�}���N���A
	}

	if(sts_gpsinf.b_spddeg_ok == OK){				// ���x�L���Ȃ�
		// ���s�^�C�}�̌v��
		if((u1s_roadJdgSts == ROADJDG_STS_CHKHIGH)
		&& (u1s_curLmtSpd != LMTSPD_NONE)){
			u2t_lmtSpd = (U2)u1s_curLmtSpd * (U2)10 * SPD_LSB;
			if(u2t_lmtSpd >= (70*SPD_LSB)){			// �������x70mk/h�ȏ�
				if(sts_gpsinf.u2_spd >= u2t_lmtSpd){
													// �������x�ȏ�
					u2s_carRunTmr = GRDADDU2(u2s_carRunTmr, 2);
													// �J�E���g�d��2�{
				}
				else if(sts_gpsinf.u2_spd >= (u2t_lmtSpd - (30*SPD_LSB))){
													// �������x-30km/h�ȏ�Ȃ�
					INCCNTU2(u2s_carRunTmr);		// �J�E���g�d��1�{
				}
				else{								// �ȉ��Ȃ�z�[���h
					;
				}
			}
			else{									// �������x60km/h�ȉ�
				if(sts_gpsinf.u2_spd >= u2t_lmtSpd){
													// �������x�ȏ�
					INCCNTU2(u2s_carRunTmr);		// �J�E���g�d��1�{
				}
				else{								// �ȉ��Ȃ�z�[���h
					;
				}
			}
		}
		else{
			u2s_carRunTmr = 0;
		}
		
		// ��ԃ^�C�}�̌v��
		if((u1s_roadJdgSts == ROADJDG_STS_CHKNORM)
		&& (u1s_curLmtSpd != LMTSPD_NONE)){
			if(sts_gpsinf.u2_spd > u2_HIGH2NORM_SPD){
				u1s_carStopTmr = 0;					// �v����蒼��
			}
			else{
				u2t_lmtSpd = (U2)u1s_curLmtSpd * (U2)10 * SPD_LSB;
				if(u2t_lmtSpd >= (70*SPD_LSB)){		// �������x70mk/h�ȏ�
					u1s_carStopTmr = GRDADDU1(u1s_carStopTmr, 2);
													// �J�E���g�d��2�{
				}
				else{
					INCCNTU1(u1s_carStopTmr);		// �J�E���g�d��1�{
				}
			}
		}
		else{
			u1s_carStopTmr = 0;
		}
	}

	// SCP�ʉ߃^�C�}�̌v��
	if((u1s_roadJdgSts == ROADJDG_STS_HIGH) || (u1s_roadJdgSts == ROADJDG_STS_CHKHIGH)){
		if(sts_gpsinf.b_spddeg_ok == OK){
			INCCNTU2(u2s_scpPassTmr);
		}
	}
	else{
		u2s_scpPassTmr = 0;
	}


}

//--------------------------------------------------------------------------
//�y�֐��z���H���菈��														
//�y�@�\�z���H�����ԑJ�ڂ��s��											
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	RoadJudge(void){

	U1	u1t_oldSts = u1s_roadJdgSts;
	U1	u1t_lmtSpd = (U1)stg_setGps[u1g_setAcsSel].b_hwChgLmtSpd;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_outReq = OFF;								// �o�͗v���Ȃ��ŏ�����

	if(u1t_lmtSpd == OFF){								// �������x���m�ݒ�OFF
		u1s_memLmtSpd = LMTSPD_NONE;					// �������x�L���Ȃ��ɏ�����
	}

	if(sts_gpsinf.b_spddeg_ok != OK){
		return;
	}

	if(F_SCP_ANYPASS){
		u2s_scpPassTmr = 0;								// SCP�ʉ߃^�C�}�N���A
	}

	if(u1t_roadSel != SET_ROAD_AUTO){					// ���H�I���I�[�g�ȊO
		if((F_SCP_H_PASS)								// �����m��
		|| (F_SCP_HCHK_NEAR_PASS)						// or �������m��50m
		|| ((F_SCP_HCHK_PASS) && (sts_gpsinf.u2_spd >= ((U2)sts_scpPoint.un_extra.scp.b_lmtSpd * (U2)10 * SPD_LSB)))){
														// or (�������m��100m and ���x�ȏ�)
			u1t_outReq = ON;							// �o�͗v������ɂ���
		}
		u1_TransitRoadJudge();
	}
	else{
		u1t_outReq = u1_TransitRoadJudge();
	}

	// �����o�͗v��
	if((u1t_outReq) && (u1t_lmtSpd)						// �v���L and �ݒ�ON
	&& (sts_scpPoint.un_extra.scp.b_lmtSpd != LMTSPD_NONE)
														// �������x�Ȃ��łȂ�(�ꉞ�K�[�h)
	&& (u1s_memLmtSpd != sts_scpPoint.un_extra.scp.b_lmtSpd)){
														// and ���x�ύX
		sts_gpsmap[SCP_VIRTUAL_INDEX] = sts_scpPoint;
		sts_gpsmap[SCP_VIRTUAL_INDEX].u4_dataAddr = SCP_VIRTUAL_ADDRESS;
		GpsVcEnQueue(VC_GPSVAR, VCGPS_SCPCONT, SCP_VIRTUAL_ADDRESS);
		u1s_memLmtSpd = (U1)(sts_scpPoint.un_extra.scp.b_lmtSpd);
	}

	// �}�b�v�X�V�v��
	if(u1t_oldSts != u1s_roadJdgSts){					// ���H���ʏ�ԕω��Ȃ�
//		vset_flg1(EVTFLG_REQ_MAPUPD);					// �}�b�v�X�V�v��
	}

	F_ROADJDG_NOFIX = OFF;

}

static U1	u1_TransitRoadJudge(void){

	U1	u1t_outReq = OFF;

		switch(u1s_roadJdgSts){
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_NORM:							// ��ʓ�
		if(sts_gpsinf.u2_spd > u2_HIGH2NORM_SPD){	// �K�葬�x����
			if(F_SCP_H_PASS){						// �����m��Ȃ�
				u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// ���݂̐������x�l�X�V
				u1s_roadJdgSts = ROADJDG_STS_HIGH;	// ��������
				u1t_outReq = ON;					// �o�͗v��
			}
			else if((F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
													// �m��ł͂Ȃ��������ʉ߂Ȃ�
				u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// ���݂̐������x�l�X�V
				u1s_roadJdgSts = ROADJDG_STS_CHKHIGH;
													// �������ʒ���
			}
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_CHKHIGH:						// �������ʒ�
		if(F_SCP_N_PASS){							// ��ʊm��Ȃ�
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// ��ʓ���
			u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_NORMVALID_SPD){
													// �ɒᑬ���o
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// ��ʓ���
//				u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_HIGH2NORM_SPD){
													// �ᑬ���o
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// ��ʓ����蒆��
		}
		else if(F_ROADJDG_NOFIX){
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// ��ʓ����蒆��
		}
		else if(u2s_scpPassTmr >= u2_SCP_NOPASS_TIM){
													// ��ʉߎ��Ԃ���莞�Ԍo��
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// ��ʓ����蒆��
		}
		else if(F_SCP_H_PASS){						// �����m��
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// ��������
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// ���݂̐������x�l�X�V
			u1t_outReq = ON;						// �o�͗v��
		}
		else if((u1s_curLmtSpd >= LMTSPD_70KMH) && (F_SCP_HCHK_PASS)){
													// ����70km/h�ȏ� and �m��ł͂Ȃ��������ʉ�(2���)
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// ��������
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// ���݂̐������x�l�X�V
			u1t_outReq = ON;						// �o�͗v��
		}
		else if(u2s_carRunTmr >= u2_RUN_CONT_TIM){	// �A���������s ?
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// ��������
			u1t_outReq = ON;						// �o�͗v��
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_HIGH:							// ������
		if(F_SCP_N_PASS){							// ��ʊm��Ȃ�
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// ��ʓ���
			u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_NORMVALID_SPD){
													// �ɒᑬ���o
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// ��ʓ���
//				u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_HIGH2NORM_SPD){
													// �ᑬ���o
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// ��ʓ����蒆��
		}
		else if((F_SCP_H_PASS) || (F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
			u1t_outReq = ON;
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// ���݂̐������x�l�X�V
		}
		else if(u2s_scpPassTmr >= u2_SCP_NOPASS_TIM2){
													// ��ʉߎ��Ԃ���莞�Ԍo��
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// ��ʓ����蒆��
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_CHKNORM:							// ��ʓ����蒆
		if((F_SCP_N_PASS)								// ��ʊm�� or
		|| (u1s_carStopTmr >= u1_STOP_CONT_TIM)			// �ᑬ�p�� or
		|| (u2s_scpPassTmr >= u2_SCP_NOPASS_TIM2)		// �������SCP���ʉ߂�����15���o��
		|| (sts_gpsinf.u2_spd <= u2_NORMVALID_SPD)){	// �ɒᑬ���o
			u1s_roadJdgSts = ROADJDG_STS_NORM;			// ��ʓ���
			if(F_SCP_N_PASS){
				u1s_memLmtSpd = LMTSPD_NONE;
			}
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(F_SCP_H_PASS){							// �����m��
			u1s_roadJdgSts = ROADJDG_STS_HIGH;			// ��������
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
														// ���݂̐������x�l�X�V
			u1t_outReq = ON;							// �o�͗v��
		}
		else if((F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
														// ���m��SCP IN�|�C���g�ʉ�
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
														// ���݂̐������x�l�X�V
			u1s_roadJdgSts = ROADJDG_STS_CHKHIGH;		// �������ʒ���
		}
		break;
	}

	return u1t_outReq;

}

EM_ROADJDG_STS	emg_GetCurrentRoad(void){

	return	(EM_ROADJDG_STS)u1s_roadJdgSts;
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE30_AREA, ZN30_VIRTUAL_ADDRESS);
				
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE30_AREA, ZN30_VIRTUAL_ADDRESS);
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



//--------------------------------------------------------------------------
//�y�֐��z�ԏ�_���G���A��ԑJ�ڏ���										
//�y�@�\�z�ԏ�_���G���A��ԑJ�ڂ��s��										
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitShajyoSts(void){
}

//--------------------------------------------------------------------------
//�y�֐��zETC�Q�[�g���m��ԑJ�ڏ���											
//�y�@�\�zETC�Q�[�g���m��ԑJ�ڂ��s��										
//�y�����z�Ȃ�																
//�y�ߒl�z�Ȃ�																
//�y���l�z																	
//--------------------------------------------------------------------------
static void	TransitETCGuide(void){
}
//--------------------------------------------------------------------------
//�y�֐��z��A�͈͔��菈��													
//�y�@�\�z���Ԃ��^�[�Q�b�g���ʂ́}40���ȓ��ɑ��݂��邩�ǂ������肷��		
//�y�����z�Ȃ�																
//�y�ߒl�zU1 TRUE�F�͈͓�	FALSE�F�͈͊O									
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_DegAIsOk(ST_GPSMAP *psth_map){

	U1	u1t_ret = FALSE;							// �͈͊O�ŏ�����
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ��A���^�[�Q�b�g�́}40���ȓ��ɂ���Δ͈͓�
	if((s2t_deg >= s2_DEG_TGTRNG_MIN)
	&& (s2t_deg <= s2_DEG_TGTRNG_MAX)){
		u1t_ret = TRUE;								// �͈͓�
	}
	
	return	u1t_ret;
}
//--------------------------------------------------------------------------
//�y�֐��z��A�͈͔��菈��(��)												
//�y�@�\�z���Ԃ��^�[�Q�b�g���ʂ́}X���ȓ��ɑ��݂��邩�ǂ������肷��			
//�y�����z																	
//�y�ߒl�zU1 TRUE�F�͈͓�	FALSE�F�͈͊O									
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_DegAIsOk_Variable(ST_GPSMAP *psth_map, U1 u1h_deg){

	U1	u1t_ret = FALSE;							// �͈͊O�ŏ�����
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ��A���^�[�Q�b�g�́}40���ȓ��ɂ���Δ͈͓�
	if((s2t_deg >= ((S2)-1*(S2)u1h_deg*(S2)DEG_LSB))
	&& (s2t_deg <= ((S2)u1h_deg*(S2)DEG_LSB))){
		u1t_ret = TRUE;								// �͈͓�
	}
	
	return	u1t_ret;
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
//�y�֐��z��B�͈͔��菈��													
//�y�@�\�z���Ԃ��^�[�Q�b�g�Ɍ������Ă��邩�ǂ����𔻒肷��					
//�y�ߒl�zU1 TRUE�F�Ό�  FALSE�F��Ό�										
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_DegBIsOk(EM_TGTDEG emh_tgtdeg, U2 u2h_dst){

	U1	u1t_ret = FALSE;							// �͈͊O�ŏ�����
	S2	s2t_degMin;
	S2	s2t_degMax;
	S2	s2t_work;

	S2	s2t_deg = psts_chkmap->s2_degB;

	switch(emh_tgtdeg){
	case TYPE_TGTDEG_EXIST_VERY_FAR:					// �^�[�Q�b�g���ʂ���őO�����͈�
		s2t_degMin = s2_DEG_TGTFWD_MIN_VERY_FAR;
		s2t_degMax = s2_DEG_TGTFWD_MAX_VERY_FAR;
		break;
	case TYPE_TGTDEG_DEGB_FRONT:						// �O���`�F�b�N
		s2t_degMin = (S2)(-90*DEG_LSB);
		s2t_degMax = (S2)(90*DEG_LSB);
		break;
	case TYPE_TGTDEG_EXIST_HOKAN:						// �^�[�Q�b�g���ʂ���ňʒu�⊮

		if(u2h_dst < u2_DIST_1100M){					// �^�[�Q�b�g����߂�
			s2t_work = (S2)u2h_dst / s2_DEGRATE_HOKAN_NEAR;
			s2t_degMax = s2_DEG_TGTFWD_MAX_HOKAN_NEAR - s2t_work * (S2)DEG_LSB;
			s2t_degMin = -s2t_degMax;
		}
		else{											// �^�[�Q�b�g���牓��
			s2t_work = (S2)(u2h_dst - u2_DIST_1100M) / s2_DEGRATE_HOKAN_FAR;
			s2t_degMax = s2_DEG_TGTFWD_MAX_HOKAN_FAR - s2t_work * (S2)DEG_LSB;
			s2t_degMin = -s2t_degMax;
		}
		break;
	default:											// �^�[�Q�b�g���ʂȂ��E����
		if(u2h_dst <= u2_DIST_600M){					// �^�[�Q�b�g����߂��Ƃ�
			s2t_degMin = s2_DEG_TGTFWD_MIN;				// �ʏ�p�͈�
			s2t_degMax = s2_DEG_TGTFWD_MAX;
		}
		else{											// �^�[�Q�b�g���牓���Ƃ�
			if(u2h_dst >= u2_DIST_1100M){
				s2t_degMin = s2_DEG_TGTFWD_MIN_FAR;		// �����p�͈�
				s2t_degMax = s2_DEG_TGTFWD_MAX_FAR;		// �����p�͈�
			}
			else{										// �������ɐ��`�⊮����
				s2t_work = (S2)(u2h_dst - u2_DIST_600M) * s2_DEGRATE_FAR_AND_NORM / (S2)10;
				s2t_degMin = s2_DEG_TGTFWD_MIN + s2t_work;
				s2t_degMax = s2_DEG_TGTFWD_MAX - s2t_work;
			}
		}
		break;
	}

	// ���Ԃ��^�[�Q�b�g�Ɍ������Ă��邩����
	
	if((s2t_deg >= s2t_degMin)
	&& (s2t_deg <= s2t_degMax)){
		u1t_ret = TRUE;								// �͈͓�
	}

	return	u1t_ret;
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

//--------------------------------------------------------------------------
//�y�֐��z�`�F�b�N�T���Z�o����												
//�y�@�\�z���Z�`�F�b�N�T�������Z���ĕԂ�									
//�y�����z*pu1h_ptr : �Ώۃf�[�^�|�C���^									
//        u1h_size  : �f�[�^�T�C�Y											
//�y�ߒl�zU1 �`�F�b�N�T���l													
//�y���l�zU1���I�[�o�[�t���[�������͖�������								
//--------------------------------------------------------------------------
U1	u1_CalChkSum(U1 *pu1h_ptr, U1 u1h_size){

	U1	u1t_sum = 0;									// �`�F�b�N�T�����Z�l
	U1	i;												// �T�C�Y���[�v�J�E���^

	for(i=0; i<u1h_size; i++){
		u1t_sum += *pu1h_ptr;
		pu1h_ptr++;
	}

	return	u1t_sum;
}

//--------------------------------------------------------------------------
//�y�֐��z�I�[�r�X�{�C�X�v������											
//�y�@�\�z�ڍו����p�f�[�^���쐬���ďo�͗v������							
//�y�����zpsth_map : ����}�b�v												
//        emh_typ  : �{�C�X�^�C�v											
//		  psth_vcGps : �o�̓{�C�X											
//�y�ߒl�zU1 OK�F�o�͂���  NG�F�o�͂��Ȃ�									
//�y���l�z��Ό��A1km�x���600m���������B�̏ꍇ�͏o�͂��Ȃ�					
//--------------------------------------------------------------------------
static U1	u1_GpsOrbisVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	U1			u1t_spdOver = OFF;						// ���x���ߔ��茋��
	U1			u1t_lmtSpd = OFF;
	Bool		nofix_tgt = FALSE;

	// ���łɒʉ߉������v�����ꂽ��̏ꍇ�͂��ׂĊ��p����
	if(psth_map->u1_wrnSts == STS_ORBIS_50M){
		return NG;
	}

	// ���x���ߔ���
	if((stg_setGps[u1g_setAcsSel].b_hwOrbisLmtSpd)		// �I�[�r�X�������x���m�ݒ�ON
	&& (psth_map->un_extra.orbis.b_lmtSpd != LMTSPD_NONE)){
		u1t_lmtSpd = ON;
		if(stg_setGps[u1g_setAcsSel].b_spdOver){		// ���x���ߍ��mON
			u1t_spdOver = ON;
		}
	}

	switch(emh_typ){
	case VCGPS_ORBIS_2KM:								// �I�[�r�X2km
	case VCGPS_ORBIS_1KM:								// �I�[�r�X1km
	case VCGPS_ORBIS_500M:								// �I�[�r�X500m
		// (1)���ʁF���
		switch(psth_map->un_type.bit.b_code){
		case TGT_NOFIX_RD_ORBIS:
			nofix_tgt = TRUE;
		case TGT_RD_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;
			break;
		case TGT_NOFIX_LHSYS_ORBIS:
			nofix_tgt = TRUE;
		case TGT_LHSYS_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_LHSYS_ORBIS;
			break;
		case TGT_NOFIX_LOOP_ORBIS:
			nofix_tgt = TRUE;
		case TGT_LOOP_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_LP_ORBIS;
			break;
		case TGT_NOFIX_HSYS_ORBIS:
			nofix_tgt = TRUE;
		case TGT_HSYS_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_HSYS_ORBIS;
			break;
		default:
			return NG;
		}
		// (2)���ʁF�g���l��
		psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
		// �g���l���o���g��
		if((psth_map->un_type.bit.b_tunnel == TUNNEL_OUT) && (nofix_tgt)){
			psth_vcGps->common.b_tn_ext = ON;			// �g���l���g���w��
		}

		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

			// (3)���ʁF�����i��g���l���j
			if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
				return NG;
			}
			// (4)���ʁF�����i��g���l���j
			if(emh_typ == VCGPS_ORBIS_2KM){				// 2km���m
				if(SubVcReqUpper1000m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else if(emh_typ == VCGPS_ORBIS_1KM){		// 1km���m
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else{										// 1km�ȊO
				SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
			}
		}

		if(emh_typ == VCGPS_ORBIS_500M){
			if((stg_setMisc.b_idvOrbisVoice)
			&& (psth_map->un_extra.orbis.u2_voiceNum <= INDIV_ORBIS_PHRASE_MASK)){
				psth_vcGps->common.b_idvVc = ON;
				psth_vcGps->extra.orbis.idvVoiceLo = (U1)(psth_map->un_extra.orbis.u2_voiceNum & 0x00FF);
				psth_vcGps->extra.orbis.idvVoiceHi = (U1)((psth_map->un_extra.orbis.u2_voiceNum & 0x0700) >> 8);
			}
		}

		if(psth_map->un_type.bit.b_tunnel != TUNNEL_OUT){	// �g���l���o���f�[�^����
			// (5)�g���F�������x
			if((emh_typ == VCGPS_ORBIS_1KM)
			&& (u1t_lmtSpd)){
				psth_vcGps->extra.orbis.b_lmtSpd = ON;
				psth_vcGps->extra.orbis.lmtSpdVal = psth_map->un_extra.orbis.b_lmtSpd;
				if(stg_setGps[u1g_setAcsSel].b_hwChgLmtSpd){
														// �������x�ؑփ|�C���g���m�ݒ�ON
					u1s_memLmtSpd = psth_map->un_extra.orbis.b_lmtSpd;
														// ���x�l���L�������ؑփ|�C���g��
														// �������x������Ȃ�
					u1s_curLmtSpd = u1s_memLmtSpd;
				}
				// (6)�g���F���x���ߏ��
				if(u1t_spdOver){
					psth_vcGps->extra.orbis.b_spdOver = ON;
				}
			}
			// (7)�g���F�J�����ʒu
			if((emh_typ == VCGPS_ORBIS_500M)
			&& (psth_map->u2_dst >= u2_DIST_300M)		// ���܂�ɋߋ����ɂȂ��Ă���ꍇ�͎��Ԃ��Ȃ��̂ŏȂ�
			&& (psth_map->un_extra.orbis.b_camera != CAMERA_NONE)
			&& (stg_setGps[u1g_setAcsSel].b_orbisCameraPos)){
														// �J�������mON
				psth_vcGps->extra.orbis.b_camera = ON;
				psth_vcGps->extra.orbis.cameraPos = psth_map->un_extra.orbis.b_camera;
			}
		}

		// (8)���ʁF����
		if(psth_map->un_type.bit.b_road == ROAD_HIGH){
			psth_vcGps->common.b_highway = ON;
		}
		break;

	case VCGPS_ORBIS_PASSSPD:
		if((psth_map->u2_dst <= u2_DIST_200M) || (psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX)){
														// ����200m�ȓ��ɓ��B���Ă����璆�~ or �ʉ߂��I����Ă��Ă����~
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;	// �I�[�r�X�Ȃ�΂悢�i����Ȃ��j
		psth_vcGps->extra.orbis.b_runSpd = ON;			// ���s���x
		psth_vcGps->extra.orbis.lmtSpdVal = psth_map->un_extra.orbis.b_lmtSpd;
		if(u1t_spdOver){
			psth_vcGps->extra.orbis.b_spdOver = ON;		// ���x����
		}
		break;
#if 0
	case VCGPS_ORBIS_PASS:
		if((psth_map->u2_dst > u2_DIST_50M)
		&& ((psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX))){
														// �|�C���g��50m�ȏ����ɂȂ��Ă��܂�����
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;	// �I�[�r�X�Ȃ�΂悢�i����Ȃ��j
		psth_vcGps->extra.orbis.b_pass = ON;			// �ʉ�
		break;
#endif
	}
	return	OK;
}

//--------------------------------------------------------------------------
//�y�֐��z�]�[���{�C�X�v������												
//�y�@�\�z�]�[���̃{�C�X�����쐬���ďo�͂���								
//�y�����zpsth_map : ����}�b�v												
//        emh_typ  : �{�C�X�^�C�v(1km, 1km����, ���O)						
//�y�ߒl�zU1 OK�F�o�͂���  NG�F�o�͂��Ȃ�									
//�y���l�z��Ό��A1km�x���600m���������B�̏ꍇ�͏o�͂��Ȃ�					
//--------------------------------------------------------------------------
static U1	u1_GpsZoneVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	Bool	nofix_tgt = FALSE;

	// (1)���ʁF�^�[�Q�b�g
	switch(psth_map->un_type.bit.b_code){
	case TGT_NOFIX_CHKPNT_ZONE:
		nofix_tgt = TRUE;
	case TGT_CHKPNT_ZONE:
		psth_vcGps->common.u1_trgt = VCTGT_CHKPNT_ZONE;
		break;
	case TGT_NOFIX_TRAP_ZONE:
		nofix_tgt = TRUE;
	default:
		psth_vcGps->common.u1_trgt = VCTGT_TRAP_ZONE;
		break;
	}

	// (2)�g���F���O
	if(emh_typ == VCGPS_ZONE_OUT){							// ���O���m
		psth_vcGps->extra.trapchk.b_rngOut = ON;
	}
	else{													// ���O�ȊO
		// (3)���ʁF�g���l��
		psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
		// �g���l���o���g��
		if((psth_map->un_type.bit.b_tunnel == TUNNEL_OUT) && (nofix_tgt)){
			psth_vcGps->common.b_tn_ext = ON;				// �g���l���g���w��
		}

		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

			// (4)���ʁF����
			if((emh_typ != VCGPS_ZONE_100M)
			&& (SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG)){
				return NG;
			}
			// (5)���ʁF����
			if(emh_typ == VCGPS_ZONE_1KM){					// 1km���m
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else if(emh_typ == VCGPS_ZONE_500M){			// 500m���m
				psth_vcGps->common.b_dist = VCDIST_NONE;	// ����Ȃ�
			}
			else{
				psth_vcGps->common.b_dist = VCDIST_50M;		// ������
			}
		}

		if(emh_typ == VCGPS_ZONE_500M){
			// (6)�g���F�������x(500m���E����̂�)
			if((psth_vcGps->common.u1_trgt != VCTGT_CHKPNT_ZONE)
			&& (psth_map->un_extra.trapchk.b_lmtSpd != LMTSPD_NONE)){
				psth_vcGps->extra.trapchk.b_lmtSpd = ON;
				psth_vcGps->extra.trapchk.lmtSpdVal = psth_map->un_extra.trapchk.b_lmtSpd;
				// (7)�g���F���x����
				if(stg_setGps[u1g_setAcsSel].b_spdOver){// ���x���ߍ��mON�H
					psth_vcGps->extra.trapchk.b_spdOver = ON;
				}
			}
			// (8)�g���F�ߐ�(500m��)
			psth_vcGps->extra.trapchk.b_near = ON;
		}
		// (9)�g���F��@(�g���l���s��)
		psth_vcGps->extra.trapchk.method = psth_map->un_extra.trapchk.u1_method;

		// (10)���ʁF����(�g���l���s��)
		if(psth_map->un_type.bit.b_road == ROAD_HIGH){
			psth_vcGps->common.b_highway = ON;
		}

		// (11)���x��(�g���l���s��)
		if(psth_map->un_extra.trapchk.b_level != TRAPCHK_LEVEL_INVALID){
															// ���x�����L���Ȃ�
			psth_vcGps->extra.trapchk.b_level = ON;
			psth_vcGps->extra.trapchk.levelVal = psth_map->un_extra.trapchk.b_level;
		}
	}

	return	OK;
}
//--------------------------------------------------------------------------
//�y�֐��z1km�^�C�v�R���e���c�{�C�X�v������									
//�y�@�\�z1km�����Ŕ��肷��R���e���c�̉��������쐬���ďo�͗v������		
//�y�����zpsth_map : ����}�b�v												
//        emh_typ  : �{�C�X�^�C�v(1km�x��, 500m�x��)						
//�y�ߒl�zU1 OK�F�o�͂���  NG�F�o�͂��Ȃ�									
//�y���l�z��Ό��A1km�x���600m���������B�̏ꍇ�͏o�͂��Ȃ�					
//--------------------------------------------------------------------------
static U1	u1_Gps1kmContVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	U1	u1t_req_smart_gas = FALSE;

	// (1)���ʁF���
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_MICHINOEKI:
		psth_vcGps->common.u1_trgt = VCTGT_MICHINOEKI;
		break;
	//-------------------------------------------------------------------
	case TGT_SA:
		psth_vcGps->common.u1_trgt = VCTGT_SA;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_PA:
		psth_vcGps->common.u1_trgt = VCTGT_PA;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_HWOASYS:
		psth_vcGps->common.u1_trgt = VCTGT_HWOASYS;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_TUNNEL:
		psth_vcGps->common.u1_trgt = VCTGT_TUNNEL;
		psth_vcGps->extra.tunnel.b_tunnel = ON;
		psth_vcGps->extra.tunnel.tunnelType = psth_map->un_extra.tunnel.b_tunnelType;
		break;
	//-------------------------------------------------------------------
	case TGT_TORUPA:
		psth_vcGps->common.u1_trgt = VCTGT_TORUPA;
		break;
	}

	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

		// (3)���ʁF����
		if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
			return NG;
		}
		// (4)���ʁF����
		if(emh_typ == VCGPS_1KMCONT_1KM){					// 1km�x��
			if(u1t_req_smart_gas){
				// ���̃^�C�v�̓����V���b�g�̂��߁A600m�ȉ��ł�����
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
				}
				// (5)�g���F�X�}�[�gIC
				// SMART IC
				if((stg_setGps[u1g_setAcsSel].b_smartIC)
				&& (psth_map->un_extra.sapaoa.b_smartIC == SMART_IC_EXIST)){
														// SMART IC����
					psth_vcGps->extra.sapaoa.b_smartIC = ON;
					psth_vcGps->extra.sapaoa.smartIC = psth_map->un_extra.sapaoa.b_smartIC;
				}
				// (6)�g���F�K�\�����X�^���h
				if((stg_setGps[u1g_setAcsSel].b_hwGas)
				&& (psth_map->un_extra.sapaoa.u1_gasStation != GS_NONE)){
														// �K�\�����X�^���h����
					psth_vcGps->extra.sapaoa.b_gas = ON;
					psth_vcGps->extra.sapaoa.gasBrand = psth_map->un_extra.sapaoa.u1_gasStation;
				}
			}
			else{
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
		}
		else{
			SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
		}
	}
	// (7)����
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}

//--------------------------------------------------------------------------
//�y�֐��z500m�^�C�v�R���e���c�{�C�X�v������								
//�y�@�\�z500m�����łŔ��肷��R���e���c�̉��������쐬���ďo�͗v������	
//�y�����zpsth_map : ����}�b�v												
//�y�ߒl�zU1 OK�F�o�͂���  NG�F�o�͂��Ȃ�									
//�y���l�z��Ό��̏ꍇ�͏o�͂��Ȃ�											
//--------------------------------------------------------------------------
static U1	u1_Gps500mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	U1	u1t_curve_phrase = FALSE;

	// (1)���ʁF���
	switch(psth_map->un_type.bit.b_code){
	case TGT_POLICE:
		if(psth_map->un_extra.police.b_policeType == POLICE_TYPE_STATION){
			psth_vcGps->common.u1_trgt = VCTGT_POLICE_STATION;
		}
		else{
			psth_vcGps->common.u1_trgt = VCTGT_HIGHWAY_POLICE;
		}
		break;

	case TGT_KOBAN:
		psth_vcGps->common.u1_trgt = VCTGT_KOBAN;
		break;

	case TGT_CURVE:
		psth_vcGps->common.u1_trgt = VCTGT_CURVE;
		psth_vcGps->extra.curve.b_curve = ON;
		psth_vcGps->extra.curve.curveType = psth_map->un_extra.curve.b_curveType;
		u1t_curve_phrase = TRUE;
		break;
	}

	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){
		// (3)���ʁF����
		if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
			return NG;
		}
		// (4)���ʁF����
		if(u1t_curve_phrase == TRUE){
			psth_vcGps->common.b_dist = VCDIST_900M;		// "���̐�"
		}
		else{
			SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
		}
	}

	// (5)���ʁF����
	if((psth_map->un_type.bit.b_road == ROAD_HIGH)
	&& (psth_vcGps->common.u1_trgt != VCTGT_HIGHWAY_POLICE)){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}
//--------------------------------------------------------------------------
//�y�֐��z300m�^�C�v�R���e���c�{�C�X�v������								
//�y�@�\�z300m�����łŔ��肷��R���e���c�̉��������쐬���ďo�͗v������	
//�y�����zpsth_map : ����}�b�v												
//�y�ߒl�zU1 OK�F�o�͂���  NG�F�o�͂��Ȃ�									
//�y���l�z�A�i�E���X�ȊO��Ό��̏ꍇ�͏o�͂��Ȃ�							
//		  �A�i�E���X�̏ꍇ�A��ނƃg���l���ȊO�͌���Ȃ�					
//--------------------------------------------------------------------------
static U1	u1_Gps300mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	U1	u1t_announce = FALSE;

	// (1)���ʁF���
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_NSYS:
		psth_vcGps->common.u1_trgt = VCTGT_NSYS;
		break;
	//-------------------------------------------------------------------
	case TGT_TRFCHK:
		psth_vcGps->common.u1_trgt = VCTGT_TRFCHK;
		break;
	//-------------------------------------------------------------------
	case TGT_ACCIDENT:
		psth_vcGps->common.u1_trgt = VCTGT_ACCIDENT;
		break;
	//-------------------------------------------------------------------
	case TGT_CROSSING:
		psth_vcGps->common.u1_trgt = VCTGT_CROSSING;
		break;
	//-------------------------------------------------------------------
	case TGT_SIGNAL:
		psth_vcGps->common.u1_trgt = VCTGT_SIGNAL;
		break;
	//-------------------------------------------------------------------
	case TGT_BRAJCT:
		psth_vcGps->common.u1_trgt = VCTGT_BRAJCT;
		psth_vcGps->extra.brajct.b_brajct = ON;
		psth_vcGps->extra.brajct.brajctType = psth_map->un_extra.brajct.b_braJctType;
		u1t_announce = TRUE;
		break;
	}

	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(u1t_announce == FALSE){
		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){
			// (3)���ʁF����
			if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
				return NG;
			}
			// (4)����
			if(psth_map->u2_dst >= u2_DIST_50M){
				psth_vcGps->common.b_dist = VCDIST_50M;		// ������
			}
			else{
				psth_vcGps->common.b_dist = VCDIST_NONE;	// ����Ȃ�
			}
		}
	}
	else{
		// (4)����
		psth_vcGps->common.b_dist = VCDIST_900M;			// ���̐�

		if((psth_map->u2_dst > u2_DIST_50M)
		&& ((psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX))){
															// �|�C���g��50m�ȏ����ɂȂ��Ă��܂�����
			return NG;										// �o�͂��Ȃ�
		}
	}

	// (5)����
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}


	return	OK;
}

//--------------------------------------------------------------------------
//�y�֐��z100m�^�C�v�R���e���c�{�C�X�v������								
//�y�@�\�z100m�����łŔ��肷��R���e���c�̉��������쐬���ďo�͗v������	
//�y�����zpsth_map : ����}�b�v												
//�y�ߒl�zU1 OK�F�o�͂���  													
//�y���l�z��Ό��̏ꍇ�ł��o�͂���											
//--------------------------------------------------------------------------
static U1	u1_Gps100mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	// (1)���ʁF���
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_HWRADIO:
		psth_vcGps->common.u1_trgt = VCTGT_HWRADIO;		// �n�C�E�F�C���W�I
		break;
	case TGT_KENKYO:
		if(u1s_kenkyo_mem != psth_map->un_extra.kenkyo.u1_kenkyoNum){
			psth_vcGps->common.u1_trgt = VCTGT_KENKYO;	// ����
			psth_vcGps->extra.kenkyo.b_kenkyo = ON;
			psth_vcGps->extra.kenkyo.kenkyoNum = psth_map->un_extra.kenkyo.u1_kenkyoNum;
			u1s_kenkyo_mem = psth_map->un_extra.kenkyo.u1_kenkyoNum;
		}
		else{
			return NG;
		}
		break;
	}

	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

		// (3)���ʁF����
		psth_vcGps->common.b_dir = VCDIR_NONE;		// ����Ȃ�

		// (4)���ʁF����
		if(psth_map->un_type.bit.b_code == TGT_KENKYO){
			psth_vcGps->common.b_dist = VCDIST_900M;// ���̐�
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_NONE;// ����Ȃ�
		}
	}

	// (5)���ʁF����
	if((psth_map->un_type.bit.b_road == ROAD_HIGH)
	&& (psth_map->un_type.bit.b_code != TGT_KENKYO)){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}
//--------------------------------------------------------------------------
//�y�֐��z���x�ؑփR���e���c�{�C�X�v������									
//�y�@�\�z���x�ؑփR���e���c�̉��������쐬���ďo�͗v������				
//�y�����zpsth_map : ����}�b�v												
//�y�ߒl�zU1 OK�F�o�͂���  													
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_GpsScpContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	// (1)���ʁF���
	psth_vcGps->common.u1_trgt = VCTGT_SCP;
	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
	// (3)���ʁF����
	psth_vcGps->common.b_dir = VCDIR_NONE;			// ����Ȃ�
	// (4)���ʁF����
	psth_vcGps->common.b_dist = VCDIST_NONE;		// ����Ȃ�
	// (5)���ʁF����
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}

	// (6)�g���F�������x
	psth_vcGps->extra.scp.b_lmtSpd = ON;
	psth_vcGps->extra.scp.lmtSpdVal = psth_map->un_extra.scp.b_lmtSpd;

	// (7)�g���F���x����
	if(stg_setGps[u1g_setAcsSel].b_spdOver){
		psth_vcGps->extra.scp.b_spdOver = ON;
	}

	return	OK;
}

//--------------------------------------------------------------------------
//�y�֐��z�G���A�{�C�X�v������												
//�y�@�\�z�G���A�{�C�X���쐬���ďo�͗v������								
//�y�����zpsth_map : ����}�b�v												
//�y�ߒl�zU1 OK�F�o�͂���  													
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_GpsAreaVcReq(EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	// (1)���ʁF���
	switch(emh_typ){
	case VCGPS_PCHK_SZ_AREA:
		if(u1s_parkChkAreaSts != PCHK_WRN_INSZ_SNDOUT){	// ���łɊY���G���A�O�Ȃ璆�~
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_SZ_AREA;
		break;

	case VCGPS_PCHK_Z_AREA:
		if(u1s_parkChkAreaSts != PCHK_WRN_INZ_SNDOUT){	// ���łɊY���G���A�O�Ȃ璆�~
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_Z_AREA;
		break;

	case VCGPS_ZONE30_AREA:
		if(u1s_zone30AreaSts != ZONE30_WRN_IN_SNDOUT){		// ���łɊY���G���A�O�Ȃ璆�~{
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_ZONE30_AREA;
		break;
	}

	// (2)���ʁF�g���l��
	psth_vcGps->common.b_tunnel = NOT_TUNNEL;
	// (3)���ʁF����
	psth_vcGps->common.b_dir = VCDIR_NONE;					// ����Ȃ�
	// (4)���ʁF����
	psth_vcGps->common.b_dist = VCDIST_NONE;				// ����Ȃ�
	// (5)���ʁF����
	psth_vcGps->common.b_highway = OFF;

	return OK;
}
//--------------------------------------------------------------------------
//	�{�C�X�쐬�T�u�֐�
//--------------------------------------------------------------------------
static U1	SubVcReqDir(S2 s2h_degB, ST_VCGPS *psth_vcGps){

	if((s2h_degB < s2_DEG_TGTFWD_MIN)
	|| (s2h_degB > s2_DEG_TGTFWD_MAX)){				// �Ό����Ă��Ȃ�
		return NG;									// �o�͂��Ȃ�
	}

	if(s2h_degB < s2_DEG_TGTLEFT_MIN){
		psth_vcGps->common.b_dir = VCDIR_LEFT;		// ������
	}
	else if(s2h_degB > s2_DEG_TGTRIGHT_MIN){
		psth_vcGps->common.b_dir = VCDIR_RIGHT;		// �E����
	}
	else{
		psth_vcGps->common.b_dir = VCDIR_FWRD;		// �O����
	}

	return OK;
}
//--------------------------------------------------------------------------
static U1	SubVcReqUpper1000m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_1900M){					// ����1900m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_2KM;		// "2km��"
	}
	else{											// ����1900m�����̂Ƃ�
		if(u2h_dst <= u2_DIST_1100M){				// 2km�x��Ȃ̂Ɋ���1km�x�񂪐������鋗��
													// (=���̌シ����1km�x�񂪏o���)
			return NG;								// �o�͒��~
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_1900M;// "���̐�"
		}
	}

	return OK;
}
//--------------------------------------------------------------------------
static U1	SubVcReqUpper500m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_900M){					// ����900m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_1KM;		// "1km��"
	}
	else{
		if(u2h_dst <= u2_DIST_600M){				// 1km�x��Ȃ̂Ɋ���500m�x�񂪐������鋗��
													// (=���̌シ����500m�x�񂪏o���)
			return NG;								// �o�͒��~
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_900M;// "���̐�"
		}
	}

	return OK;
}
//--------------------------------------------------------------------------
static void	SubVcReqUnder500m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_400M){					// ����400m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_500M;	// "500m��"
	}
	else if(u2h_dst >= u2_DIST_300M){				// ����300m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_300M;	// "300m��"
	}
	else if(u2h_dst >= u2_DIST_200M){				// ����200m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_200M;	// "200m��"
	}
	else if(u2h_dst >= u2_DIST_100M){				// ����100m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_100M;	// "100m��"
	}
	else if(u2h_dst >= u2_DIST_50M){				// ����50m�ȏ�̂Ƃ�
		psth_vcGps->common.b_dist = VCDIST_50M;		// "������"
	}
	else{
		psth_vcGps->common.b_dist = VCDIST_NONE;	// ����Ȃ�
	}
}

//--------------------------------------------------------------------------
//�y�֐��z�{�C�X�L���[�i�[����												
//�y�@�\�z�{�C�X�L���[�ɉ����v�����i�[����									
//�y�����zu1h_voice : �����ԍ�												
//        sth_vcGps : GPS�t�����											
//        u4h_addr  : �f�[�^�A�h���X										
//�y���l�zGPS�^�X�N�ȊO���瑀�삳���̂Ńf�B�X�p�b�`�֎~�ŏ�������			
//--------------------------------------------------------------------------
void	GpsVcEnQueue(EM_VC u1h_voice, EM_VCGPS_TYPE emh_vcType, U4 u4h_addr){

	ST_GPSVC_QUEUE_CTRL	*pstt_queue_ctrl;

	if(emh_vcType >= VC_LOPRI_QUEUE_STA){
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_LOPRI];
	}
	else if(emh_vcType >= VC_MIDPRI_QUEUE_STA){
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_MIDPRI];
	}
	else{
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_HIPRI];
	}

	// �ʏ�̃L���[�}���v�����̏���
	if(pstt_queue_ctrl->sts != VC_QUEUE_FULL){					// �L���[���t���łȂ��Ƃ�
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].live_count = u4s_live_counter;
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].u1_vcNum = u1h_voice;		// �{�C�X�ԍ��L��
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].em_vcType = emh_vcType;		// �{�C�X�^�C�v
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].u4_addr = u4h_addr;			// �A�h���X
		pstt_queue_ctrl->wrPos++;
		if(pstt_queue_ctrl->wrPos >= pstt_queue_ctrl->queueSize){
			pstt_queue_ctrl->wrPos = 0;
		}
		if(pstt_queue_ctrl->wrPos == pstt_queue_ctrl->rdPos){
			pstt_queue_ctrl->sts = VC_QUEUE_FULL;
		}
		else{
			pstt_queue_ctrl->sts = VC_QUEUE_DATON;
		}
	}
}

//--------------------------------------------------------------------------//
//�y�֐��z�D��t�H�[�J�X�^�[�Q�b�g���菈��									//
//�y�@�\�z																	//
//�y�����zST_GPSMAP *psth_map  	�F�Ώۃ}�b�v�v�f							//
//		  U2		u2h_nearDst	�F�ڋߔ��苗��(0�Őڋߔ��肵�Ȃ�)			//
//		  U2		u2h_baseDst	�F����苗��								//
//		  U1		u1h_degChk	�F���ԑΌ����茋��							//
//�y���l�z																	//
//--------------------------------------------------------------------------//
static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus){

	U1	u1t_focus = PRI_FOCUS_TGT_NONE;				// �t�H�[�J�X�Ȃ��ŏ�����
	U1	isPriWin = FALSE;
	U2	u2t_absDegB = (U2)(ABSO(psth_map->s2_degB));
	U2	u2t_dstSub;

	if(sound_focus)
	{
		u1t_focus = PRI_FOCUS_TGT_POLIGON_SOUNDING;
		isPriWin = TRUE;
		goto FOCUS_JDGEND;
	}

	// ��΋��������o��
	u2t_dstSub = ABS_SUB(psth_map->u2_dst, psth_focus_tgt->u2_dst);

	if((u1h_degChk) && (u2h_nearDst != 0) && (psth_map->u2_dst <= u2h_nearDst)){
		u1t_focus = PRI_FOCUS_TGT_POLIGON;
	}
	else if(psth_map->u2_dst <= u2h_baseDst)
	{
		// �A�C�R���t�H�[�J�X�����߂�
		// ���Ԃ̑O���ɑΌ����đ��݂��� ?
		if(psth_map->hys_ahead)
		{
			u1t_focus = PRI_FOCUS_TGT_ICON;
		}
	}

	// �񑪈ʈڍs���A�g���l���������I�[�r�X�̃|���S���t�H�[�J�X������ΕK���ŗD��ɂ���
	if((stg_gpsSts.b_curSokui == NG)
	&& (psth_map->un_type.bit.b_tunnel == TUNNEL_IN)
	&& (u1t_focus == PRI_FOCUS_TGT_POLIGON)){
		psth_focus_tgt->u1_priTunnelIn = TRUE;
		isPriWin = TRUE;							// �D��
		goto FOCUS_JDGEND;
	}

	// ����܂ł̍ō��D��x�����肷��
	if(u1t_focus > psth_focus_tgt->u1_sts){			// ��Ԃ����D��H
		isPriWin = TRUE;
		goto FOCUS_JDGEND;
	}
	if((u1t_focus < psth_focus_tgt->u1_sts)			// ��Ԃ���D��H
	|| (psth_focus_tgt->u1_priTunnelIn == TRUE)){	// ���łɍ��D��g���l��������
		goto FOCUS_JDGEND;
	}

	// ��Ԃ�����

	if(u2t_dstSub >= u2_DIST_100M){					// 100m�ȏ㍷������Ƃ�
		if(psth_map->u2_dst < psth_focus_tgt->u2_dst){
			isPriWin = TRUE;						// �߂����Ȃ�D��
		}
		// �߂��Ȃ��Ȃ��D��m��
		goto FOCUS_JDGEND;
	}
	else{											// ����100m�����̂Ƃ�
		// �����D��
		if(psth_map->u2_dst < psth_focus_tgt->u2_dst){
			isPriWin = TRUE;						// �߂����Ȃ�D��
			goto FOCUS_JDGEND;						// �D�揟��
		}
		else if(psth_map->u2_dst > psth_focus_tgt->u2_dst){
													// �����ق��Ȃ�
			goto FOCUS_JDGEND;						// �D�敉��
		}

		// �����������Ȃ�Ό��p�Ŕ���
		if(u2t_absDegB < psth_focus_tgt->u2_absDegB){	// �Ό��p�̏���������D��
			isPriWin = TRUE;
		}
		else{
			// ���Ƃ���Ύ�Ԃ̔�r
		}
	}

FOCUS_JDGEND:

	{
		ST_FOCUS_TGT	*pstt_2ndTgt;
		if(psth_focus_tgt == &sts_warning_focusTgt_Primary)
		{
			pstt_2ndTgt = &sts_warning_focusTgt_Secondary;
		}
		else
		{
			pstt_2ndTgt = &sts_secondary_focusTgt;
		}

		if(isPriWin){
			// �D�揟���X�V�O�ɂQ�Ԗڂ̗D��ɃR�s�[����
			if(psth_focus_tgt->u1_sts != PRI_FOCUS_TGT_NONE)
			{
				*pstt_2ndTgt = *psth_focus_tgt;
			}
			// �D�揟�������̂ōX�V����
			psth_focus_tgt->u1_sts = u1t_focus;				// ��ԍX�V
			psth_focus_tgt->u2_dst = psth_map->u2_dst;		// �����L��
			psth_focus_tgt->u2_num = u2s_sub_phase;			// �^�[�Q�b�g�ԍ����L��
			psth_focus_tgt->u2_absDegB = u2t_absDegB;

		}
		else{
			if(u1t_focus != PRI_FOCUS_TGT_NONE){
				// ���炩�̃t�H�[�J�X������ꍇ�A�ŗD��ɕ������̂ŁA��2�D��ɊY�����邩���肷��
				ChkFocusTgt_2nd(pstt_2ndTgt, psth_map, u2t_dstSub, u2t_absDegB, u1t_focus);
			}
		}

	}

}

//--------------------------------------------------------------------------//
//�y�֐��z��2�D��t�H�[�J�X�^�[�Q�b�g���菈��								//
//�y�@�\�z																	//
//�y���l�z																	//
//--------------------------------------------------------------------------//
static void	ChkFocusTgt_2nd(ST_FOCUS_TGT *psth_2nd_focus_tgt, ST_GPSMAP *psth_map, U2 u2t_dstSub, U2 u2t_absDegB, U1 u1t_focus){

	U1	is2ndWin = FALSE;

	// ����܂ł̍ō��D��x�����肷��
	if(u1t_focus > psth_2nd_focus_tgt->u1_sts){	// ��Ԃ����D��H
		is2ndWin = TRUE;
		goto FOCUS_2ND_JDGEND;
	}
	
	// ��Ԃ���D��
	if(u1t_focus < psth_2nd_focus_tgt->u1_sts){	// ��Ԃ���D��H
		goto FOCUS_2ND_JDGEND;
	}

	// ��Ԃ�����
	if(u2t_dstSub >= u2_DIST_100M){					// 100m�ȏ㍷������Ƃ�
		if(psth_map->u2_dst < psth_2nd_focus_tgt->u2_dst){
			is2ndWin = TRUE;						// �߂����Ȃ�D��
		}
		// �߂��Ȃ��Ȃ��D��m��
		goto FOCUS_2ND_JDGEND;
	}
	else{											// ����100m�����̂Ƃ�
		// �����D��
		if(psth_map->u2_dst < psth_2nd_focus_tgt->u2_dst){
			is2ndWin = TRUE;						// �߂����Ȃ�D��
			goto FOCUS_2ND_JDGEND;					// �D�揟��
		}
		else if(psth_map->u2_dst > psth_2nd_focus_tgt->u2_dst){
													// �����ق��Ȃ�
			goto FOCUS_2ND_JDGEND;					// �D�敉��
		}

		// �����������Ȃ�Ό��p�Ŕ���
		if(u2t_absDegB < psth_2nd_focus_tgt->u2_absDegB){
													// �Ό��p�̏���������D��
			is2ndWin = TRUE;
		}
		else{
			// ���Ƃ���Ύ�Ԃ̔�r
		}
	}

FOCUS_2ND_JDGEND:

	if(is2ndWin){
		// �D�揟�������̂ōX�V����
		psth_2nd_focus_tgt->u1_sts = u1t_focus;			// ��ԍX�V
		psth_2nd_focus_tgt->u2_dst = psth_map->u2_dst;	// �����L��
		psth_2nd_focus_tgt->u2_num = u2s_sub_phase;		// �^�[�Q�b�g�ԍ����L��
		psth_2nd_focus_tgt->u2_absDegB = u2t_absDegB;
	}
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

	// �J�X�^���f�[�^�͏펞���H����s�v
	if(psth_map->un_type.bit.b_dataArea)
	{
		u1t_roadChk = ROADCHK_OFF;					// ���H����s�v
	}

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
			if((u1t_roadSel == SET_ROAD_AUTO)		// �I�[�g�I����
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)		// and ����70km/h�ȉ�
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)				// and ����������
			&& (u1s_roadJdgSts == ROADJDG_STS_NORM)						// and ���H���ʂ���ʓ����s���
			&& (gps_distToHighway >= 0) && (gps_distToHighway <= 100)){	// and �������H�܂ł̋�����0�`100m
				u1t_roadChk = ROADCHK_OFF;			// ���H����s�v
			}
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
			if((u1t_roadSel == SET_ROAD_AUTO)		// �I�[�g�I����
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)		// and ����70km/h�ȉ�
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)				// and ����������
			&& (u1s_roadJdgSts == ROADJDG_STS_NORM)						// and ���H���ʂ���ʓ����s���
			&& (gps_distToHighway >= 0) && (gps_distToHighway <= 100)){	// and �������H�܂ł̋�����0�`100m
				u1t_roadChk = ROADCHK_OFF;			// ���H����s�v
			}
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

static void	TransitDummy(EM_TGTDEG emh_tgtDeg){

}

/**
 * ���݁C�}�C�G���A�����H
 */
Bool GpsCtrl_InMyArea(void)
{
    return g_InMyArea;
}

/**
 * ���݁C�}�C�L�����Z���G���A�����H
 */
Bool GpsCtrl_InMyCancelArea(void)
{
    return g_InMyCancelArea;
}
//--------------------------------------------------------------------------
//�y�֐��z���C���f�b�N�X���X�g�ǉ�����									
//�y�@�\�z																	
//�y���l�z																	
//--------------------------------------------------------------------------
static void	AddVisibleIdxLst(U2 u2h_idx){

	if(srcRefIdxListSize < GPS_VISIBLE_TGT_MAX){
		s_SrcRefIdxList[srcRefIdxListSize] = u2h_idx;
		srcRefIdxListSize++;
	}
}
//--------------------------------------------------------------------------
//�y�֐��z���C���f�b�N�X���X�g�\�[�g����									
//�y�@�\�z																	
//�y�����zU1 u1h_priIdx �F�D��^�[�Q�b�g�C���f�b�N�X						
//�y���l�z																	
//--------------------------------------------------------------------------
static void	SortVisibleIdxLst(U2 u2h_priIdx){

	U2	i,j;

	if(srcRefIdxListSize == 0){
		// �T�C�Y0�ŗD��^�[�Q�b�g������Ƃ��͂��Ȃ킿virtual
		if(u2h_priIdx != TGTNUM_NOTGT){
			s_SrcRefIdxList[0] = u2h_priIdx;
			srcRefIdxListSize = 1;
		}
		return;
	}
	else{
		sortSrcRefIdxByDistance();					// �܂������Ń\�[�g
	}

	// �ŗD��t�H�[�J�X�^�[�Q�b�g������΍ŗD��ɂ���
	if(u2h_priIdx != TGTNUM_NOTGT){
		//////////////////////////////////////////////////////////////////
		// 				�ŗD�悪�C���f�b�N�X���X�g�̉��Ԗڂ��T��		//
		//		i						��								//
		//pSrcRefIdxList[]	�����������������������������E�E�E�E		//
		//																//
		//////////////////////////////////////////////////////////////////
		for(i = 0; i < srcRefIdxListSize; i++){
			if(u2h_priIdx == s_SrcRefIdxList[i]){
				break;
			}
		}
		//////////////////////////////////////////////////////////////////
		// 	�擪���猩�������ꏊ�܂ł�S��1���Ɉړ�����			//
		//	�擪�ɍŗD��������Ă���									//
		//		j						i								//
		//pSrcRefIdxList[]	�@�A�B�C�D�E���G�H�I�J�K�L�M�E�E�E�E		//
		//		��														//
		//pSrcRefIdxList[]	���@�A�B�C�D�E�G�H�I�J�K�L�M�E�E�E�E		//
		//																//
		//////////////////////////////////////////////////////////////////
		if(i >= srcRefIdxListSize){		// �C���f�b�N�X��������Ȃ��Ƃ�(virtual target)
			// �擪�ȊO��1���ɂ��炷
			for(j = srcRefIdxListSize; j > 0 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// �擪���ŗD��̂��̂ɂ���
			s_SrcRefIdxList[0] = u2h_priIdx;
			srcRefIdxListSize++;
		}
		else if (i != 0){				// �擪�ȊO�Ŕ���������l�߂�
			for(j = i; j >= 1 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// �擪���ŗD��̂��̂ɂ���
			s_SrcRefIdxList[0] = u2h_priIdx;
		}
	}
}

//--------------------------------------------------------------------------
//�y�֐��z���C���f�b�N�X���X�g�\�[�g����									
//�y�@�\�z																	
//�y�����zU1 u1h_priIdx �F�D��^�[�Q�b�g�C���f�b�N�X						
//�y���l�z																	
//--------------------------------------------------------------------------
static void	SortVisibleIdxLst_2nd(U2 u2h_secondIdx){

	U2	i,j;

	if(u2h_secondIdx == TGTNUM_NOTGT || srcRefIdxListSize == 0)
	{
		return;
	}

	if(srcRefIdxListSize == 1){
		// �T�C�Y1�ő�2�D��^�[�Q�b�g������Ƃ�
		s_SrcRefIdxList[1] = u2h_secondIdx;
		srcRefIdxListSize = 2;
		return;
	}

	// ��2�D��t�H�[�J�X�^�[�Q�b�g��T���A��2�D��ɂ���
	if(u2h_secondIdx != TGTNUM_NOTGT){
		//////////////////////////////////////////////////////////////////
		// 				��2�D�悪�C���f�b�N�X���X�g�̉��Ԗڂ��T��		//
		//		i						��								//
		//pSrcRefIdxList[]	�����������������������������E�E�E�E		//
		//																//
		//////////////////////////////////////////////////////////////////
		for(i = 1; i < srcRefIdxListSize; i++){
			if(u2h_secondIdx == s_SrcRefIdxList[i]){
				break;
			}
		}
		//////////////////////////////////////////////////////////////////
		// 	2�Ԗڂ��猩�������ꏊ�܂ł�S��1���Ɉړ�����			//
		//	��2�D��������Ă���											//
		//		j						i								//
		//pSrcRefIdxList[]	�@�A�B�C�D�E���G�H�I�J�K�L�M�E�E�E�E		//
		//		��														//
		//pSrcRefIdxList[]	�@���A�B�C�D�E�G�H�I�J�K�L�M�E�E�E�E		//
		//																//
		//////////////////////////////////////////////////////////////////
		if(i != 1){				// 2�ԖڈȊO�Ŕ���������l�߂�
			for(j = i; j >= 2 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// �擪���ŗD��̂��̂ɂ���
			s_SrcRefIdxList[1] = u2h_secondIdx;
		}
	}
}

/**
 * �����i�� �� ���j���ɎQ�ƃC���f�b�N�X���\�[�g
 *
 * �V�F���\�[�g
 */
static void sortSrcRefIdxByDistance(void)
{
    U2 h, i, j;

    for (h = 1; h < (srcRefIdxListSize / 9); h = h * 3 + 1);

    for (; h > 0; h /= 3) {
        for (i = h; i < srcRefIdxListSize; i++) {
            j = i;
            while (j >= h
                   && sts_gpsmap[ s_SrcRefIdxList[j - h] ].u2_dst > sts_gpsmap[ s_SrcRefIdxList[j] ].u2_dst
                   ) {
                U2 temp = s_SrcRefIdxList[j];
                s_SrcRefIdxList[j] = s_SrcRefIdxList[j - h];
                s_SrcRefIdxList[j - h] = temp;
                j -= h;
            }
        }
    }
}

static void GuardPictNumber(ST_GPSMAP *psth_map){

	switch(psth_map->un_type.bit.b_code){
	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_TRAP_ZONE:
	case TGT_CHKPNT_ZONE:
	case TGT_MYAREA:
	case TGT_MYCANCEL:
	case TGT_ICANCEL:
	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_TRAP_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
	case TGT_NOFIX_SENSOR_YUDO:
		// �K�[�h���Ȃ��ł��̂܂�
		break;

	default:
		psth_map->un_extra.common.u2_mapPctNum = 0;
		break;
	}

}

// �����^�[�Q�b�g�ԍ��̕ύX
static Bool	UpdateSoundTarget(U2 *pu2h_sndTgtNum, U4 *pu4h_orgAddress, const ST_GPSMAP *psth_map, U2 u2h_mapNum){ 

	U2 	i;
	Bool	ret = FALSE;

	if(*pu2h_sndTgtNum != TGTNUM_NOTGT){					// �����^�[�Q�b�g�w�肠��
		for(i=0 ; i<u2h_mapNum ; i++, psth_map++){
			if(*pu4h_orgAddress == psth_map->u4_dataAddr){	// �f�[�^�̌��A�h���X��v�H
				ret = TRUE;
				*pu2h_sndTgtNum = i;						// �����^�[�Q�b�g�ԍ��X�V
				break;
			}
		}
		if(!ret){											// �Ȃ�����
			*pu2h_sndTgtNum = TGTNUM_NOTGT;					// �Ȃ�
			*pu4h_orgAddress = 0;							// �ꉞ�N���A
		}
	}

	return ret;

}

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt)
{
	psth_tgt->u1_sts = PRI_FOCUS_TGT_NONE;
	psth_tgt->u2_dst = U2_MAX;
	psth_tgt->u2_num = TGTNUM_NOTGT;
	psth_tgt->u2_absDegB = (U2)(180*DEG_LSB);
	psth_tgt->u1_priTunnelIn = FALSE;
}

Bool Has_tgtDeg(EM_TGTCODE code)
{
	if(TBL_TGTTRANS_PRM[code] == TYPE_TGTDEG_EXIST)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

static void update_gps_warning_lvl(EM_SYS_WARNING_LVL src_lvl, U2 dist)
{
	if(u1s_gpsWarningLvl_wrk == src_lvl)
	{
		if(u2s_gpsWarningLvl_dist_wrk > dist)
		{
			u2s_gpsWarningLvl_dist_wrk = dist;
		}
	}
	else if(u1s_gpsWarningLvl_wrk < src_lvl)
	{
		u1s_gpsWarningLvl_wrk = src_lvl;
		u2s_gpsWarningLvl_dist_wrk = dist;
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

static void TransitAreaStatusForRd(ST_GPSMAP *psth_map)
{
}

static void TransitAreaStatusForSc(ST_GPSMAP *psth_map, U1 degChk)
{
}

// �J�X�^���f�[�^�̗L�����ԃ`�F�b�N
typedef struct{
	U2 year;
	U1 month;
	U1 day;
	U1 hour;
	U1 min;
}ST_CUSTOM_DATE;

static U4 month2day(U1 month, U2 year)
{
						   //  1  2  3  4  5  6  7  8  9  10 11 12
	static const U4 cDay[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
	
	if(month == 2){
		return 28 + (1 / ((U4)year % 4 + 1)) * (1 - 1 / ((U4)year % 100 + 1)) + (1 / ((U4)year % 400 + 1));
	}
	else{
		return cDay[month-1];
	}
}

static U4 getMinute(ST_CUSTOM_DATE date, U2 base_year)
{
	S4 n = (S4)date.year - (S4)base_year;
	U4 min = 0;
	U1 i;
	
	if(n < 0) return 0;
	

	for(; n>0; n--){
		for(i=1; i<=12 ; i++){
			min += month2day(i, base_year) * 24 * 60;
		}
	}
	
	for(i=1; i<date.month; i++){
		min += month2day(i, date.year) * 24 * 60;
	}
	min += (U4)(date.day-1) * 24 * 60 + (U4)date.hour * 60 + (U4)date.min;
	return min;
}

static Bool isCustomDateValid(ST_HEADER *header)
{
#define GUARD_MIN (24 * 60) // �K�[�h���� 24����(24*60��)
	ST_CUSTOM_DATE ctm;
	ST_CUSTOM_DATE gps;
	
	// gps
	gps.year  = 2000 + (U2)stg_gpsSts.u1_year;
	gps.month = stg_gpsSts.u1_month;
	gps.day   = stg_gpsSts.u1_day;
	gps.hour  = stg_gpsSts.u1_hour;
	gps.min   = stg_gpsSts.u1_min;
	// custom
	ctm.year  = 2000 + (U2)header->u1_year;
	ctm.month = header->u1_month;
	ctm.day   = header->u1_day;
	ctm.hour  = header->u1_hour;
	ctm.min   = header->u1_min;

	// gps<ctm �G���[�iGPS���Â����Ƃ͂��肦�Ȃ�)
	if((U4)gps.year * 100 + (U4)gps.month < (U4)ctm.year * 100 + (U4)ctm.month){
		return FALSE;
	}
	if((gps.year == ctm.year)
	&& (gps.month == ctm.month)
	&& ((U4)gps.day * 100 * 100 + (U4)gps.hour * 100 + (U4)gps.min < 
	    (U4)ctm.day * 100 * 100 + (U4)ctm.hour * 100 + (U4)ctm.min)){
		return FALSE;
	}
	// 2�N�ȏ�
	if(gps.year - ctm.year >= 2){
		return FALSE;
	}

	if(gps.year == ctm.year && gps.month == ctm.month){
		// ���A�N�������B�������𕪂ɂ��Ĕ�r
		U4 gps_min = gps.day * 24 * 60 + gps.hour * 60 + gps.min;
		U4 ctm_min = ctm.day * 24 * 60 + ctm.hour * 60 + ctm.min;
		return (gps_min - ctm_min >= GUARD_MIN) ? FALSE : TRUE;
	}
	else{
		// ���A�N���ׂ��Bctm_year��̕����r
		U4 gps_min = getMinute(gps, ctm.year);
		U4 ctm_min = getMinute(ctm, ctm.year);
		return (gps_min - ctm_min >= GUARD_MIN) ? FALSE : TRUE;
	}
}



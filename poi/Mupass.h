//--------------------------------------------------------------------------//
//	MUPASS�w�b�_�t�@�C��													//
//--------------------------------------------------------------------------//
// ��d�C���N���[�h�h�~
#ifndef __MUPASS_H__
#define	__MUPASS_H__

#include "common.h"

//--------------------------------------------------------------------------//
//  �^��`																	//
//--------------------------------------------------------------------------//
// �I�[�r�X�n���m
typedef struct{
// �g����̗v��
	U1	b_lmtSpd	:1;							// �������x��
	U1	b_spdOver	:1;							// ���x���ߋ�
	U1	b_camera	:1;							// �J�������m
	U1	b_runSpd	:1;							// ���s���x��
	U1	idvVoiceHi	:3;							// �ʃ{�C�XHi
	U1				:1;							// �\��
// �g�����
	U1	lmtSpdVal	:4;							// �������x�l
	U1	cameraPos	:3;							// �J�����ʒu
	U1				:1;

	U1	idvVoiceLo;								// �ʃ{�C�XLo
}ST_ORBIS_VCGPS;

// ����E����G���A���m
typedef struct{
// �g����̗v��
	U1	b_lmtSpd	:1;							// �������x��
	U1	b_spdOver	:1;							// ���x���ߋ�
	U1	b_level		:1;							// ���x����
	U1	b_near		:1;							// �ߐ�
	U1	b_rngOut	:1;							// ���O��
	U1				:3;							// �\��
// �g�����
	U1	lmtSpdVal	:4;							// �������x�l
	U1	levelVal	:3;							// ���x���l
	U1				:1;							// �\��
	
	U1	method;									// ��@
}ST_TRAPCHK_VCGPS;

// SA�EPA�EOA���m
typedef struct{
// �g����̗v��
	U1	b_smartIC	:1;							// �X�}�[�gIC��
	U1	b_gas		:1;							// �K�\�����X�^���h��
	U1				:6;							// �\��
// �g�����
	U1	smartIC		:3;							// �X�}�[�gIC���
	U1				:5;							// �\��

	U1	gasBrand;								// �K�\�������
}ST_SAPAOA_VCGPS;

typedef struct{
// �g����̗v��
	U1	b_lmtSpd	:1;							// �������x��
	U1	b_spdOver	:1;							// ���x���ߋ�
	U1				:6;							// �\��
// �g�����
	U1	lmtSpdVal	:4;							// �������x�l
	U1				:4;							// �\��

	U1	rsv;									// �\��
}ST_SCP_VCGPS;

// �}�J�[�u
typedef struct{
// �g����̗v��
	U1	b_curve		:1;							// �J�[�u��ʋ�
	U1				:7;
// �g�����
	U1	curveType	:3;							// �J�[�u���
	U1				:5;							// �\��

	U1	rsv;									// �\��
}ST_CURVE_VCGPS;

// ����E�����|�C���g
typedef struct{
// �g����̗v��
	U1	b_brajct	:1;							// ����E������ʋ�
	U1				:7;
// �g�����
	U1	brajctType	:3;							// ���򍇗����
	U1				:5;							// �\��

	U1	rsv;									// �\��
}ST_BRAJCT_VCGPS;

// ����
typedef struct{
// �g����̗v��
	U1	b_kenkyo	:1;							// �s���{����
	U1				:7;							// �\��
// �g�����
	U1	kenkyoNum;								// �s���{���ԍ�
	U1	rsv;									// �\��
}ST_KENKYO_VCGPS;

// �g���l��
typedef struct{
// �g����̗v��
	U1	b_tunnel	:1;							// ������
	U1				:7;							// �\��
// �g�����
	U1	tunnelType	:3;							// �g���l�����
	U1				:5;							// �\��

	U1	rsv;									// �\��
}ST_TUNNEL_VCGPS;

// ETC���[���K�C�h
typedef struct{
// �g����̗v��
	U1	b_etc		:1;							// ETC��
	U1				:7;							// �\��
// �t�����
	U1	etcGuide	:3;							// ETC�K�C�h���
	U1				:5;							// �\��

	U1	rsv;									// �\��
}ST_ETC_VCGPS;

// �^�[�Q�b�g���m�^
typedef struct{
	// ���ʕ���(3byte)
	struct{
		U1	u1_trgt;								// �^�[�Q�b�g

		U1	b_tunnel	:2;							// �g���l����
		U1	b_dir		:2;							// ������
		U1	b_dist		:4;							// ������

		U1	b_highway	:1;							// ���H������
		U1	b_tn_ext	:1;							// �g���l���g��(�o������)
		U1	b_idvVc		:1;							// �ʃ{�C�X�w��
		U1	b_countdown	:1;							// �J�E���g�_�E��
		U1				:4;							// �\��
	}common;
	// �g������(3byte)
	union{
		ST_ORBIS_VCGPS		orbis;
		ST_TRAPCHK_VCGPS	trapchk;
		ST_SAPAOA_VCGPS		sapaoa;
		ST_SCP_VCGPS		scp;
		ST_CURVE_VCGPS		curve;
		ST_BRAJCT_VCGPS		brajct;
		ST_KENKYO_VCGPS		kenkyo;
		ST_TUNNEL_VCGPS		tunnel;
		ST_ETC_VCGPS		etc;
		U1					byte[3];
	}extra;
}ST_VCGPS;


//--------------------------------------------------------------------------//
//  �}�N����`																//
//--------------------------------------------------------------------------//

enum{
	VC_CH_LEIPRI_HI = 0,
	VC_CH_LEIPRI_LOW,
};

// ���������`�����l����`
typedef enum{
	VC_CH0 = 0,					// CH0. �ō��D��(�V�X�e���E�ꕔ��RD)
	VC_CH1,						// CH1. RD�֘A
	VC_CH2,						// CH2. GPS�֘A
	VC_CH3,						// CH3. �X�L���i�֘A
	VC_CH4,						// CH4. Lei����
	VC_CH_MAX = VC_CH4,
	VC_CH_ALL,					// �SCH
	VC_CH_NONE,
}EM_VC_CH;

// �^�[�Q�b�g���m�T�u���
// ����
enum{
	VCDIR_NONE = 0,				// ����Ȃ�
	VCDIR_FWRD,					// �O
	VCDIR_LEFT,					// ��
	VCDIR_RIGHT,				// �E
	VCDIR_CANT_REJUDGE,			// �Ĕ���s��
};
// ����
enum{
	VCDIST_NONE = 0,			// ����Ȃ�
	VCDIST_50M,					// ������
	VCDIST_100M,				// 100m��
	VCDIST_200M,				// 200m��
	VCDIST_300M,				// 300m��
	VCDIST_500M,				// 500m��
	VCDIST_900M,				// ���̐�
	VCDIST_1KM,					// 1Km��
	VCDIST_1900M,				// ���̐�
	VCDIST_2KM,					// 2km��
	VCDIST_SOON,				// �܂��Ȃ�
};
// �^�[�Q�b�g��
enum{
	VCTGT_RD_ORBIS = 0,			// ���[�_�[���I�[�r�X
	VCTGT_LP_ORBIS,				// ���[�v�R�C�����I�[�r�X
	VCTGT_HSYS_ORBIS,			// H�V�X�e�����I�[�r�X
	VCTGT_LHSYS_ORBIS,			// LH�V�X�e�����I�[�r�X

	VCTGT_TRAP_ZONE,			// �g���b�v�]�[��
	VCTGT_CHKPNT_ZONE,			// �`�F�b�N�|�C���g�]�[��
	VCTGT_MYAREA,				// �}�C�G���A
	VCTGT_CROSSING,				// �����_�Ď�
	VCTGT_SIGNAL,				// �M�������}�~�V�X�e��
	VCTGT_SZ_AREA,				// ���֍ŏd�_�G���A
	VCTGT_Z_AREA,				// ���֏d�_�G���A
	VCTGT_HIGHWAY_POLICE,		// �����x�@��
	VCTGT_SCP,					// ���x�ؑփ|�C���g
	VCTGT_SHAJYO_AREA,			// �ԏ�_���G���A
	VCTGT_ZONE30_AREA,			// �]�[��30

	VCTGT_NSYS,					// N�V�X�e��
	VCTGT_TRFCHK,				// ��ʊĎ��V�X�e��
	VCTGT_POLICE_STATION,		// �x�@��
	VCTGT_ACCIDENT,				// ���̑����G���A
	VCTGT_CURVE,				// �}�J�[�u
	VCTGT_BRAJCT,				// ����E����

	VCTGT_MICHINOEKI,			// ���̉w
	VCTGT_SA,					// �T�[�r�X�G���A
	VCTGT_PA,					// �p�[�L���O�G���A
	VCTGT_HWOASYS,				// �n�C�E�F�C�I�A�V�X
	VCTGT_HWRADIO,				// �n�C�E�F�C���W�I
	VCTGT_KENKYO,				// ����
	VCTGT_TUNNEL,				// �g���l��
	VCTGT_TORUPA,				// �Ƃ��
	VCTGT_ETC,					// ETC�Q�[�g

	VCTGT_KOBAN,				// ���
};

// ����������ʒ�`
typedef enum{
	VC_NONE = 0,
// CH0
	VC_STOP0,
	VC_DOWNSTA,
	VC_DOWNEND,
	VC_DATTXEND,
	VC_PWRON1,								// �d��ON1
	VC_PWRON2,								// �d��ON2
	VC_OPRT1,								// ���쉹1
	VC_OPRT2,								// ���쉹2
	VC_OPRT3,								// ���쉹3
	VC_OPRT4,								// ���쉹4							10
	VC_OPRT5,								// ���쉹5
	VC_OPRT6,								// ���쉹6
	VC_OPRT7,								// ���쉹7
	VC_OPRT8,								// ���쉹8
	VC_OPRT9,								// ���쉹9
#if 0
// �u�U�[��p�e�X�g
	VC_BZ_OPRT,								// �d�q���쉹
	VC_BZ_OPRT2,							// �d�q���쉹2
	VC_BZ_OPRT3,							// �d�q���쉹3
	VC_BZ_OPRTNACK,							// ���싑�ۉ�
	VC_BZ_REGACK,							// �o�^OK
	VC_BZ_REGNACK,							// �o�^NG
	VC_BZ_FUNCON,							// �@�\ON
	VC_BZ_FUNCOFF,							// �@�\OFF
	VC_BZ_REJECT,							// ���ۉ�
	VC_BZ_MANRST,							// ���Z�b�g��
	VC_BZ_ADJMDRDY,							// �������[�h������
	VC_BZ_ADJMD,							// �������[�h��
	VC_BZ_FORMAT,
	VC_BZ_DEFRAG,
#endif
	// GPS����֘A
	VC_MYAREA_REG_ACK,						// �}�C�G���A�o�^��
	VC_MYAREA_DELAYED_REG_ACK,				// �}�C�G���A�o�^��
	VC_MYAREA_REG_NACK,						// �}�C�G���A�o�^���ۉ�
	VC_MYAREA_DELAYED_REG_NACK,				// �}�C�G���A�o�^���ۉ�
	VC_MYAREA_DEL,							// �}�C�G���A�폜��
	VC_MYAREA_DELAYED_DEL,					// �}�C�G���A�폜��
	VC_MYCANCEL_REG_ACK,					// �}�C�L�����Z���o�^��
	VC_MYCANCEL_DELAYED_REG_ACK,			// �}�C�L�����Z���o�^��
	VC_MYCANCEL_REG_NACK,					// �}�C�L�����Z���o�^���ۉ�
	VC_MYCANCEL_DELAYED_REG_NACK,			// �}�C�L�����Z���o�^���ۉ�
	VC_MYCANCEL_DEL,						// �}�C�L�����Z���폜��
	VC_MYCANCEL_DELAYED_DEL,				// �}�C�L�����Z���폜��
	VC_MYCANCEL_SET_OFF,					// �}�C�L�����Z���ݒ�OFF��
	VC_MYCANCEL_DELAYED_SET_OFF,			// �}�C�L�����Z���폜��
	VC_GPS_OPRT_WAIT,						// GPS����҂�
	VC_GPS_DELAYED_OPRT_WAIT,				// GPS����҂�
	VC_GPS_OPRT_FAIL,				//32	// GPS���쎸�s

	VC_TST0M,								// �e�X�g��0�����f�B
	VC_TST0M2,								// �e�X�g��0�����f�B2
	VC_TST0M3,								// �e�X�g��0�����f�B3
	VC_TST0M_ROT,							// �e�X�g��0�����f�B���[�e�[�V����
	VC_TST0V,								// �e�X�g��0�{�C�X		30
	VC_TST0QV,								// �e�X�g��0�{�C�X
	VC_TST1,								// �e�X�g��1
	VC_TST2,								// �e�X�g��2
	VC_TST3,						//41	// �e�X�g��3
	VC_TSTALL,

	VC_TST_OPENING,							// �J�X�^���e�X�g �I�[�v�j���O
	VC_TST_ORBIS_JINGLE,					// �J�X�^���e�X�g �I�[�r�X�W���O��
	VC_TST_GPSWRN_JINGLE,					// �J�X�^���e�X�g GPS�x��W���O��
	VC_TST_GPSINF_JINGLE,					// �J�X�^���e�X�g GPS���m�W���O��
	VC_TST_SC_JINGLE,						// �J�X�^���e�X�g �����W���O��
	VC_TST_GPS_1STFIX,						// �J�X�^���e�X�g GPS��������
	VC_TST_RD_MELODY,						// �J�X�^���e�X�g ���[�_�[�����f�B

	VC_TST_GOOD_MORNING,					// �e�X�g ��
	VC_TST_GOOD_AFTERNOON,					// �e�X�g ��
	VC_TST_GOOD_EVENING,					// �e�X�g ��

	VC_RD_STEALTH,							// RD �X�e���X
	VC_RD_STEALTH2,							// RD �X�e���X
	VC_RD_HSYS,								// RD H�V�X�e��
	VC_ORBIS_PASS,							// �I�[�r�X�ʉ�
	VC_MYAREA_PASS,							// �}�C�G���A�ʉ�
	VC_TSTMODE,								// �e�X�g���[�h��
	VC_LOWBATT,						//48	// ���[�o�b�e���[��
	VC_MOUSE_TRAP_RD,						// �l�Y�~�߂�RD�x��
	VC_MOVE_ORBIS_RD,						// �ړ��I�[�r�XRD�x��

// CH1
	VC_STOP,								// ��~
	VC_FORCE_STOP,							// ������~
	VC_RD_MELODY,							// RD �����f�B
	VC_RD_MELODY2,							// RD �����f�B2
	VC_RD_MELODY3,							// RD �����f�B3
	VC_RD_MELODY4,							// RD �����f�B4
	VC_RD_VOICE,							// RD �{�C�X
	VC_RD_VOICE2,							// RD �{�C�X2

	VC_ICANCEL_ACT,							// I�L�����Z���쓮��
	VC_SELFTST_MELODY,
	VC_SELFTST_VOICE,

// CH3
	VC_SC_TRAP,								// ���� �g���b�v		10
	VC_SC_CARLOC_FAR,						// ���� �J�[���P����
	VC_SC_CARLOC_NEAR,						// ���� �J�[���P�ߐ�
	VC_SC_CARLOC_OUT,						// ���� �J�[���P���O
	VC_SC_DIGITAL,							// ���� �f�W�^��
	VC_SC_HELITELE,							// ���� �w���e��
	VC_SC_TOKUSHOU,							// ���� ����
	VC_SC_KEIDEN,							// ���� �x�@�d�b
	VC_SC_KATSUDOU,							// ���� �x�@����
	VC_SC_SHOKATSU,							// ���� �����n
	VC_SC_FIRE,								// ���� ���h			20
	VC_SC_FIRE_HELITELE,					// ���� ���h�w���e��
	VC_SC_WRECKER,							// ���� ���b�J�[
	VC_SC_EMERGENCY,						// ���� �~�}
	VC_SC_JH,								// ���� JH
	VC_SC_KEIBI,							// ���� �x��
	VC_SC_HEISO_TSUIBI,						// ���� �����ǔ�
	VC_SC_SURECHIGAI,						// ���� ���ꂿ����
	VC_SC_TORISHIMARI,						// ���� ���
	VC_SC_KENMON,							// ���� ����

	VC_STOP3,
// CH2
	VC_GPS_1STFIX,							// GPS��������
	VC_GPS_1STFIX_GODD_MORNING,				// GPS�������� ���͂悤�������܂�
	VC_GPS_1STFIX_GODD_AFTERNOON,			// GPS�������� ����ɂ���
	VC_GPS_1STFIX_GODD_EVENING,				// GPS�������� ����΂��
	VC_GPS_1STFIX_HAPPY_NEW_YEAR,			// GPS�������� �����܂��Ă��߂łƂ��������܂�
	VC_GPS_1STFIX_MERRY_XMAS,				// GPS�������� �����[�N���X�}�X
	VC_GPS_SOKUI,							// GPS����
	VC_GPS_NOSOKUI,							// GPS�񑪈�
	VC_GPS_SEARCH,							// GPS�T�[�`��
	VC_RELAX_CHIME,							// �����b�N�X�`���C��
	VC_AM_0_OCLOCK,
	VC_AM_1_OCLOCK,
	VC_AM_2_OCLOCK,
	VC_AM_3_OCLOCK,
	VC_AM_4_OCLOCK,
	VC_AM_5_OCLOCK,
	VC_AM_6_OCLOCK,
	VC_AM_7_OCLOCK,
	VC_AM_8_OCLOCK,
	VC_AM_9_OCLOCK,
	VC_AM_10_OCLOCK,
	VC_AM_11_OCLOCK,
	VC_PM_0_OCLOCK,
	VC_PM_1_OCLOCK,
	VC_PM_2_OCLOCK,
	VC_PM_3_OCLOCK,
	VC_PM_4_OCLOCK,
	VC_PM_5_OCLOCK,
	VC_PM_6_OCLOCK,
	VC_PM_7_OCLOCK,
	VC_PM_8_OCLOCK,
	VC_PM_9_OCLOCK,
	VC_PM_10_OCLOCK,
	VC_PM_11_OCLOCK,

	VC_ENG_AM_0_OCLOCK,
	VC_ENG_AM_1_OCLOCK,
	VC_ENG_AM_2_OCLOCK,
	VC_ENG_AM_3_OCLOCK,
	VC_ENG_AM_4_OCLOCK,
	VC_ENG_AM_5_OCLOCK,
	VC_ENG_AM_6_OCLOCK,
	VC_ENG_AM_7_OCLOCK,
	VC_ENG_AM_8_OCLOCK,
	VC_ENG_AM_9_OCLOCK,
	VC_ENG_AM_10_OCLOCK,
	VC_ENG_AM_11_OCLOCK,
	VC_ENG_PM_0_OCLOCK,
	VC_ENG_PM_1_OCLOCK,
	VC_ENG_PM_2_OCLOCK,
	VC_ENG_PM_3_OCLOCK,
	VC_ENG_PM_4_OCLOCK,
	VC_ENG_PM_5_OCLOCK,
	VC_ENG_PM_6_OCLOCK,
	VC_ENG_PM_7_OCLOCK,
	VC_ENG_PM_8_OCLOCK,
	VC_ENG_PM_9_OCLOCK,
	VC_ENG_PM_10_OCLOCK,
	VC_ENG_PM_11_OCLOCK,

	VC_DIMMER_NIGHT,
	VC_DIMMER_SENSOR,
	VC_ECOPOINT_FULL,
	VC_ECOPOINT_DEC,
	VC_REMINDER_OIL,
	VC_REMINDER_OIL_ELEMENT,
	VC_REMINDER_TIRE,
	VC_REMINDER_BATTERY,
	VC_AMBIENT_HIGH,
	VC_THROTTLE_OVER,
	VC_ENGLOAD_OVER,
	VC_TACHO_OVER,

	VC_STOP2,

	VC_GPSVAR,								// GPS�ω�
	
	VC_LEI_DUMMY,							// LEI�pdummy

	VC_ANYON,								// �����ꂩ��CH ON
	VC_MAX,									// �K�[�h�l				58
}EM_VC;

#endif

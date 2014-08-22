
#ifndef __SETMGR_H__
#define __SETMGR_H__

#include "comdat.h"

//--------------------------------------------------------------------------//
//	�}�N����`																//
//--------------------------------------------------------------------------//
// �ݒ�l�A�N�Z�X�I��
typedef enum{
	SET_ACS_LCL = 0,						// ���[�J�����[�h�ݒ�l
	SET_ACS_DRV,							// �h���C�u���[�h�ݒ�l
	SET_ACS_ALL,							// �I�[��ON���[�h�ݒ�l
	SET_ACS_NORMAL,							// �m�[�}���ݒ�l
	SET_ACS_MINIMUM,						// �~�j�}���ݒ�l
	SET_ACS_SPECIAL,						// �X�y�V�����ݒ�l
	SET_ACS_MAX,							// �K�[�h�l
}EM_ACS_SEL;

// ���H�I��
typedef enum{
	SET_ROAD_NORM = 0,						// ��ʓ��̂�
	SET_ROAD_HIGH,							// �������̂�
	SET_ROAD_ALL,							// �S��
	SET_ROAD_AUTO,							// �I�[�g
}EM_SET_ROAD;

// ����x�񃌃x��
typedef enum{
	SET_TRAPCHK_WRNLVL_OFF = 0,				// �x�񂵂Ȃ�
	SET_TRAPCHK_WRNLVL_ALL,					// �S���x��
	SET_TRAPCHK_WRNLVL_2,					// ���x��2�ȏ�
	SET_TRAPCHK_WRNLVL_3,					// ���x��3�ȏ�
	SET_TRAPCHK_WRNLVL_4,					// ���x��4�ȏ�
	SET_TRAPCHK_WRNLVL_5,					// ���x��5�ȏ�
}EM_SET_TRAPCHK_WRNLVL;

// GPS�ݒ�
typedef struct{
	U1							:3;			// Reserved
	U1			b_orbis			:1;			// �I�[�r�X���m(ON/OFF)
	U1			b_orbisPassSpd	:1;			// �I�[�r�X�ʉߑ��x���m(ON/OFF)
	U1			b_orbisPass		:1;			// �I�[�r�X�ʉߍ��m(ON/OFF)
	U1			b_hwOrbisLmtSpd	:1;			// �������I�[�r�X�������x���m(ON/OFF)
	U1			b_orbisCameraPos:1;			// �I�[�r�X�J�����ʒu���m(ON/OFF)

	U1			b_nSys			:1;			// N�V�X�e�����m(ON/OFF)
	U1			b_trafChkSys	:1;			// ��ʊĎ��V�X�e�����m(ON/OFF)
	U1			b_hwPolice		:1;			// ��ʌx�@��(ON/OFF)
	U1			b_crossing		:1;			// �����_�Ď����m(ON/OFF)
	U1			b_signal		:1;			// �M�������}�~�V�X�e��(ON/OFF)
	U1			b_accident		:1;			// ���̑����G���A���m(ON/OFF)
	U1			b_police		:1;			// �x�@�����m(ON/OFF)
	U1			b_michinoeki	:1;			// ���̉w���m(ON/OFF)

	U1			b_sa			:1;			// �T�[�r�X�G���A���m(ON/OFF)
	U1			b_pa			:1;			// �p�[�L���O�G���A���m(ON/OFF)
	U1			b_hwOasys		:1;			// �n�C�E�F�C�I�A�V�X���m(ON/OFF)
	U1			b_hwRadio		:1;			// �n�C�E�F�C���W�I��M�G���A���m(ON/OFF)
	U1			b_hwChgLmtSpd	:1;			// �������������x�ؑ֒n�_���m(ON/OFF)
	U1			b_spdOver		:1;			// ���x���ߍ��m(ON/OFF)
	U1			b_parkChkArea	:1;			// ���֊Ď��G���A���m(ON/OFF)
	U1			b_parking		:1;			// ���ԏ�\��(ON/OFF)

	U1			b_trapWrnLvl	:3;			// ����x�񃌃x��(EM_SET_TRAPCHK_WRNLVL)
	U1			b_chkWrnLvl		:3;			// ����x�񃌃x��(EM_SET_TRAPCHK_WRNLVL)
	U1			b_smartIC		:1;			// �X�}�[�gIC(ON/OFF)
	U1			b_hwGas			:1;			// �������K�X�X�e�[�V����(ON/OFF)

	U1			b_curve			:1;			// �}�J�[�u(ON/OFF)
	U1			b_brajct		:1;			// ����E����(ON/OFF)
	U1			b_tunnel		:1;			// �����E�A���g���l��(ON/OFF)
	U1			b_kenkyo		:1;			// ����(ON/OFF)
	U1			b_etcLane		:1;			// ETC���[��(ON/OFF)
	U1			b_torupa		:1;			// �r���[�|�C���g�p�[�L���O(ON/OFF)
	U1			b_shajyoArea	:1;			// �ԏ�_�������G���A(ON/OFF)
	U1			b_toilet		:1;			// ���O�g�C��(ON/OFF)

	U1			b_tmpstop		:1;			// �ꎞ��~���Ӄ|�C���g(ON/OFF)
	U1			b_fumikiri		:1;			// ����(ON/OFF)
	U1			b_koban			:1;			// ���(ON/OFF)
	U1			b_fire			:1;			// ���h��(ON/OFF)
	U1			b_hoiku			:1;			// �c�t���E�ۈ牀(ON/OFF)
	U1							:1;			// �����o�Y(ON/OFF)
	U1			b_zone30		:1;			// �]�[��30(ON/OFF)
	U1							:1;			// reserved

}ST_SET_GPS;

// ���̑��ݒ�
typedef struct{

	U1			b_areaMode		:2;			// �G���A���[�h(EM_SET_AREA)
	U1			b_voiceMode		:2;			// �������[�h(EM_VOICE_MODE)
	U1			b_panelMode		:2;			// �p�l�����[�h(EM_PANEL_MODE)
	U1			b_spdWarn		:2;			// ���x�x��(EM_SET_SPDWRN)

	U1			b_idvOrbisVoice	:1;			// �ʃI�[�r�X�{�C�X(ON/OFF)
	U1			b_scClassic		:1;			// �����N���V�b�N(ON/OFF)
	U1			b_voiceType		:3;			// �����^�C�v(EM_VOICE_TYPE)
	U1			b_photo_zoom	:2;			// �ʐ^�Y�[������(EM_PHOTO_ZOOM)
	U1			b_in_photo_clock:1;			// �ʐ^�����v(ON/OFF)

	U1			b_photo_effect	:3;			// �ʐ^�G�t�F�N�g(EM_PHOTO_EFFECT)
	U1			b_photo_chgtim	:3;			// �ʐ^�ؑ֎���(EM_PHOTO_CHGTIM)
	U1			b_photo_color	:2;			// �ʐ^�F(EM_PHOTO_COLOR)
	
	U1			b_settingMode	:3;			// �V�X�e���ݒ�L��(EM_SETTING_MODE)
	U1			b_brightness	:2;			// ���邳(EM_BRIGHTNESS)
	U1			b_relaxChime	:2;			// �����b�N�X�`���C��(EM_RELAX_CHIME)
	U1			b_oprtSnd		:1;			// ���쉹(ON/OFF)

	U1			b_dspRev		:1;			// ��ʔ��](ON/OFF)
	U1			b_vol			:3;			// ���ʐݒ�(0�`7 �傫�������ʑ�)
	U1			b_manner		:1;			// �}�i�[���[�h(ON/OFF)
	U1			b_headingUp		:1;			// ���[�_�[��ʂ̕\���`��(EM_HEADING_UP)
	U1			b_wrnDspChg		:2;			// �x��\���؂�ւ�(EM_DSP_CHG_PT)

	U1			b_logDisp		:1;			// ���O�̃p�[�Z���g�\���iON/OFF�j
	U1			b_wrnPhoto		:1;			// �x��ʐ^�\��(ON/OFF)
	U1			b_lcdDsp		:1;			// LCD�\��(ON/OFF)
	U1			b_logFunc		:1;			// ���O�@�\(ON/OFF)
	U1			b_obdIllCtrl	:1;			// OBD�C���~�R���g���[��(ON/OFF)
	U1							:1;			// ��
	U1			b_demo			:2;			// �X���f��(EM_SET_DEMO)

	U1			b_wtDsp			:5;			// �Ҏ���(EM_SET_WTDSP)
	U1			b_tideAuto		:1;			// �����������I��(ON/OFF)
	U1			b_ledmode		:2;			// LED(EM_LED_MODE)

	U1			b_rdSnd			:5;			// ���[�_�[�x�񉹐ݒ�(EM_SET_RDSND)
	U1			b_rdSens		:3;			// ���[�_�[��M���x���[�h�ݒ�(EM_SET_SENS)

	U1			b_roadSel		:2;			// GPS���H�I��ݒ�(EM_SET_ROAD)
	U1			b_sc			:2;			// �����x��ݒ�(EM_SET_SC)
	U1			b_sokuiAna		:1;			// ���ʃA�i�E���X(ON/OFF)
	U1			b_dispAlways	:1;			// �펞�\��(ON/OFF)
	U1			b_jiho			:1;			// ����(ON/OFF)
	U1			b_dimmer		:1;			// �f�B�}�[(EM_SET_FLEX_DIMMER)

	U1			u1_car_maker;				// �ԗ����[�J�[

	U2			u2_observer;				// �������ԍ�
	
	U1			b_mapMode		:3;			// �}�b�v���[�h(EM_SET_MAP_MODE)
	U1							:1;			// ��
	U1			b_movSmoothing	:1;			// �ړ��X���[�W���O(ON/OFF)
	U1			b_mapScrollFocus:1;			// �X�N���[���t�H�[�J�X(ON/OFF)
	U1			b_wideRangeSrch	:1;			// �^�[�Q�b�g���C�h�����W�T�[�`(ON/OFF)
	U1			b_mapZoom		:1;			// �Y�[���\��

	U1			u1_classicScopeMtr;
	
	union{
		U1		u1_mapIcon;											// �n�}�A�C�R���\��
		
		struct{
			U1			etc									:1;		// ���̑�
			U1			cv									:1;		// �R���r�j
			U1			ff									:1;		// �t�@�[�X�g�t�[�h
			U1			fr									:1;		// �t�@�~���X
			U1			gs									:1;		// �K�\�����X�^���h
		}b_mapIcon;
	};
	
	U1			u1_turnOnPhoto;				// �^�[���I���ʐ^
	
	U1			b_roadPres		:1;			// ���H�C���ݒ�(ON/OFF)
	U1			b_WLAN			:1;			// WLAN�ݒ�(ON/OFF)
	U1			b_turnOnMovie	:3;			// �^�[���I�����[�r�[(EM_TURNON_MOVIE)
	U1			b_auto_chgtim	:3;			// �Ҏ�AUTO�ؑ֎���(EM_AUTO_CHGTIM)
	
	U1			b_scopeType		:2;			// �X�R�[�v�^�C�v(EM_SET_SCOPE_TYPE)
	U1			b_toriDisp		:2;			// ���J����\��(EM_SET_TORI_DISP)
	U1			b_mapColor		:2;			// �}�b�v�F(EM_SET_MAP_COLOR)
	U1			b_lei			:1;			// Lei mode
	U1			b_leiCharaSD	:1;			// Lei �L����normal,SD(���у��C)
	U1			rsv00[12];

}ST_SET_MISC;

#endif


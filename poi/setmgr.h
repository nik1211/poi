//--------------------------------------------------------------------------//
//	�ݒ�}�l�[�W���w�b�_�t�@�C��											//
//--------------------------------------------------------------------------//
#ifndef __SETMGR_H__
#define	__SETMGR_H__

#include "comdat.h"

//--------------------------------------------------------------------------//
//	�}�N����`																//
//--------------------------------------------------------------------------//
// �ݒ�l�A�N�Z�X�I��
typedef enum{
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
	U1			b_roadSel		:2;			// GPS���H�I��ݒ�(EM_SET_ROAD)
}ST_SET_MISC;

extern ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS�ݒ�l
extern ST_SET_MISC				stg_setMisc;					// ���̑��ݒ�l
extern EM_ACS_SEL				u1g_setAcsSel;					// �ݒ�l�I��

extern void	SetMgrInit();

#endif


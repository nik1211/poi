//--------------------------------------------------------------------------//
//�y���O�z�ݒ�}�l�[�W��													//
//�y�@�\�z�ݒ�l�̓Ǐo���E�L���𓝊��Ǘ�����								//
//�y���l�z																	//
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
// �C���N���[�h�t�@�C��														//
//--------------------------------------------------------------------------//
#include "stdafx.h"
#include "SetMgr.h"
#include "SetMgr.prm"

//--------------------------------------------------------------------------//
// �O���ϐ���`																//
//--------------------------------------------------------------------------//
ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS�ݒ�l
ST_SET_MISC				stg_setMisc;					// ���̑��ݒ�l
EM_ACS_SEL				u1g_setAcsSel;					// �ݒ�l�I��

//--------------------------------------------------------------------------//
//�y�֐��z����������														//
//�y�@�\�z�ݒ�l��EEPROM����ǂݏo��										//
//--------------------------------------------------------------------------//
void	SetMgrInit(void){

	memcpy(&stg_setGps[0], &u1_TBL_SETGPS_DEF[0][0], sizeof(ST_SET_GPS)*4);
	memcpy(&stg_setMisc, &u1_TBL_SETMISC_DEF[0], sizeof(ST_SET_MISC));
	u1g_setAcsSel = SET_ACS_ALL;
}
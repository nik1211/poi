//--------------------------------------------------------------------------//
//【名前】設定マネージャ													//
//【機能】設定値の読出し・記憶を統括管理する								//
//【備考】																	//
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
// インクルードファイル														//
//--------------------------------------------------------------------------//
#include "stdafx.h"
#include "SetMgr.h"
#include "SetMgr.prm"

//--------------------------------------------------------------------------//
// 外部変数定義																//
//--------------------------------------------------------------------------//
ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS設定値
ST_SET_MISC				stg_setMisc;					// その他設定値
EM_ACS_SEL				u1g_setAcsSel;					// 設定値選択

//--------------------------------------------------------------------------//
//【関数】初期化処理														//
//【機能】設定値をEEPROMから読み出す										//
//--------------------------------------------------------------------------//
void	SetMgrInit(void){

	memcpy(&stg_setGps[0], &u1_TBL_SETGPS_DEF[0][0], sizeof(ST_SET_GPS)*4);
	memcpy(&stg_setMisc, &u1_TBL_SETMISC_DEF[0], sizeof(ST_SET_MISC));
	u1g_setAcsSel = SET_ACS_ALL;
}

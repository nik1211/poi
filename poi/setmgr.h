//--------------------------------------------------------------------------//
//	設定マネージャヘッダファイル											//
//--------------------------------------------------------------------------//
#ifndef __SETMGR_H__
#define	__SETMGR_H__

#include "comdat.h"

//--------------------------------------------------------------------------//
//	マクロ定義																//
//--------------------------------------------------------------------------//
// 設定値アクセス選択
typedef enum{
	SET_ACS_ALL,							// オールONモード設定値
	SET_ACS_NORMAL,							// ノーマル設定値
	SET_ACS_MINIMUM,						// ミニマム設定値
	SET_ACS_SPECIAL,						// スペシャル設定値
	SET_ACS_MAX,							// ガード値
}EM_ACS_SEL;

// 道路選択
typedef enum{
	SET_ROAD_NORM = 0,						// 一般道のみ
	SET_ROAD_HIGH,							// 高速道のみ
	SET_ROAD_ALL,							// 全て
	SET_ROAD_AUTO,							// オート
}EM_SET_ROAD;

// 取締警報レベル
typedef enum{
	SET_TRAPCHK_WRNLVL_OFF = 0,				// 警報しない
	SET_TRAPCHK_WRNLVL_ALL,					// 全レベル
	SET_TRAPCHK_WRNLVL_2,					// レベル2以上
	SET_TRAPCHK_WRNLVL_3,					// レベル3以上
	SET_TRAPCHK_WRNLVL_4,					// レベル4以上
	SET_TRAPCHK_WRNLVL_5,					// レベル5以上
}EM_SET_TRAPCHK_WRNLVL;

// GPS設定
typedef struct{
	U1							:3;			// Reserved
	U1			b_orbis			:1;			// オービス告知(ON/OFF)
	U1			b_orbisPassSpd	:1;			// オービス通過速度告知(ON/OFF)
	U1			b_orbisPass		:1;			// オービス通過告知(ON/OFF)
	U1			b_hwOrbisLmtSpd	:1;			// 高速道オービス制限速度告知(ON/OFF)
	U1			b_orbisCameraPos:1;			// オービスカメラ位置告知(ON/OFF)

	U1			b_nSys			:1;			// Nシステム告知(ON/OFF)
	U1			b_trafChkSys	:1;			// 交通監視システム告知(ON/OFF)
	U1			b_hwPolice		:1;			// 交通警察隊(ON/OFF)
	U1			b_crossing		:1;			// 交差点監視告知(ON/OFF)
	U1			b_signal		:1;			// 信号無視抑止システム(ON/OFF)
	U1			b_accident		:1;			// 事故多発エリア告知(ON/OFF)
	U1			b_police		:1;			// 警察署告知(ON/OFF)
	U1			b_michinoeki	:1;			// 道の駅告知(ON/OFF)

	U1			b_sa			:1;			// サービスエリア告知(ON/OFF)
	U1			b_pa			:1;			// パーキングエリア告知(ON/OFF)
	U1			b_hwOasys		:1;			// ハイウェイオアシス告知(ON/OFF)
	U1			b_hwRadio		:1;			// ハイウェイラジオ受信エリア告知(ON/OFF)
	U1			b_hwChgLmtSpd	:1;			// 高速道制限速度切替地点告知(ON/OFF)
	U1			b_spdOver		:1;			// 速度超過告知(ON/OFF)
	U1			b_parkChkArea	:1;			// 駐禁監視エリア告知(ON/OFF)
	U1			b_parking		:1;			// 駐車場表示(ON/OFF)

	U1			b_trapWrnLvl	:3;			// 取締警報レベル(EM_SET_TRAPCHK_WRNLVL)
	U1			b_chkWrnLvl		:3;			// 検問警報レベル(EM_SET_TRAPCHK_WRNLVL)
	U1			b_smartIC		:1;			// スマートIC(ON/OFF)
	U1			b_hwGas			:1;			// 高速道ガスステーション(ON/OFF)

	U1			b_curve			:1;			// 急カーブ(ON/OFF)
	U1			b_brajct		:1;			// 分岐・合流(ON/OFF)
	U1			b_tunnel		:1;			// 長い・連続トンネル(ON/OFF)
	U1			b_kenkyo		:1;			// 県境(ON/OFF)
	U1			b_etcLane		:1;			// ETCレーン(ON/OFF)
	U1			b_torupa		:1;			// ビューポイントパーキング(ON/OFF)
	U1			b_shajyoArea	:1;			// 車上狙い多発エリア(ON/OFF)
	U1			b_toilet		:1;			// 公衆トイレ(ON/OFF)

	U1			b_tmpstop		:1;			// 一時停止注意ポイント(ON/OFF)
	U1			b_fumikiri		:1;			// 踏切(ON/OFF)
	U1			b_koban			:1;			// 交番(ON/OFF)
	U1			b_fire			:1;			// 消防署(ON/OFF)
	U1			b_hoiku			:1;			// 幼稚園・保育園(ON/OFF)
	U1							:1;			// 高速バズ(ON/OFF)
	U1			b_zone30		:1;			// ゾーン30(ON/OFF)
	U1							:1;			// reserved

}ST_SET_GPS;

// その他設定
typedef struct{
	U1			b_roadSel		:2;			// GPS道路選択設定(EM_SET_ROAD)
}ST_SET_MISC;

extern ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS設定値
extern ST_SET_MISC				stg_setMisc;					// その他設定値
extern EM_ACS_SEL				u1g_setAcsSel;					// 設定値選択

extern void	SetMgrInit();

#endif


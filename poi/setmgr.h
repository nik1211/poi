
#ifndef __SETMGR_H__
#define __SETMGR_H__

#include "comdat.h"

//--------------------------------------------------------------------------//
//	マクロ定義																//
//--------------------------------------------------------------------------//
// 設定値アクセス選択
typedef enum{
	SET_ACS_LCL = 0,						// ローカルモード設定値
	SET_ACS_DRV,							// ドライブモード設定値
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

	U1			b_areaMode		:2;			// エリアモード(EM_SET_AREA)
	U1			b_voiceMode		:2;			// 音声モード(EM_VOICE_MODE)
	U1			b_panelMode		:2;			// パネルモード(EM_PANEL_MODE)
	U1			b_spdWarn		:2;			// 速度警告(EM_SET_SPDWRN)

	U1			b_idvOrbisVoice	:1;			// 個別オービスボイス(ON/OFF)
	U1			b_scClassic		:1;			// 無線クラシック(ON/OFF)
	U1			b_voiceType		:3;			// 音声タイプ(EM_VOICE_TYPE)
	U1			b_photo_zoom	:2;			// 写真ズーム方式(EM_PHOTO_ZOOM)
	U1			b_in_photo_clock:1;			// 写真内時計(ON/OFF)

	U1			b_photo_effect	:3;			// 写真エフェクト(EM_PHOTO_EFFECT)
	U1			b_photo_chgtim	:3;			// 写真切替時間(EM_PHOTO_CHGTIM)
	U1			b_photo_color	:2;			// 写真色(EM_PHOTO_COLOR)
	
	U1			b_settingMode	:3;			// システム設定記憶(EM_SETTING_MODE)
	U1			b_brightness	:2;			// 明るさ(EM_BRIGHTNESS)
	U1			b_relaxChime	:2;			// リラックスチャイム(EM_RELAX_CHIME)
	U1			b_oprtSnd		:1;			// 操作音(ON/OFF)

	U1			b_dspRev		:1;			// 画面反転(ON/OFF)
	U1			b_vol			:3;			// 音量設定(0～7 大きい程音量大)
	U1			b_manner		:1;			// マナーモード(ON/OFF)
	U1			b_headingUp		:1;			// レーダー画面の表示形式(EM_HEADING_UP)
	U1			b_wrnDspChg		:2;			// 警報表示切り替え(EM_DSP_CHG_PT)

	U1			b_logDisp		:1;			// ログのパーセント表示（ON/OFF）
	U1			b_wrnPhoto		:1;			// 警報写真表示(ON/OFF)
	U1			b_lcdDsp		:1;			// LCD表示(ON/OFF)
	U1			b_logFunc		:1;			// ログ機能(ON/OFF)
	U1			b_obdIllCtrl	:1;			// OBDイルミコントロール(ON/OFF)
	U1							:1;			// 空き
	U1			b_demo			:2;			// 店頭デモ(EM_SET_DEMO)

	U1			b_wtDsp			:5;			// 待受画面(EM_SET_WTDSP)
	U1			b_tideAuto		:1;			// 検潮所自動選択(ON/OFF)
	U1			b_ledmode		:2;			// LED(EM_LED_MODE)

	U1			b_rdSnd			:5;			// レーダー警報音設定(EM_SET_RDSND)
	U1			b_rdSens		:3;			// レーダー受信感度モード設定(EM_SET_SENS)

	U1			b_roadSel		:2;			// GPS道路選択設定(EM_SET_ROAD)
	U1			b_sc			:2;			// 無線警報設定(EM_SET_SC)
	U1			b_sokuiAna		:1;			// 測位アナウンス(ON/OFF)
	U1			b_dispAlways	:1;			// 常時表示(ON/OFF)
	U1			b_jiho			:1;			// 時報(ON/OFF)
	U1			b_dimmer		:1;			// ディマー(EM_SET_FLEX_DIMMER)

	U1			u1_car_maker;				// 車両メーカー

	U2			u2_observer;				// 検潮所番号
	
	U1			b_mapMode		:3;			// マップモード(EM_SET_MAP_MODE)
	U1							:1;			// 空き
	U1			b_movSmoothing	:1;			// 移動スムージング(ON/OFF)
	U1			b_mapScrollFocus:1;			// スクロールフォーカス(ON/OFF)
	U1			b_wideRangeSrch	:1;			// ターゲットワイドレンジサーチ(ON/OFF)
	U1			b_mapZoom		:1;			// ズーム表示

	U1			u1_classicScopeMtr;
	
	union{
		U1		u1_mapIcon;											// 地図アイコン表示
		
		struct{
			U1			etc									:1;		// その他
			U1			cv									:1;		// コンビニ
			U1			ff									:1;		// ファーストフード
			U1			fr									:1;		// ファミレス
			U1			gs									:1;		// ガソリンスタンド
		}b_mapIcon;
	};
	
	U1			u1_turnOnPhoto;				// ターンオン写真
	
	U1			b_roadPres		:1;			// 道路気圧設定(ON/OFF)
	U1			b_WLAN			:1;			// WLAN設定(ON/OFF)
	U1			b_turnOnMovie	:3;			// ターンオンムービー(EM_TURNON_MOVIE)
	U1			b_auto_chgtim	:3;			// 待受AUTO切替時間(EM_AUTO_CHGTIM)
	
	U1			b_scopeType		:2;			// スコープタイプ(EM_SET_SCOPE_TYPE)
	U1			b_toriDisp		:2;			// 公開取締表示(EM_SET_TORI_DISP)
	U1			b_mapColor		:2;			// マップ色(EM_SET_MAP_COLOR)
	U1			b_lei			:1;			// Lei mode
	U1			b_leiCharaSD	:1;			// Lei キャラnormal,SD(ちびレイ)
	U1			rsv00[12];

}ST_SET_MISC;

#endif


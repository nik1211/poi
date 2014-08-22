
#ifndef __GPSCTRL_H__
#define __GPSCTRL_H__

#include "comdat.h"

// GPSボイスタイプ

typedef enum{
	VCGPS_NON_TARGET = 0,							// 非ターゲットタイプ(測位音他)		優先度高
	VCGPS_ORBIS_2KM,								// オービス2km						優先度高
	VCGPS_ORBIS_1KM,								// オービス1km						優先度高
	VCGPS_ORBIS_500M,								// オービス500m						優先度高
	VCGPS_ORBIS_PASSSPD,							// オービス通過速度					優先度高
	VCGPS_ORBIS_COUNTDOWN100,
	VCGPS_ORBIS_COUNTDOWN200,
	VCGPS_ORBIS_COUNTDOWN300,
	VCGPS_ORBIS_COUNTDOWN400,
	VCGPS_ORBIS_COUNTDOWN500,
	VCGPS_ORBIS_COUNTDOWN600,
	VCGPS_ORBIS_COUNTDOWN700,
	VCGPS_ORBIS_COUNTDOWN800,
	VCGPS_ORBIS_COUNTDOWN900,
	VCGPS_MYAREA_1KM,								// マイエリア1km					優先度高
	VCGPS_MYAREA_500M,								// マイエリア500m					優先度高

	VCGPS_ZONE_1KM,									// ゾーン1km						優先度中
	VCGPS_ZONE_500M,								// ゾーン500m						優先度中
	VCGPS_ZONE_100M,								// ゾーン100m						優先度中
	VCGPS_ZONE_OUT,									// ゾーン圏外						優先度中
	VCGPS_1KMCONT_1KM,								// 1km系コンテンツ1km				優先度中
	VCGPS_1KMCONT_500M,								// 1km系コンテンツ500m				優先度中
	VCGPS_500MCONT,									// 500m系コンテンツ					優先度中
	VCGPS_300MCONT,									// 300m系コンテンツ					優先度中
	VCGPS_100MCONT,									// 100m系コンテンツ					優先度中
	VCGPS_ETC,										// ETC								優先度中

	VCGPS_SCPCONT,									// 速度切替コンテンツ				優先度低
	VCGPS_PCHK_SZ_AREA,								// 駐禁最重点エリア					優先度低
	VCGPS_PCHK_Z_AREA,								// 駐禁重点エリア					優先度低
	VCGPS_SHAJYO_AREA,								// 車上狙い多発エリア				優先度低
	VCGPS_ZONE30_AREA,								// ゾーン30エリア					優先度低

	VCGPS_NONE,
}EM_VCGPS_TYPE;

//--------------------------------------------------------------------------
//  外部公開マクロ
//--------------------------------------------------------------------------
#define	SPD_LSB				10							// 速度LSBの逆数
#define	DEG_LSB				10							// 角度LSBの逆数

// ゾーン警報状態定義
enum{
	STS_ZONE_NOALARM = 0,							// 非警報状態
	STS_ZONE_1100M,									// 1100m警報状態
	STS_ZONE_600M,									// 600m警報状態
	STS_ZONE_500M,									// 500m警報状態
	STS_ZONE_100M,									// 100m警報状態
	STS_ZONE_OUTWT_FAR								// 完全離脱待ち状態
};

#define	LATAREA_MAX			((U1)4)						// 緯度エリア最大値

//--------------------------------------------------------------------------
//  型定義																	
//--------------------------------------------------------------------------
// GPS情報型
typedef struct{
	U1 	b_sokui  :1; // 測位
    U1 	b_spdAcc :1; // 速度確からしさ(TRUE/FALSE)
    U1 	b_rx 	 :1; // 受信あり(motionでclear)
    U1 	b_rsv 	 :5;
	U2 	u2_spd; // 速度[単位:0.1km/h]
	U2 	u2_deg; // 方位[単位:0.1deg]
	S2 	s2_height; // 高度[単位:1m]
	U4	lat; //緯度
	U4	lon; // 経度
	U1  u1_vetAccEst; // 高度確からしさ[0.1m] 仮
}stGpsPos;

typedef struct{
	U1 	b_sokui       :1; // 測位
	U1 	b_height_ok   :1; // 高度有効
	U1 	b_spddeg_ok   :1; // 速度方位有効
	U1	b_tim_ok 	  :1; // 時刻OK
	U1 	b_mode		  :1; // GPS動作モード
	U1 	b_rsv         :3;

	stGpsPos st_pos; // uartからposデータを格納

	U4	lat; // 緯度
	U4	lon; // 経度
	U2 	u2_spd; // 速度
	U2	u2_deg; // 方位
	U1	u1_usesatnum; // 測位使用衛星数
	U1	u1_satnum; // 追尾衛星数
	S2 	s2_height; // 高度

	U1 	u1_latarea;	// 緯度エリア
	U2 	u2_lonarea;	// 経度エリア

	U1	u1_year; // 年
	U1	u1_month; // 月
	U1	u1_day;	// 日
	U1	u1_hour; // 時
	U1	u1_min; // 分
	U1	u1_sec; // 秒
}ST_GPSINF;

#define	LATAREA_MAX			((U1)4)						// 緯度エリア最大値

// POI設定セレクト
enum{
	SEL_POI_NORMAL = 0,
	SEL_POI_TUNNEL,
};

void PoiSample(const char* fname, U2 dataSpec);

#endif


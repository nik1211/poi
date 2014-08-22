//--------------------------------------------------------------------------//
//	共有データヘッダファイル												//
//--------------------------------------------------------------------------//

#ifndef __COMDAT_H__
#define __COMDAT_H__

#include <limits.h>

typedef	unsigned char	U1;
typedef	unsigned short	U2;
typedef unsigned long	U4;
typedef signed	char	S1;
typedef	signed	short	S2;
typedef	signed	long	S4;
typedef int Bool;

typedef union{
	U4  dword;
	U4	word;
	struct{
		U2	lo;
		U2	hi;
	}hword;
	U1	byte[4];
	struct{
		U4	bit00:1;
		U4	bit01:1;
		U4	bit02:1;
		U4	bit03:1;
		U4	bit04:1;
		U4	bit05:1;
		U4	bit06:1;
		U4	bit07:1;

		U4	bit08:1;
		U4	bit09:1;
		U4	bit10:1;
		U4	bit11:1;
		U4	bit12:1;
		U4	bit13:1;
		U4	bit14:1;
		U4	bit15:1;

		U4	bit16:1;
		U4	bit17:1;
		U4	bit18:1;
		U4	bit19:1;
		U4	bit20:1;
		U4	bit21:1;
		U4	bit22:1;
		U4	bit23:1;

		U4	bit24:1;
		U4	bit25:1;
		U4	bit26:1;
		U4	bit27:1;
		U4	bit28:1;
		U4	bit29:1;
		U4	bit30:1;
		U4	bit31:1;
	}bit;
}F4;

#define	NG		0
#define	OK		1

#define	FALSE	0
#define	TRUE	1

#define	OFF		0
#define	ON		1

#define	SMALL	0
#define	LARGE	1

#define	LO		0
#define	HI		1

#define	U1_MAX	UCHAR_MAX
#define	U2_MAX	USHRT_MAX
#define	U4_MAX	ULONG_MAX

#define	S1_MAX	SCHAR_MAX
#define	S1_MIN	SCHAR_MIN
#define	S2_MAX	SHRT_MAX
#define	S2_MIN	SHRT_MIN
#define	S4_MAX	LONG_MAX
#define	S4_MIN	LONG_MIN

#ifndef NULL
#define	NULL	0
#endif

// 絶対値差分算出マクロ
#define	ABS_SUB(a,b)		((a >= b) ? (a-b) : (b-a))

// 種番
typedef enum{
	TGT_DUMMY = 0,							// ダミー									0
	TGT_RD_ORBIS,							// RD式オービス								1
	TGT_HSYS_ORBIS,							// Hシステム式オービス						2
	TGT_LHSYS_ORBIS,						// LHシステム式オービス						3
	TGT_LOOP_ORBIS,							// ループコイル式オービス					4
	TGT_KODEN_ORBIS,						// 光電式オービス							5
	
	TGT_TRAP_ZONE,							// トラップゾーン							6
	TGT_CHKPNT_ZONE,						// チェックポイントゾーン					7
	
	TGT_NSYS,								// Nシステム								8
	TGT_TRFCHK,								// 交通監視システム							9

	TGT_MYAREA,								// マイエリア								10
	TGT_MYCANCEL,							// マイキャンセルエリア						11
	TGT_ICANCEL,							// Iキャンセル(自動登録)					12
	
	TGT_POLICE,								// 警察署									13
	TGT_CROSSING,							// 交差点監視システム						14
	TGT_ACCIDENT,							// 事故多発エリア							15
	TGT_SIGNAL,								// 信号無視抑止システム						16
	TGT_MICHINOEKI,							// 道の駅									17
	TGT_HWOASYS,							// ハイウェイオアシス						18
	TGT_SA,									// サービスエリア							19
	TGT_PA,									// パーキングエリア							20
	TGT_HWRADIO,							// ハイウェイラジオ							21
#ifdef	__TGT_HW__
	TGT_HWCHGLMTSPD_40KMH,					// 高速道制限速度切替地点 40km/h
	TGT_HWCHGLMTSPD_50KMH,					// 高速道制限速度切替地点 50km/h
	TGT_HWCHGLMTSPD_60KMH,					// 高速道制限速度切替地点 60km/h
	TGT_HWCHGLMTSPD_70KMH,					// 高速道制限速度切替地点 70km/h
	TGT_HWCHGLMTSPD_80KMH,					// 高速道制限速度切替地点 80km/h
	TGT_HWCHGLMTSPD_90KMH,					// 高速道制限速度切替地点 90km/h
	TGT_HWCHGLMTSPD_100KMH,					// 高速道制限速度切替地点 100km/h

	TGT_HWORBIS_LMTSPD_40KMH,				// 高速道オービス地点制限速度 40km/h
	TGT_HWORBIS_LMTSPD_50KMH,				// 高速道オービス地点制限速度 50km/h
	TGT_HWORBIS_LMTSPD_60KMH,				// 高速道オービス地点制限速度 60km/h
	TGT_HWORBIS_LMTSPD_70KMH,				// 高速道オービス地点制限速度 70km/h
	TGT_HWORBIS_LMTSPD_80KMH,				// 高速道オービス地点制限速度 80km/h
	TGT_HWORBIS_LMTSPD_90KMH,				// 高速道オービス地点制限速度 90km/h
	TGT_HWORBIS_LMTSPD_100KMH,				// 高速道オービス地点制限速度 100km/h
	TGT_PARKCHK_AREA,
#endif
	TGT_SCP_ICI,							// 高速道制限速度切替地点 本線入口			22
	TGT_SCP_ICO,							// 高速道制限速度切替地点 本線出口			23
	TGT_SCP_JC,								// 高速道制限速度切替地点 ジャンクション	24
	TGT_SCP_SPD,							// 高速道制限速度切替地点 本線				25
	TGT_SCP_PO,								// 高速道制限速度切替地点 パーキング出口	26

	TGT_PARKCHK_AREA_SZ,					// 駐車禁止取締り最重点エリア				27
	TGT_PARKCHK_AREA_Z,						// 駐車禁止取締り重点エリア					28
	TGT_PARKING,							// 駐車場									29

	TGT_FCANCEL,							// 固定キャンセルエリア						30
	TGT_CURVE,								// 急カーブ									31
	TGT_BRAJCT,								// 分岐・合流ポイント						32
	TGT_KENKYO,								// 県境										33
	TGT_TUNNEL,								// 長いトンネル								34
	TGT_ETC,								// ETCレーン								35
	TGT_STEEP_SLOPE,						// 急勾配									36
	TGT_RAILROAD_CROSSING,					// 踏切										37

	TGT_PIN,								// ピン										38
	TGT_LNKUNT,								// リンクユニット							39
	TGT_RT_TRAP_ZONE,						// リアルタイム取締エリア					40
	TGT_TORUPA,								// とるぱ									41
	TGT_TMPSTOP,							// 一時停止									42
	TGT_SHAJYOU_AREA,						// 車上狙い多発エリア						43

	TGT_NOFIX_YUDO,							// 非測位誘導データ							44
	TGT_NOFIX_RD_ORBIS,						// 非測位RD式オービス						45
	TGT_NOFIX_HSYS_ORBIS,					// 非測位Hシステム式オービス				46
	TGT_NOFIX_LHSYS_ORBIS,					// 非測位LHシステム式オービス				47
	TGT_NOFIX_LOOP_ORBIS,					// 非測位ループコイル式オービス				48
	TGT_NOFIX_KODEN_ORBIS,					// 非測位光電管式オービス					49

	TGT_NOFIX_TRAP_ZONE,					// 非測位取締エリア							50
	TGT_NOFIX_CHKPNT_ZONE,					// 非測位検問エリア							51

	TGT_TOILET,								// 公衆トイレ								52
	TGT_NOFIX_SENSOR_YUDO,					// 非測位センサ誘導データ					53
	TGT_KOBAN,								// 交番										54

	TGT_FIRE,								// 消防署									55
	TGT_HOIKU,								// 保育園・幼稚園							56

	TGT_HWBUS_STOP,							// 高速バス停								57
	TGT_ZONE30,								// ゾーン30エリア							58

	TGT_GUARD_MAX,							// ガード値

}EM_TGTCODE;

// 方位情報
#define	DEG_INVALID		0xFF				// 方位無効

// 道路判別状態
typedef enum{
	ROADJDG_STS_NORM = 0,					// 一般道走行中
	ROADJDG_STS_CHKHIGH,					// 高速道走行判定中
	ROADJDG_STS_HIGH,						// 高速道走行中
	ROADJDG_STS_CHKNORM,					// 一般道走行判定中
	ROADJDG_STS_NORMHIGH,					// 両道路走行中
}EM_ROADJDG_STS;

// 駐車禁止エリア内状態
typedef enum{
	PCHK_AREA_OUT = 0,						// 駐車禁止エリア外
	PCHK_AREA_SZ,							// 最重点監視エリア内
	PCHK_AREA_Z,							// 重点監視エリア内
}EM_PCHK_AREA_STS;

// 車上狙い多発エリア内状態
typedef enum{
	SHAJYO_AREA_OUT = 0,					// 車上狙い多発エリア外
	SHAJYO_AREA_IN,							// 車上狙い多発エリア内
}EM_SHAJYO_AREA_STS;

// ゾーン30エリア内状態
typedef enum{
	ZONE30_AREA_OUT = 0,					// ゾーン30エリア外
	ZONE30_AREA_IN,							// ゾーン30エリア内
}EM_ZONE30_AREA_STS;

// 種番基本情報
// 道路属性
typedef enum{
	ROAD_NORM = 0,							// 一般道
	ROAD_HIGH,								// 高速道
	ROAD_NORM_TOLL,							// 一般有料道
}EM_ATTRIB_ROAD;

// トンネル属性
typedef enum{
	NOT_TUNNEL = 0,							// 非トンネル
	TUNNEL_OUT,								// トンネル出口
	TUNNEL_IN,								// トンネル内
}EM_ATTRIB_TUNNEL;

// データエリア属性
typedef enum{
	MAKER_DATA_AREA = 0,					// メーカーデータエリア
	CUSTOM_DATA_AREA,						// カスタムデータエリア
}EM_ATTRIB_DATA_AREA;

// 種番拡張情報
// カメラ位置
typedef enum{
	CAMERA_NONE = 0,						// カメラなし
	CAMERA_UP,								// カメラ位置上
	CAMERA_LEFT,							// カメラ位置左
	CAMERA_RIGHT,							// カメラ位置右
}EM_EXTRA_CAMERA;

// 高速・一般道路判別可否
typedef enum{
	ROADJDG_INVALID = 0,					// 高速・一般の区別ができない
	ROADJDG_HIGHWAY,						// 高速道として判別可
	ROADJDG_NORMWAY,						// 一般道として判別可
}EM_EXTRA_ROADJDG;

// 駐禁・車上狙いエリア半径
typedef enum{
	AREA_ROUND_500M = 0,					// 半径500m
	AREA_ROUND_1000M,						// 半径1000m
	AREA_ROUND_1500M,						// 半径1500m
	AREA_ROUND_2000M,						// 半径2000m
	AREA_ROUND_2500M,						// 半径2500m
	AREA_ROUND_50M,							// 半径50m
	AREA_ROUND_100M,						// 半径100m
	AREA_ROUND_300M,						// 半径300m
}EM_EXTRA_AREA_ROUND;

// 制限速度
typedef enum{
	LMTSPD_NONE = 0,						// 制限速度情報なし
	LMTSPD_10KMH,							// 制限速度10km/h
	LMTSPD_20KMH,							// 制限速度20km/h
	LMTSPD_30KMH,							// 制限速度30km/h
	LMTSPD_40KMH,							// 制限速度40km/h
	LMTSPD_50KMH,							// 制限速度50km/h
	LMTSPD_60KMH,							// 制限速度60km/h
	LMTSPD_70KMH,							// 制限速度70km/h
	LMTSPD_80KMH,							// 制限速度80km/h
	LMTSPD_90KMH,							// 制限速度90km/h
	LMTSPD_100KMH,							// 制限速度100km/h
	LMTSPD_110KMH,							// 制限速度110km/h
	LMTSPD_120KMH,							// 制限速度120km/h
}EM_EXTRA_LMTSPD;

typedef enum{
	TRAPCHK_LEVEL_INVALID = 0,				// レベル無効
	TRAPCHK_LEVEL1,							// レベル1
	TRAPCHK_LEVEL2,							// レベル2
	TRAPCHK_LEVEL3,							// レベル3
	TRAPCHK_LEVEL4,							// レベル4
	TRAPCHK_LEVEL5,							// レベル5
}EM_EXTRA_TRAPCHK_LEVEL;

// 取締・検問手法
typedef enum{
	METHOD_UNKNOWN = 0,						// 特定不能
	METHOD_MOVABLE_ORBIS_RADAR,				// 移動オービス(レーダー)
	METHOD_MOVABLE_ORBIS_STEALTH,			// 移動オービス(ステルス)
	METHOD_MOVABLE_ORBIS_KODEN,				// 移動オービス(光電管)
	METHOD_MOUSE_TRAP_RADAR,				// ネズミ捕り(レーダー)
	METHOD_MOUSE_TRAP_STEALTH,				// ネズミ捕り(ステルス)
	METHOD_MOUSE_TRAP_KODEN,				// ネズミ捕り(光電管)
	METHOD_MOTORCYCLE_CHASE,				// 追尾(白バイ)
	METHOD_MASKED_POLICECAR_CHASE,			// 追尾(覆面パトカー)
	METHOD_POLICECAR_CHASE,					// 追尾(パトカー)
	METHOD_TEMPORARY_STOP,					// 一時停止
	METHOD_SIGNAL,							// 信号無視
	METHOD_SEATBELT,						// シートベルト
	METHOD_PHONE_DRIVING,					// 携帯電話
	METHOD_DRUNKEN_DRIVING,					// 飲酒運転
}EM_EXTRA_TRAPCHK_METHOD;

// 駐車・車上狙い禁止エリアの形
typedef enum{
	AREA_SHAPE_CIRCLE = 0,					// 円形
	AREA_SHAPE_OVAL250M,					// 250m長円
	AREA_SHAPE_OVAL500M,					// 500m長円
	AREA_SHAPE_OVAL750M,					// 750m長円
	AREA_SHAPE_OVAL1000M,					// 1000m長円
	AREA_SHAPE_OVAL1500M,					// 1500m長円
	AREA_SHAPE_OVAL2000M,					// 2000m長円
	AREA_SHAPE_OVAL2500M,					// 2500m長円
}EM_EXTRA_AREA_SHAPE;

// 警察種別
typedef enum{
	POLICE_TYPE_STATION = 0,				// 警察署
	POLICE_TYPE_HIGHWAY,					// 高速警察隊
}EM_EXTRA_POLICE_TYPE;

// スマートIC情報
typedef enum{
	SMART_IC_NOINFO = 0,					// スマートIC情報なし
	SMART_IC_NOEXIST,						// スマートICなし
	SMART_IC_EXIST,							// スマートICあり
}EM_EXTRA_SMART_IC;

// ガソリンスタンド情報
typedef enum{
	GS_NONE = 0,							// ガソリンスタンドなし
	GS_JOMO,								// JOMO
	GS_USAMI,								// 宇佐美
	GS_ESSO,								// ESSO
	GS_CARENEX,								// カーエネクス
	GS_KYGNUS,								// キグナス
	GS_COSMO,								// コスモ
	GS_KYUSHU,								// 九州石油
	GS_IDEMITSU,							// 出光
	GS_SHELL,								// 昭和シェル
	GS_ENEOS,								// ENEOS
	GS_TAIYO,								// 太陽石油
	GS_GENERAL,								// ゼネラル
	GS_MOBIL,								// モービル
	GS_KOUNAN,								// コーナン
	GS_TAKARA,								// タカラフリート
	GS_SORATO,								// ソラト
	GS_MAX,
}EM_EXTRA_GAS_STATION;

// カーブ種別
typedef enum{
	LEFT_CURVE = 0,							// 左カーブ
	RIGHT_CURVE,							// 右カーブ
	LEFT_TO_RIGHT_CURVE,					// 左→右カーブ
	RIGHT_TO_LEFT_CURVE,					// 右→左カーブ
	BOTH_LEFT_AND_RIGHT_CURVE,				// 左右分岐カーブ
}EM_EXTRA_CURVE_TYPE;

// 高速道路種別
typedef enum{
	NORMAL_HIGHWAY = 0,						// 通常高速
	URBAN_HIGHWAY,							// 都市高速
}EM_EXTRA_HIGHWAY_TYPE;

// 分岐・合流種別
typedef enum{
	BRAJCT_BRA_LEFT = 0,					// 左分岐
	BRAJCT_BRA_RIGHT,						// 右分岐
	BRAJCT_BRA_LEFT_RIGHT,					// 左右分岐
	BRAJCT_JCT_LEFT,						// 左合流
	BRAJCT_JCT_RIGHT,						// 右合流
	BRAJCT_JCT_LEFT_RIGHT,					// 左右合流
}EM_EXTRA_BRAJCT_TYPE;

// トンネル種別
typedef enum{
	TUNNEL_TYPE_LONG = 0,					// 長いトンネル
	TUNNEL_TYPE_CONT,						// 連続トンネル
}EM_TUNNEL_TYPE;

// ETCガイド
typedef enum{
	ETC_GUIDE_LANE_LEFT = 0,				// ETCレーン左側
	ETC_GUIDE_LANE_CENTER,					// ETCレーン中央
	ETC_GUIDE_LANE_RIGHT,					// ETCレーン右側
	ETC_GUIDE_LANE_BOTH_SIDE,				// ETCレーン両端
	ETC_GUIDE_LANE_NONE,					// ETCレーンなし
}EM_ETC_GUIDE;

// ETCポイント種別
typedef enum{
	ETC_POINT_IN_START = 0,					// 高速入口案内開始ポイント
	ETC_POINT_OUT_START,					// 高速出口案内開始ポイント
	ETC_POINT_STOP,							// 案内終了ポイント
}EM_ETC_POINT_TYPE;

// 最優先ターゲットフォーカス定義
typedef enum{
	PRI_FOCUS_TGT_NONE = 0,							// 最優先ターゲットフォーカスなし
	PRI_FOCUS_TGT_ICON,								// 最優先ターゲットアイコンフォーカス
	PRI_FOCUS_TGT_POLIGON,							// 最優先ターゲットポリゴンフォーカス
	PRI_FOCUS_TGT_POLIGON_SOUNDING,					// 最優先ターゲットポリゴンフォーカス(音声出力中)
}EM_PRI_FOCUS_TGT;

// 誘導タイプ
typedef enum{
	YUDO_TYPE_COMMON = 0,					// 共通誘導
	YUDO_TYPE_VPS_ONLY,						// VPS専用誘導
}EM_EXTRA_YUDO_TYPE;

// 誘導番号無効
#define	u1_YUDO_NUM_INVALID	0

typedef enum{
	DIMMER_PHASE_NIGHT_OR_DAYLIGHT,
	DIMMER_PHASE_NIGHT_TO_MORNING,
	DIMMER_PHASE_MORNING_TO_DAYLIGHT,
	DIMMER_PHASE_DAYLIGHT_TO_EVENING,
	DIMMER_PHASE_EVENING_TO_NIGHT,
}EM_DIMMER_PHASE;

//--------------------------------------------------------------------------//
//	型定義																	//
//--------------------------------------------------------------------------//

#define	GPSMAP_MAX			500				// 最大ターゲットデータ数
#define GPS_VISIBLE_TGT_MAX	150				// 可視ターゲット最大数
#define	EXTRA_DATA_SIZE		8				// 拡張情報データ長

// 種番共用体
typedef union{
	struct{
		U2						:3;			// 予約
		U2		b_road			:2;			// 道路属性(EM_ATTRIB_ROAD)
		U2		b_tunnel		:2;			// トンネル属性(EM_ATTRIB_TUNNEL)
		U2		b_dataArea		:1;			// データエリア属性(EM_ATTRIB_DATA_AREA)
		U2		b_code			:8;			// ターゲット(EM_TGTCODE)
	}bit;
	U2	word;
}UN_TYPE;

#pragma pack(1)
// 種番別拡張型
// オービス
typedef struct{
	// extra1
	U1	b_camera		:3;					// カメラ(EM_EXTRA_CAMERA)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_photoNum;						// 写真番号
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_voiceNum;						// ボイス番号
}ST_EXTRA_ORBIS;
// 取締・検問エリア
typedef union{
	U2	hword;
	struct{
		U2		day		:5;
		U2		month	:4;
		U2		year	:7;
	}mbr;
}UN_REGDAY;

typedef struct{
	// extra1
	U1	b_level			:3;					// レベル(EM_EXTRA_TRAPCHK_LEVEL)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_method;							// 方法(EM_EXTRA_TRAPCHK_METHOD)
	// extra4-5
	UN_REGDAY	un_regday;					// 登録日
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_TRAPCHK;

// 警察署関連
typedef struct{
	// extra1
	U1	b_policeType	:3;					// 警察区分(EM_EXTRA_POLICE_TYPE)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_POLICE;

// SA・PA・OA関連
typedef struct{
	// extra1
	U1	b_smartIC		:3;					// スマートIC情報(EM_EXTRA_SMART_IC)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_gasStation;						// ガソリンスタンド情報(EM_EXTRA_GAS_STATION)
	// extra4-5
	U2	u2_idNumber;						// SA・PA・OA番号
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_SAPAOA;

// 制限速度切替
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_SCP;

// 駐禁エリア
typedef struct{
	// extra1
	U1	b_areaRound		:3;					// エリア半径(EM_EXTRA_AREA_ROUND)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_shape			:3;					// エリアの形(EM_EXTRA_AREA_SHAPE)
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_NOPARKING;

// 駐車場
typedef struct{
	// extra1
	U1	b_charge		:3;					// 料金情報
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_openTime;						// 営業開始時間(単位:0.25h)
	// extra4
	U1	u1_closeTime;						// 営業終了時間(単位:0.25h)
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_PARKING;

// 急カーブ
typedef struct{
	// extra1
	U1	b_curveType		:3;					// カーブ種別(EM_EXTRA_CURVE_TYPE)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_highwayType	:3;					// 高速種別(EM_EXTRA_HIGHWAY_TYPE)
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_CURVE;

// 分岐・合流ポイント
typedef struct{
	// extra1
	U1	b_braJctType	:3;					// 分岐・合流種別(EM_EXTRA_BRAJCT_TYPE)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_BRAJCT;

// 県境
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_kenkyoNum;						// 都道府県番号(0-46)
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_KENKYO;

// トンネル
typedef struct{
	// extra1
	U1	b_tunnelType	:3;					// トンネル種別(EM_TUNNEL_TYPE)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_length;							// トンネル全長(単位：m)
	// extra5-6
	U2	u2_nameNumber;						// トンネル名称番号
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_TUNNEL;

// ETCレーン
typedef struct{
	// extra1
	U1	b_ETCGuide		:3;					// ETCガイド(EM_ETC_GUIDE)
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1	b_pointType		:3;					// ETCポイント種別(EM_ETC_POINT_TYPE)
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;

	// extra3-6
	U4	u4_laneDetail;						// ETCレーン詳細
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_ETC;

// 道の駅・ビューポイントパーキング
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4-5
	U2	u2_idNumber;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_MICHI_TORUPA;

// 車上狙い多発エリア
typedef struct{
	// extra1
	U1	b_areaRound		:3;					// エリア半径
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4-5
	U2	u2_idNumber;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_SHAJYOU;

// 動的
typedef struct{
	// extra1
	U1	u1_mark;							// 有効・無効マーク
	// extra2-3
	UN_REGDAY	un_regday;					// 初回登録年月日
	// extra4
	U1	u1_sum;								// チェックサム
	// extra5-6
	U1	u1_oldFlags[2];						// 旧フラグ
	// extra7-8
	U1	u1_newFlags[2];						// 新フラグ
}ST_EXTRA_DYNAMIC;

// その他特殊項目なしの汎用
typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1					:8;
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1	u1_tunnelRem;						// トンネル残距離
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号

}ST_EXTRA_COMMON;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_yudo_type;						// 誘導タイプ(EM_EXTRA_YUDO_TYPE)
	// extra4
	U1					:8;
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7
	U1					:8;
	// extra8
	U1					:8;
}ST_EXTRA_YUDO;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3
	U1	u1_syudo_number;					// 誘導番号
	// extra4
	U1	u1_syudo_rad;						// 誘導半径(LSB 100m)
	// extra5
	U1	u1_syudo_deg;						// 誘導角度(LSB 1°)
	// extra6
	U1					:8;
	// extra7
	U1					:8;
	// extra8
	U1					:8;
}ST_EXTRA_SYUDO;

typedef struct{
	// extra1
	U1					:3;
	U1	b_roadJdg		:3;					// 道路判別(EM_EXTRA_ROADJDG)
	U1					:2;
	// extra2
	U1					:3;
	U1	b_lmtSpd		:4;					// 制限速度(EM_EXTRA_LMTSPD)
	U1					:1;
	// extra3-4
	U2	u2_radius;							// 誘導タイプ(EM_EXTRA_YUDO_TYPE)
	// extra5
	U1					:8;
	// extra6
	U1					:8;
	// extra7-8
	U2	u2_mapPctNum;						// 地図・画像番号
}ST_EXTRA_ZONE30;


// EXTRAアクセス共用体
typedef union{
	ST_EXTRA_ORBIS			orbis;			// オービス系種番用
	ST_EXTRA_TRAPCHK		trapchk;		// 取締・検問種番用
	ST_EXTRA_POLICE			police;			// 警察署種番用
	ST_EXTRA_MICHI_TORUPA	michiTorupa;	// 道の駅・とるぱ種番用
	ST_EXTRA_SAPAOA			sapaoa;			// サービスエリア・パーキングエリア・ハイウェイオアシス種番用
	ST_EXTRA_SCP			scp;			// 制限速度切替種番用
	ST_EXTRA_NOPARKING		no_parking;		// 駐禁エリア種番用
	ST_EXTRA_PARKING		parking;		// 駐車場種番用
	ST_EXTRA_CURVE			curve;			// 急カーブ種番用
	ST_EXTRA_BRAJCT			brajct;			// 分岐・合流種番用
	ST_EXTRA_KENKYO			kenkyo;			// 県境種番用
	ST_EXTRA_TUNNEL			tunnel;			// トンネル種番用
	ST_EXTRA_ETC			etc;			// ETC種番用
	ST_EXTRA_SHAJYOU		shajyou;		// 車上狙いエリア種番用
	ST_EXTRA_DYNAMIC		dynamic;		// 動的用
	ST_EXTRA_COMMON			common;			// その他共通
	ST_EXTRA_YUDO			yudo;			// 誘導
	ST_EXTRA_SYUDO			syudo;			// センサ誘導
	ST_EXTRA_ZONE30			zone30;			// ゾーン30用
	U1						byte[EXTRA_DATA_SIZE];
											// byteアクセス
}UN_EXTRA;

// ターゲットマップ型
typedef struct{
	U4			u4_tgtLat;					// ターゲット緯度(単位 10^-3分)
	U4			u4_tgtLon;					// ターゲット経度(単位 10^-3分)
	U4			u4_dataAddr;				// データ元アドレス
	U2			u2_dst;						// 自車-ターゲット間距離(単位 m)
	U2			u2_tgtDeg;					// ターゲット方位(単位 0.1°)
	U2			u2_degA;					// ターゲットから見た自車方位(単位 0.1°)
	S2			s2_degB;					// 自車のターゲット対向角(単位 0.1° -180°< deg ≦ +180°：右方向を正)
	U2			u2_countdown_dist;
	U2			u2_dst_old;					// 前回の距離
	U1			hys_ahead;					// 前方状態
	U1			ahead_hys_count;			// 前方ヒスカウント
	U1			u1_areaStsRd;				// RD用エリア状態
	U1			u1_areaStsSc;				// 無線用エリア状態
	U1			u1_areaScSnd;				// エリア無線音記憶
	UN_TYPE		un_type;					// ターゲット種番
	UN_EXTRA	un_extra;					// 拡張情報
	U1			u1_wrnSts;					// 警報状態(＝0:非警報 ≠0:警報中もしくは離反待ち)
}ST_GPSMAP;
#pragma pack()

// GPSターゲット情報型
typedef struct{
	ST_GPSMAP	*pst_gpsMap;				// ターゲットマップポインタ(ST_GPSMAP型 配列の先頭ポインタ)
	U2			u2_tgtNum;					// ターゲット総数
}ST_GPSMAP_INF;

// ヘッダ構造
typedef struct{
	U4	u4_headCode;								// ヘッダコード
	U4	u4_staAdr;									// 開始アドレス
	U4	u4_endAdr;									// 終了アドレス
	U4	u4_allChksum;								// 全データチェックサム

	union{
		struct{
			U2	b_orbisDataArea		:1;				// オービスデータエリア
			U2	u2_orbisDataSpec	:15;			// オービスデータスペック
		}orbis;
		U2	u2_mapMainVer;							// マップメインバージョン
	}mainVer;
	U2	u2_subVer;									// サブバージョン

	U1	u1_hour;									// 時
	U1	u1_day;										// 日
	U1	u1_month;									// 月
	U1	u1_year;									// 年

	U1	u1_min;										// 分
	U1	u1_sec;										// 秒
	U1	u1_reserved[2];								// 予約
	U4	u4_headerChksum;							// ヘッダチェックサム
}ST_HEADER;

// INDEX1情報構造
#pragma pack(2)
typedef struct{
	U2	u2_elenum;									// ブロック内要素数
	U4	u4_adr;										// アドレス
}ST_INDEX1;
#pragma pack()

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
}ST_ROMIMAGE;
#pragma pack()

// 衛星情報型
#pragma pack(1)
typedef struct{
	U1		id;										// 0の時以下の情報は無効、GPS: 1(G1)-32(G32), GLONASS 65(R1)-96(R32), SBAS 120(S120)-151(S151), QZSS 193(Q1)-197(Q5)
	S1		ev;										// [deg]
	S2		az;										// [deg]
	U1		cn;										// [dbHz]
	U1		flg;									// bSvFlg
#define	bSvFlgUsed				bit0					// SV is used for navigation
#define	bSvFlgCorrct			bit1					// Differential correction data is available for this SV
#define	bSvFlgEpAl				bit2					// Orbit information is available for this SV (Ephemeris or Almanach)
#define	bSvFlgEp				bit3					// Orbit information is Ephemeris
#define	bSvFlgUnhelth			bit4					// SV is unhealthy / shall not be used
//								bit5
//								bit6
//								bit7
#define	kSvFlgInvalid			(0xFF)					// flgを無効にする、bit0とbit4が同時に有効になることはないだろう
}ST_SAT_INF;

// GPS状態型
typedef	struct{
	U4			u4_carLat;					// 自車緯度 (単位：10^-3分)
	U4			u4_carLon;					// 自車経度 (単位：10^-3分)
	U2			u2_carDeg;					// 自車方位 (単位：0.1°)	
	S2			s2_carHight;				// 自車高度 (単位：1m)
	U2			u2_carSpd;					// 自車車速 (単位：0.1km/h)

	U1			b_curSokui		:1;			// 現在の測位状態(モジュールデータそのまま)
	U1			b_sysSokui		:1;			// システム測位状態(表示や音に使用する状態)
	U1			b_degValid		:1;			// 方位有効フラグ
	U1			b_hightValid	:1;			// 高度有効フラグ
	U1			b_timValid		:1;			// 時刻有効フラグ
	U1			b_timDim		:2;			// タイムディマー状態(EM_DIMMER)
	U1			b_spdValid		:1;			// 速度確定フラグ

	U1			b_gpsModSts		:3;			// GPSモジュール制御状態(EM_GPSMOD_STS)
	U1			b_rsv1			:4;			// 旧：測位使用衛星数(0～12)
	U1			b_sys1stFix		:1;			// システム1stFixフラグ

	U1			u1_latarea;					// 緯度エリア
	U2			u2_lonarea;					// 経度エリア

	U1			u1_year;					// 年(2000年からのオフセット)
	U1			u1_month;					// 月(1～12)
	U1			u1_day;						// 日(1～31)
	U1			u1_hour;					// 時(0～23)
	U1			u1_min;						// 分(0～59)
	U1			u1_sec;						// 秒(0～59)

	U1 			b_carSpdValid   :1;     	// 自車速度有効(表示は"u2_carSpd" & "b_carSpdValid"で判断する)
	U1			b_CarAveMaxSpdValid	:1;
	U1			b_forceGyroDegUse	:1;
	U1 			b_rsv           :5;     	// 

	EM_DIMMER_PHASE	dim_phase;
	U2			dim_sec_offs;
	
	U2			u2_carAveSpd;				// 平均速度
	U2			u2_carMaxSpd;				// 最高速度

#define	SAT_INF_NUM				40			// 衛星情報数
	U1			u1_viewSatNum;				// 視野内衛星数
	ST_SAT_INF	st_satInf[SAT_INF_NUM];		// 衛星情報

}ST_GPS_STS;
#pragma pack()
#define	GPSMAP_MAX			500				// 最大ターゲットデータ数
#define GPS_VISIBLE_TGT_MAX	150				// 可視ターゲット最大数
#define	EXTRA_DATA_SIZE		8				// 拡張情報データ長

#endif


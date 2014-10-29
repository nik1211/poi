//--------------------------------------------------------------------------//
//	MUPASSヘッダファイル													//
//--------------------------------------------------------------------------//
// 二重インクルード防止
#ifndef __MUPASS_H__
#define	__MUPASS_H__

#include "common.h"

//--------------------------------------------------------------------------//
//  型定義																	//
//--------------------------------------------------------------------------//
// オービス系告知
typedef struct{
// 拡張句の要否
	U1	b_lmtSpd	:1;							// 制限速度句
	U1	b_spdOver	:1;							// 速度超過句
	U1	b_camera	:1;							// カメラ告知
	U1	b_runSpd	:1;							// 走行速度句
	U1	idvVoiceHi	:3;							// 個別ボイスHi
	U1				:1;							// 予備
// 拡張情報
	U1	lmtSpdVal	:4;							// 制限速度値
	U1	cameraPos	:3;							// カメラ位置
	U1				:1;

	U1	idvVoiceLo;								// 個別ボイスLo
}ST_ORBIS_VCGPS;

// 取締・検問エリア告知
typedef struct{
// 拡張句の要否
	U1	b_lmtSpd	:1;							// 制限速度句
	U1	b_spdOver	:1;							// 速度超過句
	U1	b_level		:1;							// レベル句
	U1	b_near		:1;							// 近接
	U1	b_rngOut	:1;							// 圏外句
	U1				:3;							// 予備
// 拡張情報
	U1	lmtSpdVal	:4;							// 制限速度値
	U1	levelVal	:3;							// レベル値
	U1				:1;							// 予備
	
	U1	method;									// 手法
}ST_TRAPCHK_VCGPS;

// SA・PA・OA告知
typedef struct{
// 拡張句の要否
	U1	b_smartIC	:1;							// スマートIC句
	U1	b_gas		:1;							// ガソリンスタンド句
	U1				:6;							// 予備
// 拡張情報
	U1	smartIC		:3;							// スマートIC情報
	U1				:5;							// 予備

	U1	gasBrand;								// ガソリン情報
}ST_SAPAOA_VCGPS;

typedef struct{
// 拡張句の要否
	U1	b_lmtSpd	:1;							// 制限速度句
	U1	b_spdOver	:1;							// 速度超過句
	U1				:6;							// 予備
// 拡張情報
	U1	lmtSpdVal	:4;							// 制限速度値
	U1				:4;							// 予備

	U1	rsv;									// 予備
}ST_SCP_VCGPS;

// 急カーブ
typedef struct{
// 拡張句の要否
	U1	b_curve		:1;							// カーブ種別句
	U1				:7;
// 拡張情報
	U1	curveType	:3;							// カーブ情報
	U1				:5;							// 予備

	U1	rsv;									// 予備
}ST_CURVE_VCGPS;

// 分岐・合流ポイント
typedef struct{
// 拡張句の要否
	U1	b_brajct	:1;							// 分岐・合流種別句
	U1				:7;
// 拡張情報
	U1	brajctType	:3;							// 分岐合流情報
	U1				:5;							// 予備

	U1	rsv;									// 予備
}ST_BRAJCT_VCGPS;

// 県境
typedef struct{
// 拡張句の要否
	U1	b_kenkyo	:1;							// 都道府県句
	U1				:7;							// 予備
// 拡張情報
	U1	kenkyoNum;								// 都道府県番号
	U1	rsv;									// 予備
}ST_KENKYO_VCGPS;

// トンネル
typedef struct{
// 拡張句の要否
	U1	b_tunnel	:1;							// 方向句
	U1				:7;							// 予備
// 拡張情報
	U1	tunnelType	:3;							// トンネル種別
	U1				:5;							// 予備

	U1	rsv;									// 予備
}ST_TUNNEL_VCGPS;

// ETCレーンガイド
typedef struct{
// 拡張句の要否
	U1	b_etc		:1;							// ETC句
	U1				:7;							// 予備
// 付加情報
	U1	etcGuide	:3;							// ETCガイド情報
	U1				:5;							// 予備

	U1	rsv;									// 予備
}ST_ETC_VCGPS;

// ターゲット告知型
typedef struct{
	// 共通部分(3byte)
	struct{
		U1	u1_trgt;								// ターゲット

		U1	b_tunnel	:2;							// トンネル句
		U1	b_dir		:2;							// 方向句
		U1	b_dist		:4;							// 距離句

		U1	b_highway	:1;							// 道路属性句
		U1	b_tn_ext	:1;							// トンネル拡張(出口直後)
		U1	b_idvVc		:1;							// 個別ボイス指定
		U1	b_countdown	:1;							// カウントダウン
		U1				:4;							// 予備
	}common;
	// 拡張部分(3byte)
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
//  マクロ定義																//
//--------------------------------------------------------------------------//

enum{
	VC_CH_LEIPRI_HI = 0,
	VC_CH_LEIPRI_LOW,
};

// 音声合成チャンネル定義
typedef enum{
	VC_CH0 = 0,					// CH0. 最高優先(システム・一部のRD)
	VC_CH1,						// CH1. RD関連
	VC_CH2,						// CH2. GPS関連
	VC_CH3,						// CH3. スキャナ関連
	VC_CH4,						// CH4. Lei音声
	VC_CH_MAX = VC_CH4,
	VC_CH_ALL,					// 全CH
	VC_CH_NONE,
}EM_VC_CH;

// ターゲット告知サブ情報
// 方向
enum{
	VCDIR_NONE = 0,				// 言わない
	VCDIR_FWRD,					// 前
	VCDIR_LEFT,					// 左
	VCDIR_RIGHT,				// 右
	VCDIR_CANT_REJUDGE,			// 再判定不可
};
// 距離
enum{
	VCDIST_NONE = 0,			// 言わない
	VCDIST_50M,					// すぐ先
	VCDIST_100M,				// 100m先
	VCDIST_200M,				// 200m先
	VCDIST_300M,				// 300m先
	VCDIST_500M,				// 500m先
	VCDIST_900M,				// この先
	VCDIST_1KM,					// 1Km先
	VCDIST_1900M,				// この先
	VCDIST_2KM,					// 2km先
	VCDIST_SOON,				// まもなく
};
// ターゲット名
enum{
	VCTGT_RD_ORBIS = 0,			// レーダー式オービス
	VCTGT_LP_ORBIS,				// ループコイル式オービス
	VCTGT_HSYS_ORBIS,			// Hシステム式オービス
	VCTGT_LHSYS_ORBIS,			// LHシステム式オービス

	VCTGT_TRAP_ZONE,			// トラップゾーン
	VCTGT_CHKPNT_ZONE,			// チェックポイントゾーン
	VCTGT_MYAREA,				// マイエリア
	VCTGT_CROSSING,				// 交差点監視
	VCTGT_SIGNAL,				// 信号無視抑止システム
	VCTGT_SZ_AREA,				// 駐禁最重点エリア
	VCTGT_Z_AREA,				// 駐禁重点エリア
	VCTGT_HIGHWAY_POLICE,		// 高速警察隊
	VCTGT_SCP,					// 速度切替ポイント
	VCTGT_SHAJYO_AREA,			// 車上狙いエリア
	VCTGT_ZONE30_AREA,			// ゾーン30

	VCTGT_NSYS,					// Nシステム
	VCTGT_TRFCHK,				// 交通監視システム
	VCTGT_POLICE_STATION,		// 警察署
	VCTGT_ACCIDENT,				// 事故多発エリア
	VCTGT_CURVE,				// 急カーブ
	VCTGT_BRAJCT,				// 分岐・合流

	VCTGT_MICHINOEKI,			// 道の駅
	VCTGT_SA,					// サービスエリア
	VCTGT_PA,					// パーキングエリア
	VCTGT_HWOASYS,				// ハイウェイオアシス
	VCTGT_HWRADIO,				// ハイウェイラジオ
	VCTGT_KENKYO,				// 県境
	VCTGT_TUNNEL,				// トンネル
	VCTGT_TORUPA,				// とるぱ
	VCTGT_ETC,					// ETCゲート

	VCTGT_KOBAN,				// 交番
};

// 音声合成種別定義
typedef enum{
	VC_NONE = 0,
// CH0
	VC_STOP0,
	VC_DOWNSTA,
	VC_DOWNEND,
	VC_DATTXEND,
	VC_PWRON1,								// 電源ON1
	VC_PWRON2,								// 電源ON2
	VC_OPRT1,								// 操作音1
	VC_OPRT2,								// 操作音2
	VC_OPRT3,								// 操作音3
	VC_OPRT4,								// 操作音4							10
	VC_OPRT5,								// 操作音5
	VC_OPRT6,								// 操作音6
	VC_OPRT7,								// 操作音7
	VC_OPRT8,								// 操作音8
	VC_OPRT9,								// 操作音9
#if 0
// ブザー代用テスト
	VC_BZ_OPRT,								// 電子操作音
	VC_BZ_OPRT2,							// 電子操作音2
	VC_BZ_OPRT3,							// 電子操作音3
	VC_BZ_OPRTNACK,							// 操作拒否音
	VC_BZ_REGACK,							// 登録OK
	VC_BZ_REGNACK,							// 登録NG
	VC_BZ_FUNCON,							// 機能ON
	VC_BZ_FUNCOFF,							// 機能OFF
	VC_BZ_REJECT,							// 拒否音
	VC_BZ_MANRST,							// リセット音
	VC_BZ_ADJMDRDY,							// 調整モード準備音
	VC_BZ_ADJMD,							// 調整モード音
	VC_BZ_FORMAT,
	VC_BZ_DEFRAG,
#endif
	// GPS操作関連
	VC_MYAREA_REG_ACK,						// マイエリア登録音
	VC_MYAREA_DELAYED_REG_ACK,				// マイエリア登録音
	VC_MYAREA_REG_NACK,						// マイエリア登録拒否音
	VC_MYAREA_DELAYED_REG_NACK,				// マイエリア登録拒否音
	VC_MYAREA_DEL,							// マイエリア削除音
	VC_MYAREA_DELAYED_DEL,					// マイエリア削除音
	VC_MYCANCEL_REG_ACK,					// マイキャンセル登録音
	VC_MYCANCEL_DELAYED_REG_ACK,			// マイキャンセル登録音
	VC_MYCANCEL_REG_NACK,					// マイキャンセル登録拒否音
	VC_MYCANCEL_DELAYED_REG_NACK,			// マイキャンセル登録拒否音
	VC_MYCANCEL_DEL,						// マイキャンセル削除音
	VC_MYCANCEL_DELAYED_DEL,				// マイキャンセル削除音
	VC_MYCANCEL_SET_OFF,					// マイキャンセル設定OFF音
	VC_MYCANCEL_DELAYED_SET_OFF,			// マイキャンセル削除音
	VC_GPS_OPRT_WAIT,						// GPS操作待ち
	VC_GPS_DELAYED_OPRT_WAIT,				// GPS操作待ち
	VC_GPS_OPRT_FAIL,				//32	// GPS操作失敗

	VC_TST0M,								// テスト音0メロディ
	VC_TST0M2,								// テスト音0メロディ2
	VC_TST0M3,								// テスト音0メロディ3
	VC_TST0M_ROT,							// テスト音0メロディローテーション
	VC_TST0V,								// テスト音0ボイス		30
	VC_TST0QV,								// テスト音0ボイス
	VC_TST1,								// テスト音1
	VC_TST2,								// テスト音2
	VC_TST3,						//41	// テスト音3
	VC_TSTALL,

	VC_TST_OPENING,							// カスタムテスト オープニング
	VC_TST_ORBIS_JINGLE,					// カスタムテスト オービスジングル
	VC_TST_GPSWRN_JINGLE,					// カスタムテスト GPS警報ジングル
	VC_TST_GPSINF_JINGLE,					// カスタムテスト GPS告知ジングル
	VC_TST_SC_JINGLE,						// カスタムテスト 無線ジングル
	VC_TST_GPS_1STFIX,						// カスタムテスト GPS初期測位
	VC_TST_RD_MELODY,						// カスタムテスト レーダーメロディ

	VC_TST_GOOD_MORNING,					// テスト 朝
	VC_TST_GOOD_AFTERNOON,					// テスト 昼
	VC_TST_GOOD_EVENING,					// テスト 夜

	VC_RD_STEALTH,							// RD ステルス
	VC_RD_STEALTH2,							// RD ステルス
	VC_RD_HSYS,								// RD Hシステム
	VC_ORBIS_PASS,							// オービス通過
	VC_MYAREA_PASS,							// マイエリア通過
	VC_TSTMODE,								// テストモード音
	VC_LOWBATT,						//48	// ローバッテリー音
	VC_MOUSE_TRAP_RD,						// ネズミ捕りRD警報音
	VC_MOVE_ORBIS_RD,						// 移動オービスRD警報音

// CH1
	VC_STOP,								// 停止
	VC_FORCE_STOP,							// 強制停止
	VC_RD_MELODY,							// RD メロディ
	VC_RD_MELODY2,							// RD メロディ2
	VC_RD_MELODY3,							// RD メロディ3
	VC_RD_MELODY4,							// RD メロディ4
	VC_RD_VOICE,							// RD ボイス
	VC_RD_VOICE2,							// RD ボイス2

	VC_ICANCEL_ACT,							// Iキャンセル作動音
	VC_SELFTST_MELODY,
	VC_SELFTST_VOICE,

// CH3
	VC_SC_TRAP,								// 無線 トラップ		10
	VC_SC_CARLOC_FAR,						// 無線 カーロケ遠方
	VC_SC_CARLOC_NEAR,						// 無線 カーロケ近接
	VC_SC_CARLOC_OUT,						// 無線 カーロケ圏外
	VC_SC_DIGITAL,							// 無線 デジタル
	VC_SC_HELITELE,							// 無線 ヘリテレ
	VC_SC_TOKUSHOU,							// 無線 特小
	VC_SC_KEIDEN,							// 無線 警察電話
	VC_SC_KATSUDOU,							// 無線 警察活動
	VC_SC_SHOKATSU,							// 無線 署活系
	VC_SC_FIRE,								// 無線 消防			20
	VC_SC_FIRE_HELITELE,					// 無線 消防ヘリテレ
	VC_SC_WRECKER,							// 無線 レッカー
	VC_SC_EMERGENCY,						// 無線 救急
	VC_SC_JH,								// 無線 JH
	VC_SC_KEIBI,							// 無線 警備
	VC_SC_HEISO_TSUIBI,						// 無線 並走追尾
	VC_SC_SURECHIGAI,						// 無線 すれちがい
	VC_SC_TORISHIMARI,						// 無線 取締
	VC_SC_KENMON,							// 無線 検問

	VC_STOP3,
// CH2
	VC_GPS_1STFIX,							// GPS初期測位
	VC_GPS_1STFIX_GODD_MORNING,				// GPS初期測位 おはようございます
	VC_GPS_1STFIX_GODD_AFTERNOON,			// GPS初期測位 こんにちは
	VC_GPS_1STFIX_GODD_EVENING,				// GPS初期測位 こんばんは
	VC_GPS_1STFIX_HAPPY_NEW_YEAR,			// GPS初期測位 あけましておめでとうございます
	VC_GPS_1STFIX_MERRY_XMAS,				// GPS初期測位 メリークリスマス
	VC_GPS_SOKUI,							// GPS測位
	VC_GPS_NOSOKUI,							// GPS非測位
	VC_GPS_SEARCH,							// GPSサーチ中
	VC_RELAX_CHIME,							// リラックスチャイム
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

	VC_GPSVAR,								// GPS可変音
	
	VC_LEI_DUMMY,							// LEI用dummy

	VC_ANYON,								// いずれかのCH ON
	VC_MAX,									// ガード値				58
}EM_VC;

#endif

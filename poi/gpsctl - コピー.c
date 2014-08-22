//--------------------------------------------------------------------------
//【モジュール】GPSコントローラ												
//【機能】      GPS位置情報を入力として各種GPS警報を出力する				
//【備考】																	
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//  インクルードファイル                                                    
//--------------------------------------------------------------------------

#include "stdafx.h"
#include "gpsctrl.h"
#include "gpsctrl.prm"
#include "crypt.h"
#include "math.h"
#include "setmgr.h"

//--------------------------------------------------------------------------
//  マクロ定義                                                              
//--------------------------------------------------------------------------

#define ZMATH_PI			3.14159265358979323846f
#define ZMATH_PI_2			1.57079632679489661923f // pi / 2

// オービス警報状態定義
enum{
	STS_ORBIS_NOALARM = 0,							// 非警報状態
	STS_ORBIS_2100M,								// 2100m警報状態
	STS_ORBIS_1100M,								// 1100m警報状態
	STS_ORBIS_600M,									// 600m警報状態
	STS_ORBIS_300M,									// 300m警報状態
	STS_ORBIS_50M,									// 50m警報状態
};

// マイエリア
enum{
	STS_MYAREA_NOALARM = 0,							// 非警報状態
	STS_MYAREA_1100M,								// 1100m警報状態
	STS_MYAREA_600M,								// 600m警報状態
	STS_MYAREA_50M,									// 50m警報状態
};
// マイキャンセル
enum{
	STS_MYCANCEL_RNGOUT = 0,						// 圏外状態
	STS_MYCANCEL_200M,								// 200m圏内
	STS_MYCANCEL_200M_RD,							// 200m圏内(RD受信あり)
};
// 自動キャンセル
enum{
	STS_ATCANCEL_RNGOUT = 0,						// 圏外状態
	STS_ATCANCEL_REG1ST,							// 登録1回目通過
	STS_ATCANCEL_200M_NORD,							// 200m圏内(RD受信なし)
	STS_ATCANCEL_100M_NORD,							// 100m圏内(RD受信なし)
	STS_ATCANCEL_RDRX,								// 圏内RD受信あり
	STS_ATCANCEL_AACDIS,							// 圏内AAC禁止
};
// 自動キャンセル登録
typedef enum{
	REGSTS_IDLE = 0,								// 登録・削除待ち状態
	REGSTS_SRCHPOS,									// 30～150m位置サーチ状態
	REGSTS_CHK300M,									// 300m圏内チェック状態
	REGSTS_CHK500M,									// 500m圏内チェック状態
}EM_ATCANCEL_REGSTS;

// SCPコンテンツ状態定義
enum{
	STS_SCP_RNGOUT = 0,								// 圏外
	STS_SCP_100M,									// 100m圏内状態
	STS_SCP_50M,									// 50m圏内状態
};
// ワンショットコンテンツ状態定義
enum{
	STS_ONESHOTCONT_NOALARM = 0,
	STS_ONESHOTCONT_ALARM,
};

// 1kmコンテンツ状態定義
enum{
	STS_1KMCONT_NOALARM = 0,						// 非警報状態
	STS_1KMCONT_1100M,								// 1100m警報状態
	STS_1KMCONT_600M,								// 600m警報状態
};
// エリアタイプコンテンツ状態定義
enum{
	STS_AREACONT_OUTAREA = 0,						// エリア外状態
	STS_AREACONT_INAREA,							// エリア内状態
};

// ETCゲート状態定義
enum{
	STS_ETC_NOPASS = 0,								// 未通過
	STS_ETC_PASS,									// 通過
};

// 駐禁警報状態
enum{
	PCHK_WRN_OUT = 0,								// 圏外
	PCHK_WRN_INSZ_NOSNDOUT,							// 最重点エリア音声未出力
	PCHK_WRN_INSZ_SNDOUT,							// 最重点エリア音声出力済み
	PCHK_WRN_INZ_NOSNDOUT,							// 重点エリア音声未出力
	PCHK_WRN_INZ_SNDOUT,							// 重点エリア音声出力済み
};

// ゾーン30状態
enum{
	ZONE30_WRN_OUT = 0,								// 圏外
	ZONE30_WRN_IN_NOSNDOUT,							// 圏内音声未出力
	ZONE30_WRN_IN_SNDOUT,							// 圏内音声出力済み
};

// 車上狙い状態
enum{
	SHAJYO_WRN_OUT = 0,								// 圏外
	SHAJYO_WRN_IN_NOSNDOUT,							// 圏内音声未出力
	SHAJYO_WRN_IN_SNDOUT,							// 圏内音声出力済み
};

// トンネル内オービス警報状態定義
enum{
	STS_TUNNEL_ORBIS_NOALARM = 0,
	STS_TUNNEL_ORBIS_2100M,
	STS_TUNNEL_ORBIS_1100M,
	STS_TUNNEL_ORBIS_600M,
};

// MAP生成フーズ中のサブフェーズ
enum{
	MAP_SPHASE_INBLK,								// ブロック内バイナリサーチ
	MAP_SPHASE_GET_SMALL,							// 初期サーチ点からの緯度小サーチ
	MAP_SPHASE_GET_LARGE,							// 初期サーチ点からの緯度大サーチ
};

#define	u4_LON_SPLIT_BASE		((U4)0x00740000)	// 分割基準経度
#define	u4_LON_SPLIT_WIDTH		((U4)0x00000800)	// 経度分割幅
#define	u2_DATA_SPLIT_NUM		((U2)576)			// データ分割数
#define	u1_MAPDATA_SIZE			((U1)16)			// オービスROMデータサイズ

enum{
	TYPE_LAT = 0,
	TYPE_LON,
};

typedef enum{
	TYPE_TGTDEG_EXIST = 0,							// 角度あり（通常）
	TYPE_TGTDEG_NOEXIST,							// 角度なし
	TYPE_TGTDEG_EXIST_VERY_FAR,						// 角度あり(degB最遠方)
	TYPE_TGTDEG_EXIST_DEGA_WIDE,					// 角度あり(degA広い範囲)
	TYPE_TGTDEG_NOEXIST_BACK,						// 角度なし後方チェック
	TYPE_TGTDEG_DEGB_FRONT,							// 前方チェック(degB広い範囲)
	TYPE_TGTDEG_EXIST_HOKAN,						// 角度あり(位置補完)
}EM_TGTDEG;

// サーチ結果定義
enum{
	u1_SRCH_OK = 0,									// サーチ発見
	u1_SRCH_NG_SMALL,								// サーチ発見できず上
	u1_SRCH_NG_LARGE,								// サーチ発見できず下
	u1_SRCH_NG_NONE,								// サーチ発見できずポイントなし
	u1_SRCH_NG_DEV_BUSY,							// デバイスビジー
	u1_SRCH_END,									// サーチ終了
	u1_SRCH_MAX_ERR									// サーチ数最大エラー
};

// マップ登録結果定義
enum{
	REGOK = 0,
	REGNG_LAT_SMALL,
	REGNG_LAT_LARGE,
	REGNG_LON_SMALL,
	REGNG_LON_LARGE,
	REGNG_MAP_MAX,
	REGNG_EEP_BUSY
};

enum{
	RUNSTOP_INIT = 0,
	RUNSTOP_RUN,
	RUNSTOP_STOP,
};

#define	VIRTUAL_INDEX_TYPES		7
enum{
	SCP_VIRTUAL_INDEX = 0,
	SZZ_VIRTUAL_INDEX,
	TZ_VIRTUAL_INDEX,
	KENKYO_VIRTUAL_INDEX,
	ETC_VIRTUAL_INDEX,
	CURVE_VIRTUAL_INDEX,
	ZN30_VIRTUAL_INDEX,
};

enum{
	NON_TARGET_ADDRESS = 0,
	SCP_VIRTUAL_ADDRESS,
	SZZ_VIRTUAL_ADDRESS,
	TZ_VIRTUAL_ADDRESS,
	KENKYO_VIRTUAL_ADDRESS,
	ETC_VIRTUAL_ADDRESS,
	CV_VIRTUAL_ADDRESS,
	ZN30_VIRTUAL_ADDRESS,
};

// エリア状態
enum{
	AREA_STATUS_OUT = 0,
	AREA_STATUS_IN_NOFIRE,
	AREA_STATUS_IN_FIRING,
	AREA_STATUS_IN_FIRED,
};

typedef enum{
	EM_TGT_DATA_MAKER = 0,
	EM_TGT_DATA_CUSTOM,
	EM_TGT_DATA_DYNAMIC,
#if __FREE_DATA_DOWNLOAD__
	EM_TGT_DATA_PAY,		//有料版
#endif
}EM_TGT_DATA_TYPE;

//--------------------------------------------------------------------------
//  内部型定義                                                              
//--------------------------------------------------------------------------
// GPSターゲットROMデータ
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

	U4	u4_addr;
}ST_GPSROM;

// 優先フォーカスターゲット
typedef struct{
	U2			u2_dst;								// 距離
	U2			u2_absDegB;
	U1			u1_priTunnelIn;						// 高優先トンネル内あり
	U1			u1_sts;								// フォーカス状態
	U2			u2_num;								// ターゲット番号
}ST_FOCUS_TGT;

//--------------------------------------------------------------------------
//  外部公開変数                                                            
//--------------------------------------------------------------------------
ST_GPS_STS	stg_gpsSts;

//--------------------------------------------------------------------------
//  変数定義
//--------------------------------------------------------------------------
static ST_GPSINF	sts_gpsinf;						// GPS入力情報
static U1			u1s_poiSetSel;					// POI設定セレクト
F1					f1g_gpsdat_flg;

static F4			f4s_gpsctl_flg;					// フラグ(警報フェーズ開始時初期化)
#define	F_RORBIS_CAN	(f4s_gpsctl_flg.bit.bit00)	// 反対車線オービスキャンセルフラグ
#define	F_RORBIS_NOCAN	(f4s_gpsctl_flg.bit.bit01)	// 反対車線オービスキャンセル否定フラグ
#define	F_MYAREA_IN		(f4s_gpsctl_flg.bit.bit02)	// マイエリア圏内フラグ
#define	F_MYCANCEL_IN	(f4s_gpsctl_flg.bit.bit03)	// マイキャンセル圏内フラグ
#define	F_ATCANCEL_IN	(f4s_gpsctl_flg.bit.bit04)	// オートキャンセル圏内フラグ
#define	F_CANCEL_NG		(f4s_gpsctl_flg.bit.bit05)	// キャンセルNGフラグ
#define	F_RDAREA_IN		(f4s_gpsctl_flg.bit.bit06)	// レーダーエリア内フラグ
#define	F_TRAPZONE_IN	(f4s_gpsctl_flg.bit.bit07)	// トラップゾーン圏内フラグ
#define	F_PCHK_SZ_IN	(f4s_gpsctl_flg.bit.bit08)	// 駐禁監視最重点エリアフラグ
#define	F_PCHK_Z_IN		(f4s_gpsctl_flg.bit.bit09)	// 駐禁監視重点エリアフラグ
#define	F_SCP_H_PASS	(f4s_gpsctl_flg.bit.bit10)	// 道路判別H通過フラグ
#define	F_SCP_N_PASS	(f4s_gpsctl_flg.bit.bit11)	// 道路判別N通過フラグ
#define	F_SCP_HCHK_PASS	(f4s_gpsctl_flg.bit.bit12)	// 道路判別H判定通過フラグ
#define	F_SCP_NCHK_PASS	(f4s_gpsctl_flg.bit.bit13)	// 道路判別N判定通過フラグ
#define	F_SCP_HCHK_NEAR_PASS	\
						(f4s_gpsctl_flg.bit.bit14)	// 道路判別H判定近接通過フラグ
#define	F_SCP_ANYPASS	(f4s_gpsctl_flg.bit.bit15)	// SCP通過フラグ
#define	F_ATCANCEL_NEAR	(f4s_gpsctl_flg.bit.bit16)	// オートキャンセル近接フラグ
#define	F_ETC_START		(f4s_gpsctl_flg.bit.bit17)	// ETCゲート案内STARTフラグ
#define	F_ETC_STOP		(f4s_gpsctl_flg.bit.bit18)	// ETCゲート案内STOPフラグ
#define	F_SHAJYO_IN		(f4s_gpsctl_flg.bit.bit19)	// 車上狙いエリアフラグ
#define	F_RDSCOPE_IN	(f4s_gpsctl_flg.bit.bit20)	// レーダースコープフラグ
#define	F_CHKPNT_IN		(f4s_gpsctl_flg.bit.bit21)	// チェックポイントゾーン圏内フラグ
#define F_ZONE30_IN		(f4s_gpsctl_flg.bit.bit22)	// ゾーン30エリア圏内フラグ

static F1			f1s_gpsctl_flg2;				// フラグ(フェーズ開始時保存)
#define	F_MAPUPD		(f1s_gpsctl_flg2.bit.b2)	// マップ更新通知

static F1			f1s_gpsctl_flg4;
#define	F_TRAP_INCOMING	(f1s_gpsctl_flg4.bit.b0)	// 取締圏内間近
#define	F_CHKPNT_INCOMING	(f1s_gpsctl_flg4.bit.b1)	// 検問圏内間近
#define	F_TRAPOUT_REQ	(f1s_gpsctl_flg4.bit.b2)	// 取締圏外出力要求
#define	F_CHKOUT_REQ	(f1s_gpsctl_flg4.bit.b3)	// 検問圏外出力要求

static U2			u2s_sub_phase;					// GPS処理サブフェーズ

static U2			u2s_mkmap_hiPri_num;			// 高優先マップ作成数
static U2			u2s_mkmap_loPri_num;			// 低優先マップ作成数

static U2			u2s_mkmap_num;					// 作成マップ要素数
static U2			u2s_chkmap_num;					// 判定マップ要素数
static U2			u2s_oldmap_num;
static U1			u1s_parkChkAreaSts;				// 駐禁監視エリア状態
static U1			u1s_zone30AreaSts;				// ゾーン30エリア状態
static U1			u1s_roadJdgSts;					// 道路判定状態
static U2			u2s_inisrch_pos;				// 初期サーチポイント
static U2			u2s_lonarea_old;				// 経度エリア前回値
static U2			u2s_srch_pos;					// サーチしているポイント
static U2			u2s_high_pos;					// サーチ上端
static U2			u2s_low_pos;					// サーチ下端
static U2			u2s_srch_center_grp;			// サーチ中央グループ

// エリア関連
static U1			u1s_runStopSts;

ST_GPSMAP			sts_gpsmap[GPSMAP_MAX];			// GPSマップ
static ST_GPSMAP	*psts_chkmap;					// 判定用マップポインタ
static ST_GPSMAP	sts_oldmap[GPSMAP_MAX];			// GPSマップ入れ替え用前回データ
static U4			u4s_latDiffPrmMinus;
static U4			u4s_latDiffPrmPlus;
static U4			u4s_lonDiffPrmMinus;
static U4			u4s_lonDiffPrmPlus;
static ST_GPSROM	sts_gpsrom;						// GPS ROMデータワーク

static ST_FOCUS_TGT	sts_focusTgt;					// フォーカスターゲット
static ST_FOCUS_TGT	sts_secondary_focusTgt;			// セカンダリフォーカスターゲット
static ST_FOCUS_TGT	sts_warning_focusTgt_Primary;	// プライマリ警報フォーカスターゲット
static ST_FOCUS_TGT	sts_warning_focusTgt_Secondary;	// セカンダリ警報フォーカスターゲット

static ST_INDEX1	sts_index1[3];					// 周辺INDEX1情報
static ST_INDEX1 	*psts_index1;					// INDEX1ポインタ

// GPS警報レベル
static U1			u1s_gpsWarningLvl_wrk;			// ワーク
U1					u1g_gpsWarningLvl;				// 公開データ
static U2			u2s_gpsWarningLvl_dist_wrk;		// ワーク
U2					u2g_gpsWarningLvl_dist;			// 公開データ

static UN_REGDAY	uns_today;						// 日付比較用

static U2	s_SrcRefIdxList[GPS_VISIBLE_TGT_MAX];	// 可視インデックスリスト
static U2	srcRefIdxListSize;						// 可視インデックスリスト数
static float k_lon2_meter;
static Bool is_orbisCountDown;

ST_SET_GPS				stg_setGps[SET_ACS_MAX];		// GPS設定値
ST_SET_MISC				stg_setMisc;					// その他設定値
U1						u1g_setAcsSel;					// 設定値選択
static FILE	*gpsDataFile;
const char	*gpspoi_filename;
static U2	u2g_dataSpec;

//--------------------------------------------------------------------------
//  内部関数プロトタイプ宣言												
//--------------------------------------------------------------------------
static void	PhaseChkMov(void);						// 移動判定フェーズ処理
static void	PhaseMakeMap(void);						// MAP生成フェーズ処理
static void	PhaseChgMap(void);						// MAP変更判定フェーズ処理
static void	PhaseChkWrn(void);						// 警報判定フェーズ処理

static void	UpdLatLonArea(void);					// 緯度・経度範囲更新処理
static void	UpdIndex1(void);						// INDEX1更新処理
static U1	u1_BlkSrch(EM_TGT_DATA_TYPE);			// ブロック検索
static U1	u1_PntSrch(U1, EM_TGT_DATA_TYPE);		// ポイントサーチ
static U1	u1_ReadGpsData(U4, EM_TGT_DATA_TYPE);	// GPSデータ読み出し処理
static U1	u1_RegMap(void);						// MAP登録処理
static Bool Can_data_pre_expire(ST_GPSROM *);		// 事前データ棄却判定処理
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type);	// スクランブル解除処理
static	U2	Shahen(U2, U2);							// 斜辺算出処理
static U2	u2_CalDstAxis(U4, U1);					// 軸方向距離算出処理
static U2	u2_CalDegA(U2,U2, ST_GPSMAP *);			// ∠A算出処理
static S2	s2_CalDegB(U2);							// ∠B算出処理
static void TransitDummy(EM_TGTDEG);				// ダミー
static void	TransitOrbis(EM_TGTDEG);				// オービス警報遷移
static void	TransitZone(EM_TGTDEG);					// ゾーン警報遷移
static void TransitMyArea(EM_TGTDEG);				// マイエリア処理
static void	TransitMyCancel(EM_TGTDEG);				// マイキャンセル処理
static void	TransitAtCancel(EM_TGTDEG);				// オートキャンセル処理
static void	TransitContScp(EM_TGTDEG);				// 速度切替コンテンツ処理
static void TransitPin(EM_TGTDEG);					// ピン状態遷移

static void	TransitCont1kmType(EM_TGTDEG);			// 1kmタイプコンテンツ処理
static void	TransitContOneShotType(EM_TGTDEG);		// ワンショットタイプコンテンツ処理
static void	TransitContAreaType(EM_TGTDEG);			// エリアタイプコンテンツ処理
static void	TransitETCGateType(EM_TGTDEG);			// ETCコンテンツ処理
static void TransitZone30Type(EM_TGTDEG emh_tgtDeg);// ゾーン30タイプコンテンツ処理

static void TransitTunnelOrbis(EM_TGTDEG);			// トンネルオービス警報遷移
static void TransitYudo(EM_TGTDEG);					// 誘導遷移処理
static void TransitSyudo(EM_TGTDEG);				// センサ誘導遷移処理
static void	TransitTunnelZone(EM_TGTDEG);			// トンネルゾーン警報遷移処理

static void	TransitPchkSts(void);					// 駐禁監視状態遷移
static void TransitZone30Sts(void);					// ゾーン30状態遷移
static void	TransitShajyoSts(void);					// 車上狙い状態遷移
static void	TransitETCGuide(void);					// ETCガイド管理

static U1	u1_ChkDegRng(EM_TGTDEG, U2);			// 角度範囲判定処理

static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus);

S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2);
static U1	u1_DegAWideIsOk(ST_GPSMAP *psth_map);

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt);

static U1	IsJudgePOISet(U1);						// POI設定判定処理
static U1	u1_JudgePOISet(ST_GPSMAP *);			// 
static U1	u1_JudgePOISetTunnel(ST_GPSMAP *);		// 

static void update_ahead_hys(ST_GPSMAP *psth_map);
static void update_distance(ST_GPSMAP *psth_map);

//--------------------------------------------------------------------------
//  内部const data定義
//--------------------------------------------------------------------------
static const EM_TGTDEG	TBL_TGTTRANS_PRM[] = {

	TYPE_TGTDEG_NOEXIST,					// dummy					0
	TYPE_TGTDEG_EXIST,						// RD式オービス				1
	TYPE_TGTDEG_EXIST,						// Hシステム式オービス		2
	TYPE_TGTDEG_EXIST,						// LHシステム式オービス		3
	TYPE_TGTDEG_EXIST,						// ループコイル式オービス	4
	TYPE_TGTDEG_EXIST,						// 光電式オービス			5

	TYPE_TGTDEG_EXIST,						// 取締エリア				6
	TYPE_TGTDEG_EXIST,						// 検問エリア				7

	TYPE_TGTDEG_EXIST,						// Nシステム				8
	TYPE_TGTDEG_EXIST,						// 交通監視システム			9

	TYPE_TGTDEG_NOEXIST,					// マイエリア				10
	TYPE_TGTDEG_NOEXIST,					// マイキャンセルエリア		11
	TYPE_TGTDEG_NOEXIST,					// Iキャンセルエリア		12

	TYPE_TGTDEG_NOEXIST,					// 警察署					13
	TYPE_TGTDEG_NOEXIST,					// 交差点監視ポイント		14
	TYPE_TGTDEG_NOEXIST,					// 事故多発エリア			15
	TYPE_TGTDEG_NOEXIST,					// 信号無視抑止システム		16
	
	TYPE_TGTDEG_NOEXIST,					// 道の駅					17
	TYPE_TGTDEG_NOEXIST,					// ハイウェイオアシス		18
	TYPE_TGTDEG_EXIST,						// サービスエリア			19
	TYPE_TGTDEG_EXIST,						// パーキングエリア			20
	TYPE_TGTDEG_EXIST,						// ハイウェイラジオ			21

	TYPE_TGTDEG_EXIST,						// SCP本線入口				22
	TYPE_TGTDEG_EXIST,						// SCP本線出口				23
	TYPE_TGTDEG_EXIST,						// SCPジャンクション		24
	TYPE_TGTDEG_EXIST,						// SCP本線					25
	TYPE_TGTDEG_EXIST,						// SCPパーキング出口		26

	TYPE_TGTDEG_NOEXIST,					// 駐禁最重点エリア			27
	TYPE_TGTDEG_NOEXIST,					// 駐禁重点エリア			28
	TYPE_TGTDEG_NOEXIST,					// 駐車場					29

	TYPE_TGTDEG_NOEXIST,					// 固定キャンセルエリア		30
	TYPE_TGTDEG_EXIST,						// 急カーブ					31
	TYPE_TGTDEG_EXIST,						// 分岐・合流ポイント		32
	TYPE_TGTDEG_EXIST,						// 県境						33
	TYPE_TGTDEG_EXIST,						// 長いトンネル				34
	TYPE_TGTDEG_EXIST,						// ETCレーン				35
	TYPE_TGTDEG_EXIST,						// 急勾配					36
	TYPE_TGTDEG_NOEXIST,					// 踏切						37
	TYPE_TGTDEG_EXIST,						// ユーザーピン				38
	TYPE_TGTDEG_NOEXIST,					// リンクユニット			39
	TYPE_TGTDEG_EXIST,						// リアルタイム取締エリア	40
	TYPE_TGTDEG_NOEXIST,					// とるぱ					41
	TYPE_TGTDEG_EXIST,						// 一時停止					42
	TYPE_TGTDEG_NOEXIST,					// 車上狙い多発エリア		43

	TYPE_TGTDEG_EXIST,						// 非測位誘導データ				44
	TYPE_TGTDEG_EXIST,						// 非測位RD式オービス			45
	TYPE_TGTDEG_EXIST,						// 非測位Hシステム式オービス	46
	TYPE_TGTDEG_EXIST,						// 非測位LHシステム式オービス	47
	TYPE_TGTDEG_EXIST,						// 非測位ループコイル式オービス	48
	TYPE_TGTDEG_EXIST,						// 非測位光電管式オービス		49

	TYPE_TGTDEG_EXIST,						// 非測位取締エリア			50
	TYPE_TGTDEG_EXIST,						// 非測位検問エリア			51

	TYPE_TGTDEG_NOEXIST,					// 公衆トイレ				52
	TYPE_TGTDEG_EXIST,						// 非測位センサ誘導データ	53
	TYPE_TGTDEG_NOEXIST,					// 交番						54

	TYPE_TGTDEG_NOEXIST,					// 消防署					55
	TYPE_TGTDEG_NOEXIST,					// 保育園・幼稚園			56
	
	TYPE_TGTDEG_NOEXIST,					// 高速バス停				57
	TYPE_TGTDEG_NOEXIST,					// ゾーン30					58
};

static void (*const fnc_TBL_TGTTRANS[])(EM_TGTDEG) = {
	
	TransitDummy,							// dummy					0
/*	TransitOrbis,							// RD式オービス				1
	TransitOrbis,							// Hシステム式オービス		2
	TransitOrbis,							// LHシステム式オービス		3
	TransitOrbis,							// ループコイル式オービス	4
	TransitOrbis,							// 光電式オービス			5
	
	TransitZone,							// 取締エリア				6
	TransitZone,							// 検問エリア				7

	TransitContOneShotType,					// Nシステム				8
	TransitContOneShotType,					// 交通監視システム			9

	TransitMyArea,							// マイエリア				10
	TransitMyCancel,						// マイキャンセルエリア		11
	TransitAtCancel,						// Iキャンセルエリア		12

	TransitContOneShotType,					// 警察署					13
	TransitContOneShotType,					// 交差点監視ポイント		14
	TransitContOneShotType,					// 事故多発エリア			15
	TransitContOneShotType,					// 信号無視抑止システム		16

	TransitCont1kmType,						// 道の駅					17
	TransitContOneShotType,					// ハイウェイオアシス		18
	TransitContOneShotType,					// サービスエリア			19
	TransitContOneShotType,					// パーキングエリア			20
	TransitContOneShotType,					// ハイウェイラジオ			21

	TransitContScp,							// SCP本線入口				22
	TransitContScp,							// SCP本線出口				23
	TransitContScp,							// SCPジャンクション		24
	TransitContScp,							// SCP本線					25
	TransitContScp,							// SCPパーキング出口		26

	TransitContAreaType,					// 駐禁最重点エリア			27
	TransitContAreaType,					// 駐禁重点エリア			28
	TransitContOneShotType,					// 駐車場					29

	TransitDummy,							// 固定キャンセルエリア		30
	TransitContOneShotType,					// 急カーブ					31
	TransitContOneShotType,					// 分岐・合流ポイント		32
	TransitContOneShotType,					// 県境						33
	TransitCont1kmType,						// 長いトンネル				34
	TransitETCGateType,						// ETCレーン				35
	TransitDummy,							// 急勾配					36
	TransitContOneShotType,					// 踏切						37
	TransitPin,								// ユーザーピン				38
	TransitDummy,							// リンクユニット			39
	TransitZone,							// リアルタイム取締エリア	40
	TransitCont1kmType,						// とるぱ					41
	TransitContOneShotType,					// 一時停止					42
	TransitContAreaType,					// 車上狙い多発エリア		43

	TransitYudo,							// 非測位誘導データ				44
	TransitTunnelOrbis,						// 非測位RD式オービス			45
	TransitTunnelOrbis,						// 非測位Hシステム式オービス	46
	TransitTunnelOrbis,						// 非測位LHシステム式オービス	47
	TransitTunnelOrbis,						// 非測位ループコイル式オービス	48
	TransitTunnelOrbis,						// 非測位光電管式オービス		49

	TransitTunnelZone,						// 非測位取締エリア			50
	TransitTunnelZone,						// 非測位検問エリア			51
	
	TransitContOneShotType,					// 公衆トイレ				52
	TransitSyudo,							// 非測位センサ誘導データ	53
	TransitContOneShotType,					// 交番						54

	TransitContOneShotType,					// 消防署					55
	TransitContOneShotType,					// 保育園・幼稚園			56

	TransitDummy,							// 高速バス停				57
	TransitZone30Type,						// ゾーン30					58*/
};

//--------------------------------------------------------------------------
//  外部関数定義															
//--------------------------------------------------------------------------
void PoiSample(const char* fname, U2 dataSpec){

	gpspoi_filename = fname;
	u2g_dataSpec = dataSpec;

	sts_gpsinf.lat = 2107021;
	sts_gpsinf.lon = 8220672;
	u2s_lonarea_old = U2_MAX;						// 前回経度エリア範囲外に

	PhaseChkMov();
	PhaseMakeMap();
	PhaseChgMap();
}

//--------------------------------------------------------------------------
//【関数】移動判定フェーズ													
//【機能】自車移動距離を計測して、移動していればINDEX1を更新する			
//--------------------------------------------------------------------------
static void PhaseChkMov(void){

	ST_GPSMAP	*pstt_oldmap;
	ST_GPSMAP	*pstt_curmap;

	UpdLatLonArea();								// 緯度・経度範囲の決定

	UpdIndex1();								// INDEX1更新
	// マップ作成前に旧データの退避
	pstt_oldmap = &sts_oldmap[0];
	pstt_curmap = &sts_gpsmap[0];

	u2s_oldmap_num = u2s_chkmap_num;
	if(u2s_oldmap_num != 0){
		memcpy(pstt_oldmap, pstt_curmap, sizeof(ST_GPSMAP)*u2s_oldmap_num);
	}
	// 緯度・経度の許容範囲パラメータ決定
	u4s_latDiffPrmMinus = sts_gpsinf.lat - u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_latDiffPrmPlus = sts_gpsinf.lat + u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmMinus = sts_gpsinf.lon - u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmPlus = sts_gpsinf.lon + u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];

	u2s_mkmap_hiPri_num = VIRTUAL_INDEX_TYPES;	// 高優先マップ作成数初期化(仮想ターゲット分は確保)
	u2s_mkmap_loPri_num = 0;					// 低優先マップ作成数初期化
	u2s_mkmap_num = VIRTUAL_INDEX_TYPES;		// 作成MAP要素数クリア(仮想ターゲット分は確保)
}
//--------------------------------------------------------------------------
//【関数】MAP生成フェーズ処理												
//【機能】自車位置基準でROMデータを取り出し、MAPを生成する					
//【備考】ブロック番号が大きい程、緯度も大きい								
//--------------------------------------------------------------------------
static void PhaseMakeMap(void){

	U2	u2t_pos_bak;									// 中点保存値
	U1	u1t_groupEnd;
	U1	u1t_srchStep;
	U4	u4t_lon_center;
	ST_INDEX1	*pstt_index1;
	EM_TGT_DATA_TYPE type;

	pstt_index1 = &sts_index1[0];
	type = EM_TGT_DATA_MAKER;

	// 初期化
	u2s_srch_pos = 0;									// サーチ位置初期化
	for(u1t_srchStep = 0; u1t_srchStep <=2 ; u1t_srchStep++){
		switch(u1t_srchStep){
		case 0:
			psts_index1 = &pstt_index1[1];				// 初めは中央から
			break;
		case 1:
			// 現在位置が左寄りなら左優先、右寄りなら右優先
			u4t_lon_center = u4_LON_SPLIT_BASE + (U4)u2s_srch_center_grp*u4_LON_SPLIT_WIDTH + (u4_LON_SPLIT_WIDTH / 2);
			if(sts_gpsinf.lon < u4t_lon_center){		// 左寄り？
				psts_index1 = &pstt_index1[0];			// 次は左
			}
			else{
				psts_index1 = &pstt_index1[2];			// 次は右
			}
			break;
		case 2:
			if(psts_index1 == &pstt_index1[0]){			// まだ読んでない側にする
				psts_index1 = &pstt_index1[2];
			}
			else{
				psts_index1 = &pstt_index1[0];
			}
			break;
		}
		// 1つのグループを検索
		// バイナリサーチ準備
		// 1つのグループを検索
		if(psts_index1->u2_elenum == 0){
			continue;
		}
		u1t_groupEnd = OFF;
		u2s_low_pos = 0;								// 下点
		u2s_high_pos = psts_index1->u2_elenum;			// 上点
		u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1);
														// 中点算出
		u2s_sub_phase = MAP_SPHASE_INBLK;				// バイナリサーチ開始

		for(;;){
			switch(u2s_sub_phase){
		//-----------------ブロック中点からのサーチ-------------
			case MAP_SPHASE_INBLK:
				u2t_pos_bak = u2s_srch_pos;
				switch(u1_BlkSrch(type)){				// ブロック内中点サーチ
				
				case u1_SRCH_OK:						// 初期サーチ点発見
					u2s_sub_phase = MAP_SPHASE_GET_SMALL;
														// 上方に抽出開始
					break;
					
				case u1_SRCH_NG_LARGE:					// 緯度大側に範囲外＝緯度小側を探す
					u2s_high_pos = u2s_srch_pos;		// 下点そのまま 上点は今の中点
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// 中点算出
					if(u2s_srch_pos == u2t_pos_bak){	// 中点に変更なし(ポイントなし)？
						u1t_groupEnd = ON;				// グループ終了
					}
					break;
					
				case u1_SRCH_NG_SMALL:					// 緯度小側に範囲外＝緯度大側を探す
					u2s_low_pos = u2s_srch_pos;			// 下点は今の中点 上点はそのまま
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// 中点算出
					if(u2s_srch_pos == u2t_pos_bak){	// 中点に変更なし(ポイントなし)？
						u1t_groupEnd = ON;				// グループ終了
					}
					break;
//				case u1_SRCH_NG_DEV_BUSY:				// デバイスビジー
//				case u1_SRCH_NG_NONE:					// なし
				default:
					goto srch_end;
				}
				break;

		//----------初期サーチ点からの緯度小方向サーチ----------------------------
			case MAP_SPHASE_GET_SMALL:
				switch(u1_PntSrch(SMALL, type)){		// 緯度小側にXkm探す
				case u1_SRCH_END:						// 緯度小側終了なら
					u2s_sub_phase = MAP_SPHASE_GET_LARGE;
														// 次は緯度大側を探す
					u2s_srch_pos = u2s_inisrch_pos;		// 初期サーチ位置をロード
					break;
				case u1_SRCH_OK:						// 登録OK
														// 引き続き探す
					break;
//				case u1_SRCH_MAX_ERR:					// 登録最大NG
//				case u1_SRCH_NG_DEV_BUSY:				// デバイスビジー
				default:
					goto srch_end;						// フェーズ終了

				}
				break;
		//----------初期サーチ点からの緯度大方向サーチ----------------------------
			case MAP_SPHASE_GET_LARGE:
				switch(u1_PntSrch(LARGE, type)){		// 緯度大側にXkm探す
				case u1_SRCH_END:
					u1t_groupEnd = ON;					// グループ終了
					break;

				case u1_SRCH_OK:						// 登録OK -> 引き続き探す
					break;

//				case u1_SRCH_MAX_ERR:
//				case u1_SRCH_NG_DEV_BUSY:				// デバイスビジー
				default:
					goto srch_end;						// サーチ終了

				}
				break;
			}
			
			if(u1t_groupEnd){							// グループ終了 ?
				break;
			}
		}
	}
srch_end:
	return;
}

//--------------------------------------------------------------------------
//【関数】MAPポイント変更チェックフェーズ処理								
//【機能】MAP入れ替えによる警報状態の初期化、警報終了処置を行う				
//--------------------------------------------------------------------------
static void	PhaseChgMap(void){

	U2	i;											// 作成MAPループカウンタ
	U2	j;											// 判定MAPループカウンタ
	ST_GPSMAP	*pstt_oldmap;
	ST_GPSMAP	*pstt_newmap;

	// マップを詰める
	if((u2s_mkmap_loPri_num != 0)					// 低優先データあり and
			&& (u2s_mkmap_num < GPSMAP_MAX)){				// マップフル以外のとき
		// 低優先データを高優先データのすぐ後ろに移動する
		memcpy(&sts_gpsmap[u2s_mkmap_hiPri_num], &sts_gpsmap[GPSMAP_MAX - u2s_mkmap_loPri_num], sizeof(ST_GPSMAP)*(int)u2s_mkmap_loPri_num);
	}

	if((u2s_oldmap_num != 0)						// 旧マップ数0でない？
			&& (u2s_mkmap_num != 0)){						// 新マップ数0でない？
		// 旧マップ基準で比較する
		pstt_oldmap = &sts_oldmap[0];				// 判定旧MAPポインタ設定

		for(i=0;i<u2s_oldmap_num; i++, pstt_oldmap++){
			pstt_newmap = &sts_gpsmap[0];		// 新MAPポインタ設定
			// 作成マップとアドレス比較する
			for(j=0;j<u2s_mkmap_num; j++, pstt_newmap++){
				if(pstt_newmap->u4_dataAddr == pstt_oldmap->u4_dataAddr){
					// 同一データアドレスあり
					*pstt_newmap = *pstt_oldmap;
					break;						// 発見したので次の判定マップ
				}
			}
		}
	}
	u2s_chkmap_num = u2s_mkmap_num;
	F_MAPUPD = ON;
}

//--------------------------------------------------------------------------
//【関数】警報判定フェーズ処理												
//【機能】GPS警報を判定する													
//--------------------------------------------------------------------------
static void PhaseChkWrn(void){

	U2	u2t_deltaX;
	U2	u2t_deltaY;

	// フェーズ初期化
	u2s_sub_phase = 0;								// サブフェーズ初期化
	psts_chkmap = &sts_gpsmap[VIRTUAL_INDEX_TYPES];	// 判定マップポインタ初期化
	f4s_gpsctl_flg.word = 0;
	f1s_gpsctl_flg4.byte = 0;
	u1s_gpsWarningLvl_wrk = SYS_NO_WARNING;			// 警報なしで初期化
	u2s_gpsWarningLvl_dist_wrk = 10000;				// 10km遠方で初期化

	// フォーカスターゲットワーク初期化
	initFocusTgt(&sts_focusTgt);
	initFocusTgt(&sts_secondary_focusTgt);
	initFocusTgt(&sts_warning_focusTgt_Primary);
	initFocusTgt(&sts_warning_focusTgt_Secondary);

	// 可視インデックスリスト初期化
	srcRefIdxListSize = 0;

	// オービスカウントダウン初期化
	is_orbisCountDown = FALSE;

	if(u2s_chkmap_num > VIRTUAL_INDEX_TYPES){		// マップポイントありなら
		for(u2s_sub_phase = VIRTUAL_INDEX_TYPES; u2s_sub_phase < u2s_chkmap_num ; u2s_sub_phase++){
			// 自車<->ターゲット間距離の更新
			// 軸方向成分の差分を算出
			u2t_deltaX = u2_CalDstAxis(psts_chkmap->u4_tgtLat, TYPE_LAT);
			u2t_deltaY = u2_CalDstAxis(psts_chkmap->u4_tgtLon, TYPE_LON);

			// 距離最新値算出
			psts_chkmap->u2_dst = Shahen(u2t_deltaX, u2t_deltaY);

			// ターゲットとの自車を結んだ直線の角度（∠Ａ）を更新
			psts_chkmap->u2_degA = u2_CalDegA(u2t_deltaX, u2t_deltaY, psts_chkmap);
			
			// ∠Ａと自車方位との差分（∠Ｂ）を更新
			psts_chkmap->s2_degB = s2_CalDegB(psts_chkmap->u2_degA);

			// 距離を更新
			update_distance(psts_chkmap);

			// 前方検出ヒスを更新
			update_ahead_hys(psts_chkmap);

			// 種類別警報状態遷移
			if(IsJudgePOISet(u1s_poiSetSel) == OK){
				fnc_TBL_TGTTRANS[psts_chkmap->un_type.bit.b_code](TBL_TGTTRANS_PRM[psts_chkmap->un_type.bit.b_code]);
			}
			else{
				psts_chkmap->u1_wrnSts = 0;						// 警報状態初期化
			}
			psts_chkmap++;										// MAPポインタを進める
		}
	}

	// 道路判定処理
	// RoadJudge();

	// 駐禁エリアの遷移処理
	TransitPchkSts();

	// ゾーン30の遷移処理
	TransitZone30Sts();
	
}

//-----------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------
//【関数】緯度・経度範囲更新処理											
//【機能】現在の緯度が5分割したエリアのどの位置にあるかを決定する			
//		  現在の経度が576分割したエリアのどの位置にあるかを決定する			
//【備考】エリア番号は1～574まで(端は1つ内側に入れる						
//--------------------------------------------------------------------------
static void	UpdLatLonArea(void){

	U1	i;											// ループカウンタ
	U4	u4t_lonwrk;

	// 緯度エリアの決定
	for(i = 0; i< LATAREA_MAX; i++){
		if(((sts_gpsinf.lat & 0x00FFFF00) >> 8) < u2_TBL_LATAREA[i]){
													// 上位2byteサイズで決定
													// しきい値より小なら
			break;									// エリア決定
		}
	}
	sts_gpsinf.u1_latarea = i;

	u4t_lonwrk = sts_gpsinf.lon;					// 経度
	if(u4t_lonwrk >= 0x860000){
		u4t_lonwrk = 0x860000 - 1;
	}
	else if(u4t_lonwrk <= u4_LON_SPLIT_BASE){
		u4t_lonwrk = u4_LON_SPLIT_BASE;
	}

	u4t_lonwrk -= u4_LON_SPLIT_BASE;
	sts_gpsinf.u2_lonarea =  (U2)(u4t_lonwrk / u4_LON_SPLIT_WIDTH);
	if(sts_gpsinf.u2_lonarea == 0){
		sts_gpsinf.u2_lonarea = 1;
	}
	else if(sts_gpsinf.u2_lonarea >= (u2_DATA_SPLIT_NUM - 1)){
		sts_gpsinf.u2_lonarea = u2_DATA_SPLIT_NUM - 2;
	}

    stg_gpsSts.u1_latarea = sts_gpsinf.u1_latarea;
    stg_gpsSts.u2_lonarea = sts_gpsinf.u2_lonarea;
}

//--------------------------------------------------------------------------
//【関数】INDEX1更新処理													
//【機能】エリアを算出し、エリア変更したら新しいINDEX1に更新する			
//【備考】																	
//--------------------------------------------------------------------------
static void UpdIndex1(void){

	U4	u4t_adr;									// 読出しアドレス

	// エリア変更なら新しくINDEX1を構築する
	if(u2s_lonarea_old != sts_gpsinf.u2_lonarea){	// 変化有り？

		u2s_srch_center_grp = sts_gpsinf.u2_lonarea;

		// YPデータ INDEX1
		u4t_adr = sizeof(ST_HEADER) + ((U4)(u2s_srch_center_grp-1) * (U4)sizeof(ST_INDEX1));
		if((fopen_s(&gpsDataFile, gpspoi_filename, "rb") == 0)
			&& (fseek(gpsDataFile, u4t_adr, SEEK_CUR) == 0)
			&& (fread(&sts_index1[0], sizeof(unsigned char), sizeof(ST_INDEX1) * 3, gpsDataFile) == sizeof(ST_INDEX1)*3))
		{
			// 暗号解除
			DataCryptPoi(sizeof(ST_INDEX1)*3, u4t_adr, (U1 *)&sts_index1[0], u2g_dataSpec);
		}
		fclose(gpsDataFile);

		u2s_lonarea_old = sts_gpsinf.u2_lonarea;	// 前回経度エリアを保存
	}
	// 経度エリア変更なしならINDEX1は変わらない

}

//--------------------------------------------------------------------------//
//【関数】GPSデータ読み出し処理												//
//【機能】指定ブロック、指定位置のデータをRAMに展開する						//
//【戻値】OK：読み出し成功		NG：読み出し失敗							//
//【備考】dynamicとstaticの切替時は必ずcache disableにすること				//
//        →読み出し元が複数になり、物理アドレスが重複するため				//
//--------------------------------------------------------------------------//

enum{
	LOWER_CACHE = 0,
	UPPER_CACHE,
};
static U1	u1_ReadGpsData(U4 u4h_dataAddr, EM_TGT_DATA_TYPE type){

	if((fopen_s(&gpsDataFile, gpspoi_filename, "rb") != 0)
	|| (fseek(gpsDataFile, u4h_dataAddr, SEEK_CUR) != 0)
	|| (fread(&sts_gpsrom, sizeof(unsigned char), u1_MAPDATA_SIZE, gpsDataFile) != u1_MAPDATA_SIZE)){
		fclose(gpsDataFile);
		return NG;
	}
	fclose(gpsDataFile);
	DataCryptPoi(u1_MAPDATA_SIZE, u4h_dataAddr, (U1 *)&sts_gpsrom, u2g_dataSpec);

	sts_gpsrom.u4_addr = u4h_dataAddr;	// データの物理アドレスを保存

	return	OK;
}

//--------------------------------------------------------------------------
//【関数】ブロック内サーチ処理												
//【機能】ブロックの指定位置データが範囲内かチェックする					
//【引数】u1h_romType ：対象ROMタイプ										
//【戻値】u1_SRCH_NG_NONE	: ブロック内にポイントなしのため失敗			
//		  u1_SRCH_NG_DEV_BUSY: デバイスアクセス失敗							
//		  u1_SRCH_NG_SMALL	: 緯度小のため失敗								
//		  u1_SRCH_NG_LARGE	: 緯度大のため失敗								
//		  u1_SRCH_OK		: サーチ成功									
//--------------------------------------------------------------------------
static U1 u1_BlkSrch(EM_TGT_DATA_TYPE type){

	U1	u1t_ret;									// 戻値
	U4	u4t_dataAddr;

	// INDEX1の該当ブロック要素数をチェック
	if(psts_index1->u2_elenum == 0){				// ポイントなし
		u1t_ret = u1_SRCH_NG_NONE;					// ポイントなしにより失敗
	}
	else{
		// INDEX1のグループ先頭位置と位置からデータアドレスを出す
		u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
		// 読み出す
		if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
													// 読み出し失敗なら
			u1t_ret = u1_SRCH_NG_DEV_BUSY;			// DEVICE BUSYを返す
		}
		// MAP登録を試みる
		else{
			switch(u1_RegMap()){					// 登録試行結果
			case REGNG_LAT_SMALL:					// 現在緯度より小
				u1t_ret = u1_SRCH_NG_SMALL;
				break;
			
			case REGNG_LAT_LARGE:					// 現在緯度より大
				u1t_ret = u1_SRCH_NG_LARGE;
				break;
			
			default:								// 緯度範囲内なら
				u1t_ret = u1_SRCH_OK;				// 初期サーチとしてはOK
				// 初期サーチ点情報を記憶
				u2s_inisrch_pos = u2s_srch_pos;		// ブロック内位置記憶
				break;
			}
		}
	}
	return	u1t_ret;
}

//--------------------------------------------------------------------------
//【関数】ポイントサーチ処理												
//【機能】現在のポイントから緯度上下の指定方向にポイントを取り出す			
//【引数】u1h_romType ：対象ROMタイプ										
//		  u1h_dir：取り出す方向		SMALL / LARGE							
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_PntSrch(U1 u1h_dir, EM_TGT_DATA_TYPE type){
	
	U1	u1t_ret = u1_SRCH_OK;						// 戻値
	U4	u4t_dataAddr;
	
	// 次のデータを取り出すため、ブロック内位置・ブロック番号を更新する
	// 緯度小時
	if(u1h_dir == SMALL){							// 緯度小方向時
		if(u2s_srch_pos == 0){						// 位置が一番上？
			return	u1_SRCH_END;					// 終了
		}
		else{										// 位置が一番上でない
			u2s_srch_pos--;							// 位置移動
		}
	}
	else{											// 緯度大方向時
		if(u2s_srch_pos >= (psts_index1->u2_elenum - (U2)1)){
													// ブロック最下点に到達？
			return u1_SRCH_END;
		}
		else{
			u2s_srch_pos++;							// 位置移動
		}
	}

	// データを読み出す
	u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
	if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
		return	u1_SRCH_NG_DEV_BUSY;
	}

	// MAP登録を試みる
	switch(u1_RegMap()){

	case REGNG_MAP_MAX:								// 登録件数オーバーで登録されず
		u1t_ret = u1_SRCH_MAX_ERR;					// 最大数エラーを通知
		break;

	case REGNG_LAT_SMALL:							// 緯度小のため登録されず
	case REGNG_LAT_LARGE:							// 緯度大のため登録されず
		u1t_ret = u1_SRCH_END;						// 緯度方向にはもうないため終了
		break;

//	case REGOK:										// 範囲内のため登録された
//	case REGNG_LON_SMALL:							// 経度小のため登録されず
//	case REGNG_LON_LARGE:							// 経度大のため登録されず
	default:
		u1t_ret = u1_SRCH_OK;						// 緯度方向は入っているので継続
		break;
	}
	
	return	u1t_ret;
}
//--------------------------------------------------------------------------
//【関数】マップ登録処理													
//【機能】ポイントが範囲内であればマップに登録する							
//【戻値】OK：登録可能		NG：登録不可能									
//--------------------------------------------------------------------------
static U1	u1_RegMap(void){

	U4	u4t_diffLat;								// 緯度差分
	U4	u4t_diffLon;								// 経度差分
	U4	u4t_lat;									// 緯度
	U4	u4t_lon;									// 経度
	ST_GPSMAP	*pstt_mkmap;
	U1	u1t_highway;
	Bool in_range = FALSE;

	u4t_lat = u4_RomDeScrmble(&sts_gpsrom, TYPE_LAT);	// 緯度値スクランブル解除

	if(u4t_lat < u4s_latDiffPrmMinus){
		return	REGNG_LAT_SMALL;					// 下限値より下のためNG
	}
	if(u4t_lat > u4s_latDiffPrmPlus){
		return	REGNG_LAT_LARGE;					// 上限値より上のためNG
	}

	u4t_lon = u4_RomDeScrmble(&sts_gpsrom, TYPE_LON);	// 経度値スクランブル解除

	if(u4t_lon < u4s_lonDiffPrmMinus){
		return	REGNG_LON_SMALL;					// 下限値より下のためNG
	}
	if(u4t_lon > u4s_lonDiffPrmPlus){
		return	REGNG_LON_LARGE;					// 上限値より上のためNG
	}

	u4t_diffLat = ABS_SUB(sts_gpsinf.lat, u4t_lat);
	u4t_diffLon = ABS_SUB(sts_gpsinf.lon, u4t_lon);

	// POIデータが大量にあるので、数の多いものは事前に捨てるようにする
	if(Can_data_pre_expire(&sts_gpsrom) == TRUE){
		return REGOK;								// 次に進む
	}

	if(sts_gpsrom.road == ROAD_HIGH){
		u1t_highway = ON;
	}
	else{
		u1t_highway = OFF;
	}
	
	{
		U1 distRng = em_TBL_DISTRNG[sts_gpsrom.u1_type][u1t_highway];

		if((u4t_diffLat <= u4_TBL_LATDIST[distRng][sts_gpsinf.u1_latarea])
		&& (u4t_diffLon <= u4_TBL_LONDIST[distRng][sts_gpsinf.u1_latarea])){
			in_range = TRUE;
		}
	}

	// 範囲内確定
	if(in_range){
		// 優先度判定
		if(u1_TBL_MAPPRI[sts_gpsrom.u1_type] == LO){
			if(u2s_mkmap_num >= GPSMAP_MAX){		// すでに低優先を入れる余地なし？
				return	REGOK;						// 登録はしないが、まだ高優先があるかもしれないので
													// 次の作成に進む
			}
			pstt_mkmap = &sts_gpsmap[(GPSMAP_MAX - 1) - u2s_mkmap_loPri_num];
													// 低優先マップ作成位置
			u2s_mkmap_loPri_num++;					// 低優先マップ作成数更新
		}
		else{
			if(u2s_mkmap_hiPri_num >= GPSMAP_MAX){	// すでにすべて高優先で入れる余地なし？
				return	REGNG_MAP_MAX;				// 最大個数のため登録不可
			}
			pstt_mkmap = &sts_gpsmap[u2s_mkmap_hiPri_num];
													// 高優先マップ位置
			u2s_mkmap_hiPri_num++;					// 高優先マップ作成数更新
			if((u2s_mkmap_loPri_num != 0)			// 低優先マップ１件以上あり and
			&& ((u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num) > GPSMAP_MAX)){
													// 高優先が低優先の上書きした？
				u2s_mkmap_loPri_num--;				// 低優先マップ１件削除
			}
		}
		u2s_mkmap_num = u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num;
													// マップ作成数更新

		pstt_mkmap->u4_tgtLat = u4t_lat;			// 緯度保存
		pstt_mkmap->u4_tgtLon = u4t_lon;			// 経度保存
		pstt_mkmap->u4_dataAddr = sts_gpsrom.u4_addr;
													// 元アドレス保存
		pstt_mkmap->u2_tgtDeg = (U2)sts_gpsrom.u1_deg * (U2)DEGDAT_LSB * (U2)DEG_LSB;
		pstt_mkmap->un_type.bit.b_code = sts_gpsrom.u1_type;
													// 種番
		pstt_mkmap->un_type.bit.b_road = sts_gpsrom.road;
													// 道路属性
		pstt_mkmap->un_type.bit.b_tunnel = sts_gpsrom.tunnel;
													// トンネル属性
		pstt_mkmap->un_type.bit.b_dataArea = sts_gpsrom.area;
													// エリア属性保存
		pstt_mkmap->u1_wrnSts = 0;					// 警報状態初期化
		pstt_mkmap->hys_ahead = FALSE;				// 前方検出初期化
		pstt_mkmap->ahead_hys_count = 0;			// 前方ヒスカウンタ初期化
		pstt_mkmap->u2_countdown_dist = U2_MAX;
		pstt_mkmap->u2_dst_old = U2_MAX;
		pstt_mkmap->u1_areaStsRd = AREA_STATUS_OUT;
		pstt_mkmap->u1_areaStsSc = AREA_STATUS_OUT;

		if(sts_gpsrom.u1_type == TGT_ICANCEL){		// Iキャンセルのとき
			if(sts_gpsrom.un_extra.dynamic.un_regday.hword == uns_today.hword){
				pstt_mkmap->u1_wrnSts = STS_ATCANCEL_REG1ST;
													// キャンセルさせない
			}
		}
		memcpy(&pstt_mkmap->un_extra.byte[0], &sts_gpsrom.un_extra.byte[0], EXTRA_DATA_SIZE);
													// EXTRA保存
		// 登録完了
	}
	// 範囲は無効でも登録はOKにする(初期サーチの場合はマップ情報はなくてもよい)

	return	REGOK;

}

static Bool	Can_data_pre_expire(ST_GPSROM *psth_gpsrom){
	return FALSE;
}

//--------------------------------------------------------------------------
//【関数】GPS緯度・経度情報スクランブル解除処理								
//【機能】情報を使用できる数値に変換する									
//【引数】対象タイプ  TYPE_LAT：緯度  TYPE_LON：経度						
//【戻値】U4	緯度値 or 経度値 (LSB : 10^-3分)							
//【備考】																	
//--------------------------------------------------------------------------
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type){

	U4	u4t_calwork;								// 算出ワーク
	U4	u4t_addval;									// 加算値
	
	if(u1h_type == TYPE_LAT){
		u4t_calwork = (U4)(((U4)p_rom->u1_lat3 << 16) + (U4)p_rom->u2_lat12);
		u4t_addval = u4_LAT_BASE_MIN;
	}
	else{
		u4t_calwork = (U4)(((U4)p_rom->u1_lon3 << 16) + (U4)p_rom->u2_lon12);
		u4t_addval = u4_LON_BASE_MIN;
	}

	u4t_calwork &= u4_LATLON_MSKPTN;
	u4t_calwork += u4t_addval;
	
	return	u4t_calwork;
}

//--------------------------------------------------------------------------
//【関数】自車<->任意点間軸成分距離算出										
//【機能】自車と任意点との指定軸成分の距離を算出する						
//【引数】U4	u4h_latlon：緯度 or 経度									
//        U1	u1h_type：緯度 or 経度										
//【戻値】U2	距離(LSB 1m)												
//--------------------------------------------------------------------------
static U2	u2_CalDstAxis(U4 u4h_latlon, U1 u1h_type){

	U4	u4t_diff;

	if(u1h_type == TYPE_LAT){						// 緯度成分指定(X軸)
		u4t_diff = ABS_SUB(u4h_latlon, sts_gpsinf.lat);
													// 経度差算出
		u4t_diff *= (U4)u2_TBL_LAT2MTR[sts_gpsinf.u1_latarea];
													// m距離に変換
		u4t_diff += 500;							// 0.1mの位で四捨五入
		u4t_diff /= 1000;							// LSB 0.001m→1m
	}
	else{											// 経度成分指定(Y軸)
		u4t_diff = ABS_SUB(u4h_latlon, sts_gpsinf.lon);
													// 経度差算出
		u4t_diff = (U4)((float)u4t_diff * k_lon2_meter + 0.5f);
													// 1mに変換
	}

	// 一応ガードする
	if(u4t_diff > (U4)U2_MAX){
		u4t_diff = (U4)U2_MAX;
	}

	return	(U2)u4t_diff;
}

//	-----------------------------------------------------------------------------------------------
static	U2	Shahen(U2 inX, U2 inY){

	U2	z;

	if(inX == 0){
		z = (U2)inY;
	}
	else if(inY == 0){
		z = (U2)inX;
	}
	else
	{
		U4 wrk = (U4)sqrtf((float)((S4)inX*(S4)inX + (S4)inY*(S4)inY));
		if(wrk > U2_MAX)
		{
			wrk = U2_MAX;
		}
		z = (U2)wrk;
	}

	return (U2)z;

}

//--------------------------------------------------------------------------
//【関数】自車<->指定点直線角度算出											
//【機能】自車と指定点とを結んだ直線の指定点基準角度を算出する				
//【引数】u2h_deltaX:緯度差		u2h_deltaY：経度差							
//【戻値】U2 角度(LSB 0.1°)												
//--------------------------------------------------------------------------
static U2	u2_CalDegA(U2 u2h_deltaX, U2 u2h_deltaY, ST_GPSMAP *psth_map){

	U1	u1t_deg_area;								// 象限
	U2	u2t_deg = 0;								// 算出角度

	// 象限の特定
	if(psth_map->u4_tgtLat <= sts_gpsinf.lat){
													// 指定点から見て車両は緯度＋側？
		// 第１もしくは第４象限
		if(psth_map->u4_tgtLon <= sts_gpsinf.lon){
													// 指定点から見て車両は経度＋側？
			u1t_deg_area = 1;						// 第１象限
		}
		else{										// 第４象限
			u1t_deg_area = 4;
		}
	}
	else{											// 指定点から見て車両は緯度－側
		// 第２もしくは第３象限
		if(psth_map->u4_tgtLon <= sts_gpsinf.lon){
													// 指定点から見て車両は経度＋側？
			u1t_deg_area = 2;						// 第２象限
		}
		else{
			u1t_deg_area = 3;						// 第３象限
		}
	}

	// 角度算出
	if(u2h_deltaX == 0){							// 分母0で計算不能時
		u2t_deg = (U2)(90*DEG_LSB);					// 90°にする
	}
	else{
		u2t_deg = (U2)(atan2f((float)u2h_deltaY, (float)u2h_deltaX) * (180.0f * (float)DEG_LSB / ZMATH_PI));
	}

	// 象限毎の角度
	switch(u1t_deg_area){
	case 1:											// 第１象限
													// ∠Aそのまま
		break;
	case 2:											// 第２象限
		u2t_deg = (U2)(180*DEG_LSB) - u2t_deg;		// ∠A = 180°-∠A
		break;
	case 3:											// 第３象限
		u2t_deg = (U2)(180*DEG_LSB) + u2t_deg;		// ∠A = 180°+∠A
		break;
	case 4:											// 第４象限
		u2t_deg = (U2)(360*DEG_LSB) - u2t_deg;		// ∠A = 360°-∠A
		break;
	}

	return	u2t_deg;
}
//--------------------------------------------------------------------------
//【関数】ポイント対向角算出												
//【機能】自車がポイントに対して向いている角度を算出する					
//【引数】U2 ∠A (LSB 0.1°)												
//【戻値】S2 ∠B (LSB 0.1°)												
//--------------------------------------------------------------------------
static S2	s2_CalDegB(U2 u2h_degA){

	U2	u2t_degA;
	S2	s2t_degB;

	// ∠Aをポイント基準->自車基準にする（180°反対側にする）
	if(u2h_degA >= (180*DEG_LSB)){						// 180°以上なら
		u2t_degA = u2h_degA - (180*DEG_LSB);
	}
	else{												// 180°未満なら
		u2t_degA = u2h_degA + (180*DEG_LSB);
	}
	// 自車方位基準で∠Aの真逆方向角との差分を算出
	s2t_degB = s2_CalDegSub(u2t_degA, sts_gpsinf.u2_deg);

	return	s2t_degB;
}

//--------------------------------------------------------------------------
//【関数】角度差算出処理													
//【機能】角度2から見た角度1の差分を算出する								
//【引数】u2h_deg1:角度1(LSB 0.1°)		u2h_deg2：角度2(LSB 0.1°)			
//【戻値】角度差：S2 (LSB 0.1°)											
//【備考】右回転に+、左回転に-、レンジは-180°～+180°						
//--------------------------------------------------------------------------
S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2){

	S2 s2t_degdiff = (S2)u2h_deg1 - (S2)u2h_deg2;

	if(s2t_degdiff >= 180*DEG_LSB)
	{
		s2t_degdiff -= 360*DEG_LSB;
	}
	else if(s2t_degdiff <= -180*DEG_LSB)
	{
		s2t_degdiff += 360*DEG_LSB;
	}

	return	s2t_degdiff;
}
//--------------------------------------------------------------------------
//【関数】オービス警報遷移処理												
//【機能】オービス警報を判定する											
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitOrbis(EM_TGTDEG emh_tgtDeg){
	
	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U2	u2t_rngOutDst = u2_DIST_1500M;						// 1500mで初期化
	U2	u2t_passDst = u2_DIST_60M;							// 60m通過判定で初期化

	// 離脱距離
	if((psts_chkmap->un_type.bit.b_road == ROAD_HIGH)		// 高速属性時
	&& (psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL)){	// and 非トンネル
		u2t_rngOutDst = u2_DIST_2500M;
	}
	// 通過距離
	if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){		// 高速属性時
		u2t_passDst = u2_DIST_100M;							// 100mで通過判定
	}

	if(u1s_poiSetSel == SEL_POI_NORMAL){					// トンネル誘導状態でないなら警報処理

		if(psts_chkmap->u2_dst > u2t_rngOutDst){				// 圏外距離より大なら
			psts_chkmap->u1_wrnSts = STS_ORBIS_NOALARM;			// 非警報状態へ
		}
		else{
			// 警報状態遷移
			switch(psts_chkmap->u1_wrnSts){
		//-------非警報状態-------------------------------------------------------//
			case STS_ORBIS_NOALARM:									// 非警報状態
				// 扇範囲内でターゲットに向かうと警報開始
				if(u1t_degChk != OK){								// 角度NG？
					break;
				}
				if(psts_chkmap->un_type.bit.b_tunnel == TUNNEL_IN){	// トンネル内判定
					break;
				}
				if(psts_chkmap->u2_dst <= u2_DIST_330M){			// 330m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m警報状態へ
																	// 500m警報音要求
					// ダイレクトに300mに飛び込んだとき、通過速度は告知しない（間に合わない）
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_600M;		// 600m警報状態へ
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M)	{	// 1100m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m警報状態へ
				}
				else{
					if((psts_chkmap->u2_dst <= u2_DIST_2100M)		// 2100m以内
					&& (u1_ChkDegRng(TYPE_TGTDEG_EXIST_VERY_FAR, psts_chkmap->u2_dst) == OK)
																	// 対向角度OK
					&& (psts_chkmap->un_type.bit.b_road == ROAD_HIGH)
																	// 高速属性
					&& (psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL)
																	// 非トンネル
					&& ((stg_setMisc.b_roadSel != SET_ROAD_AUTO) || (u1s_roadJdgSts == ROADJDG_STS_HIGH))
																	// 道路選択オート以外か、オートで高速走行状態
					&& (sts_gpsinf.b_spddeg_ok == OK)					// 速度方位有効で
					&& (sts_gpsinf.u2_spd >= ((U2)psts_chkmap->un_extra.orbis.b_lmtSpd * (U2)10 * SPD_LSB))){
																	// 制限速度以上
						psts_chkmap->u1_wrnSts = STS_ORBIS_2100M;	// 2100m警報状態へ
					}
				}
				break;
		//-------2100m警報状態----------------------------------------------------//
			case STS_ORBIS_2100M:									// 2100m警報状態
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// 角度OK and 1100m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m警報状態へ
				}
				break;
		//-------1100m警報状態----------------------------------------------------//
			case STS_ORBIS_1100M:									// 1100m警報状態
				if(u1t_degChk == OK){								// 角度OK
					if(psts_chkmap->u2_dst <= u2_DIST_330M){		// 330m範囲内?
						psts_chkmap->u1_wrnSts = STS_ORBIS_300M;	// 300m警報状態へ
						// ダイレクトに300mに飛び込んだとき、通過速度は告知しない（間に合わない）
					}
				 	else if(psts_chkmap->u2_dst <= u2_DIST_600M){	// 600m範囲内なら
						psts_chkmap->u1_wrnSts = STS_ORBIS_600M;	// 600m警報状態へ
					}
				}
				break;
		//-------600m警報状態----------------------------------------------------//
			case STS_ORBIS_600M:									// 600m警報状態
				if(psts_chkmap->u2_dst <= u2t_passDst){				// 通過判定距離内？
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m警報状態へ
				}
				else if((psts_chkmap->u2_dst <= u2_DIST_330M) && (u1t_degChk == OK)){
																	// 330m範囲内 and 角度OK 
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m警報状態へ
				}
				break;
		//-------300m警報状態----------------------------------------------------//
			case STS_ORBIS_300M:									// 300m警報状態
				if(psts_chkmap->u2_dst <= u2t_passDst){				// 通過判定距離内？
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m警報状態へ
				}
				break;

			default:
				break;
			}
		}

		// オービスでのフォーカスターゲット判定
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){		// 高速オービス
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, FALSE);
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
			else{													// 一般オービス
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
		}

		// GPS警報レベルの判定
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			if(u1t_degChk == OK){
				// HIの判定
				if(psts_chkmap->u2_dst <= u2_DIST_600M){
					// 通過距離以内なら赤色から開放する
					if(psts_chkmap->u2_dst > u2t_passDst)
					{
						update_gps_warning_lvl(SYS_RED_WARNING_HI, psts_chkmap->u2_dst);
					}
				}
				// MIDの判定
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
					update_gps_warning_lvl(SYS_RED_WARNING_MID, psts_chkmap->u2_dst);
				}
				// LOの判定
				else if(psts_chkmap->u2_dst <= u2_DIST_2100M){
					update_gps_warning_lvl(SYS_RED_WARNING_LO, psts_chkmap->u2_dst);
				}
			}
		}
	}
}
//--------------------------------------------------------------------------
//【関数】角度範囲判定処理													
//【機能】引数で指定されたタイプで角度条件を判定する						
//【引数】emh_tgtdeg : ターゲット方位有無									
//		  u2h_dst ： 自車～ターゲット中心点距離								
//【戻値】U1 OK : 角度範囲内  NG:角度範囲外									
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_ChkDegRng(EM_TGTDEG emh_tgtdeg, U2 u2h_dst){

	U1	u1t_ret = NG;									// 不成立で初期化

	switch(emh_tgtdeg){
	default:
		if((u1_ChkDegValid())							// 角度有効
		&& (u1_DegBIsOk(emh_tgtdeg, u2h_dst) == TRUE)){	// 対向角有効
			if((emh_tgtdeg == TYPE_TGTDEG_NOEXIST)		// ターゲットに方位がないとき
			|| (u1_DegAIsOk(psts_chkmap) == TRUE)){		//  or 扇内角度
				u1t_ret = OK;							// 成立
			}
		}
		break;
	case TYPE_TGTDEG_EXIST_DEGA_WIDE:
		if((u1_ChkDegValid())							// 角度有効
		&& (u1_DegBIsOk(emh_tgtdeg,u2h_dst) == TRUE)	// 対向角有効
		&& (u1_DegAWideIsOk(psts_chkmap) == TRUE)){		// 扇広範囲
			u1t_ret = OK;
		}
		break;
	case TYPE_TGTDEG_NOEXIST_BACK:						// 背後(ターゲットの中心が後ろ)
		if((u1_ChkDegValid())							// 角度有効
		&& (u1_DegBIsOk(TYPE_TGTDEG_DEGB_FRONT, u2h_dst) == FALSE)){
														// 前方に存在しない
			u1t_ret = OK;
		}
		break;
	}

	return	u1t_ret;

}
static void update_distance(ST_GPSMAP *psth_map)
{

	// 前回値がないとき最新値のまま
	if(psth_map->u2_dst_old == U2_MAX)
	{
		psth_map->u2_dst_old = psth_map->u2_dst;
	}
	else
	{
		S2 diff = (S2)psth_map->u2_dst - (S2)psth_map->u2_dst_old;

		// 距離減少 もしくは +20m以上後退
		if(diff <= 0 || diff >= 20)
		{
			psth_map->u2_dst_old = psth_map->u2_dst;
		}
		else
		{
			psth_map->u2_dst = psth_map->u2_dst_old;	// 前回値で距離を保つ
		}
	}
}

static void update_ahead_hys(ST_GPSMAP *psth_map)
{
	Bool face = FALSE;

	if(TBL_TGTTRANS_PRM[psth_map->un_type.bit.b_code] == TYPE_TGTDEG_EXIST)
	{
		if(u1_DegAWideIsOk(psth_map) == TRUE)
		{
			face = TRUE;
		}
	}
	else
	{
		face = TRUE;
	}

	// 状態を遷移する
	if(psth_map->hys_ahead)
	{
		// 方位付き場合、その方位の方向にいない場合は即時無効
		if(!face)
		{
			psth_map->hys_ahead = FALSE;
			psth_map->ahead_hys_count = 0;
		}
		else
		{
			// 前方に存在しない
			if(psth_map->s2_degB < s2_DEG_TGTFWD_MIN || psth_map->s2_degB > s2_DEG_TGTFWD_MAX)
			{
				// 回頭運動などで極端に後方になった場合は一発判定
				if(psth_map->s2_degB < s2_NOAHEAD_ABS_DEG_MIN || psth_map->s2_degB > s2_NOAHEAD_ABS_DEG_MAX)
				{
					psth_map->hys_ahead = FALSE;
					psth_map->ahead_hys_count = 0;
				}
				else
				{
					// 連続でn回継続すれば非前方へ遷移
					if(++(psth_map->ahead_hys_count) >= u1_NOAHEAD_HYS_COUNT)
					{
						psth_map->hys_ahead = FALSE;
						psth_map->ahead_hys_count = 0;
					}
				}
			}
			else
			{
				psth_map->ahead_hys_count = 0;
			}
		}
	}
	else
	{
		// 対向していて、前方に存在するなら即時前方確定
		if(face && psth_map->s2_degB >= s2_DEG_TGTFWD_MIN && psth_map->s2_degB <= s2_DEG_TGTFWD_MAX)
		{
			psth_map->hys_ahead = TRUE;
			psth_map->ahead_hys_count = 0;
		}
	}
}

//--------------------------------------------------------------------------
//【関数】∠A範囲判定処理(広範囲)											
//【機能】自車がターゲット方位の±90°未満に存在するかどうか判定する		
//【引数】なし																
//【戻値】U1 TRUE：範囲内	FALSE：範囲外									
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_DegAWideIsOk(ST_GPSMAP *psth_map){

	U1	u1t_ret = FALSE;							// 範囲外で初期化
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ∠Aがターゲットの±90°未満にあれば範囲内
	if((s2t_deg > s2_DEG_TGTRNG_WIDE_MIN)
	&& (s2t_deg < s2_DEG_TGTRNG_WIDE_MAX)){
		u1t_ret = TRUE;								// 範囲内
	}
	
	return	u1t_ret;
}

//--------------------------------------------------------------------------
//【関数】POI設定判定処理													
//【機能】対象としているPOIデータが設定上有効かどうか判定する				
//【備考】																	
//--------------------------------------------------------------------------
enum{
	ROADCHK_OFF,
	ROADCHK_ON,
	ROADCHK_ONLY_MAN,								// マニュアル時のみチェック
};
static	U1	IsJudgePOISet(U1 u1h_sel)
{
	if(u1h_sel == SEL_POI_NORMAL){						// ノーマル
		return (u1_JudgePOISet(psts_chkmap));			// 
	}
	else{												// トンネル
		return (u1_JudgePOISetTunnel(psts_chkmap));			// 
	}
}

static	U1	u1_JudgePOISet(ST_GPSMAP *psth_map){

	U1	u1t_ret = NG;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_roadChk = ROADCHK_ON;					// 道路チェック要

	// 種番判定
	switch(psth_map->un_type.bit.b_code){
	default:
		break;

	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
		}
		break;

	case TGT_TRAP_ZONE:
	case TGT_RT_TRAP_ZONE:
	case TGT_NOFIX_TRAP_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_trapWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// 全禁止以外
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// レベル無効 or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_trapWrnLvl)){
																				// 設定レベル以上
				u1t_ret = OK;
			}
		}
		break;

	case TGT_CHKPNT_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_chkWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// 全禁止以外
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// レベル無効 or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_chkWrnLvl)){
																				// 設定レベル以上
				u1t_ret = OK;
			}
		}
		break;

	case TGT_NSYS:
		if(stg_setGps[u1g_setAcsSel].b_nSys){		// Nシステム設定ON
			u1t_ret = OK;
		}
		break;

	case TGT_TRFCHK:
		if(stg_setGps[u1g_setAcsSel].b_trafChkSys){	// 交通監視システムON
			u1t_ret = OK;
		}
		break;

	case TGT_POLICE:
		if(psth_map->un_extra.police.b_policeType == POLICE_TYPE_STATION){
			if(stg_setGps[u1g_setAcsSel].b_police){	// 警察署ON
				u1t_ret = OK;
			}
		}
		else{
			if(stg_setGps[u1g_setAcsSel].b_hwPolice){
													// 高速警察隊ON
				u1t_ret = OK;
			}
		}
		break;

	case TGT_ACCIDENT:
		if(stg_setGps[u1g_setAcsSel].b_accident){	// 事故多発ON
			u1t_ret = OK;
		}
		break;

	case TGT_MICHINOEKI:
		if(stg_setGps[u1g_setAcsSel].b_michinoeki){	// 道の駅ON
			u1t_ret = OK;
		}
		break;

	case TGT_CROSSING:
		if(stg_setGps[u1g_setAcsSel].b_crossing){	// 交差点監視システムON
			u1t_ret = OK;
		}
		break;

	case TGT_SIGNAL:
		if(stg_setGps[u1g_setAcsSel].b_signal){		// 信号無視抑止システムON
			u1t_ret = OK;
		}
		break;

	case TGT_SA:
		if(stg_setGps[u1g_setAcsSel].b_sa){			// サービスエリアON
			u1t_ret = OK;
		}
		break;

	case TGT_PA:
		if(stg_setGps[u1g_setAcsSel].b_pa){			// パーキングエリアON
			u1t_ret = OK;
		}
		break;

	case TGT_HWOASYS:
		if(stg_setGps[u1g_setAcsSel].b_hwOasys){	// ハイウェイオアシスON
			u1t_roadChk = ROADCHK_OFF;
			u1t_ret = OK;
		}
		break;

	case TGT_HWRADIO:
		if(stg_setGps[u1g_setAcsSel].b_hwRadio){	// ハイウェイラジオON
			u1t_ret = OK;
		}
		break;

	case TGT_PARKCHK_AREA_SZ:
	case TGT_PARKCHK_AREA_Z:
		if(stg_setGps[u1g_setAcsSel].b_parkChkArea){// 駐禁エリアON
			u1t_ret = OK;
		}
		break;

	case TGT_SHAJYOU_AREA:
		if(stg_setGps[u1g_setAcsSel].b_shajyoArea){	// 車上狙い多発エリアON
			u1t_ret = OK;
		}
		break;

	case TGT_PARKING:
		if(stg_setGps[u1g_setAcsSel].b_parking){	// 駐車場表示ON
			u1t_ret = OK;
		}
		break;

	case TGT_MYAREA:								// 道路属性に依存しない
	case TGT_MYCANCEL:
	case TGT_PIN:
		u1t_roadChk = ROADCHK_OFF;
		u1t_ret = OK;
		break;

	case TGT_ETC:
		if(stg_setGps[u1g_setAcsSel].b_etcLane){
			if(u1t_roadSel != SET_ROAD_NORM){
				u1t_ret = OK;
				if(u1t_roadSel == SET_ROAD_AUTO){
					if(psth_map->un_extra.etc.b_pointType == ETC_POINT_STOP){
													// STOP点は
						u1t_roadChk = ROADCHK_OFF;	// 道路属性不問
					}
				}
				else{								// オール or 高速
					u1t_roadChk = ROADCHK_OFF;		// 道路属性不問
				}
			}
		}
		break;

	case TGT_CURVE:
		if(stg_setGps[u1g_setAcsSel].b_curve){		// 急カーブON
			u1t_ret = OK;
		}
		break;

	case TGT_BRAJCT:
		if(stg_setGps[u1g_setAcsSel].b_brajct){		// 分岐・合流ON
			u1t_ret = OK;
		}
		break;

	case TGT_KENKYO:
		if(stg_setGps[u1g_setAcsSel].b_kenkyo){		// 県境ON
			u1t_ret = OK;
		}
		break;

	case TGT_TUNNEL:
		if(stg_setGps[u1g_setAcsSel].b_tunnel){		// トンネルON
			u1t_ret = OK;
		}
		break;

	case TGT_TORUPA:
		if(stg_setGps[u1g_setAcsSel].b_torupa){		// とるぱON
			u1t_ret = OK;
		}
		break;

	case TGT_SCP_ICI:
	case TGT_SCP_ICO:
	case TGT_SCP_JC:
	case TGT_SCP_SPD:
	case TGT_SCP_PO:
		u1t_roadChk = ROADCHK_ONLY_MAN;				// 手動時のみチェック要
		u1t_ret = OK;
		break;

	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
			if((u1t_roadSel == SET_ROAD_AUTO)		// オート選択時
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)){
													// 制限70km/h以下の高速道属性
				u1t_roadChk = ROADCHK_OFF;			// 道路判定不要
			}
		}
		break;

	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_SENSOR_YUDO:
		u1t_ret = OK;
		u1t_roadChk = ROADCHK_OFF;					// 道路判定不要
		break;

	case TGT_RAILROAD_CROSSING:						// 踏切
	case TGT_TMPSTOP:								// 一時停止注意ポイント
	case TGT_TOILET:
	case TGT_KOBAN:
	case TGT_FIRE:
	case TGT_HOIKU:
		u1t_ret = OK;								// マップ読み込み時に設定は確認済み
		break;
	case TGT_ZONE30:
		if(stg_setGps[u1g_setAcsSel].b_zone30){		// ゾーン30
			u1t_ret = OK;
		}
		break;

	}

	// 道路判定
	if(u1t_ret == OK){
		if(u1t_roadChk != ROADCHK_OFF){						// 道路チェック要
			if(psth_map->un_type.bit.b_road != ROAD_HIGH){	// 一般道POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// 道路自動判定設定時
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_HIGH)){// 現在高速道
						u1t_ret = NG;
					}
				}
				else{										// 手動設定時
					if(u1t_roadSel == SET_ROAD_HIGH){		// 高速のみ告知設定
						u1t_ret = NG;
					}
				}
			}
			else{											// 高速道POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// 道路自動判定設定時
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_NORM)){
															// 一般道告知のみ要 ?
						u1t_ret = NG;
					}
				}
				else{										// 手動設定時
					if(u1t_roadSel == SET_ROAD_NORM){		// 一般のみ告知設定
						u1t_ret = NG;
					}
				}
			}
		}
	}
	return	u1t_ret;

}

//--------------------------------------------------------------------------
//【関数】POI設定判定処理（トンネル時）										
//【機能】対象としているPOIデータが設定上有効かどうか判定する				
//【備考】																	
//--------------------------------------------------------------------------
static	U1	u1_JudgePOISetTunnel(ST_GPSMAP *psth_map){

	U1	u1t_ret = NG;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_roadChk = ROADCHK_ON;					// 道路チェック要

	// 種番判定
	switch(psth_map->un_type.bit.b_code){
	default:
		break;
// オービスは既存の(Y)も誘導に使う
	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
		}
		break;

	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_KODEN_ORBIS:
		if(stg_setGps[u1g_setAcsSel].b_orbis){
			u1t_ret = OK;
			if((u1t_roadSel == SET_ROAD_AUTO)		// オート選択時
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)){
													// 制限70km/h以下の高速道属性
				u1t_roadChk = ROADCHK_OFF;			// 道路判定不要
			}
		}
		break;

	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_SENSOR_YUDO:
		u1t_ret = OK;
		u1t_roadChk = ROADCHK_OFF;					// 道路判定不要
		break;

	case TGT_NOFIX_TRAP_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_trapWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// 全禁止以外
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// レベル無効 or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_trapWrnLvl)){
																				// 設定レベル以上
				u1t_ret = OK;
			}
		}
		break;

	case TGT_NOFIX_CHKPNT_ZONE:
		if(stg_setGps[u1g_setAcsSel].b_chkWrnLvl != SET_TRAPCHK_WRNLVL_OFF){
													// 全禁止以外
			if((psth_map->un_extra.trapchk.b_level == TRAPCHK_LEVEL_INVALID)	// レベル無効 or
			|| (psth_map->un_extra.trapchk.b_level >= stg_setGps[u1g_setAcsSel].b_chkWrnLvl)){
																				// 設定レベル以上
				u1t_ret = OK;
			}
		}
		break;
	}

	// 道路判定
	if(u1t_ret == OK){
		if(u1t_roadChk != ROADCHK_OFF){						// 道路チェック要
			if(psth_map->un_type.bit.b_road != ROAD_HIGH){	// 一般道POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// 道路自動判定設定時
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_HIGH)){// 現在高速道
						u1t_ret = NG;
					}
				}
				else{										// 手動設定時
					if(u1t_roadSel == SET_ROAD_HIGH){		// 高速のみ告知設定
						u1t_ret = NG;
					}
				}
			}
			else{											// 高速道POI
				if(u1t_roadSel == SET_ROAD_AUTO){			// 道路自動判定設定時
					if((u1t_roadChk == ROADCHK_ON)
					&& (u1s_roadJdgSts == ROADJDG_STS_NORM)){
															// 一般道告知のみ要 ?
						u1t_ret = NG;
					}
				}
				else{										// 手動設定時
					if(u1t_roadSel == SET_ROAD_NORM){		// 一般のみ告知設定
						u1t_ret = NG;
					}
				}
			}
		}
	}
	return	u1t_ret;
}

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt)
{
	psth_tgt->u1_sts = PRI_FOCUS_TGT_NONE;
	psth_tgt->u2_dst = U2_MAX;
	psth_tgt->u2_num = TGTNUM_NOTGT;
	psth_tgt->u2_absDegB = (U2)(180*DEG_LSB);
	psth_tgt->u1_priTunnelIn = FALSE;
}

//--------------------------------------------------------------------------
//【関数】駐禁監視エリア状態遷移処理										
//【機能】駐禁監視エリア状態遷移を行う										
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitPchkSts(void){

	if(stg_setGps[u1g_setAcsSel].b_parkChkArea == OFF){		// 設定OFF ?
		u1s_parkChkAreaSts = PCHK_WRN_OUT;						// 圏外にする
		return;
	}

	switch(u1s_parkChkAreaSts){
	case PCHK_WRN_OUT:											// エリア外
		if(F_PCHK_SZ_IN){										// 最重点エリア内 ?
			if(u1s_runStopSts == RUNSTOP_STOP){					// 走行後停車 ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// 最重点音声出力済みへ
				
				// 仮想マップ更新
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// 音声キュー格納要求
																// 警報音出力要求
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// 最重点音声未出力へ
			}
		}
		else if(F_PCHK_Z_IN){									// 重点エリア内 ?
			if(u1s_runStopSts == RUNSTOP_STOP){					// 走行後停車 ?
				u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;		// 重点音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// 音声キュー格納要求
																// 警報音出力要求
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INZ_NOSNDOUT;		// 重点音声未出力へ
			}
		}
		break;

	case PCHK_WRN_INSZ_NOSNDOUT:								// 最重点で音声未出力
		if(!F_PCHK_SZ_IN){										// 最重点からは離脱
			if(F_PCHK_Z_IN){									// 重点エリア内 ?
				if(u1s_runStopSts == RUNSTOP_STOP){				// 走行後停車 ?
					u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;	// 重点音声出力済みへ
					// 仮想マップ更新
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
					// 音声キュー格納要求
																// 警報音出力要求
				}
				else{
					u1s_parkChkAreaSts = PCHK_WRN_INZ_NOSNDOUT;	// 重点音声未出力へ
				}
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// 圏外へ
			}
		}
		else{													// 最重点エリア継続
			if(u1s_runStopSts == RUNSTOP_STOP){					// 走行後停車 ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// 最重点音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// 音声キュー格納要求
																// 警報音出力要求
			}
		}
		break;

	case PCHK_WRN_INSZ_SNDOUT:									// 最重点で音声出力済み
		if(!F_PCHK_SZ_IN){										// 最重点からは離脱
			if(F_PCHK_Z_IN){									// 重点エリア内 ?
				u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;		// 重点音声出力済みへ
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// 圏外へ
			}
		}
		break;

	case PCHK_WRN_INZ_NOSNDOUT:									// 重点で音声未出力
		if(F_PCHK_SZ_IN){										// 最重点圏内
			if(u1s_runStopSts == RUNSTOP_STOP){					// 走行後停車 ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// 最重点音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
				// 音声キュー格納要求
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// 最重点音声未出力へ
			}
		}
		else{
			if(F_PCHK_Z_IN){									// 重点継続?
				if(u1s_runStopSts == RUNSTOP_STOP){				// 走行後停車 ?
					u1s_parkChkAreaSts = PCHK_WRN_INZ_SNDOUT;	// 重点音声出力済みへ
					// 仮想マップ更新
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_Z;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
					sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;
					// 音声キュー格納要求
				}
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// 圏外へ
			}
		}
		break;

	case PCHK_WRN_INZ_SNDOUT:									// 重点で音声出力済み
		if(F_PCHK_SZ_IN){										// 最重点圏内
			if(u1s_runStopSts == RUNSTOP_STOP){					// 走行後停車 ?
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_SNDOUT;		// 最重点音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_type.bit.b_code = TGT_PARKCHK_AREA_SZ;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].u4_dataAddr = SZZ_VIRTUAL_ADDRESS;
				sts_gpsmap[SZZ_VIRTUAL_INDEX].un_extra.no_parking.u2_mapPctNum = 0;

				// 音声キュー格納要求
			}
			else{
				u1s_parkChkAreaSts = PCHK_WRN_INSZ_NOSNDOUT;	// 最重点音声未出力へ
			}
		}
		else{
			if(!F_PCHK_Z_IN){									// 重点圏外?
				u1s_parkChkAreaSts = PCHK_WRN_OUT;				// 圏外へ
			}
		}
		break;

	default:
		u1s_parkChkAreaSts = PCHK_WRN_OUT;
		break;
	}
}

//--------------------------------------------------------------------------
//【関数】ゾーン30エリア状態遷移処理										
//【機能】ゾーン30エリア状態遷移を行う										
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void TransitZone30Sts(void){

	switch(u1s_zone30AreaSts){
	case ZONE30_WRN_OUT:										// エリア外
		if(F_ZONE30_IN){										// エリア内？
			if(u1s_runStopSts == RUNSTOP_RUN){					// 走行中
				u1s_zone30AreaSts = ZONE30_WRN_IN_SNDOUT;		// 圏内音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_type.bit.b_code = TGT_ZONE30; // 58
				sts_gpsmap[ZN30_VIRTUAL_INDEX].u4_dataAddr = ZN30_VIRTUAL_ADDRESS;
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_extra.zone30.u2_mapPctNum = 0;
				// 音声キュー格納要素
			}
			else{
				u1s_zone30AreaSts = ZONE30_WRN_IN_NOSNDOUT;		// 圏内音声未出力へ
			}
		}
		break;
	
	case ZONE30_WRN_IN_NOSNDOUT:								// 圏内音声未出力
		if(!F_ZONE30_IN){
			u1s_zone30AreaSts = ZONE30_WRN_OUT;
		}
		else{
			if(u1s_runStopSts == RUNSTOP_RUN){					// 走行中
				u1s_zone30AreaSts = ZONE30_WRN_IN_SNDOUT;		// 圏内音声出力済みへ
				// 仮想マップ更新
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_type.bit.b_code = TGT_ZONE30; // 58
				sts_gpsmap[ZN30_VIRTUAL_INDEX].u4_dataAddr = ZN30_VIRTUAL_ADDRESS;
				sts_gpsmap[ZN30_VIRTUAL_INDEX].un_extra.zone30.u2_mapPctNum = 0;
				// 音声キュー格納要素
			}
		}
		break;
	
	case ZONE30_WRN_IN_SNDOUT:									// 圏内音声出力済み
		if(!F_ZONE30_IN){
			u1s_zone30AreaSts = ZONE30_WRN_OUT;
		}
		break;
	default:
		u1s_zone30AreaSts = ZONE30_WRN_OUT;
		break;
	}
}


static void	TransitDummy(EM_TGTDEG emh_tgtDeg){

}

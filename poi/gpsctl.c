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
#include "mupass.h"

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
// キュー状態
enum{
	VC_QUEUE_EMPTY = 0,								// キュー空
	VC_QUEUE_DATON,									// キューデータあり
	VC_QUEUE_FULL,									// キューフル
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

enum{
	INDX_RDRX_OFF = 0,
	INDX_RDRX_ON,
};

enum{
	GPSTGT_VC_STS_INIT = 0,							// 初期状態
	GPSTGT_VC_STS_FIXOUTWT,							// 測位音出力待ち状態
	GPSTGT_VC_STS_FIXOUT,							// 測位音出力中状態
	GPSTGT_VC_STS_OUTOK,							// 出力許可状態
};

typedef enum{
	MYREG_RET_REGNG = 0,							// 登録できなかった
	MYREG_RET_REGOK_PTFULL,							// 登録できたがマップがフル
	MYREG_RET_REGOK,								// 登録できた
}EM_MYREG_RET;

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
#pragma pack()

// GPSボイスキュー
typedef struct{
	EM_VC			u1_vcNum;							// ボイス番号
	EM_VCGPS_TYPE	em_vcType;							// ボイスタイプ
	U4				u4_addr;							// ターゲットデータアドレス
	U4				live_count;							// 生存カウント値
}ST_GPSVC_QUEUE;

// GPSボイスキュー管理情報
typedef struct{
	ST_GPSVC_QUEUE	*queue;
	U1				queueSize;
	U1				sts;
	U1				rdPos;
	U1				wrPos;
}ST_GPSVC_QUEUE_CTRL;
enum{
	VCGPS_HIPRI = 0,
	VCGPS_MIDPRI,
	VCGPS_LOPRI,
};

// 優先フォーカスターゲット
typedef struct{
	U2			u2_dst;								// 距離
	U2			u2_absDegB;
	U1			u1_priTunnelIn;						// 高優先トンネル内あり
	U1			u1_sts;								// フォーカス状態
	U2			u2_num;								// ターゲット番号
}ST_FOCUS_TGT;

// MAP選択用情報
typedef struct{
	ST_GPSMAP	*pst_map;
	U1			isDegOk;
}ST_MAPSEL;

typedef struct{
	U4	u4_dataAddr;								// データアドレス
	U2	u2_countdown_dist;							// カウントダウン距離
	U2	u2_dst_old;									// 距離前回値
	U1	u1_wrnSts;									// 警報状態
	U1	hys_ahead;									// 前方検出状態
	U1	ahead_hys_count;							// 前方ヒスカウンタ
	U1	u1_areaStsRd;								// レーダーエリア状態
	U1	u1_areaStsSc;								// 無線エリア状態
	U1	u1_areaScSnd;								// エリア無線音記憶
}ST_OLDMAP;

#define		__ENABLE_DATA_CACHE__

#ifdef		__ENABLE_DATA_CACHE__
//#define	__CACHE_DEBUG__
#define	u2_DATA_CACHE_SIZE	256
typedef struct{
	U1	u1_cache[u2_DATA_CACHE_SIZE];
	U1	u1_valid;
	U4	u4_topAddr;
	U4	u4_endAddr;
}ST_DATA_CACHE;
#endif

typedef enum{
	HOKAN_MARK_INVALID = 0,
	HOKAN_MARK_VALID_NOPRI,
	HOKAN_MARK_VALID_PRI,
}EM_HOKAN_MARK;

// 補完ターゲット
typedef struct{
	U4	u4_data_address;		// データアドレス
	EM_HOKAN_MARK	mark;		// マーク
	U2	u2_deg;					// 方向
	U2	u2_dst;					// 距離
	U2	u2_lmtspd;				// 制限速度
	U1	u1_yudo_number;			// 誘導番号
}ST_HOKAN_TARGET;

#if __I2C_NCYC_SEPARATE__ == 1
	#define SEM_I2C_GPSCTL SEM_I2C_2
#else
	#define SEM_I2C_GPSCTL SEM_I2C
#endif

//--------------------------------------------------------------------------
//  変数定義
//--------------------------------------------------------------------------
ST_GPS_STS	stg_gpsSts;

// 外部変数
ST_GPSMAP_INF		stg_gpsMapInf;					// GPSマップ情報
ST_GPSWRN_INF		stg_gpsWrnInf;					// GPS警報情報
F1					f1g_gpsFlg;						// GPSフラグ
// 全体
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
#define	F_STS_ATCANCEL	(f1s_gpsctl_flg2.bit.b0)	// 自動キャンセル内部状態
#define	F_STS_MYCANCEL	(f1s_gpsctl_flg2.bit.b1)	// マイキャンセル内部状態
#define	F_MAPUPD		(f1s_gpsctl_flg2.bit.b2)	// マップ更新通知
#define	F_ATCANCEL_DEL	(f1s_gpsctl_flg2.bit.b3)	// オートキャンセル削除要求フラグ
#define	F_MYCANCEL_DEL	(f1s_gpsctl_flg2.bit.b4)	// マイキャンセル削除要求フラグ
#define	F_MYAREA_DEL	(f1s_gpsctl_flg2.bit.b5)	// マイエリア削除要求フラグ
#define	F_ROADJDG_NOFIX	(f1s_gpsctl_flg2.bit.b6)
#define	F_PRIPLAYSND	(f1s_gpsctl_flg2.bit.b7)	// 優先サウンド再生フラグ

static F1			f1s_gpsctl_flg3;				// フラグ(電源ON中保存)
#define	F_REQ_DEFRAG	(f1s_gpsctl_flg3.bit.b0)	// デフラグ要求記憶

static F1			f1s_gpsctl_flg4;
#define	F_TRAP_INCOMING	(f1s_gpsctl_flg4.bit.b0)	// 取締圏内間近
#define	F_CHKPNT_INCOMING	(f1s_gpsctl_flg4.bit.b1)	// 検問圏内間近
#define	F_TRAPOUT_REQ	(f1s_gpsctl_flg4.bit.b2)	// 取締圏外出力要求
#define	F_CHKOUT_REQ	(f1s_gpsctl_flg4.bit.b3)	// 検問圏外出力要求

static U1			u1s_gpsctl_phase;				// GPS処理フェーズ
static U2			u2s_sub_phase;					// GPS処理サブフェーズ
static U2			u2s_mkmap_hiPri_num;			// 高優先マップ作成数
static U2			u2s_mkmap_loPri_num;			// 低優先マップ作成数

static U2			u2s_mkmap_num;					// 作成マップ要素数
static U2			u2s_chkmap_num;					// 判定マップ要素数
static U2			u2s_oldmap_num;
static U1			u1s_parkChkAreaSts;				// 駐禁監視エリア状態
static U1			u1s_zone30AreaSts;				// ゾーン30エリア状態
static U1			u1s_shajyoAreaSts;				// 車上狙いエリア状態
static U1			u1s_roadJdgSts;					// 道路判定状態
static U1			u1s_carStopTmr;					// 停車タイマ
static U1			u1s_carNosokuiTmr;				// 走行・停車非測位タイマ
static U1			u1s_curLmtSpd;					// 現在の制限速度値
static U1			u1s_memLmtSpd;					// 制限速度告知記憶値
static U1			u1s_relaxSetOld;				// リラックスチャイム設定前回値
static U1			u1s_myswEvtTmr;					// マイSWイベントタイマ
static U1			u1s_dynamicTargetFace;			// 動的ターゲット面
static U1			ems_atCancelRegSts;				// オートキャンセル登録状態(EM_ATCANCEL_REGSTS)
static U1			u1s_wrt_wrk;					// 書き込みワーク
static U2			u2s_inisrch_pos;				// 初期サーチポイント
static U2			u2s_lonarea_old;				// 経度エリア前回値
static U2			u2s_srch_pos;					// サーチしているポイント
static U2			u2s_high_pos;					// サーチ上端
static U2			u2s_low_pos;					// サーチ下端
static U2			u2s_srch_box;					// サーチ中BOX
static U2			u2s_srch_center_grp;			// サーチ中央グループ
static U2			u2s_srch_inbox_pos;				// サーチBOX内位置
static U2			u2s_rdNoRxTmr;					// RD非受信タイマ
static U2			u2s_scpPassTmr;					// SCP通過タイマ
static U2			u2s_myswTimOut;					// マイSWタイムアウト
static U2			u2s_carRunTmr;					// 走行タイマ
static U2			u2s_dynamicBoxUsedNum;			// 使用済みBOX数

// エリア関連
static U1			u1s_runStopSts;

// ETC関連
static U1			u1s_ETC_guide_sts;
static U2			u2s_ETC_guide_tmr;
static ST_GPSMAP	sts_ETC_guide_point;

// オービス・TRAP・N関連
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

static U2			u2s_sndTgtNum;					// 音声出力ターゲット番号
static U4			u4s_sndTgtAddress;

static ST_GPSMAP	sts_scpPoint;
static ST_GPSMAP	*psts_trapOutMap;
static ST_GPSMAP	*psts_chkOutMap;

static U4			u4s_mapupd_lat;					// マップ更新時緯度
static U4			u4s_mapupd_lon;					// マップ更新時軽度
static ST_INDEX1	sts_index1[3];					// 周辺INDEX1情報
static ST_INDEX1	sts_index1_dynamic[3];			// 周辺INDEX1情報(動的)
static ST_INDEX1	sts_index1_custom[3];			// 周辺INDEX1情報(カスタム)
static ST_INDEX1 	*psts_index1;					// INDEX1ポインタ
#if __FREE_DATA_DOWNLOAD__
static ST_INDEX1	sts_index1_pay[3];				// 周辺INDEX1情報(有料版)
#endif
static U4			u4s_srch_address;

static U4			u4s_relaxTmr;					// リラックスチャイムタイマ

// SW関連
U2					u2g_failTmr;					// フェールタイマ

// キュー関連
static ST_GPSVC_QUEUE	sts_gpsVcHiPriQueue[VC_HIPRI_QUEUE_MAX];
													// 高優先ボイスキュー
static ST_GPSVC_QUEUE	sts_gpsVcMidPriQueue[VC_MIDPRI_QUEUE_MAX];
													// 中優先ボイスキュー
static ST_GPSVC_QUEUE	sts_gpsVcLoPriQueue[VC_LOPRI_QUEUE_MAX];
													// 低優先ボイスキュー

static ST_GPSVC_QUEUE_CTRL	sts_gpsVcQueueCtrl[3];

// キャッシュ
#ifdef __ENABLE_DATA_CACHE__
static ST_DATA_CACHE	sts_dataCache;
#ifdef __CACHE_DEBUG__
static U2			u2s_cacheHitNum;
static U2			u2s_cacheMissHitNum;
#endif
#endif

// GPS警報レベル
static U1			u1s_gpsWarningLvl_wrk;			// ワーク
U1					u1g_gpsWarningLvl;				// 公開データ
static U2			u2s_gpsWarningLvl_dist_wrk;		// ワーク
U2					u2g_gpsWarningLvl_dist;			// 公開データ
U1					u1g_gpsRdScopeShow;
static U1			u1s_gpsRdScopeTmr;
// 日付比較用
static UN_REGDAY	uns_today;

static Bool			g_InMyArea;						// マイエリア内か
static Bool			g_InMyCancelArea;				// マイキャンセルエリア内か

static U2	s_SrcRefIdxList[GPS_VISIBLE_TGT_MAX];	// 可視インデックスリスト
static U2	srcRefIdxListSize;						// 可視インデックスリスト数

static U2	u2s_gpsHoldTmr;							// GPS保持タイマ

static U1	u1s_max_rxperc_for_icancel;

static U1	u1s_gpsTgtVcSts;
static U1	u1s_kenkyo_mem;

ST_GPSVC_QUEUE		sts_lastRequested_queue;

static S4	gps_distToHighway;

static float k_lon2_meter;

static U4	u4s_live_counter;

static ST_GPSMAP *psts_rd_bp2_pri_map;				// RD最優先
static ST_GPSMAP *psts_sc_bp2_pri_map;				// SC最優先

static Bool is_orbisCountDown;

static U2 u2s_vcsts_failsafe_tmr;

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

static void CustomDataChk(void);					// カスタムデータチェック

static U1	u1_ChkGpsMapUpd(void);					// MAP更新判定処理
static void	UpdLatLonArea(void);					// 緯度・経度範囲更新処理
static void	UpdIndex1(void);						// INDEX1更新処理
static U1	u1_BlkSrch(EM_TGT_DATA_TYPE);			// ブロック検索
static U1	u1_PntSrch(U1, EM_TGT_DATA_TYPE);		// ポイントサーチ
static U1	u1_ReadGpsData(U4, EM_TGT_DATA_TYPE);	// GPSデータ読み出し処理
static U1	u1_RegMap(void);						// MAP登録処理
static Bool Can_data_pre_expire(ST_GPSROM *);		// 事前データ棄却判定処理
static void	RomEnScrmble(U1, U4, U1 *);				// スクランブル処理
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
static void	RoadJudge(void);						// 道路判定処理
static U1	u1_TransitRoadJudge(void);
static U1	u1_ChkDegRng(EM_TGTDEG, U2);			// 角度範囲判定処理
static U1	u1_DegAIsOk(ST_GPSMAP *);				// ∠A範囲判定処理
static U1	u1_DegAIsOk_Variable(ST_GPSMAP *psth_map, U1 u1h_deg);
													// ∠A範囲判定処理(可変)
static U1	u1_DegAWideIsOk(ST_GPSMAP *);			// ∠A範囲判定処理(広範囲)
static U1	u1_DegBIsOk(EM_TGTDEG, U2);				// ∠B範囲判定処理
static U1	u1_GpsOrbisVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// GPSオービスボイス要求
static U1	u1_GpsZoneVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// ゾーンボイス要求
static U1	u1_Gps1kmContVcReq(ST_GPSMAP *, EM_VCGPS_TYPE, ST_VCGPS *);
													// 1kmタイプコンテンツボイス要求
static U1	u1_Gps500mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 500mタイプコンテンツボイス要求
static U1	u1_Gps300mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 300mタイプコンテンツボイス要求
static U1	u1_Gps100mContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 100mタイプコンテンツボイス要求
static U1	u1_GpsScpContVcReq(ST_GPSMAP *, ST_VCGPS *);
													// 速度切替コンテンツボイス要求
static U1	u1_GpsAreaVcReq(EM_VCGPS_TYPE, ST_VCGPS *);
													// エリアボイス要求
static U1	SubVcReqDir(S2 , ST_VCGPS *);			// サブ関数：ボイス方向処理
static U1	SubVcReqUpper1000m(U2, ST_VCGPS *);		// サブ関数：ボイス1000m以上距離処理
static U1	SubVcReqUpper500m(U2, ST_VCGPS *);		// サブ関数：ボイス500m以上距離処理
static void	SubVcReqUnder500m(U2, ST_VCGPS *);		// サブ関数：ボイス500m未満距離処理
static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus);
static void	ChkFocusTgt_2nd(ST_FOCUS_TGT *psth_2nd_focus_tgt, ST_GPSMAP *psth_map, U2 u2t_dstSub, U2 u2t_absDegB, U1 u1t_focus);

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt);

static void ChkRevOrbisArea(ST_GPSMAP *);			// 反対車線オービス判定処理
static U1	IsJudgePOISet(U1);						// POI設定判定処理
static U1	u1_JudgePOISet(ST_GPSMAP *);			// 
static U1	u1_JudgePOISetTunnel(ST_GPSMAP *);		// 

static void	RoadJdgRoutine(void);					// 道路判定定期処理

static void	AddVisibleIdxLst(U2);					// 可視インデックスリスト追加処理
static void	SortVisibleIdxLst(U2);					// 可視インデックスリストソート処理
static void	SortVisibleIdxLst_2nd(U2 u2h_secondIdx);

static void sortSrcRefIdxByDistance(void);			// インデックスリストソート距離ソート処理

//static void	CalTunnelInRemDist(void);				// トンネル内残距離算出処理
static void	ChkZoneGpsWrnLvl(U1);					// ゾーンGPS警報レベル判定処理

static void	GuardPictNumber(ST_GPSMAP *);
static Bool	UpdateSoundTarget(U2 *pu2h_sndTgtNum, U4 *pu4h_orgAddress, const ST_GPSMAP *psth_map, U2 u2h_mapNum); 

static void update_gps_warning_lvl(EM_SYS_WARNING_LVL src_lvl, U2 dist);

static void update_ahead_hys(ST_GPSMAP *psth_map);
static void update_distance(ST_GPSMAP *psth_map);

static void TransitAreaStatusForRd(ST_GPSMAP *psth_map);
static void TransitAreaStatusForSc(ST_GPSMAP *psth_map, U1 degChk);

static Bool isCustomDateValid(ST_HEADER *header);

S2	s2_CalDegSub(U2 u2h_deg1, U2 u2h_deg2);

void	GpsVcEnQueue(EM_VC u1h_voice, EM_VCGPS_TYPE emh_vcType, U4 u4h_addr);
void	VcReq(EM_VC u1h_req){
}
U1	u1_ChkDegValid(void){
	return	(U1)TRUE;
}
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
	TransitOrbis,							// RD式オービス				1
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
	TransitZone30Type,						// ゾーン30					58
};

// 駐禁状態変換
static const U1	u1_TBL_CONV_PCHKSTS[] = {
	PCHK_AREA_OUT,
	PCHK_AREA_SZ,
	PCHK_AREA_SZ,
	PCHK_AREA_Z,
	PCHK_AREA_Z,
};
// ゾーン30状態変換
static const U1	u1_TBL_CONV_ZONE30STS[] = {
	ZONE30_AREA_OUT,
	ZONE30_AREA_IN,
	ZONE30_AREA_IN,
};

// キューデフォルト値
static const ST_GPSVC_QUEUE_CTRL	st_TBL_gpsVcQueueDef[] = {

	{	&sts_gpsVcHiPriQueue[0],	VC_HIPRI_QUEUE_MAX,		VC_QUEUE_EMPTY,		0,		0,		},
	{	&sts_gpsVcMidPriQueue[0],	VC_MIDPRI_QUEUE_MAX,	VC_QUEUE_EMPTY,		0,		0,		},
	{	&sts_gpsVcLoPriQueue[0],	VC_LOPRI_QUEUE_MAX,		VC_QUEUE_EMPTY,		0,		0,		},
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
//【関数】GPSマップ更新判定処理												
//【機能】前回マップ更新した位置からの移動距離でMAP更新を判定する			
//【戻値】TRUE：更新要		FALSE：更新不要									
//--------------------------------------------------------------------------
static U1	u1_ChkGpsMapUpd(void){

	U4	u4t_latdiff;								// 緯度差分
	U4	u4t_londiff;								// 経度差分
	U1	u1t_judge = FALSE;

	if(0){//vpol_flg1(EVTFLG_REQ_MAPUPD, TWF_CLR) == E_OK){
		u1t_judge = TRUE;
		u2s_lonarea_old = U2_MAX;					// INDEX強制更新
	}
	else{
		// 最終マップ更新位置からの距離を測定
		// 緯度成分絶対値差分
		u4t_latdiff = ABS_SUB(sts_gpsinf.lat, u4s_mapupd_lat);

		// 経度成分絶対値差分
		u4t_londiff = ABS_SUB(sts_gpsinf.lon, u4s_mapupd_lon);

		// 移動量が規定距離を越えている or 更新要求発生 ならマップを更新させる
		if((u4t_latdiff >= u4_TBL_LATDIST[DISTRNG_200M][sts_gpsinf.u1_latarea])
		|| (u4t_londiff >= u4_TBL_LONDIST[DISTRNG_200M][sts_gpsinf.u1_latarea])){
			u1t_judge = TRUE;
		}
	}

	if(u1t_judge){
		// 更新位置を記憶
		u4s_mapupd_lat = sts_gpsinf.lat;
		u4s_mapupd_lon = sts_gpsinf.lon;
	}

	// 移動量が少なければ更新不要

	return	u1t_judge;
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

		memcpy(&pstt_mkmap->un_extra.byte[0], &sts_gpsrom.un_extra.byte[0], EXTRA_DATA_SIZE);
													// EXTRA保存
		// 登録完了
	}
	// 範囲は無効でも登録はOKにする(初期サーチの場合はマップ情報はなくてもよい)

	return	REGOK;

}

static Bool	Can_data_pre_expire(ST_GPSROM *psth_gpsrom){

	Bool	ret = FALSE;
	// 密集度が高いものはマップに載せる時点で設定で捨てる
	// 踏切、一時停止、公衆トイレ、交番
	
	// 旧誘導は捨てる
	
	switch(psth_gpsrom->u1_type){
	case TGT_RAILROAD_CROSSING:
		if(!stg_setGps[u1g_setAcsSel].b_fumikiri){
			ret = TRUE;
		}
		break;
	case TGT_TMPSTOP:
		if(!stg_setGps[u1g_setAcsSel].b_tmpstop){
			ret = TRUE;
		}
		break;

	case TGT_NOFIX_YUDO:
		if(psth_gpsrom->un_extra.yudo.u1_yudo_type != YUDO_TYPE_COMMON){
			ret = TRUE;
		}
		break;

	case TGT_TOILET:
		if(!stg_setGps[u1g_setAcsSel].b_toilet){
			ret = TRUE;
		}
		break;
	case TGT_KOBAN:
		if(!stg_setGps[u1g_setAcsSel].b_koban){
			ret = TRUE;
		}
		break;

	case TGT_FIRE:
		if(!stg_setGps[u1g_setAcsSel].b_fire){
			ret = TRUE;
		}
		break;

	case TGT_HOIKU:
		if(!stg_setGps[u1g_setAcsSel].b_hoiku){
			ret = TRUE;
		}
		break;
	}
	
	return ret;
}

//--------------------------------------------------------------------------
//【関数】GPS緯度・経度情報スクランブル処理									
//【機能】オフセット値に変換して格納する									
//【備考】																	
//--------------------------------------------------------------------------
static void	RomEnScrmble(U1 u1h_type, U4 u4h_input, U1 *pu1h_output){

	if(u1h_type == TYPE_LAT){						// 緯度
		u4h_input -= u4_LAT_BASE_MIN;
	}
	else{											// 経度
		u4h_input -= u4_LON_BASE_MIN;
	}

	// 1→2→3の順で格納
	pu1h_output[0] = (U1)(u4h_input & (U4)0x000000FF);
	pu1h_output[1] = (U1)((u4h_input & (U4)0x0000FF00) >> 8);
	pu1h_output[2] = (U1)((u4h_input & (U4)0x00FF0000) >> 16);
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
			psts_chkmap->u2_countdown_dist = U2_MAX;			// カウントダウンクリア
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
																	// 500m警報音要求
					// ダイレクトに300mに飛び込んだとき、通過速度は告知しない（間に合わない）
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_600M;		// 600m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M)	{	// 1100m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
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
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_2KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------2100m警報状態----------------------------------------------------//
			case STS_ORBIS_2100M:									// 2100m警報状態
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// 角度OK and 1100m範囲内?
					psts_chkmap->u1_wrnSts = STS_ORBIS_1100M;		// 1100m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------1100m警報状態----------------------------------------------------//
			case STS_ORBIS_1100M:									// 1100m警報状態
				if(u1t_degChk == OK){								// 角度OK
					if(psts_chkmap->u2_dst <= u2_DIST_330M){		// 330m範囲内?
						psts_chkmap->u1_wrnSts = STS_ORBIS_300M;	// 300m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
						// ダイレクトに300mに飛び込んだとき、通過速度は告知しない（間に合わない）
					}
				 	else if(psts_chkmap->u2_dst <= u2_DIST_600M){	// 600m範囲内なら
						psts_chkmap->u1_wrnSts = STS_ORBIS_600M;	// 600m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------600m警報状態----------------------------------------------------//
			case STS_ORBIS_600M:									// 600m警報状態
				if(psts_chkmap->u2_dst <= u2t_passDst){				// 通過判定距離内？
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m警報状態へ
					if((u1_ChkDegRng(TYPE_TGTDEG_EXIST_DEGA_WIDE, psts_chkmap->u2_dst)== OK)
																	// 通過角度OK
					&& (psts_chkmap->un_type.bit.b_tunnel == OFF)	// トンネル以外
					&& (stg_setGps[u1g_setAcsSel].b_orbisPass)){	// オービス通過告知ONなら
						if(u1s_gpsTgtVcSts == GPSTGT_VC_STS_OUTOK){
							VcReq(VC_ORBIS_PASS);					// 通過告知
							F_PRIPLAYSND = TRUE;
							u2s_sndTgtNum = u2s_sub_phase;
							u4s_sndTgtAddress = psts_chkmap->u4_dataAddr;
						}
					}
				}
				else if((psts_chkmap->u2_dst <= u2_DIST_330M) && (u1t_degChk == OK)){
																	// 330m範囲内 and 角度OK 
					psts_chkmap->u1_wrnSts = STS_ORBIS_300M;		// 300m警報状態へ
					if((psts_chkmap->un_type.bit.b_tunnel == OFF)	// トンネル以外
					&& (stg_setGps[u1g_setAcsSel].b_orbisPassSpd)	// オービス通過速度告知ONなら
					&& (psts_chkmap->u2_dst >= u2_DIST_100M)){
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_PASSSPD, psts_chkmap->u4_dataAddr);
																	// 通過速度告知
					}
				}
				break;
		//-------300m警報状態----------------------------------------------------//
			case STS_ORBIS_300M:									// 300m警報状態
				if(psts_chkmap->u2_dst <= u2t_passDst){				// 通過判定距離内？
					psts_chkmap->u1_wrnSts = STS_ORBIS_50M;			// 50m警報状態へ
					if((u1_ChkDegRng(TYPE_TGTDEG_EXIST_DEGA_WIDE, psts_chkmap->u2_dst)== OK)
																	// 通過角度OK
					&& (psts_chkmap->un_type.bit.b_tunnel == OFF)	// トンネル以外
					&& (stg_setGps[u1g_setAcsSel].b_orbisPass)){	// オービス通過告知ONなら
						if(u1s_gpsTgtVcSts == GPSTGT_VC_STS_OUTOK){
							VcReq(VC_ORBIS_PASS);				// 通過告知
							F_PRIPLAYSND = TRUE;
							u2s_sndTgtNum = u2s_sub_phase;
							u4s_sndTgtAddress = psts_chkmap->u4_dataAddr;
						}
					}
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
			//	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_2100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
			}
			else{													// 一般オービス
				ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
			//	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, ((u1_GetActVc(VC_CH0) == VC_ORBIS_PASS) && (u4s_sndTgtAddress == psts_chkmap->u4_dataAddr)));
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
//【関数】反対車線オービスキャンセルエリア判定処理							
//【機能】反対車線オービスキャンセルエリア成立・否定を判定する				
//【引数】psth_map：判定するマップポイントへのポインタ						
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	ChkRevOrbisArea(ST_GPSMAP *psth_map){

	S2	s2t_degDiff;										// 角度差

	// ターゲット角度と自車方位角との差を算出
	s2t_degDiff = s2_CalDegSub(psth_map->u2_tgtDeg, sts_gpsinf.u2_deg);
	if(s2t_degDiff < 0){
		s2t_degDiff *= (S2)-1;								// 絶対値にする
	}
	if(s2t_degDiff <= 90*DEG_LSB){							// 90°以内(対向していない側)
		if((psth_map->u2_dst <= u2_DIST_600M)				// ターゲット中心から600m以内 or
		|| ((psth_map->u2_dst <= u2_DIST_1100M) && (u1_DegAIsOk(psth_map) == TRUE))){
															// ターゲット方位の1100m扇範囲内にいる ?
			F_RORBIS_CAN = ON;								// キャンセルフラグセット
		}
	}
	else{													// 90°より大(対向している側)
		if((psth_map->u2_dst <= u2_DIST_1100M) && (u1_DegAIsOk(psth_map) == TRUE)){
															// ターゲット中心から1100m以内 ?
			F_RORBIS_NOCAN = ON;							// キャンセル否定フラグセット
		}
	}
}
#if 0
//--------------------------------------------------------------------------//
//【関数】トンネル内残距離算出処理											//
//【機能】トンネル内属性データの残距離を算出する							//
//【引数】なし																//
//【戻値】なし																//
//【備考】毎秒のGPS位置データが更新されるごとに、入力(u2_dst)は仮想点からの	//
//        距離が入っている。												//
//        可視ターゲットの距離ソート終了後しか行ってはいけない				//
//--------------------------------------------------------------------------//
static void	CalTunnelInRemDist(void){

	int	i;
	ST_GPSMAP	*pstt_map;

	// 可視ターゲットの中から対象があるか検索し、あったら計算する。
	if(srcRefIdxListSize != 0){
		for(i=0 ; i<srcRefIdxListSize ; i++){
			pstt_map = &sts_gpsmap[s_SrcRefIdxList[i]];
			if(pstt_map->un_type.bit.b_tunnel == TUNNEL_IN){
													// トンネル内属性
				pstt_map->u2_dst = GRDADDU2(pstt_map->u2_dst, ((U2)pstt_map->un_extra.common.u1_tunnelRem*(U2)100));
			}
		}
	}
}
#endif
//--------------------------------------------------------------------------
//【関数】ゾーン警報遷移処理												
//【機能】ゾーン警報を判定する												
//【引数】方位有無															
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitZone(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U1	u1t_thisTargetOut = OFF;
	Bool	isTrapZone = TRUE;

	if(psts_chkmap->un_type.bit.b_tunnel != NOT_TUNNEL){// トンネル内判定
		return;											// 見えない・警報しない
	}

	switch(psts_chkmap->un_type.bit.b_code){
	case TGT_CHKPNT_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
		isTrapZone = FALSE;
		break;
	default:
		break;
	}

	if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){
		switch(psts_chkmap->u1_wrnSts){
	//-------非警報状態--------------------------------------------------------//
		case STS_ZONE_NOALARM:
			if((u1t_degChk == OK)								// 角度OK？
			&& (psts_chkmap->u2_dst <= u2_DIST_100M)){			// 100m範囲内?
				psts_chkmap->u1_wrnSts = STS_ZONE_100M;			// 100m警報状態へ
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_100M, psts_chkmap->u4_dataAddr);
			}
			break;
		case STS_ZONE_100M:
			if(psts_chkmap->u2_dst > u2_DIST_600M){				// 距離離反?
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
			}
			break;
		}

		// ゾーンでのフォーカスターゲット判定
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
	}
	else{
		// 警報状態遷移
		switch(psts_chkmap->u1_wrnSts){
	//-------非警報状態--------------------------------------------------------//
		case STS_ZONE_NOALARM:
			// 扇範囲内でターゲットに向かうと警報開始
			if(u1t_degChk == OK){								// 角度OK？
				if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m範囲内?
					if(psts_chkmap->u2_dst <= u2_DIST_500M){	// 500m範囲内なら
						psts_chkmap->u1_wrnSts = STS_ZONE_500M;	// 500m警報状態へ
					}
					else{
						psts_chkmap->u1_wrnSts = STS_ZONE_600M;	// 600m警報状態へ
					}
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
				}
				else{
					if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m範囲内?
						psts_chkmap->u1_wrnSts = STS_ZONE_1100M;// 1100m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_1KM, psts_chkmap->u4_dataAddr);
					}
				}
			}
			break;
	//-------1100m警報状態-----------------------------------------------------//
		case STS_ZONE_1100M:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外？
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
			}
			else if((u1t_degChk == OK)							// 角度OK and
				 && (psts_chkmap->u2_dst <= u2_DIST_600M)){		// 600m範囲内なら
				if(psts_chkmap->u2_dst <= u2_DIST_500M){		// 500m範囲内なら
					psts_chkmap->u1_wrnSts = STS_ZONE_500M;		// 500m警報状態へ
				}
				else{
					psts_chkmap->u1_wrnSts = STS_ZONE_600M;		// 600m警報状態へ
				}
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
			}
			break;
	//-------600m警報状態------------------------------------------------------//
		case STS_ZONE_600M:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外？
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
			}
			// 扇内の500m圏内に入ったか確認する
			else if((u1t_degChk == OK)							// 角度OK and
				 && (psts_chkmap->u2_dst <= u2_DIST_500M)){		// 500m範囲内なら
				psts_chkmap->u1_wrnSts = STS_ZONE_500M;			// 500m警報状態へ
			}
			break;
	//-------500m警報状態------------------------------------------------------//
		case STS_ZONE_500M:
			if(psts_chkmap->u2_dst > u2_DIST_500M){				// 500m範囲外？
				psts_chkmap->u1_wrnSts = STS_ZONE_OUTWT_FAR;	// 完全離脱待ち状態へ
				if(isTrapZone){
					F_TRAPOUT_REQ = ON;
					psts_trapOutMap = psts_chkmap;				// ポインタ保存
				}
				else{
					F_CHKOUT_REQ = ON;
					psts_chkOutMap = psts_chkmap;				// ポインタ保存
				}
				u1t_thisTargetOut = ON;
			}
			break;
	//-------エリア内完全離脱待ち状態-----------------------------------------//
		case STS_ZONE_OUTWT_FAR:
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外
				psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
			}
			break;
		}

		// ゾーンでのフォーカスターゲット判定
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
	}

	// 	インデックスリスト追加
	AddVisibleIdxLst(u2s_sub_phase);

	// トラップゾーン内での特別処理
	if(isTrapZone){
		// 一時停止：キャンセルNG範囲にしない、RD感度上昇不要、圏外不要
		// 交差点：キャンセルNG範囲にしない、RD感度上昇不要

		if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_TEMPORARY_STOP){
																// 一時停止以外のとき
			if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_SIGNAL){
																// さらに交差点以外のとき
				// キャンセルNG判定
			 	if(psts_chkmap->u2_dst <= u2_DIST_600M){
					F_CANCEL_NG = ON;							// キャンセルNG成立
				}
				// RD感度上昇用判定
				if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){	// 半径500m圏内 ?
					F_TRAPZONE_IN = ON;
				}
			}
			// このゾーンに入ってきているかどうかの判定
			if((u1t_thisTargetOut == OFF)						// このターゲットが今圏外要求していない
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)		// 
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)	// 
			&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// and 1100m圏内
				F_TRAP_INCOMING = ON;
			}
		}
	}
	else{														// 検問エリア
		if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){			// 半径500m圏内 ?
			F_CHKPNT_IN = ON;
		}

		if((u1t_thisTargetOut == OFF)							// このターゲットが今圏外要求していない
		&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)			// 
		&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)		// 
		&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){				// and 1100m圏内
			F_CHKPNT_INCOMING = ON;
		}
	}

	// エリア状態遷移
	TransitAreaStatusForRd(psts_chkmap);
	TransitAreaStatusForSc(psts_chkmap, u1t_degChk);

	ChkZoneGpsWrnLvl(u1t_degChk);								// ゾーンGPS警報レベル判定
}

//--------------------------------------------------------------------------
//【関数】ゾーンGPS警報レベル判定処理										
//【機能】ゾーンの警報レベルを判定する										
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	ChkZoneGpsWrnLvl(U1 u1h_degChk){

	// GPS警報レベルの判定
	if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){

		if(u1h_degChk == OK){								// 中心に向かっている
			if(psts_chkmap->u2_dst <= u2_DIST_100M)			// 100m以内
			{
				update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
			}
			else if(psts_chkmap->u2_dst <= u2_DIST_300M)	// 300m以内
			{
				update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
			}
		}
	}
	else{
		// YELLOW警報レベルの判定
		if((psts_chkmap->u2_dst <= u2_DIST_600M)			// 600m以内で
		&& (u1h_degChk == OK)){								// 中心に向かっている
			update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
		}
		// MIDの判定
		else if((psts_chkmap->u1_wrnSts == STS_ZONE_500M)	// 圏外待ちか
		|| ((psts_chkmap->u2_dst <= u2_DIST_1100M) && (u1h_degChk == OK))){
															// 1100m以内で中心に向かっている
			update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
		}
	}

}

//--------------------------------------------------------------------------
//【関数】マイエリア警報遷移処理											
//【機能】マイエリア告知を判定する											
//【引数】なし																
//【戻値】なし																
//【備考】1500m離反で再警報可能												
//--------------------------------------------------------------------------
static void TransitMyArea(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//【関数】ユーザーピン状態遷移処理											
//【機能】フォーカスを判定する												
//【引数】なし																
//【戻値】なし																
//--------------------------------------------------------------------------
static void TransitPin(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtDeg, psts_chkmap->u2_dst);

	// ピンのフォーカスターゲット判定
	ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
	ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);

	// インデックスリスト追加
	AddVisibleIdxLst(u2s_sub_phase);
}

//--------------------------------------------------------------------------
//【関数】マイキャンセル処理												
//【機能】手動登録のIキャンセル警報状態遷移を行い、圏内判定する				
//【引数】なし																
//【戻値】なし																
//【備考】F_MYCANCEL_INは警報判定フェーズ開始時に初期化済み					
//--------------------------------------------------------------------------
static void TransitMyCancel(EM_TGTDEG emh_tgtDeg){
}
//--------------------------------------------------------------------------
//【関数】オートキャンセル遷移処理											
//【機能】自動登録のIキャンセル警報状態遷移を行い、圏内および削除判定する	
//【引数】なし																
//【戻値】なし																
//【備考】F_ATCANCEL_INは警報判定フェーズ開始時に初期化済み					
//		  レギュラーはAAC禁止でも受信する？									
//		  ソーラーはAAC禁止なら受信しないので受信なし通過判定しない			
//--------------------------------------------------------------------------
static void TransitAtCancel(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//【関数】1kmタイプコンテンツ処理											
//【機能】1kmコンテンツ告知状態遷移を行う									
//【引数】emh_tgtdeg : ターゲット方位有無									
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitCont1kmType(EM_TGTDEG emh_tgtdeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtdeg, psts_chkmap->u2_dst);

	// 状態遷移

	if(psts_chkmap->u2_dst > u2_DIST_1500M){					// 1500m範囲外 ?
		psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;			// 非警報状態へ
	}
	else{
		switch(psts_chkmap->u1_wrnSts){
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_NOALARM:								// 非警報状態
			if(u1t_degChk == OK){								// 角度OK
				if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m範囲内 ?
					psts_chkmap->u1_wrnSts = STS_1KMCONT_600M;	// 600m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_500M, psts_chkmap->u4_dataAddr);
				}
				else{
					if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m範囲内?
						psts_chkmap->u1_wrnSts = STS_1KMCONT_1100M;
																// 1100m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_1KM, psts_chkmap->u4_dataAddr);
					}
				}
			}
			break;
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_1100M:									// 1100m警報状態
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外 ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;	// 非警報状態へ
			}
			else if((u1t_degChk == OK) && (psts_chkmap->u2_dst <= u2_DIST_600M)){
																// 600m範囲内 and 角度OK ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_600M;		// 600m警報状態へ
				GpsVcEnQueue(VC_GPSVAR, VCGPS_1KMCONT_500M, psts_chkmap->u4_dataAddr);
			}
			break;
	//-----------------------------------------------------------------------------
		case STS_1KMCONT_600M:									// 600m警報状態
			if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外 ?
				psts_chkmap->u1_wrnSts = STS_1KMCONT_NOALARM;	// 非警報状態へ
			}
			break;
		}

		// 1kmコンテンツの最接近対向ターゲット判定
		if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		}
	}

	// インデックスリスト追加
	if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
		AddVisibleIdxLst(u2s_sub_phase);
	}
}

//--------------------------------------------------------------------------
//【関数】ワンショットタイプコンテンツ処理									
//【機能】ワンショットコンテンツ告知状態遷移を行う							
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitContOneShotType(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(emh_tgtDeg, psts_chkmap->u2_dst);
	U2	u2t_inDst;
	U2	u2t_outDst;
	EM_VCGPS_TYPE	u1t_vcGps = VCGPS_NONE;
	U1	u1t_reqFocus = TRUE;					// フォーカス要で初期化
	U1  u1t_wrnFocus = FALSE;
	U2	u2t_focusNearDst = 0;					// 近接フォーカス判定なしで初期化
	U1	u1t_visible = TRUE;						// 可視ターゲットかどうか

	switch(psts_chkmap->un_type.bit.b_code){
	// 1100mグループ
	case TGT_SA:
	case TGT_PA:
	case TGT_HWOASYS:
		u2t_inDst = u2_DIST_1100M;
		u2t_outDst = u2_DIST_1600M;
		u1t_vcGps = VCGPS_1KMCONT_1KM;
		u2t_focusNearDst = u2_DIST_1100M;		// 1100mより近接
		break;
	// 600mグループ
	case TGT_CURVE:								// 急カーブ
		u2t_inDst = u2_DIST_600M;
		u2t_outDst = u2_DIST_1100M;
		u1t_vcGps = VCGPS_500MCONT;
		u1t_reqFocus = FALSE;					// フォーカス不要
		u1t_visible = FALSE;					// 見えない
		break;
	case TGT_POLICE:							// 警察署
		if(psts_chkmap->un_extra.police.b_policeType != POLICE_TYPE_STATION)
		{
			u1t_wrnFocus = TRUE;
		}
		// fall
	case TGT_KOBAN:								// 交番
		u2t_inDst = u2_DIST_600M;
		u2t_outDst = u2_DIST_1100M;
		u1t_vcGps = VCGPS_500MCONT;
		u2t_focusNearDst = u2_DIST_600M;		// 600mより近接
		break;
	// 300mグループ
	case TGT_CROSSING:							// 交差点監視ポイント
	case TGT_SIGNAL:							// 信号無視抑止システム
		u1t_wrnFocus = TRUE;
		// fall 
	case TGT_NSYS:								// Nシステム
	case TGT_TRFCHK:							// 交通監視システム
	case TGT_ACCIDENT:							// 事故多発エリア
		u2t_inDst = u2_DIST_300M;
		u2t_outDst = u2_DIST_800M;
		u1t_vcGps = VCGPS_300MCONT;
		u2t_focusNearDst = u2_DIST_300M;		// 300mより近接
		break;
	// 200mグループ
	case TGT_BRAJCT:							// 分岐・合流
		u2t_inDst = u2_DIST_200M;
		u2t_outDst = u2_DIST_700M;
		u1t_reqFocus = FALSE;					// フォーカス不要
		u1t_visible = FALSE;					// 見えない
		u1t_vcGps = VCGPS_300MCONT;
		break;
	// 100mグループ
	case TGT_KENKYO:							// 県境
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// フォーカス不要
		u1t_visible = FALSE;					// 見えない
		u1t_vcGps = VCGPS_100MCONT;
		break;

	case TGT_PARKING:							// 駐車場
	case TGT_RAILROAD_CROSSING:					// 踏切
	case TGT_TOILET:							// 公衆トイレ
	case TGT_FIRE:								// 消防署
	case TGT_HOIKU:								// 保育園
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// フォーカス不要
		u1t_vcGps = VCGPS_NONE;					// 音声不要
		break;

	case TGT_TMPSTOP:							// 一時停止注意ポイント
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_reqFocus = FALSE;					// フォーカス不要
		u1t_vcGps = VCGPS_NONE;					// 音声不要
		// 角度判定結果を可視判定結果にする
		u1t_visible = u1t_degChk;
		break;

	case TGT_HWRADIO:							// ハイウェイラジオ
		u2t_inDst = u2_DIST_100M;
		u2t_outDst = u2_DIST_600M;
		u1t_vcGps = VCGPS_100MCONT;
		u2t_focusNearDst = u2_DIST_100M;		// 100mより近接
		break;
	}

	// 状態遷移
	if(psts_chkmap->u1_wrnSts == STS_ONESHOTCONT_NOALARM){
		if((psts_chkmap->u2_dst <= u2t_inDst)				// 指定範囲内
		&& (u1t_degChk == OK)){								// ターゲット対向
			psts_chkmap->u1_wrnSts = STS_ONESHOTCONT_ALARM;	// 警報状態へ
			if(u1t_vcGps != VCGPS_NONE){					// 音声出力要
				switch(psts_chkmap->un_type.bit.b_code){
				case TGT_KENKYO:
					sts_gpsmap[KENKYO_VIRTUAL_INDEX] = *psts_chkmap;
															// 一度全部コピー
					sts_gpsmap[KENKYO_VIRTUAL_INDEX].u4_dataAddr = KENKYO_VIRTUAL_ADDRESS;
															// アドレスを仮想ターゲットにする
					sts_gpsmap[KENKYO_VIRTUAL_INDEX].un_extra.common.u2_mapPctNum = 0;
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, KENKYO_VIRTUAL_ADDRESS);
					break;
				case TGT_CURVE:
					sts_gpsmap[CURVE_VIRTUAL_INDEX] = *psts_chkmap;
															// 一度全部コピー
					sts_gpsmap[CURVE_VIRTUAL_INDEX].u4_dataAddr = CV_VIRTUAL_ADDRESS;
															// アドレスを仮想ターゲットにする
					sts_gpsmap[CURVE_VIRTUAL_INDEX].un_extra.common.u2_mapPctNum = 0;
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, CV_VIRTUAL_ADDRESS);
					break;
				default:
					GpsVcEnQueue(VC_GPSVAR, u1t_vcGps, psts_chkmap->u4_dataAddr);
					break;
				}
			}
		}
	}
	else{
		if(psts_chkmap->u2_dst > u2t_outDst){				// 範囲外に出た？
			psts_chkmap->u1_wrnSts = STS_ONESHOTCONT_NOALARM;
															// 非警報状態へ
		}
	}
	if(psts_chkmap->un_type.bit.b_tunnel == NOT_TUNNEL){
		// コンテンツのフォーカスターゲット判定
		if(u1t_reqFocus){
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2t_focusNearDst, u2_DIST_1100M, u1t_degChk, FALSE);
			if(u1t_wrnFocus)
			{
				ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2t_focusNearDst, u2_DIST_1100M, u1t_degChk, FALSE);
				// 警報レベル判定
				if(u1t_degChk)
				{
					U2	near_dist;
					U2	far_dist;
					if(psts_chkmap->un_type.bit.b_code == TGT_POLICE)
					{
						// 高速警察
						near_dist = 600;
						far_dist = 1100;
					}
					else
					{
						near_dist = 300;
						far_dist = 600;
					}
					if(psts_chkmap->u2_dst <= near_dist)
					{
						update_gps_warning_lvl(SYS_YELLOW_WARNING_HI, psts_chkmap->u2_dst);
					}
					else if(psts_chkmap->u2_dst <= far_dist)
					{
						update_gps_warning_lvl(SYS_YELLOW_WARNING_MID, psts_chkmap->u2_dst);
					}
				}
			}
		}
		// 可視ターゲットのインデックスリスト追加
		if(u1t_visible){
			AddVisibleIdxLst(u2s_sub_phase);
		}
	}
}
//--------------------------------------------------------------------------
//【関数】速度切替コンテンツ処理											
//【機能】速度切替コンテンツの通過を判別する								
//【引数】なし																
//【戻値】なし																
//【備考】不可視ターゲット(発音中の可視ターゲットは仮想ターゲット)			
//--------------------------------------------------------------------------

static void	TransitContScp(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//【関数】エリアタイプコンテンツ処理										
//【機能】エリアタイプコンテンツ告知状態遷移を行う							
//【引数】なし																
//【戻値】なし																
//【備考】不可視ターゲット(発音中の可視ターゲットは仮想ターゲット)			
//--------------------------------------------------------------------------
static void	TransitContAreaType(EM_TGTDEG emh_tgtDeg){

	U2	u2t_inDst;

	// 状態遷移
	if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_SZ){
															// 最重点エリアのとき
		u2t_inDst = u2_DIST_500M;							// 範囲内距離
	}
	else{
		u2t_inDst = u2_TBL_AREA_ROUND[psts_chkmap->un_extra.no_parking.b_areaRound];
	}
	
	if(psts_chkmap->u1_wrnSts == STS_AREACONT_OUTAREA){		// エリア外状態
		if(psts_chkmap->u2_dst <= u2t_inDst){				// 範囲内侵入?
			psts_chkmap->u1_wrnSts = STS_AREACONT_INAREA;	// エリア内状態へ
		}
	}
	else{													// エリア内状態
		if(psts_chkmap->u2_dst > (u2t_inDst + u2_DIST_100M)){
															// 範囲外離脱？
			psts_chkmap->u1_wrnSts = STS_AREACONT_OUTAREA;	// 非警報状態へ
		}
	}

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_INAREA){		// エリア内状態
		if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_SZ){
			F_PCHK_SZ_IN = ON;								// 最重点ありで上書き
		}
		else if(psts_chkmap->un_type.bit.b_code == TGT_PARKCHK_AREA_Z){
			F_PCHK_Z_IN = ON;								// 重点ありで上書き
		}
		else{												// 車上狙い
			F_SHAJYO_IN = ON;
		}
	}

}
//--------------------------------------------------------------------------//
//【関数】ETCゲートコンテンツ処理											//
//【機能】ETCゲートSTART・STOPの通過判定を行う								//
//【引数】方位有無															//
//【戻値】なし																//
//【備考】																	//
//--------------------------------------------------------------------------//
static void	TransitETCGateType(EM_TGTDEG emh_tgtDeg){
}

//--------------------------------------------------------------------------
//【関数】ゾーン30タイプコンテンツ処理										
//【機能】ゾーン30タイプコンテンツ告知状態遷移を行う						
//【引数】なし																
//【戻値】なし																
//【備考】不可視ターゲット(発音中の可視ターゲットは仮想ターゲット)			
//--------------------------------------------------------------------------
static void TransitZone30Type(EM_TGTDEG emh_tgtDeg){

	U2 u2t_inDst;
	
	// 状態遷移
	u2t_inDst = psts_chkmap->un_extra.zone30.u2_radius;

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_OUTAREA){		// エリア外状態
		if(psts_chkmap->u2_dst <= u2t_inDst){				// 範囲内進入？
			psts_chkmap->u1_wrnSts = STS_AREACONT_INAREA;	// エリア内状態へ
		}
	}
	else{													// エリア内状態
		if(psts_chkmap->u2_dst > (u2t_inDst + u2_DIST_20M)){
															// 範囲外離脱？
			psts_chkmap->u1_wrnSts = STS_AREACONT_OUTAREA;	// 非警報状態へ
		}
	}

	if(psts_chkmap->u1_wrnSts == STS_AREACONT_INAREA){
		F_ZONE30_IN = ON;									// ゾーン30
	}
}

//--------------------------------------------------------------------------
//【関数】トンネルオービス警報遷移処理										
//【機能】トンネルオービス警報を判定する									
//【引数】方位有無															
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void TransitTunnelOrbis(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U2	u2t_rngOutDst = u2_DIST_1500M;						// 1500mで初期化
	U2	u2t_hokanJudgeDist = u2_DIST_1100M;

	if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){
		u2t_hokanJudgeDist = u2_DIST_2100M;
		u2t_rngOutDst = u2_DIST_2500M;
	}

	// SEL_POI_NORMALのときは不可視＆非警報だが、誘導判定はする

	if(u1s_poiSetSel == SEL_POI_TUNNEL){		// トンネル誘導状態なら警報

		if(psts_chkmap->u2_dst > u2t_rngOutDst){				// 圏外距離より大なら
			psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_NOALARM;	// 非警報状態へ
			psts_chkmap->u2_countdown_dist = U2_MAX;			// カウントダウンクリア
		}
		else{
			// 警報状態遷移
			switch(psts_chkmap->u1_wrnSts){
		//-------非警報状態-------------------------------------------------------//
			case STS_TUNNEL_ORBIS_NOALARM:							// 非警報状態
				// 扇範囲内でターゲットに向かうと警報開始
				if(u1t_degChk != OK){								// 角度NG？
					break;
				}
				if(psts_chkmap->u2_dst <= u2_DIST_600M){			// 600m範囲内?
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; // 600m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_1100M;// 1100m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
				}
				else if(psts_chkmap->u2_dst <= u2_DIST_2100M){		// 2100M以内
					if(psts_chkmap->un_type.bit.b_road == ROAD_HIGH){
																	// 高速属性
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_2100M;
																	// 2100m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_2KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------2100m警報状態----------------------------------------------------//
			case STS_TUNNEL_ORBIS_2100M:
				if(u1t_degChk == OK){										// 角度ＯＫ
					if(psts_chkmap->u2_dst <= u2_DIST_600M){				// 600m範囲内?
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; 	// 600m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
					}
					else if(psts_chkmap->u2_dst <= u2_DIST_1100M){			// 1100m範囲内?
						psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_1100M; 	// 1100m警報状態へ
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_1KM, psts_chkmap->u4_dataAddr);
					}
				}
				break;
		//-------1100m警報状態----------------------------------------------------//
			case STS_TUNNEL_ORBIS_1100M:
				if((u1t_degChk == OK)
				&& (psts_chkmap->u2_dst <= u2_DIST_600M)){			// 角度OK and 600m範囲内?
					psts_chkmap->u1_wrnSts = STS_TUNNEL_ORBIS_600M; // 600m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ORBIS_500M, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------600m警報状態------------------------------------------------------//
			case STS_TUNNEL_ORBIS_600M:
				break;
			}
		}

		// フォーカスターゲット判定
		ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2t_hokanJudgeDist, u1t_degChk, FALSE);
		ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2t_hokanJudgeDist, u1t_degChk, FALSE);

		// GPS警報レベルの判定
		if(u1t_degChk == OK){
			// RED HIの判定
			if(psts_chkmap->u2_dst <= u2_DIST_600M){
				update_gps_warning_lvl(SYS_RED_WARNING_HI, psts_chkmap->u2_dst);
			}
			// RED MIDの判定
			else if(psts_chkmap->u2_dst <= u2_DIST_1100M){
				update_gps_warning_lvl(SYS_RED_WARNING_MID, psts_chkmap->u2_dst);
			}
			// RED LOの判定
			else if(psts_chkmap->u2_dst <= u2_DIST_2100M){
				update_gps_warning_lvl(SYS_RED_WARNING_LO, psts_chkmap->u2_dst);
			}
		}
		// インデックスリスト追加
		AddVisibleIdxLst(u2s_sub_phase);
	}
}
//--------------------------------------------------------------------------
//【関数】トンネルゾーン警報遷移処理										
//【機能】トンネル取締／検問警報を判定する									
//【引数】方位有無															
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void TransitTunnelZone(EM_TGTDEG emh_tgtDeg){

	U1	u1t_degChk = u1_ChkDegRng(TYPE_TGTDEG_EXIST, psts_chkmap->u2_dst);
	U1	u1t_thisTargetOut = OFF;
	Bool	isTrapZone = TRUE;

	// SEL_POI_NORMALのときは不可視＆非警報だが、誘導判定はする

	if(u1s_poiSetSel == SEL_POI_TUNNEL){		// トンネル誘導状態なら警報

		switch(psts_chkmap->un_type.bit.b_code){
		case TGT_CHKPNT_ZONE:
		case TGT_NOFIX_CHKPNT_ZONE:
			isTrapZone = FALSE;
			break;
		default:
			break;
		}

		if(psts_chkmap->un_extra.trapchk.u1_method == METHOD_TEMPORARY_STOP){
			switch(psts_chkmap->u1_wrnSts){
		//-------非警報状態--------------------------------------------------------//
			case STS_ZONE_NOALARM:
				if((u1t_degChk == OK)								// 角度OK？
				&& (psts_chkmap->u2_dst <= u2_DIST_100M)){			// 100m範囲内?
					psts_chkmap->u1_wrnSts = STS_ZONE_100M;			// 100m警報状態へ
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_100M, psts_chkmap->u4_dataAddr);
				}
				break;
			case STS_ZONE_100M:
				if(psts_chkmap->u2_dst > u2_DIST_600M){				// 距離離反?
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
				}
				break;
			}

			// ゾーンでのフォーカスターゲット判定
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);
			ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_300M, u2_DIST_1100M, u1t_degChk, FALSE);

		}
		else{
			// 警報状態遷移
			switch(psts_chkmap->u1_wrnSts){
		//-------非警報状態--------------------------------------------------------//
			case STS_ZONE_NOALARM:
				// 扇範囲内でターゲットに向かうと警報開始
				if(u1t_degChk == OK){								// 角度OK？
					if(psts_chkmap->u2_dst <= u2_DIST_600M){		// 600m範囲内?
						if(psts_chkmap->u2_dst <= u2_DIST_500M){	// 500m範囲内なら
							psts_chkmap->u1_wrnSts = STS_ZONE_500M;	// 500m警報状態へ
						}
						else{
							psts_chkmap->u1_wrnSts = STS_ZONE_600M;	// 600m警報状態へ
						}
						GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
					}
					else{
						if(psts_chkmap->u2_dst <= u2_DIST_1100M){	// 1100m範囲内?
							psts_chkmap->u1_wrnSts = STS_ZONE_1100M;// 1100m警報状態へ
							GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_1KM, psts_chkmap->u4_dataAddr);
						}
					}
				}
				break;
		//-------1100m警報状態-----------------------------------------------------//
			case STS_ZONE_1100M:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外？
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
				}
				else if((u1t_degChk == OK)							// 角度OK and
					 && (psts_chkmap->u2_dst <= u2_DIST_600M)){		// 600m範囲内なら
					if(psts_chkmap->u2_dst <= u2_DIST_500M){		// 500m範囲内なら
						psts_chkmap->u1_wrnSts = STS_ZONE_500M;		// 500m警報状態へ
					}
					else{
						psts_chkmap->u1_wrnSts = STS_ZONE_600M;		// 600m警報状態へ
					}
					GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE_500M, psts_chkmap->u4_dataAddr);
				}
				break;
		//-------600m警報状態------------------------------------------------------//
			case STS_ZONE_600M:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外？
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
				}
				// 扇内の500m圏内に入ったか確認する
				else if((u1t_degChk == OK)							// 角度OK and
					 && (psts_chkmap->u2_dst <= u2_DIST_500M)){		// 500m範囲内なら
					psts_chkmap->u1_wrnSts = STS_ZONE_500M;			// 500m警報状態へ
				}
				break;
		//-------500m警報状態------------------------------------------------------//
			case STS_ZONE_500M:
				if(psts_chkmap->u2_dst > u2_DIST_500M){				// 500m範囲外？
					psts_chkmap->u1_wrnSts = STS_ZONE_OUTWT_FAR;	// 完全離脱待ち状態へ
					if(isTrapZone){
						F_TRAPOUT_REQ = ON;
						psts_trapOutMap = psts_chkmap;				// ポインタ保存
					}
					else{
						F_CHKOUT_REQ = ON;
						psts_chkOutMap = psts_chkmap;				// ポインタ保存
					}
					u1t_thisTargetOut = ON;
				}
				break;
		//-------エリア内完全離脱待ち状態-----------------------------------------//
			case STS_ZONE_OUTWT_FAR:
				if(psts_chkmap->u2_dst > u2_DIST_1500M){			// 1500m範囲外
					psts_chkmap->u1_wrnSts = STS_ZONE_NOALARM;		// 非警報状態へ
				}
				break;
			}

			// ゾーンでのフォーカスターゲット判定
			ChkFocusTgt(&sts_focusTgt, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
			ChkFocusTgt(&sts_warning_focusTgt_Primary, psts_chkmap, u2_DIST_1100M, u2_DIST_1100M, u1t_degChk, FALSE);
		}

		// 	インデックスリスト追加
		AddVisibleIdxLst(u2s_sub_phase);

		// トラップゾーン内での特別処理
		if(isTrapZone){
			// 一時停止：キャンセルNG範囲にしない、RD感度上昇不要、圏外不要
			// 交差点：キャンセルNG範囲にしない、RD感度上昇不要

			if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_TEMPORARY_STOP){
																	// 一時停止以外のとき
				if(psts_chkmap->un_extra.trapchk.u1_method != METHOD_SIGNAL){
																	// さらに交差点以外のとき
					// キャンセルNG判定
				 	if(psts_chkmap->u2_dst <= u2_DIST_600M){
						F_CANCEL_NG = ON;							// キャンセルNG成立
					}
					// RD感度上昇用判定
					if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){	// 半径500m圏内 ?
						F_TRAPZONE_IN = ON;
					}
				}
				// このゾーンに入ってきているかどうかの判定
				if((u1t_thisTargetOut == OFF)						// このターゲットが今圏外要求していない
				&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)		// 
				&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)	// 
				&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){			// and 1100m圏内
					F_TRAP_INCOMING = ON;
				}
			}
		}
		else{														// 検問エリア
			if(psts_chkmap->u1_wrnSts == STS_ZONE_500M){			// 半径500m圏内 ?
				F_CHKPNT_IN = ON;
			}

			if((u1t_thisTargetOut == OFF)							// このターゲットが今圏外要求していない
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_NOALARM)			// 
			&& (psts_chkmap->u1_wrnSts != STS_ZONE_OUTWT_FAR)		// 
			&& (psts_chkmap->u2_dst <= u2_DIST_1100M)){				// and 1100m圏内
				F_CHKPNT_INCOMING = ON;
			}
		}

		ChkZoneGpsWrnLvl(u1t_degChk);								// ゾーンGPS警報レベル判定

	}
	// エリア状態遷移
	TransitAreaStatusForRd(psts_chkmap);
	TransitAreaStatusForSc(psts_chkmap, u1t_degChk);

}
//--------------------------------------------------------------------------
//【関数】誘導遷移処理														
//【機能】誘導エリアを判定する												
//【引数】方位有無															
//【戻値】なし																
//【備考】誘導のみ行い、警報しない											
//--------------------------------------------------------------------------
static void TransitYudo(EM_TGTDEG emh_tgtDeg)
{
}
//--------------------------------------------------------------------------
//【関数】センサ誘導遷移処理												
//【機能】誘導エリアを判定する												
//【引数】方位有無															
//【戻値】なし																
//【備考】誘導のみ行い、警報しない											
//--------------------------------------------------------------------------
static void TransitSyudo(EM_TGTDEG emh_tgtDeg)
{
}

//--------------------------------------------------------------------------
//【関数】走行判定定期処理													
//【機能】																	
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void RoadJdgRoutine(void){

	U2		u2t_lmtSpd;

	// 非測位タイマの計測
	if(sts_gpsinf.b_spddeg_ok != OK){				// 速度無効なら
		INCCNTU1(u1s_carNosokuiTmr);			// 連続非測位時間計測
		// 走行タイマの無効化判定
		if(u1s_carNosokuiTmr >= u1_RUN_NOSOKUI_TIM){
												// 一定時間非測位継続
			u2s_carRunTmr = 0;					// 走行タイマ計測やり直し
			if(u1s_roadJdgSts == ROADJDG_STS_CHKHIGH){
				F_ROADJDG_NOFIX = ON;			// 非測位検出を通知
			}
		}
#if 0
		// 停車タイマの無効化判定
		if(u1s_carNosokuiTmr >= u1_STOP_NOSOKUI_TIM){
			u1s_carStopTmr = 0;
		}
#endif
	}
	else{										// 速度有効
		u1s_carNosokuiTmr = 0;					// 非測位タイマをクリア
	}

	if(sts_gpsinf.b_spddeg_ok == OK){				// 速度有効なら
		// 走行タイマの計測
		if((u1s_roadJdgSts == ROADJDG_STS_CHKHIGH)
		&& (u1s_curLmtSpd != LMTSPD_NONE)){
			u2t_lmtSpd = (U2)u1s_curLmtSpd * (U2)10 * SPD_LSB;
			if(u2t_lmtSpd >= (70*SPD_LSB)){			// 制限速度70mk/h以上
				if(sts_gpsinf.u2_spd >= u2t_lmtSpd){
													// 制限速度以上
					u2s_carRunTmr = GRDADDU2(u2s_carRunTmr, 2);
													// カウント重み2倍
				}
				else if(sts_gpsinf.u2_spd >= (u2t_lmtSpd - (30*SPD_LSB))){
													// 制限速度-30km/h以上なら
					INCCNTU2(u2s_carRunTmr);		// カウント重み1倍
				}
				else{								// 以下ならホールド
					;
				}
			}
			else{									// 制限速度60km/h以下
				if(sts_gpsinf.u2_spd >= u2t_lmtSpd){
													// 制限速度以上
					INCCNTU2(u2s_carRunTmr);		// カウント重み1倍
				}
				else{								// 以下ならホールド
					;
				}
			}
		}
		else{
			u2s_carRunTmr = 0;
		}
		
		// 停車タイマの計測
		if((u1s_roadJdgSts == ROADJDG_STS_CHKNORM)
		&& (u1s_curLmtSpd != LMTSPD_NONE)){
			if(sts_gpsinf.u2_spd > u2_HIGH2NORM_SPD){
				u1s_carStopTmr = 0;					// 計測やり直し
			}
			else{
				u2t_lmtSpd = (U2)u1s_curLmtSpd * (U2)10 * SPD_LSB;
				if(u2t_lmtSpd >= (70*SPD_LSB)){		// 制限速度70mk/h以上
					u1s_carStopTmr = GRDADDU1(u1s_carStopTmr, 2);
													// カウント重み2倍
				}
				else{
					INCCNTU1(u1s_carStopTmr);		// カウント重み1倍
				}
			}
		}
		else{
			u1s_carStopTmr = 0;
		}
	}

	// SCP通過タイマの計測
	if((u1s_roadJdgSts == ROADJDG_STS_HIGH) || (u1s_roadJdgSts == ROADJDG_STS_CHKHIGH)){
		if(sts_gpsinf.b_spddeg_ok == OK){
			INCCNTU2(u2s_scpPassTmr);
		}
	}
	else{
		u2s_scpPassTmr = 0;
	}


}

//--------------------------------------------------------------------------
//【関数】道路判定処理														
//【機能】道路判定状態遷移を行う											
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	RoadJudge(void){

	U1	u1t_oldSts = u1s_roadJdgSts;
	U1	u1t_lmtSpd = (U1)stg_setGps[u1g_setAcsSel].b_hwChgLmtSpd;
	U1	u1t_roadSel = stg_setMisc.b_roadSel;
	U1	u1t_outReq = OFF;								// 出力要求なしで初期化

	if(u1t_lmtSpd == OFF){								// 制限速度告知設定OFF
		u1s_memLmtSpd = LMTSPD_NONE;					// 制限速度記憶なしに初期化
	}

	if(sts_gpsinf.b_spddeg_ok != OK){
		return;
	}

	if(F_SCP_ANYPASS){
		u2s_scpPassTmr = 0;								// SCP通過タイマクリア
	}

	if(u1t_roadSel != SET_ROAD_AUTO){					// 道路選択オート以外
		if((F_SCP_H_PASS)								// 高速確定
		|| (F_SCP_HCHK_NEAR_PASS)						// or 高速未確定50m
		|| ((F_SCP_HCHK_PASS) && (sts_gpsinf.u2_spd >= ((U2)sts_scpPoint.un_extra.scp.b_lmtSpd * (U2)10 * SPD_LSB)))){
														// or (高速未確定100m and 速度以上)
			u1t_outReq = ON;							// 出力要求ありにする
		}
		u1_TransitRoadJudge();
	}
	else{
		u1t_outReq = u1_TransitRoadJudge();
	}

	// 音声出力要求
	if((u1t_outReq) && (u1t_lmtSpd)						// 要求有 and 設定ON
	&& (sts_scpPoint.un_extra.scp.b_lmtSpd != LMTSPD_NONE)
														// 制限速度なしでない(一応ガード)
	&& (u1s_memLmtSpd != sts_scpPoint.un_extra.scp.b_lmtSpd)){
														// and 速度変更
		sts_gpsmap[SCP_VIRTUAL_INDEX] = sts_scpPoint;
		sts_gpsmap[SCP_VIRTUAL_INDEX].u4_dataAddr = SCP_VIRTUAL_ADDRESS;
		GpsVcEnQueue(VC_GPSVAR, VCGPS_SCPCONT, SCP_VIRTUAL_ADDRESS);
		u1s_memLmtSpd = (U1)(sts_scpPoint.un_extra.scp.b_lmtSpd);
	}

	// マップ更新要求
	if(u1t_oldSts != u1s_roadJdgSts){					// 道路判別状態変化なら
//		vset_flg1(EVTFLG_REQ_MAPUPD);					// マップ更新要求
	}

	F_ROADJDG_NOFIX = OFF;

}

static U1	u1_TransitRoadJudge(void){

	U1	u1t_outReq = OFF;

		switch(u1s_roadJdgSts){
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_NORM:							// 一般道
		if(sts_gpsinf.u2_spd > u2_HIGH2NORM_SPD){	// 規定速度より大
			if(F_SCP_H_PASS){						// 高速確定なら
				u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// 現在の制限速度値更新
				u1s_roadJdgSts = ROADJDG_STS_HIGH;	// 高速道へ
				u1t_outReq = ON;					// 出力要求
			}
			else if((F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
													// 確定ではないが高速通過なら
				u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// 現在の制限速度値更新
				u1s_roadJdgSts = ROADJDG_STS_CHKHIGH;
													// 高速判別中へ
			}
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_CHKHIGH:						// 高速判別中
		if(F_SCP_N_PASS){							// 一般確定なら
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// 一般道へ
			u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_NORMVALID_SPD){
													// 極低速検出
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// 一般道へ
//				u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_HIGH2NORM_SPD){
													// 低速検出
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// 一般道判定中へ
		}
		else if(F_ROADJDG_NOFIX){
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// 一般道判定中へ
		}
		else if(u2s_scpPassTmr >= u2_SCP_NOPASS_TIM){
													// 非通過時間が一定時間経過
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// 一般道判定中へ
		}
		else if(F_SCP_H_PASS){						// 高速確定
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// 高速道へ
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// 現在の制限速度値更新
			u1t_outReq = ON;						// 出力要求
		}
		else if((u1s_curLmtSpd >= LMTSPD_70KMH) && (F_SCP_HCHK_PASS)){
													// 制限70km/h以上 and 確定ではないが高速通過(2回目)
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// 高速道へ
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// 現在の制限速度値更新
			u1t_outReq = ON;						// 出力要求
		}
		else if(u2s_carRunTmr >= u2_RUN_CONT_TIM){	// 連続高速走行 ?
			u1s_roadJdgSts = ROADJDG_STS_HIGH;		// 高速道へ
			u1t_outReq = ON;						// 出力要求
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_HIGH:							// 高速道
		if(F_SCP_N_PASS){							// 一般確定なら
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// 一般道へ
			u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_NORMVALID_SPD){
													// 極低速検出
			u1s_roadJdgSts = ROADJDG_STS_NORM;		// 一般道へ
//				u1s_memLmtSpd = LMTSPD_NONE;
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(sts_gpsinf.u2_spd <= u2_HIGH2NORM_SPD){
													// 低速検出
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// 一般道判定中へ
		}
		else if((F_SCP_H_PASS) || (F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
			u1t_outReq = ON;
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
													// 現在の制限速度値更新
		}
		else if(u2s_scpPassTmr >= u2_SCP_NOPASS_TIM2){
													// 非通過時間が一定時間経過
			u1s_roadJdgSts = ROADJDG_STS_CHKNORM;	// 一般道判定中へ
		}
		break;
//------------------------------------------------------------------------------------------------
	case ROADJDG_STS_CHKNORM:							// 一般道判定中
		if((F_SCP_N_PASS)								// 一般確定 or
		|| (u1s_carStopTmr >= u1_STOP_CONT_TIM)			// 低速継続 or
		|| (u2s_scpPassTmr >= u2_SCP_NOPASS_TIM2)		// いずれのSCPも通過せずに15分経過
		|| (sts_gpsinf.u2_spd <= u2_NORMVALID_SPD)){	// 極低速検出
			u1s_roadJdgSts = ROADJDG_STS_NORM;			// 一般道へ
			if(F_SCP_N_PASS){
				u1s_memLmtSpd = LMTSPD_NONE;
			}
			u1s_curLmtSpd = LMTSPD_NONE;
		}
		else if(F_SCP_H_PASS){							// 高速確定
			u1s_roadJdgSts = ROADJDG_STS_HIGH;			// 高速道へ
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
														// 現在の制限速度値更新
			u1t_outReq = ON;							// 出力要求
		}
		else if((F_SCP_HCHK_PASS) || (F_SCP_HCHK_NEAR_PASS)){
														// 未確定SCP INポイント通過
			u1s_curLmtSpd = sts_scpPoint.un_extra.scp.b_lmtSpd;
														// 現在の制限速度値更新
			u1s_roadJdgSts = ROADJDG_STS_CHKHIGH;		// 高速判別中へ
		}
		break;
	}

	return u1t_outReq;

}

EM_ROADJDG_STS	emg_GetCurrentRoad(void){

	return	(EM_ROADJDG_STS)u1s_roadJdgSts;
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
					GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_Z_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_PCHK_SZ_AREA, SZZ_VIRTUAL_ADDRESS);
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE30_AREA, ZN30_VIRTUAL_ADDRESS);
				
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
				GpsVcEnQueue(VC_GPSVAR, VCGPS_ZONE30_AREA, ZN30_VIRTUAL_ADDRESS);
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



//--------------------------------------------------------------------------
//【関数】車上狙いエリア状態遷移処理										
//【機能】車上狙いエリア状態遷移を行う										
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitShajyoSts(void){
}

//--------------------------------------------------------------------------
//【関数】ETCゲート告知状態遷移処理											
//【機能】ETCゲート告知状態遷移を行う										
//【引数】なし																
//【戻値】なし																
//【備考】																	
//--------------------------------------------------------------------------
static void	TransitETCGuide(void){
}
//--------------------------------------------------------------------------
//【関数】∠A範囲判定処理													
//【機能】自車がターゲット方位の±40°以内に存在するかどうか判定する		
//【引数】なし																
//【戻値】U1 TRUE：範囲内	FALSE：範囲外									
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_DegAIsOk(ST_GPSMAP *psth_map){

	U1	u1t_ret = FALSE;							// 範囲外で初期化
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ∠Aがターゲットの±40°以内にあれば範囲内
	if((s2t_deg >= s2_DEG_TGTRNG_MIN)
	&& (s2t_deg <= s2_DEG_TGTRNG_MAX)){
		u1t_ret = TRUE;								// 範囲内
	}
	
	return	u1t_ret;
}
//--------------------------------------------------------------------------
//【関数】∠A範囲判定処理(可変)												
//【機能】自車がターゲット方位の±X°以内に存在するかどうか判定する			
//【引数】																	
//【戻値】U1 TRUE：範囲内	FALSE：範囲外									
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_DegAIsOk_Variable(ST_GPSMAP *psth_map, U1 u1h_deg){

	U1	u1t_ret = FALSE;							// 範囲外で初期化
	S2	s2t_deg;
	
	s2t_deg = s2_CalDegSub(psth_map->u2_degA, psth_map->u2_tgtDeg);
	// ∠Aがターゲットの±40°以内にあれば範囲内
	if((s2t_deg >= ((S2)-1*(S2)u1h_deg*(S2)DEG_LSB))
	&& (s2t_deg <= ((S2)u1h_deg*(S2)DEG_LSB))){
		u1t_ret = TRUE;								// 範囲内
	}
	
	return	u1t_ret;
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
//【関数】∠B範囲判定処理													
//【機能】自車がターゲットに向かっているかどうかを判定する					
//【戻値】U1 TRUE：対向  FALSE：非対向										
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_DegBIsOk(EM_TGTDEG emh_tgtdeg, U2 u2h_dst){

	U1	u1t_ret = FALSE;							// 範囲外で初期化
	S2	s2t_degMin;
	S2	s2t_degMax;
	S2	s2t_work;

	S2	s2t_deg = psts_chkmap->s2_degB;

	switch(emh_tgtdeg){
	case TYPE_TGTDEG_EXIST_VERY_FAR:					// ターゲット方位ありで前方狭範囲
		s2t_degMin = s2_DEG_TGTFWD_MIN_VERY_FAR;
		s2t_degMax = s2_DEG_TGTFWD_MAX_VERY_FAR;
		break;
	case TYPE_TGTDEG_DEGB_FRONT:						// 前方チェック
		s2t_degMin = (S2)(-90*DEG_LSB);
		s2t_degMax = (S2)(90*DEG_LSB);
		break;
	case TYPE_TGTDEG_EXIST_HOKAN:						// ターゲット方位ありで位置補完

		if(u2h_dst < u2_DIST_1100M){					// ターゲットから近い
			s2t_work = (S2)u2h_dst / s2_DEGRATE_HOKAN_NEAR;
			s2t_degMax = s2_DEG_TGTFWD_MAX_HOKAN_NEAR - s2t_work * (S2)DEG_LSB;
			s2t_degMin = -s2t_degMax;
		}
		else{											// ターゲットから遠い
			s2t_work = (S2)(u2h_dst - u2_DIST_1100M) / s2_DEGRATE_HOKAN_FAR;
			s2t_degMax = s2_DEG_TGTFWD_MAX_HOKAN_FAR - s2t_work * (S2)DEG_LSB;
			s2t_degMin = -s2t_degMax;
		}
		break;
	default:											// ターゲット方位なし・あり
		if(u2h_dst <= u2_DIST_600M){					// ターゲットから近いとき
			s2t_degMin = s2_DEG_TGTFWD_MIN;				// 通常用範囲
			s2t_degMax = s2_DEG_TGTFWD_MAX;
		}
		else{											// ターゲットから遠いとき
			if(u2h_dst >= u2_DIST_1100M){
				s2t_degMin = s2_DEG_TGTFWD_MIN_FAR;		// 遠方用範囲
				s2t_degMax = s2_DEG_TGTFWD_MAX_FAR;		// 遠方用範囲
			}
			else{										// 距離毎に線形補完する
				s2t_work = (S2)(u2h_dst - u2_DIST_600M) * s2_DEGRATE_FAR_AND_NORM / (S2)10;
				s2t_degMin = s2_DEG_TGTFWD_MIN + s2t_work;
				s2t_degMax = s2_DEG_TGTFWD_MAX - s2t_work;
			}
		}
		break;
	}

	// 自車がターゲットに向かっているか判定
	
	if((s2t_deg >= s2t_degMin)
	&& (s2t_deg <= s2t_degMax)){
		u1t_ret = TRUE;								// 範囲内
	}

	return	u1t_ret;
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

//--------------------------------------------------------------------------
//【関数】チェックサム算出処理												
//【機能】加算チェックサムを演算して返す									
//【引数】*pu1h_ptr : 対象データポインタ									
//        u1h_size  : データサイズ											
//【戻値】U1 チェックサム値													
//【備考】U1をオーバーフローした分は無視する								
//--------------------------------------------------------------------------
U1	u1_CalChkSum(U1 *pu1h_ptr, U1 u1h_size){

	U1	u1t_sum = 0;									// チェックサム演算値
	U1	i;												// サイズループカウンタ

	for(i=0; i<u1h_size; i++){
		u1t_sum += *pu1h_ptr;
		pu1h_ptr++;
	}

	return	u1t_sum;
}

//--------------------------------------------------------------------------
//【関数】オービスボイス要求処理											
//【機能】詳細文言用データを作成して出力要求する							
//【引数】psth_map : 判定マップ												
//        emh_typ  : ボイスタイプ											
//		  psth_vcGps : 出力ボイス											
//【戻値】U1 OK：出力する  NG：出力しない									
//【備考】非対向、1km警報の600m距離既到達の場合は出力しない					
//--------------------------------------------------------------------------
static U1	u1_GpsOrbisVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	U1			u1t_spdOver = OFF;						// 速度超過判定結果
	U1			u1t_lmtSpd = OFF;
	Bool		nofix_tgt = FALSE;

	// すでに通過音声が要求された後の場合はすべて棄却する
	if(psth_map->u1_wrnSts == STS_ORBIS_50M){
		return NG;
	}

	// 速度超過判定
	if((stg_setGps[u1g_setAcsSel].b_hwOrbisLmtSpd)		// オービス制限速度告知設定ON
	&& (psth_map->un_extra.orbis.b_lmtSpd != LMTSPD_NONE)){
		u1t_lmtSpd = ON;
		if(stg_setGps[u1g_setAcsSel].b_spdOver){		// 速度超過告知ON
			u1t_spdOver = ON;
		}
	}

	switch(emh_typ){
	case VCGPS_ORBIS_2KM:								// オービス2km
	case VCGPS_ORBIS_1KM:								// オービス1km
	case VCGPS_ORBIS_500M:								// オービス500m
		// (1)共通：種類
		switch(psth_map->un_type.bit.b_code){
		case TGT_NOFIX_RD_ORBIS:
			nofix_tgt = TRUE;
		case TGT_RD_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;
			break;
		case TGT_NOFIX_LHSYS_ORBIS:
			nofix_tgt = TRUE;
		case TGT_LHSYS_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_LHSYS_ORBIS;
			break;
		case TGT_NOFIX_LOOP_ORBIS:
			nofix_tgt = TRUE;
		case TGT_LOOP_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_LP_ORBIS;
			break;
		case TGT_NOFIX_HSYS_ORBIS:
			nofix_tgt = TRUE;
		case TGT_HSYS_ORBIS:
			psth_vcGps->common.u1_trgt = VCTGT_HSYS_ORBIS;
			break;
		default:
			return NG;
		}
		// (2)共通：トンネル
		psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
		// トンネル出口拡張
		if((psth_map->un_type.bit.b_tunnel == TUNNEL_OUT) && (nofix_tgt)){
			psth_vcGps->common.b_tn_ext = ON;			// トンネル拡張指定
		}

		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

			// (3)共通：方向（非トンネル）
			if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
				return NG;
			}
			// (4)共通：距離（非トンネル）
			if(emh_typ == VCGPS_ORBIS_2KM){				// 2km告知
				if(SubVcReqUpper1000m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else if(emh_typ == VCGPS_ORBIS_1KM){		// 1km告知
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else{										// 1km以外
				SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
			}
		}

		if(emh_typ == VCGPS_ORBIS_500M){
			if((stg_setMisc.b_idvOrbisVoice)
			&& (psth_map->un_extra.orbis.u2_voiceNum <= INDIV_ORBIS_PHRASE_MASK)){
				psth_vcGps->common.b_idvVc = ON;
				psth_vcGps->extra.orbis.idvVoiceLo = (U1)(psth_map->un_extra.orbis.u2_voiceNum & 0x00FF);
				psth_vcGps->extra.orbis.idvVoiceHi = (U1)((psth_map->un_extra.orbis.u2_voiceNum & 0x0700) >> 8);
			}
		}

		if(psth_map->un_type.bit.b_tunnel != TUNNEL_OUT){	// トンネル出口データ除く
			// (5)拡張：制限速度
			if((emh_typ == VCGPS_ORBIS_1KM)
			&& (u1t_lmtSpd)){
				psth_vcGps->extra.orbis.b_lmtSpd = ON;
				psth_vcGps->extra.orbis.lmtSpdVal = psth_map->un_extra.orbis.b_lmtSpd;
				if(stg_setGps[u1g_setAcsSel].b_hwChgLmtSpd){
														// 制限速度切替ポイント告知設定ON
					u1s_memLmtSpd = psth_map->un_extra.orbis.b_lmtSpd;
														// 速度値を記憶させ切替ポイントで
														// 同じ速度を言わない
					u1s_curLmtSpd = u1s_memLmtSpd;
				}
				// (6)拡張：速度超過情報
				if(u1t_spdOver){
					psth_vcGps->extra.orbis.b_spdOver = ON;
				}
			}
			// (7)拡張：カメラ位置
			if((emh_typ == VCGPS_ORBIS_500M)
			&& (psth_map->u2_dst >= u2_DIST_300M)		// あまりに近距離になっている場合は時間がないので省く
			&& (psth_map->un_extra.orbis.b_camera != CAMERA_NONE)
			&& (stg_setGps[u1g_setAcsSel].b_orbisCameraPos)){
														// カメラ告知ON
				psth_vcGps->extra.orbis.b_camera = ON;
				psth_vcGps->extra.orbis.cameraPos = psth_map->un_extra.orbis.b_camera;
			}
		}

		// (8)共通：高速
		if(psth_map->un_type.bit.b_road == ROAD_HIGH){
			psth_vcGps->common.b_highway = ON;
		}
		break;

	case VCGPS_ORBIS_PASSSPD:
		if((psth_map->u2_dst <= u2_DIST_200M) || (psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX)){
														// 既に200m以内に到達していたら中止 or 通過が終わっていても中止
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;	// オービスならばよい（言わない）
		psth_vcGps->extra.orbis.b_runSpd = ON;			// 走行速度
		psth_vcGps->extra.orbis.lmtSpdVal = psth_map->un_extra.orbis.b_lmtSpd;
		if(u1t_spdOver){
			psth_vcGps->extra.orbis.b_spdOver = ON;		// 速度超過
		}
		break;
#if 0
	case VCGPS_ORBIS_PASS:
		if((psth_map->u2_dst > u2_DIST_50M)
		&& ((psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX))){
														// ポイントが50m以上後方になってしまったら
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_RD_ORBIS;	// オービスならばよい（言わない）
		psth_vcGps->extra.orbis.b_pass = ON;			// 通過
		break;
#endif
	}
	return	OK;
}

//--------------------------------------------------------------------------
//【関数】ゾーンボイス要求処理												
//【機能】ゾーンのボイス情報を作成して出力する								
//【引数】psth_map : 判定マップ												
//        emh_typ  : ボイスタイプ(1km, 1km未満, 圏外)						
//【戻値】U1 OK：出力する  NG：出力しない									
//【備考】非対向、1km警報の600m距離既到達の場合は出力しない					
//--------------------------------------------------------------------------
static U1	u1_GpsZoneVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	Bool	nofix_tgt = FALSE;

	// (1)共通：ターゲット
	switch(psth_map->un_type.bit.b_code){
	case TGT_NOFIX_CHKPNT_ZONE:
		nofix_tgt = TRUE;
	case TGT_CHKPNT_ZONE:
		psth_vcGps->common.u1_trgt = VCTGT_CHKPNT_ZONE;
		break;
	case TGT_NOFIX_TRAP_ZONE:
		nofix_tgt = TRUE;
	default:
		psth_vcGps->common.u1_trgt = VCTGT_TRAP_ZONE;
		break;
	}

	// (2)拡張：圏外
	if(emh_typ == VCGPS_ZONE_OUT){							// 圏外告知
		psth_vcGps->extra.trapchk.b_rngOut = ON;
	}
	else{													// 圏外以外
		// (3)共通：トンネル
		psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
		// トンネル出口拡張
		if((psth_map->un_type.bit.b_tunnel == TUNNEL_OUT) && (nofix_tgt)){
			psth_vcGps->common.b_tn_ext = ON;				// トンネル拡張指定
		}

		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

			// (4)共通：方向
			if((emh_typ != VCGPS_ZONE_100M)
			&& (SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG)){
				return NG;
			}
			// (5)共通：距離
			if(emh_typ == VCGPS_ZONE_1KM){					// 1km告知
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
			else if(emh_typ == VCGPS_ZONE_500M){			// 500m告知
				psth_vcGps->common.b_dist = VCDIST_NONE;	// 言わない
			}
			else{
				psth_vcGps->common.b_dist = VCDIST_50M;		// すぐ先
			}
		}

		if(emh_typ == VCGPS_ZONE_500M){
			// (6)拡張：制限速度(500m時・取締のみ)
			if((psth_vcGps->common.u1_trgt != VCTGT_CHKPNT_ZONE)
			&& (psth_map->un_extra.trapchk.b_lmtSpd != LMTSPD_NONE)){
				psth_vcGps->extra.trapchk.b_lmtSpd = ON;
				psth_vcGps->extra.trapchk.lmtSpdVal = psth_map->un_extra.trapchk.b_lmtSpd;
				// (7)拡張：速度超過
				if(stg_setGps[u1g_setAcsSel].b_spdOver){// 速度超過告知ON？
					psth_vcGps->extra.trapchk.b_spdOver = ON;
				}
			}
			// (8)拡張：近接(500m時)
			psth_vcGps->extra.trapchk.b_near = ON;
		}
		// (9)拡張：手法(トンネル不問)
		psth_vcGps->extra.trapchk.method = psth_map->un_extra.trapchk.u1_method;

		// (10)共通：高速(トンネル不問)
		if(psth_map->un_type.bit.b_road == ROAD_HIGH){
			psth_vcGps->common.b_highway = ON;
		}

		// (11)レベル(トンネル不問)
		if(psth_map->un_extra.trapchk.b_level != TRAPCHK_LEVEL_INVALID){
															// レベルが有効なら
			psth_vcGps->extra.trapchk.b_level = ON;
			psth_vcGps->extra.trapchk.levelVal = psth_map->un_extra.trapchk.b_level;
		}
	}

	return	OK;
}
//--------------------------------------------------------------------------
//【関数】1kmタイプコンテンツボイス要求処理									
//【機能】1km圏内で判定するコンテンツの音声情報を作成して出力要求する		
//【引数】psth_map : 判定マップ												
//        emh_typ  : ボイスタイプ(1km警報, 500m警報)						
//【戻値】U1 OK：出力する  NG：出力しない									
//【備考】非対向、1km警報の600m距離既到達の場合は出力しない					
//--------------------------------------------------------------------------
static U1	u1_Gps1kmContVcReq(ST_GPSMAP *psth_map, EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	U1	u1t_req_smart_gas = FALSE;

	// (1)共通：種類
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_MICHINOEKI:
		psth_vcGps->common.u1_trgt = VCTGT_MICHINOEKI;
		break;
	//-------------------------------------------------------------------
	case TGT_SA:
		psth_vcGps->common.u1_trgt = VCTGT_SA;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_PA:
		psth_vcGps->common.u1_trgt = VCTGT_PA;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_HWOASYS:
		psth_vcGps->common.u1_trgt = VCTGT_HWOASYS;
		u1t_req_smart_gas = TRUE;
		break;
	//-------------------------------------------------------------------
	case TGT_TUNNEL:
		psth_vcGps->common.u1_trgt = VCTGT_TUNNEL;
		psth_vcGps->extra.tunnel.b_tunnel = ON;
		psth_vcGps->extra.tunnel.tunnelType = psth_map->un_extra.tunnel.b_tunnelType;
		break;
	//-------------------------------------------------------------------
	case TGT_TORUPA:
		psth_vcGps->common.u1_trgt = VCTGT_TORUPA;
		break;
	}

	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

		// (3)共通：方向
		if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
			return NG;
		}
		// (4)共通：距離
		if(emh_typ == VCGPS_1KMCONT_1KM){					// 1km警報時
			if(u1t_req_smart_gas){
				// このタイプはワンショットのため、600m以下でも言う
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
				}
				// (5)拡張：スマートIC
				// SMART IC
				if((stg_setGps[u1g_setAcsSel].b_smartIC)
				&& (psth_map->un_extra.sapaoa.b_smartIC == SMART_IC_EXIST)){
														// SMART ICあり
					psth_vcGps->extra.sapaoa.b_smartIC = ON;
					psth_vcGps->extra.sapaoa.smartIC = psth_map->un_extra.sapaoa.b_smartIC;
				}
				// (6)拡張：ガソリンスタンド
				if((stg_setGps[u1g_setAcsSel].b_hwGas)
				&& (psth_map->un_extra.sapaoa.u1_gasStation != GS_NONE)){
														// ガソリンスタンドあり
					psth_vcGps->extra.sapaoa.b_gas = ON;
					psth_vcGps->extra.sapaoa.gasBrand = psth_map->un_extra.sapaoa.u1_gasStation;
				}
			}
			else{
				if(SubVcReqUpper500m(psth_map->u2_dst, psth_vcGps) == NG){
					return NG;
				}
			}
		}
		else{
			SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
		}
	}
	// (7)高速
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}

//--------------------------------------------------------------------------
//【関数】500mタイプコンテンツボイス要求処理								
//【機能】500m圏内でで判定するコンテンツの音声情報を作成して出力要求する	
//【引数】psth_map : 判定マップ												
//【戻値】U1 OK：出力する  NG：出力しない									
//【備考】非対向の場合は出力しない											
//--------------------------------------------------------------------------
static U1	u1_Gps500mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	U1	u1t_curve_phrase = FALSE;

	// (1)共通：種類
	switch(psth_map->un_type.bit.b_code){
	case TGT_POLICE:
		if(psth_map->un_extra.police.b_policeType == POLICE_TYPE_STATION){
			psth_vcGps->common.u1_trgt = VCTGT_POLICE_STATION;
		}
		else{
			psth_vcGps->common.u1_trgt = VCTGT_HIGHWAY_POLICE;
		}
		break;

	case TGT_KOBAN:
		psth_vcGps->common.u1_trgt = VCTGT_KOBAN;
		break;

	case TGT_CURVE:
		psth_vcGps->common.u1_trgt = VCTGT_CURVE;
		psth_vcGps->extra.curve.b_curve = ON;
		psth_vcGps->extra.curve.curveType = psth_map->un_extra.curve.b_curveType;
		u1t_curve_phrase = TRUE;
		break;
	}

	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){
		// (3)共通：方向
		if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
			return NG;
		}
		// (4)共通：距離
		if(u1t_curve_phrase == TRUE){
			psth_vcGps->common.b_dist = VCDIST_900M;		// "この先"
		}
		else{
			SubVcReqUnder500m(psth_map->u2_dst, psth_vcGps);
		}
	}

	// (5)共通：高速
	if((psth_map->un_type.bit.b_road == ROAD_HIGH)
	&& (psth_vcGps->common.u1_trgt != VCTGT_HIGHWAY_POLICE)){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}
//--------------------------------------------------------------------------
//【関数】300mタイプコンテンツボイス要求処理								
//【機能】300m圏内でで判定するコンテンツの音声情報を作成して出力要求する	
//【引数】psth_map : 判定マップ												
//【戻値】U1 OK：出力する  NG：出力しない									
//【備考】アナウンス以外非対向の場合は出力しない							
//		  アナウンスの場合、種類とトンネル以外は言わない					
//--------------------------------------------------------------------------
static U1	u1_Gps300mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	U1	u1t_announce = FALSE;

	// (1)共通：種類
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_NSYS:
		psth_vcGps->common.u1_trgt = VCTGT_NSYS;
		break;
	//-------------------------------------------------------------------
	case TGT_TRFCHK:
		psth_vcGps->common.u1_trgt = VCTGT_TRFCHK;
		break;
	//-------------------------------------------------------------------
	case TGT_ACCIDENT:
		psth_vcGps->common.u1_trgt = VCTGT_ACCIDENT;
		break;
	//-------------------------------------------------------------------
	case TGT_CROSSING:
		psth_vcGps->common.u1_trgt = VCTGT_CROSSING;
		break;
	//-------------------------------------------------------------------
	case TGT_SIGNAL:
		psth_vcGps->common.u1_trgt = VCTGT_SIGNAL;
		break;
	//-------------------------------------------------------------------
	case TGT_BRAJCT:
		psth_vcGps->common.u1_trgt = VCTGT_BRAJCT;
		psth_vcGps->extra.brajct.b_brajct = ON;
		psth_vcGps->extra.brajct.brajctType = psth_map->un_extra.brajct.b_braJctType;
		u1t_announce = TRUE;
		break;
	}

	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(u1t_announce == FALSE){
		if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){
			// (3)共通：方向
			if(SubVcReqDir(psth_map->s2_degB, psth_vcGps) == NG){
				return NG;
			}
			// (4)距離
			if(psth_map->u2_dst >= u2_DIST_50M){
				psth_vcGps->common.b_dist = VCDIST_50M;		// すぐ先
			}
			else{
				psth_vcGps->common.b_dist = VCDIST_NONE;	// 言わない
			}
		}
	}
	else{
		// (4)距離
		psth_vcGps->common.b_dist = VCDIST_900M;			// この先

		if((psth_map->u2_dst > u2_DIST_50M)
		&& ((psth_map->s2_degB < s2_DEG_TGTRNG_WIDE_MIN) || (psth_map->s2_degB > s2_DEG_TGTRNG_WIDE_MAX))){
															// ポイントが50m以上後方になってしまったら
			return NG;										// 出力しない
		}
	}

	// (5)高速
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}


	return	OK;
}

//--------------------------------------------------------------------------
//【関数】100mタイプコンテンツボイス要求処理								
//【機能】100m圏内でで判定するコンテンツの音声情報を作成して出力要求する	
//【引数】psth_map : 判定マップ												
//【戻値】U1 OK：出力する  													
//【備考】非対向の場合でも出力する											
//--------------------------------------------------------------------------
static U1	u1_Gps100mContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	// (1)共通：種類
	switch(psth_map->un_type.bit.b_code){
	//-------------------------------------------------------------------
	case TGT_HWRADIO:
		psth_vcGps->common.u1_trgt = VCTGT_HWRADIO;		// ハイウェイラジオ
		break;
	case TGT_KENKYO:
		if(u1s_kenkyo_mem != psth_map->un_extra.kenkyo.u1_kenkyoNum){
			psth_vcGps->common.u1_trgt = VCTGT_KENKYO;	// 県境
			psth_vcGps->extra.kenkyo.b_kenkyo = ON;
			psth_vcGps->extra.kenkyo.kenkyoNum = psth_map->un_extra.kenkyo.u1_kenkyoNum;
			u1s_kenkyo_mem = psth_map->un_extra.kenkyo.u1_kenkyoNum;
		}
		else{
			return NG;
		}
		break;
	}

	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;

	if(psth_map->un_type.bit.b_tunnel == NOT_TUNNEL){

		// (3)共通：方向
		psth_vcGps->common.b_dir = VCDIR_NONE;		// 言わない

		// (4)共通：距離
		if(psth_map->un_type.bit.b_code == TGT_KENKYO){
			psth_vcGps->common.b_dist = VCDIST_900M;// この先
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_NONE;// 言わない
		}
	}

	// (5)共通：高速
	if((psth_map->un_type.bit.b_road == ROAD_HIGH)
	&& (psth_map->un_type.bit.b_code != TGT_KENKYO)){
		psth_vcGps->common.b_highway = ON;
	}

	return	OK;
}
//--------------------------------------------------------------------------
//【関数】速度切替コンテンツボイス要求処理									
//【機能】速度切替コンテンツの音声情報を作成して出力要求する				
//【引数】psth_map : 判定マップ												
//【戻値】U1 OK：出力する  													
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_GpsScpContVcReq(ST_GPSMAP *psth_map, ST_VCGPS *psth_vcGps){

	// (1)共通：種類
	psth_vcGps->common.u1_trgt = VCTGT_SCP;
	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = (U1)psth_map->un_type.bit.b_tunnel;
	// (3)共通：方向
	psth_vcGps->common.b_dir = VCDIR_NONE;			// 言わない
	// (4)共通：距離
	psth_vcGps->common.b_dist = VCDIST_NONE;		// 言わない
	// (5)共通：高速
	if(psth_map->un_type.bit.b_road == ROAD_HIGH){
		psth_vcGps->common.b_highway = ON;
	}

	// (6)拡張：制限速度
	psth_vcGps->extra.scp.b_lmtSpd = ON;
	psth_vcGps->extra.scp.lmtSpdVal = psth_map->un_extra.scp.b_lmtSpd;

	// (7)拡張：速度超過
	if(stg_setGps[u1g_setAcsSel].b_spdOver){
		psth_vcGps->extra.scp.b_spdOver = ON;
	}

	return	OK;
}

//--------------------------------------------------------------------------
//【関数】エリアボイス要求処理												
//【機能】エリアボイスを作成して出力要求する								
//【引数】psth_map : 判定マップ												
//【戻値】U1 OK：出力する  													
//【備考】																	
//--------------------------------------------------------------------------
static U1	u1_GpsAreaVcReq(EM_VCGPS_TYPE emh_typ, ST_VCGPS *psth_vcGps){

	// (1)共通：種類
	switch(emh_typ){
	case VCGPS_PCHK_SZ_AREA:
		if(u1s_parkChkAreaSts != PCHK_WRN_INSZ_SNDOUT){	// すでに該当エリア外なら中止
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_SZ_AREA;
		break;

	case VCGPS_PCHK_Z_AREA:
		if(u1s_parkChkAreaSts != PCHK_WRN_INZ_SNDOUT){	// すでに該当エリア外なら中止
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_Z_AREA;
		break;

	case VCGPS_ZONE30_AREA:
		if(u1s_zone30AreaSts != ZONE30_WRN_IN_SNDOUT){		// すでに該当エリア外なら中止{
			return NG;
		}
		psth_vcGps->common.u1_trgt = VCTGT_ZONE30_AREA;
		break;
	}

	// (2)共通：トンネル
	psth_vcGps->common.b_tunnel = NOT_TUNNEL;
	// (3)共通：方向
	psth_vcGps->common.b_dir = VCDIR_NONE;					// 言わない
	// (4)共通：距離
	psth_vcGps->common.b_dist = VCDIST_NONE;				// 言わない
	// (5)共通：高速
	psth_vcGps->common.b_highway = OFF;

	return OK;
}
//--------------------------------------------------------------------------
//	ボイス作成サブ関数
//--------------------------------------------------------------------------
static U1	SubVcReqDir(S2 s2h_degB, ST_VCGPS *psth_vcGps){

	if((s2h_degB < s2_DEG_TGTFWD_MIN)
	|| (s2h_degB > s2_DEG_TGTFWD_MAX)){				// 対向していない
		return NG;									// 出力しない
	}

	if(s2h_degB < s2_DEG_TGTLEFT_MIN){
		psth_vcGps->common.b_dir = VCDIR_LEFT;		// 左方向
	}
	else if(s2h_degB > s2_DEG_TGTRIGHT_MIN){
		psth_vcGps->common.b_dir = VCDIR_RIGHT;		// 右方向
	}
	else{
		psth_vcGps->common.b_dir = VCDIR_FWRD;		// 前方向
	}

	return OK;
}
//--------------------------------------------------------------------------
static U1	SubVcReqUpper1000m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_1900M){					// 距離1900m以上のとき
		psth_vcGps->common.b_dist = VCDIST_2KM;		// "2km先"
	}
	else{											// 距離1900m未満のとき
		if(u2h_dst <= u2_DIST_1100M){				// 2km警報なのに既に1km警報が成立する距離
													// (=この後すぐに1km警報が出る状況)
			return NG;								// 出力中止
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_1900M;// "この先"
		}
	}

	return OK;
}
//--------------------------------------------------------------------------
static U1	SubVcReqUpper500m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_900M){					// 距離900m以上のとき
		psth_vcGps->common.b_dist = VCDIST_1KM;		// "1km先"
	}
	else{
		if(u2h_dst <= u2_DIST_600M){				// 1km警報なのに既に500m警報が成立する距離
													// (=この後すぐに500m警報が出る状況)
			return NG;								// 出力中止
		}
		else{
			psth_vcGps->common.b_dist = VCDIST_900M;// "この先"
		}
	}

	return OK;
}
//--------------------------------------------------------------------------
static void	SubVcReqUnder500m(U2 u2h_dst, ST_VCGPS *psth_vcGps){

	if(u2h_dst >= u2_DIST_400M){					// 距離400m以上のとき
		psth_vcGps->common.b_dist = VCDIST_500M;	// "500m先"
	}
	else if(u2h_dst >= u2_DIST_300M){				// 距離300m以上のとき
		psth_vcGps->common.b_dist = VCDIST_300M;	// "300m先"
	}
	else if(u2h_dst >= u2_DIST_200M){				// 距離200m以上のとき
		psth_vcGps->common.b_dist = VCDIST_200M;	// "200m先"
	}
	else if(u2h_dst >= u2_DIST_100M){				// 距離100m以上のとき
		psth_vcGps->common.b_dist = VCDIST_100M;	// "100m先"
	}
	else if(u2h_dst >= u2_DIST_50M){				// 距離50m以上のとき
		psth_vcGps->common.b_dist = VCDIST_50M;		// "すぐ先"
	}
	else{
		psth_vcGps->common.b_dist = VCDIST_NONE;	// 言わない
	}
}

//--------------------------------------------------------------------------
//【関数】ボイスキュー格納処理												
//【機能】ボイスキューに音声要求を格納する									
//【引数】u1h_voice : 音声番号												
//        sth_vcGps : GPS付加情報											
//        u4h_addr  : データアドレス										
//【備考】GPSタスク以外から操作されるのでディスパッチ禁止で処理する			
//--------------------------------------------------------------------------
void	GpsVcEnQueue(EM_VC u1h_voice, EM_VCGPS_TYPE emh_vcType, U4 u4h_addr){

	ST_GPSVC_QUEUE_CTRL	*pstt_queue_ctrl;

	if(emh_vcType >= VC_LOPRI_QUEUE_STA){
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_LOPRI];
	}
	else if(emh_vcType >= VC_MIDPRI_QUEUE_STA){
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_MIDPRI];
	}
	else{
		pstt_queue_ctrl = &sts_gpsVcQueueCtrl[VCGPS_HIPRI];
	}

	// 通常のキュー挿入要求時の処理
	if(pstt_queue_ctrl->sts != VC_QUEUE_FULL){					// キューがフルでないとき
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].live_count = u4s_live_counter;
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].u1_vcNum = u1h_voice;		// ボイス番号記憶
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].em_vcType = emh_vcType;		// ボイスタイプ
		pstt_queue_ctrl->queue[pstt_queue_ctrl->wrPos].u4_addr = u4h_addr;			// アドレス
		pstt_queue_ctrl->wrPos++;
		if(pstt_queue_ctrl->wrPos >= pstt_queue_ctrl->queueSize){
			pstt_queue_ctrl->wrPos = 0;
		}
		if(pstt_queue_ctrl->wrPos == pstt_queue_ctrl->rdPos){
			pstt_queue_ctrl->sts = VC_QUEUE_FULL;
		}
		else{
			pstt_queue_ctrl->sts = VC_QUEUE_DATON;
		}
	}
}

//--------------------------------------------------------------------------//
//【関数】優先フォーカスターゲット判定処理									//
//【機能】																	//
//【引数】ST_GPSMAP *psth_map  	：対象マップ要素							//
//		  U2		u2h_nearDst	：接近判定距離(0で接近判定しない)			//
//		  U2		u2h_baseDst	：基準判定距離								//
//		  U1		u1h_degChk	：自車対向判定結果							//
//【備考】																	//
//--------------------------------------------------------------------------//
static void	ChkFocusTgt(ST_FOCUS_TGT *psth_focus_tgt, ST_GPSMAP *psth_map, U2 u2h_nearDst, U2 u2h_baseDst, U1 u1h_degChk, Bool sound_focus){

	U1	u1t_focus = PRI_FOCUS_TGT_NONE;				// フォーカスなしで初期化
	U1	isPriWin = FALSE;
	U2	u2t_absDegB = (U2)(ABSO(psth_map->s2_degB));
	U2	u2t_dstSub;

	if(sound_focus)
	{
		u1t_focus = PRI_FOCUS_TGT_POLIGON_SOUNDING;
		isPriWin = TRUE;
		goto FOCUS_JDGEND;
	}

	// 絶対距離差を出す
	u2t_dstSub = ABS_SUB(psth_map->u2_dst, psth_focus_tgt->u2_dst);

	if((u1h_degChk) && (u2h_nearDst != 0) && (psth_map->u2_dst <= u2h_nearDst)){
		u1t_focus = PRI_FOCUS_TGT_POLIGON;
	}
	else if(psth_map->u2_dst <= u2h_baseDst)
	{
		// アイコンフォーカスを決める
		// 自車の前方に対向して存在する ?
		if(psth_map->hys_ahead)
		{
			u1t_focus = PRI_FOCUS_TGT_ICON;
		}
	}

	// 非測位移行中、トンネル内属性オービスのポリゴンフォーカスがあれば必ず最優先にする
	if((stg_gpsSts.b_curSokui == NG)
	&& (psth_map->un_type.bit.b_tunnel == TUNNEL_IN)
	&& (u1t_focus == PRI_FOCUS_TGT_POLIGON)){
		psth_focus_tgt->u1_priTunnelIn = TRUE;
		isPriWin = TRUE;							// 優先
		goto FOCUS_JDGEND;
	}

	// これまでの最高優先度か判定する
	if(u1t_focus > psth_focus_tgt->u1_sts){			// 状態が高優先？
		isPriWin = TRUE;
		goto FOCUS_JDGEND;
	}
	if((u1t_focus < psth_focus_tgt->u1_sts)			// 状態が低優先？
	|| (psth_focus_tgt->u1_priTunnelIn == TRUE)){	// すでに高優先トンネル内あり
		goto FOCUS_JDGEND;
	}

	// 状態が同じ

	if(u2t_dstSub >= u2_DIST_100M){					// 100m以上差があるとき
		if(psth_map->u2_dst < psth_focus_tgt->u2_dst){
			isPriWin = TRUE;						// 近い方なら優先
		}
		// 近くないなら低優先確定
		goto FOCUS_JDGEND;
	}
	else{											// 差が100m未満のとき
		// 距離優先
		if(psth_map->u2_dst < psth_focus_tgt->u2_dst){
			isPriWin = TRUE;						// 近い方なら優先
			goto FOCUS_JDGEND;						// 優先勝ち
		}
		else if(psth_map->u2_dst > psth_focus_tgt->u2_dst){
													// 遠いほうなら
			goto FOCUS_JDGEND;						// 優先負け
		}

		// 距離も同じなら対向角で判定
		if(u2t_absDegB < psth_focus_tgt->u2_absDegB){	// 対向角の小さい方を優先
			isPriWin = TRUE;
		}
		else{
			// やるとすれば種番の比較
		}
	}

FOCUS_JDGEND:

	{
		ST_FOCUS_TGT	*pstt_2ndTgt;
		if(psth_focus_tgt == &sts_warning_focusTgt_Primary)
		{
			pstt_2ndTgt = &sts_warning_focusTgt_Secondary;
		}
		else
		{
			pstt_2ndTgt = &sts_secondary_focusTgt;
		}

		if(isPriWin){
			// 優先勝ち更新前に２番目の優先にコピーする
			if(psth_focus_tgt->u1_sts != PRI_FOCUS_TGT_NONE)
			{
				*pstt_2ndTgt = *psth_focus_tgt;
			}
			// 優先勝ちしたので更新する
			psth_focus_tgt->u1_sts = u1t_focus;				// 状態更新
			psth_focus_tgt->u2_dst = psth_map->u2_dst;		// 距離記憶
			psth_focus_tgt->u2_num = u2s_sub_phase;			// ターゲット番号を記憶
			psth_focus_tgt->u2_absDegB = u2t_absDegB;

		}
		else{
			if(u1t_focus != PRI_FOCUS_TGT_NONE){
				// 何らかのフォーカスがある場合、最優先に負けたので、第2優先に該当するか判定する
				ChkFocusTgt_2nd(pstt_2ndTgt, psth_map, u2t_dstSub, u2t_absDegB, u1t_focus);
			}
		}

	}

}

//--------------------------------------------------------------------------//
//【関数】第2優先フォーカスターゲット判定処理								//
//【機能】																	//
//【備考】																	//
//--------------------------------------------------------------------------//
static void	ChkFocusTgt_2nd(ST_FOCUS_TGT *psth_2nd_focus_tgt, ST_GPSMAP *psth_map, U2 u2t_dstSub, U2 u2t_absDegB, U1 u1t_focus){

	U1	is2ndWin = FALSE;

	// これまでの最高優先度か判定する
	if(u1t_focus > psth_2nd_focus_tgt->u1_sts){	// 状態が高優先？
		is2ndWin = TRUE;
		goto FOCUS_2ND_JDGEND;
	}
	
	// 状態が低優先
	if(u1t_focus < psth_2nd_focus_tgt->u1_sts){	// 状態が低優先？
		goto FOCUS_2ND_JDGEND;
	}

	// 状態が同じ
	if(u2t_dstSub >= u2_DIST_100M){					// 100m以上差があるとき
		if(psth_map->u2_dst < psth_2nd_focus_tgt->u2_dst){
			is2ndWin = TRUE;						// 近い方なら優先
		}
		// 近くないなら低優先確定
		goto FOCUS_2ND_JDGEND;
	}
	else{											// 差が100m未満のとき
		// 距離優先
		if(psth_map->u2_dst < psth_2nd_focus_tgt->u2_dst){
			is2ndWin = TRUE;						// 近い方なら優先
			goto FOCUS_2ND_JDGEND;					// 優先勝ち
		}
		else if(psth_map->u2_dst > psth_2nd_focus_tgt->u2_dst){
													// 遠いほうなら
			goto FOCUS_2ND_JDGEND;					// 優先負け
		}

		// 距離も同じなら対向角で判定
		if(u2t_absDegB < psth_2nd_focus_tgt->u2_absDegB){
													// 対向角の小さい方を優先
			is2ndWin = TRUE;
		}
		else{
			// やるとすれば種番の比較
		}
	}

FOCUS_2ND_JDGEND:

	if(is2ndWin){
		// 優先勝ちしたので更新する
		psth_2nd_focus_tgt->u1_sts = u1t_focus;			// 状態更新
		psth_2nd_focus_tgt->u2_dst = psth_map->u2_dst;	// 距離記憶
		psth_2nd_focus_tgt->u2_num = u2s_sub_phase;		// ターゲット番号を記憶
		psth_2nd_focus_tgt->u2_absDegB = u2t_absDegB;
	}
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

	// カスタムデータは常時道路判定不要
	if(psth_map->un_type.bit.b_dataArea)
	{
		u1t_roadChk = ROADCHK_OFF;					// 道路判定不要
	}

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
			if((u1t_roadSel == SET_ROAD_AUTO)		// オート選択時
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)		// and 制限70km/h以下
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)				// and 高速道属性
			&& (u1s_roadJdgSts == ROADJDG_STS_NORM)						// and 道路判別が一般道走行状態
			&& (gps_distToHighway >= 0) && (gps_distToHighway <= 100)){	// and 高速道路までの距離が0～100m
				u1t_roadChk = ROADCHK_OFF;			// 道路判定不要
			}
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
			if((u1t_roadSel == SET_ROAD_AUTO)		// オート選択時
			&& (psth_map->un_extra.orbis.b_lmtSpd <= LMTSPD_70KMH)		// and 制限70km/h以下
			&& (psth_map->un_type.bit.b_road == ROAD_HIGH)				// and 高速道属性
			&& (u1s_roadJdgSts == ROADJDG_STS_NORM)						// and 道路判別が一般道走行状態
			&& (gps_distToHighway >= 0) && (gps_distToHighway <= 100)){	// and 高速道路までの距離が0～100m
				u1t_roadChk = ROADCHK_OFF;			// 道路判定不要
			}
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

static void	TransitDummy(EM_TGTDEG emh_tgtDeg){

}

/**
 * 現在，マイエリア内か？
 */
Bool GpsCtrl_InMyArea(void)
{
    return g_InMyArea;
}

/**
 * 現在，マイキャンセルエリア内か？
 */
Bool GpsCtrl_InMyCancelArea(void)
{
    return g_InMyCancelArea;
}
//--------------------------------------------------------------------------
//【関数】可視インデックスリスト追加処理									
//【機能】																	
//【備考】																	
//--------------------------------------------------------------------------
static void	AddVisibleIdxLst(U2 u2h_idx){

	if(srcRefIdxListSize < GPS_VISIBLE_TGT_MAX){
		s_SrcRefIdxList[srcRefIdxListSize] = u2h_idx;
		srcRefIdxListSize++;
	}
}
//--------------------------------------------------------------------------
//【関数】可視インデックスリストソート処理									
//【機能】																	
//【引数】U1 u1h_priIdx ：優先ターゲットインデックス						
//【備考】																	
//--------------------------------------------------------------------------
static void	SortVisibleIdxLst(U2 u2h_priIdx){

	U2	i,j;

	if(srcRefIdxListSize == 0){
		// サイズ0で優先ターゲットがあるときはすなわちvirtual
		if(u2h_priIdx != TGTNUM_NOTGT){
			s_SrcRefIdxList[0] = u2h_priIdx;
			srcRefIdxListSize = 1;
		}
		return;
	}
	else{
		sortSrcRefIdxByDistance();					// まず距離でソート
	}

	// 最優先フォーカスターゲットがあれば最優先にする
	if(u2h_priIdx != TGTNUM_NOTGT){
		//////////////////////////////////////////////////////////////////
		// 				最優先がインデックスリストの何番目か探す		//
		//		i						↓								//
		//pSrcRefIdxList[]	□□□□□□■□□□□□□□・・・・		//
		//																//
		//////////////////////////////////////////////////////////////////
		for(i = 0; i < srcRefIdxListSize; i++){
			if(u2h_priIdx == s_SrcRefIdxList[i]){
				break;
			}
		}
		//////////////////////////////////////////////////////////////////
		// 	先頭から見つかった場所までを全部1つ後ろに移動して			//
		//	先頭に最優先を持ってくる									//
		//		j						i								//
		//pSrcRefIdxList[]	①②③④⑤⑥■⑧⑨⑩⑪⑫⑬⑭・・・・		//
		//		↓														//
		//pSrcRefIdxList[]	■①②③④⑤⑥⑧⑨⑩⑪⑫⑬⑭・・・・		//
		//																//
		//////////////////////////////////////////////////////////////////
		if(i >= srcRefIdxListSize){		// インデックスが見つからないとき(virtual target)
			// 先頭以外を1つ後ろにずらす
			for(j = srcRefIdxListSize; j > 0 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// 先頭を最優先のものにする
			s_SrcRefIdxList[0] = u2h_priIdx;
			srcRefIdxListSize++;
		}
		else if (i != 0){				// 先頭以外で発見したら詰める
			for(j = i; j >= 1 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// 先頭を最優先のものにする
			s_SrcRefIdxList[0] = u2h_priIdx;
		}
	}
}

//--------------------------------------------------------------------------
//【関数】可視インデックスリストソート処理									
//【機能】																	
//【引数】U1 u1h_priIdx ：優先ターゲットインデックス						
//【備考】																	
//--------------------------------------------------------------------------
static void	SortVisibleIdxLst_2nd(U2 u2h_secondIdx){

	U2	i,j;

	if(u2h_secondIdx == TGTNUM_NOTGT || srcRefIdxListSize == 0)
	{
		return;
	}

	if(srcRefIdxListSize == 1){
		// サイズ1で第2優先ターゲットがあるとき
		s_SrcRefIdxList[1] = u2h_secondIdx;
		srcRefIdxListSize = 2;
		return;
	}

	// 第2優先フォーカスターゲットを探し、第2優先にする
	if(u2h_secondIdx != TGTNUM_NOTGT){
		//////////////////////////////////////////////////////////////////
		// 				第2優先がインデックスリストの何番目か探す		//
		//		i						↓								//
		//pSrcRefIdxList[]	□□□□□□■□□□□□□□・・・・		//
		//																//
		//////////////////////////////////////////////////////////////////
		for(i = 1; i < srcRefIdxListSize; i++){
			if(u2h_secondIdx == s_SrcRefIdxList[i]){
				break;
			}
		}
		//////////////////////////////////////////////////////////////////
		// 	2番目から見つかった場所までを全部1つ後ろに移動して			//
		//	第2優先を持ってくる											//
		//		j						i								//
		//pSrcRefIdxList[]	①②③④⑤⑥■⑧⑨⑩⑪⑫⑬⑭・・・・		//
		//		↓														//
		//pSrcRefIdxList[]	①■②③④⑤⑥⑧⑨⑩⑪⑫⑬⑭・・・・		//
		//																//
		//////////////////////////////////////////////////////////////////
		if(i != 1){				// 2番目以外で発見したら詰める
			for(j = i; j >= 2 ; j--){
				s_SrcRefIdxList[j] = s_SrcRefIdxList[j-1];
			}
			// 先頭を最優先のものにする
			s_SrcRefIdxList[1] = u2h_secondIdx;
		}
	}
}

/**
 * 距離（近 → 遠）順に参照インデックスをソート
 *
 * シェルソート
 */
static void sortSrcRefIdxByDistance(void)
{
    U2 h, i, j;

    for (h = 1; h < (srcRefIdxListSize / 9); h = h * 3 + 1);

    for (; h > 0; h /= 3) {
        for (i = h; i < srcRefIdxListSize; i++) {
            j = i;
            while (j >= h
                   && sts_gpsmap[ s_SrcRefIdxList[j - h] ].u2_dst > sts_gpsmap[ s_SrcRefIdxList[j] ].u2_dst
                   ) {
                U2 temp = s_SrcRefIdxList[j];
                s_SrcRefIdxList[j] = s_SrcRefIdxList[j - h];
                s_SrcRefIdxList[j - h] = temp;
                j -= h;
            }
        }
    }
}

static void GuardPictNumber(ST_GPSMAP *psth_map){

	switch(psth_map->un_type.bit.b_code){
	case TGT_RD_ORBIS:
	case TGT_HSYS_ORBIS:
	case TGT_LHSYS_ORBIS:
	case TGT_LOOP_ORBIS:
	case TGT_TRAP_ZONE:
	case TGT_CHKPNT_ZONE:
	case TGT_MYAREA:
	case TGT_MYCANCEL:
	case TGT_ICANCEL:
	case TGT_NOFIX_YUDO:
	case TGT_NOFIX_RD_ORBIS:
	case TGT_NOFIX_HSYS_ORBIS:
	case TGT_NOFIX_LHSYS_ORBIS:
	case TGT_NOFIX_LOOP_ORBIS:
	case TGT_NOFIX_TRAP_ZONE:
	case TGT_NOFIX_CHKPNT_ZONE:
	case TGT_NOFIX_SENSOR_YUDO:
		// ガードしないでそのまま
		break;

	default:
		psth_map->un_extra.common.u2_mapPctNum = 0;
		break;
	}

}

// 音声ターゲット番号の変更
static Bool	UpdateSoundTarget(U2 *pu2h_sndTgtNum, U4 *pu4h_orgAddress, const ST_GPSMAP *psth_map, U2 u2h_mapNum){ 

	U2 	i;
	Bool	ret = FALSE;

	if(*pu2h_sndTgtNum != TGTNUM_NOTGT){					// 音声ターゲット指定あり
		for(i=0 ; i<u2h_mapNum ; i++, psth_map++){
			if(*pu4h_orgAddress == psth_map->u4_dataAddr){	// データの元アドレス一致？
				ret = TRUE;
				*pu2h_sndTgtNum = i;						// 音声ターゲット番号更新
				break;
			}
		}
		if(!ret){											// なかった
			*pu2h_sndTgtNum = TGTNUM_NOTGT;					// なし
			*pu4h_orgAddress = 0;							// 一応クリア
		}
	}

	return ret;

}

static void initFocusTgt(ST_FOCUS_TGT *psth_tgt)
{
	psth_tgt->u1_sts = PRI_FOCUS_TGT_NONE;
	psth_tgt->u2_dst = U2_MAX;
	psth_tgt->u2_num = TGTNUM_NOTGT;
	psth_tgt->u2_absDegB = (U2)(180*DEG_LSB);
	psth_tgt->u1_priTunnelIn = FALSE;
}

Bool Has_tgtDeg(EM_TGTCODE code)
{
	if(TBL_TGTTRANS_PRM[code] == TYPE_TGTDEG_EXIST)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

static void update_gps_warning_lvl(EM_SYS_WARNING_LVL src_lvl, U2 dist)
{
	if(u1s_gpsWarningLvl_wrk == src_lvl)
	{
		if(u2s_gpsWarningLvl_dist_wrk > dist)
		{
			u2s_gpsWarningLvl_dist_wrk = dist;
		}
	}
	else if(u1s_gpsWarningLvl_wrk < src_lvl)
	{
		u1s_gpsWarningLvl_wrk = src_lvl;
		u2s_gpsWarningLvl_dist_wrk = dist;
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

static void TransitAreaStatusForRd(ST_GPSMAP *psth_map)
{
}

static void TransitAreaStatusForSc(ST_GPSMAP *psth_map, U1 degChk)
{
}

// カスタムデータの有効時間チェック
typedef struct{
	U2 year;
	U1 month;
	U1 day;
	U1 hour;
	U1 min;
}ST_CUSTOM_DATE;

static U4 month2day(U1 month, U2 year)
{
						   //  1  2  3  4  5  6  7  8  9  10 11 12
	static const U4 cDay[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
	
	if(month == 2){
		return 28 + (1 / ((U4)year % 4 + 1)) * (1 - 1 / ((U4)year % 100 + 1)) + (1 / ((U4)year % 400 + 1));
	}
	else{
		return cDay[month-1];
	}
}

static U4 getMinute(ST_CUSTOM_DATE date, U2 base_year)
{
	S4 n = (S4)date.year - (S4)base_year;
	U4 min = 0;
	U1 i;
	
	if(n < 0) return 0;
	

	for(; n>0; n--){
		for(i=1; i<=12 ; i++){
			min += month2day(i, base_year) * 24 * 60;
		}
	}
	
	for(i=1; i<date.month; i++){
		min += month2day(i, date.year) * 24 * 60;
	}
	min += (U4)(date.day-1) * 24 * 60 + (U4)date.hour * 60 + (U4)date.min;
	return min;
}

static Bool isCustomDateValid(ST_HEADER *header)
{
#define GUARD_MIN (24 * 60) // ガード時間 24時間(24*60分)
	ST_CUSTOM_DATE ctm;
	ST_CUSTOM_DATE gps;
	
	// gps
	gps.year  = 2000 + (U2)stg_gpsSts.u1_year;
	gps.month = stg_gpsSts.u1_month;
	gps.day   = stg_gpsSts.u1_day;
	gps.hour  = stg_gpsSts.u1_hour;
	gps.min   = stg_gpsSts.u1_min;
	// custom
	ctm.year  = 2000 + (U2)header->u1_year;
	ctm.month = header->u1_month;
	ctm.day   = header->u1_day;
	ctm.hour  = header->u1_hour;
	ctm.min   = header->u1_min;

	// gps<ctm エラー（GPSが古いことはありえない)
	if((U4)gps.year * 100 + (U4)gps.month < (U4)ctm.year * 100 + (U4)ctm.month){
		return FALSE;
	}
	if((gps.year == ctm.year)
	&& (gps.month == ctm.month)
	&& ((U4)gps.day * 100 * 100 + (U4)gps.hour * 100 + (U4)gps.min < 
	    (U4)ctm.day * 100 * 100 + (U4)ctm.hour * 100 + (U4)ctm.min)){
		return FALSE;
	}
	// 2年以上
	if(gps.year - ctm.year >= 2){
		return FALSE;
	}

	if(gps.year == ctm.year && gps.month == ctm.month){
		// 月、年が同じ。日時分を分にして比較
		U4 gps_min = gps.day * 24 * 60 + gps.hour * 60 + gps.min;
		U4 ctm_min = ctm.day * 24 * 60 + ctm.hour * 60 + ctm.min;
		return (gps_min - ctm_min >= GUARD_MIN) ? FALSE : TRUE;
	}
	else{
		// 月、年を跨ぐ。ctm_year基準の分を比較
		U4 gps_min = getMinute(gps, ctm.year);
		U4 ctm_min = getMinute(ctm, ctm.year);
		return (gps_min - ctm_min >= GUARD_MIN) ? FALSE : TRUE;
	}
}



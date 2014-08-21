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

//--------------------------------------------------------------------------
//  マクロ定義                                                              
//--------------------------------------------------------------------------

// MAP生成フーズ中のサブフェーズ
enum{
	MAP_SPHASE_INBLK,								// ブロック内バイナリサーチ
	MAP_SPHASE_GET_SMALL,							// 初期サーチ点からの緯度小サーチ
	MAP_SPHASE_GET_LARGE,							// 初期サーチ点からの緯度大サーチ
};

#define	u4_LON_SPLIT_BASE		((U4)0x00740000)	// 分割基準経度
#define	u4_LON_SPLIT_WIDTH		((U4)0x00000800)	// 経度分割幅
#define	u2_DATA_SPLIT_NUM		((U2)576)			// データ分割数

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

//--------------------------------------------------------------------------
//  変数定義
//--------------------------------------------------------------------------
static ST_GPSINF	sts_gpsinf;						// GPS入力情報

static U2			u2s_sub_phase;					// GPS処理サブフェーズ

static U2			u2s_mkmap_hiPri_num;			// 高優先マップ作成数
static U2			u2s_mkmap_loPri_num;			// 低優先マップ作成数

static U2			u2s_mkmap_num;					// 作成マップ要素数
static U2			u2s_chkmap_num;					// 判定マップ要素数
static U2			u2s_oldmap_num;
static U2			u2s_lonarea_old;				// 経度エリア前回値
static U2			u2s_srch_pos;					// サーチしているポイント
static U2			u2s_high_pos;					// サーチ上端
static U2			u2s_low_pos;					// サーチ下端
static U2			u2s_srch_center_grp;			// サーチ中央グループ

ST_GPSMAP			sts_gpsmap[GPSMAP_MAX];			// GPSマップ
static ST_GPSMAP	sts_oldmap[GPSMAP_MAX];			// GPSマップ入れ替え用前回データ
static U4			u4s_latDiffPrmMinus;
static U4			u4s_latDiffPrmPlus;
static U4			u4s_lonDiffPrmMinus;
static U4			u4s_lonDiffPrmPlus;

static ST_INDEX1	sts_index1[3];					// 周辺INDEX1情報
static ST_INDEX1 	*psts_index1;					// INDEX1ポインタ

//--------------------------------------------------------------------------
//  内部関数プロトタイプ宣言												
//--------------------------------------------------------------------------
static void	UpdLatLonArea(void);
static void UpdIndex1(void);
static U1 u1_BlkSrch(EM_TGT_DATA_TYPE type);
static U1	u1_PntSrch(U1 u1h_dir, EM_TGT_DATA_TYPE type);
static U1	u1_RegMap(void);
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type);
static Bool	Can_data_pre_expire(ST_GPSROM *psth_gpsrom);
static U1	u1_ReadGpsData(U4 u4h_dataAddr, EM_TGT_DATA_TYPE type);

//--------------------------------------------------------------------------
//  内部const data定義
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
//  外部関数定義															
//--------------------------------------------------------------------------
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
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------
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
		if((FWUtil_FileOpen(e_FWUtilClient_3, &gpsDataFile, gpspoi_filename, FA_OPEN_EXISTING | FA_READ) == FR_OK)
		&& (FWUtil_FileSeek(e_FWUtilClient_3, &gpsDataFile, u4t_adr) == FR_OK)
		&& (FWUtil_FileRead(e_FWUtilClient_3, &gpsDataFile, &sts_index1[0], sizeof(ST_INDEX1)*3, &rwsize) == FR_OK)
		&& (rwsize == sizeof(ST_INDEX1)*3)){
			// 暗号解除
			DataCrypt(sizeof(ST_INDEX1)*3, u4t_adr, (U1 *)&sts_index1[0]);
		}
		FWUtil_FileClose(e_FWUtilClient_3, &gpsDataFile);

		u2s_lonarea_old = sts_gpsinf.u2_lonarea;	// 前回経度エリアを保存
	}
	// 経度エリア変更なしならINDEX1は変わらない

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
		pstt_mkmap->u1_areaScSnd = SCSND_OFF;

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

	if((FWUtil_FileOpen(e_FWUtilClient_3, f, gpspoi_filename, FA_OPEN_EXISTING | FA_READ) != FR_OK)
	|| (FWUtil_FileSeek(e_FWUtilClient_3, f, u4h_dataAddr) != FR_OK)
	|| (FWUtil_FileRead(e_FWUtilClient_3, f, &sts_gpsrom, u1_MAPDATA_SIZE, &rwsize) != FR_OK)
	|| (rwsize != u1_MAPDATA_SIZE)){
		FWUtil_FileClose(e_FWUtilClient_3, f);
		return NG;
	}
	FWUtil_FileClose(e_FWUtilClient_3, f);
	DataCrypt(u1_MAPDATA_SIZE, u4h_dataAddr, (U1 *)&sts_gpsrom);

	sts_gpsrom.u4_addr = u4h_dataAddr;	// データの物理アドレスを保存

	return	OK;
}


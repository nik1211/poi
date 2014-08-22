//--------------------------------------------------------------------------
//�y���W���[���zGPS�R���g���[��												
//�y�@�\�z      GPS�ʒu������͂Ƃ��Ċe��GPS�x����o�͂���				
//�y���l�z																	
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//  �C���N���[�h�t�@�C��                                                    
//--------------------------------------------------------------------------

#include "stdafx.h"
#include "gpsctrl.h"
#include "gpsctrl.prm"
#include "crypt.h"

//--------------------------------------------------------------------------
//  �}�N����`                                                              
//--------------------------------------------------------------------------

// �I�[�r�X�x���Ԓ�`
enum{
	STS_ORBIS_NOALARM = 0,							// ��x����
	STS_ORBIS_2100M,								// 2100m�x����
	STS_ORBIS_1100M,								// 1100m�x����
	STS_ORBIS_600M,									// 600m�x����
	STS_ORBIS_300M,									// 300m�x����
	STS_ORBIS_50M,									// 50m�x����
};

// �}�C�G���A
enum{
	STS_MYAREA_NOALARM = 0,							// ��x����
	STS_MYAREA_1100M,								// 1100m�x����
	STS_MYAREA_600M,								// 600m�x����
	STS_MYAREA_50M,									// 50m�x����
};
// �}�C�L�����Z��
enum{
	STS_MYCANCEL_RNGOUT = 0,						// ���O���
	STS_MYCANCEL_200M,								// 200m����
	STS_MYCANCEL_200M_RD,							// 200m����(RD��M����)
};
// �����L�����Z��
enum{
	STS_ATCANCEL_RNGOUT = 0,						// ���O���
	STS_ATCANCEL_REG1ST,							// �o�^1��ڒʉ�
	STS_ATCANCEL_200M_NORD,							// 200m����(RD��M�Ȃ�)
	STS_ATCANCEL_100M_NORD,							// 100m����(RD��M�Ȃ�)
	STS_ATCANCEL_RDRX,								// ����RD��M����
	STS_ATCANCEL_AACDIS,							// ����AAC�֎~
};
// �����L�����Z���o�^
typedef enum{
	REGSTS_IDLE = 0,								// �o�^�E�폜�҂����
	REGSTS_SRCHPOS,									// 30�`150m�ʒu�T�[�`���
	REGSTS_CHK300M,									// 300m�����`�F�b�N���
	REGSTS_CHK500M,									// 500m�����`�F�b�N���
}EM_ATCANCEL_REGSTS;

// SCP�R���e���c��Ԓ�`
enum{
	STS_SCP_RNGOUT = 0,								// ���O
	STS_SCP_100M,									// 100m�������
	STS_SCP_50M,									// 50m�������
};
// �����V���b�g�R���e���c��Ԓ�`
enum{
	STS_ONESHOTCONT_NOALARM = 0,
	STS_ONESHOTCONT_ALARM,
};

// 1km�R���e���c��Ԓ�`
enum{
	STS_1KMCONT_NOALARM = 0,						// ��x����
	STS_1KMCONT_1100M,								// 1100m�x����
	STS_1KMCONT_600M,								// 600m�x����
};
// �G���A�^�C�v�R���e���c��Ԓ�`
enum{
	STS_AREACONT_OUTAREA = 0,						// �G���A�O���
	STS_AREACONT_INAREA,							// �G���A�����
};

// ETC�Q�[�g��Ԓ�`
enum{
	STS_ETC_NOPASS = 0,								// ���ʉ�
	STS_ETC_PASS,									// �ʉ�
};

// ���֌x����
enum{
	PCHK_WRN_OUT = 0,								// ���O
	PCHK_WRN_INSZ_NOSNDOUT,							// �ŏd�_�G���A�������o��
	PCHK_WRN_INSZ_SNDOUT,							// �ŏd�_�G���A�����o�͍ς�
	PCHK_WRN_INZ_NOSNDOUT,							// �d�_�G���A�������o��
	PCHK_WRN_INZ_SNDOUT,							// �d�_�G���A�����o�͍ς�
};

// �]�[��30���
enum{
	ZONE30_WRN_OUT = 0,								// ���O
	ZONE30_WRN_IN_NOSNDOUT,							// �����������o��
	ZONE30_WRN_IN_SNDOUT,							// ���������o�͍ς�
};

// �ԏ�_�����
enum{
	SHAJYO_WRN_OUT = 0,								// ���O
	SHAJYO_WRN_IN_NOSNDOUT,							// �����������o��
	SHAJYO_WRN_IN_SNDOUT,							// ���������o�͍ς�
};

// �g���l�����I�[�r�X�x���Ԓ�`
enum{
	STS_TUNNEL_ORBIS_NOALARM = 0,
	STS_TUNNEL_ORBIS_2100M,
	STS_TUNNEL_ORBIS_1100M,
	STS_TUNNEL_ORBIS_600M,
};

// MAP�����t�[�Y���̃T�u�t�F�[�Y
enum{
	MAP_SPHASE_INBLK,								// �u���b�N���o�C�i���T�[�`
	MAP_SPHASE_GET_SMALL,							// �����T�[�`�_����̈ܓx���T�[�`
	MAP_SPHASE_GET_LARGE,							// �����T�[�`�_����̈ܓx��T�[�`
};

#define	u4_LON_SPLIT_BASE		((U4)0x00740000)	// ������o�x
#define	u4_LON_SPLIT_WIDTH		((U4)0x00000800)	// �o�x������
#define	u2_DATA_SPLIT_NUM		((U2)576)			// �f�[�^������
#define	u1_MAPDATA_SIZE			((U1)16)			// �I�[�r�XROM�f�[�^�T�C�Y

enum{
	TYPE_LAT = 0,
	TYPE_LON,
};

// �T�[�`���ʒ�`
enum{
	u1_SRCH_OK = 0,									// �T�[�`����
	u1_SRCH_NG_SMALL,								// �T�[�`�����ł�����
	u1_SRCH_NG_LARGE,								// �T�[�`�����ł�����
	u1_SRCH_NG_NONE,								// �T�[�`�����ł����|�C���g�Ȃ�
	u1_SRCH_NG_DEV_BUSY,							// �f�o�C�X�r�W�[
	u1_SRCH_END,									// �T�[�`�I��
	u1_SRCH_MAX_ERR									// �T�[�`���ő�G���[
};

// �}�b�v�o�^���ʒ�`
enum{
	REGOK = 0,
	REGNG_LAT_SMALL,
	REGNG_LAT_LARGE,
	REGNG_LON_SMALL,
	REGNG_LON_LARGE,
	REGNG_MAP_MAX,
	REGNG_EEP_BUSY
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

// �G���A���
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
	EM_TGT_DATA_PAY,		//�L����
#endif
}EM_TGT_DATA_TYPE;

//--------------------------------------------------------------------------
//  �����^��`                                                              
//--------------------------------------------------------------------------
// GPS�^�[�Q�b�gROM�f�[�^
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
//  �O�����J�ϐ�                                                            
//--------------------------------------------------------------------------
ST_GPS_STS	stg_gpsSts;

//--------------------------------------------------------------------------
//  �ϐ���`
//--------------------------------------------------------------------------
static ST_GPSINF	sts_gpsinf;						// GPS���͏��

static U2			u2s_sub_phase;					// GPS�����T�u�t�F�[�Y

static U2			u2s_mkmap_hiPri_num;			// ���D��}�b�v�쐬��
static U2			u2s_mkmap_loPri_num;			// ��D��}�b�v�쐬��

static U2			u2s_mkmap_num;					// �쐬�}�b�v�v�f��
static U2			u2s_chkmap_num;					// ����}�b�v�v�f��
static U2			u2s_oldmap_num;
static U2			u2s_inisrch_pos;				// �����T�[�`�|�C���g
static U2			u2s_lonarea_old;				// �o�x�G���A�O��l
static U2			u2s_srch_pos;					// �T�[�`���Ă���|�C���g
static U2			u2s_high_pos;					// �T�[�`��[
static U2			u2s_low_pos;					// �T�[�`���[
static U2			u2s_srch_center_grp;			// �T�[�`�����O���[�v

ST_GPSMAP			sts_gpsmap[GPSMAP_MAX];			// GPS�}�b�v
static ST_GPSMAP	sts_oldmap[GPSMAP_MAX];			// GPS�}�b�v����ւ��p�O��f�[�^
static U4			u4s_latDiffPrmMinus;
static U4			u4s_latDiffPrmPlus;
static U4			u4s_lonDiffPrmMinus;
static U4			u4s_lonDiffPrmPlus;
static ST_GPSROM	sts_gpsrom;						// GPS ROM�f�[�^���[�N

static ST_INDEX1	sts_index1[3];					// ����INDEX1���
static ST_INDEX1 	*psts_index1;					// INDEX1�|�C���^

static UN_REGDAY	uns_today;						// ���t��r�p

static FILE	*gpsDataFile;
char	gpspoi_filename[13];

//--------------------------------------------------------------------------
//  �����֐��v���g�^�C�v�錾												
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
//  ����const data��`
//--------------------------------------------------------------------------
const U2	u2g_dataSpec = 6;		//1006xxxx.txt

//--------------------------------------------------------------------------
//  �O���֐���`															
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//�y�֐��z�ړ�����t�F�[�Y													
//�y�@�\�z���Ԉړ��������v�����āA�ړ����Ă����INDEX1���X�V����			
//--------------------------------------------------------------------------
static void PhaseChkMov(void){

	ST_GPSMAP	*pstt_oldmap;
	ST_GPSMAP	*pstt_curmap;

	UpdLatLonArea();								// �ܓx�E�o�x�͈͂̌���

	UpdIndex1();								// INDEX1�X�V
	// �}�b�v�쐬�O�ɋ��f�[�^�̑ޔ�
	pstt_oldmap = &sts_oldmap[0];
	pstt_curmap = &sts_gpsmap[0];

	u2s_oldmap_num = u2s_chkmap_num;
	if(u2s_oldmap_num != 0){
		memcpy(pstt_oldmap, pstt_curmap, sizeof(ST_GPSMAP)*u2s_oldmap_num);
	}
	// �ܓx�E�o�x�̋��e�͈̓p�����[�^����
	u4s_latDiffPrmMinus = sts_gpsinf.lat - u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_latDiffPrmPlus = sts_gpsinf.lat + u4_TBL_LATDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmMinus = sts_gpsinf.lon - u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];
	u4s_lonDiffPrmPlus = sts_gpsinf.lon + u4_TBL_LONDIST[DISTRNG_2500M][sts_gpsinf.u1_latarea];

	u2s_mkmap_hiPri_num = VIRTUAL_INDEX_TYPES;	// ���D��}�b�v�쐬��������(���z�^�[�Q�b�g���͊m��)
	u2s_mkmap_loPri_num = 0;					// ��D��}�b�v�쐬��������
	u2s_mkmap_num = VIRTUAL_INDEX_TYPES;		// �쐬MAP�v�f���N���A(���z�^�[�Q�b�g���͊m��)
}
//--------------------------------------------------------------------------
//�y�֐��zMAP�����t�F�[�Y����												
//�y�@�\�z���Ԉʒu���ROM�f�[�^�����o���AMAP�𐶐�����					
//�y���l�z�u���b�N�ԍ����傫�����A�ܓx���傫��								
//--------------------------------------------------------------------------
static void PhaseMakeMap(void){

	U2	u2t_pos_bak;									// ���_�ۑ��l
	U1	u1t_groupEnd;
	U1	u1t_srchStep;
	U4	u4t_lon_center;
	ST_INDEX1	*pstt_index1;
	EM_TGT_DATA_TYPE type;

	pstt_index1 = &sts_index1[0];
	type = EM_TGT_DATA_MAKER;

	// ������
	u2s_srch_pos = 0;									// �T�[�`�ʒu������
	for(u1t_srchStep = 0; u1t_srchStep <=2 ; u1t_srchStep++){
		switch(u1t_srchStep){
		case 0:
			psts_index1 = &pstt_index1[1];				// ���߂͒�������
			break;
		case 1:
			// ���݈ʒu�������Ȃ獶�D��A�E���Ȃ�E�D��
			u4t_lon_center = u4_LON_SPLIT_BASE + (U4)u2s_srch_center_grp*u4_LON_SPLIT_WIDTH + (u4_LON_SPLIT_WIDTH / 2);
			if(sts_gpsinf.lon < u4t_lon_center){		// �����H
				psts_index1 = &pstt_index1[0];			// ���͍�
			}
			else{
				psts_index1 = &pstt_index1[2];			// ���͉E
			}
			break;
		case 2:
			if(psts_index1 == &pstt_index1[0]){			// �܂��ǂ�łȂ����ɂ���
				psts_index1 = &pstt_index1[2];
			}
			else{
				psts_index1 = &pstt_index1[0];
			}
			break;
		}
		// 1�̃O���[�v������
		// �o�C�i���T�[�`����
		// 1�̃O���[�v������
		if(psts_index1->u2_elenum == 0){
			continue;
		}
		u1t_groupEnd = OFF;
		u2s_low_pos = 0;								// ���_
		u2s_high_pos = psts_index1->u2_elenum;			// ��_
		u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1);
														// ���_�Z�o
		u2s_sub_phase = MAP_SPHASE_INBLK;				// �o�C�i���T�[�`�J�n

		for(;;){
			switch(u2s_sub_phase){
		//-----------------�u���b�N���_����̃T�[�`-------------
			case MAP_SPHASE_INBLK:
				u2t_pos_bak = u2s_srch_pos;
				switch(u1_BlkSrch(type)){				// �u���b�N�����_�T�[�`
				
				case u1_SRCH_OK:						// �����T�[�`�_����
					u2s_sub_phase = MAP_SPHASE_GET_SMALL;
														// ����ɒ��o�J�n
					break;
					
				case u1_SRCH_NG_LARGE:					// �ܓx�呤�ɔ͈͊O���ܓx������T��
					u2s_high_pos = u2s_srch_pos;		// ���_���̂܂� ��_�͍��̒��_
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// ���_�Z�o
					if(u2s_srch_pos == u2t_pos_bak){	// ���_�ɕύX�Ȃ�(�|�C���g�Ȃ�)�H
						u1t_groupEnd = ON;				// �O���[�v�I��
					}
					break;
					
				case u1_SRCH_NG_SMALL:					// �ܓx�����ɔ͈͊O���ܓx�呤��T��
					u2s_low_pos = u2s_srch_pos;			// ���_�͍��̒��_ ��_�͂��̂܂�
					u2s_srch_pos = (U2)((u2s_high_pos - u2s_low_pos) >> 1) + u2s_low_pos;
														// ���_�Z�o
					if(u2s_srch_pos == u2t_pos_bak){	// ���_�ɕύX�Ȃ�(�|�C���g�Ȃ�)�H
						u1t_groupEnd = ON;				// �O���[�v�I��
					}
					break;
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
//				case u1_SRCH_NG_NONE:					// �Ȃ�
				default:
					goto srch_end;
				}
				break;

		//----------�����T�[�`�_����̈ܓx�������T�[�`----------------------------
			case MAP_SPHASE_GET_SMALL:
				switch(u1_PntSrch(SMALL, type)){		// �ܓx������Xkm�T��
				case u1_SRCH_END:						// �ܓx�����I���Ȃ�
					u2s_sub_phase = MAP_SPHASE_GET_LARGE;
														// ���͈ܓx�呤��T��
					u2s_srch_pos = u2s_inisrch_pos;		// �����T�[�`�ʒu�����[�h
					break;
				case u1_SRCH_OK:						// �o�^OK
														// ���������T��
					break;
//				case u1_SRCH_MAX_ERR:					// �o�^�ő�NG
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
				default:
					goto srch_end;						// �t�F�[�Y�I��

				}
				break;
		//----------�����T�[�`�_����̈ܓx������T�[�`----------------------------
			case MAP_SPHASE_GET_LARGE:
				switch(u1_PntSrch(LARGE, type)){		// �ܓx�呤��Xkm�T��
				case u1_SRCH_END:
					u1t_groupEnd = ON;					// �O���[�v�I��
					break;

				case u1_SRCH_OK:						// �o�^OK -> ���������T��
					break;

//				case u1_SRCH_MAX_ERR:
//				case u1_SRCH_NG_DEV_BUSY:				// �f�o�C�X�r�W�[
				default:
					goto srch_end;						// �T�[�`�I��

				}
				break;
			}
			
			if(u1t_groupEnd){							// �O���[�v�I�� ?
				break;
			}
		}
	}
srch_end:
	return;
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------
//�y�֐��z�ܓx�E�o�x�͈͍X�V����											
//�y�@�\�z���݂̈ܓx��5���������G���A�̂ǂ̈ʒu�ɂ��邩�����肷��			
//		  ���݂̌o�x��576���������G���A�̂ǂ̈ʒu�ɂ��邩�����肷��			
//�y���l�z�G���A�ԍ���1�`574�܂�(�[��1�����ɓ����						
//--------------------------------------------------------------------------
static void	UpdLatLonArea(void){

	U1	i;											// ���[�v�J�E���^
	U4	u4t_lonwrk;

	// �ܓx�G���A�̌���
	for(i = 0; i< LATAREA_MAX; i++){
		if(((sts_gpsinf.lat & 0x00FFFF00) >> 8) < u2_TBL_LATAREA[i]){
													// ���2byte�T�C�Y�Ō���
													// �������l��菬�Ȃ�
			break;									// �G���A����
		}
	}
	sts_gpsinf.u1_latarea = i;

	u4t_lonwrk = sts_gpsinf.lon;					// �o�x
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
//�y�֐��zINDEX1�X�V����													
//�y�@�\�z�G���A���Z�o���A�G���A�ύX������V����INDEX1�ɍX�V����			
//�y���l�z																	
//--------------------------------------------------------------------------
static void UpdIndex1(void){

	U4	u4t_adr;									// �Ǐo���A�h���X
	U4	rwsize;

	// �G���A�ύX�Ȃ�V����INDEX1���\�z����
	if(u2s_lonarea_old != sts_gpsinf.u2_lonarea){	// �ω��L��H

		u2s_srch_center_grp = sts_gpsinf.u2_lonarea;

		// YP�f�[�^ INDEX1
		u4t_adr = sizeof(ST_HEADER) + ((U4)(u2s_srch_center_grp-1) * (U4)sizeof(ST_INDEX1));
		if((fopen_s(&gpsDataFile, gpspoi_filename, "rb") == 0)
			&& (fseek(gpsDataFile, u4t_adr, SEEK_CUR) == 0)
			&& ((rwsize = fread(&sts_index1[0], sizeof(unsigned char), sizeof(ST_INDEX1) * 3, gpsDataFile)) > 0)
			&& (rwsize == sizeof(ST_INDEX1)*3))
		{
			// �Í�����
			DataCryptPoi(sizeof(ST_INDEX1)*3, u4t_adr, (U1 *)&sts_index1[0], u2g_dataSpec);
		}
		fclose(gpsDataFile);

		u2s_lonarea_old = sts_gpsinf.u2_lonarea;	// �O��o�x�G���A��ۑ�
	}
	// �o�x�G���A�ύX�Ȃ��Ȃ�INDEX1�͕ς��Ȃ�

}

//--------------------------------------------------------------------------
//�y�֐��z�u���b�N���T�[�`����												
//�y�@�\�z�u���b�N�̎w��ʒu�f�[�^���͈͓����`�F�b�N����					
//�y�����zu1h_romType �F�Ώ�ROM�^�C�v										
//�y�ߒl�zu1_SRCH_NG_NONE	: �u���b�N���Ƀ|�C���g�Ȃ��̂��ߎ��s			
//		  u1_SRCH_NG_DEV_BUSY: �f�o�C�X�A�N�Z�X���s							
//		  u1_SRCH_NG_SMALL	: �ܓx���̂��ߎ��s								
//		  u1_SRCH_NG_LARGE	: �ܓx��̂��ߎ��s								
//		  u1_SRCH_OK		: �T�[�`����									
//--------------------------------------------------------------------------
static U1 u1_BlkSrch(EM_TGT_DATA_TYPE type){

	U1	u1t_ret;									// �ߒl
	U4	u4t_dataAddr;

	// INDEX1�̊Y���u���b�N�v�f�����`�F�b�N
	if(psts_index1->u2_elenum == 0){				// �|�C���g�Ȃ�
		u1t_ret = u1_SRCH_NG_NONE;					// �|�C���g�Ȃ��ɂ�莸�s
	}
	else{
		// INDEX1�̃O���[�v�擪�ʒu�ƈʒu����f�[�^�A�h���X���o��
		u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
		// �ǂݏo��
		if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
													// �ǂݏo�����s�Ȃ�
			u1t_ret = u1_SRCH_NG_DEV_BUSY;			// DEVICE BUSY��Ԃ�
		}
		// MAP�o�^�����݂�
		else{
			switch(u1_RegMap()){					// �o�^���s����
			case REGNG_LAT_SMALL:					// ���݈ܓx��菬
				u1t_ret = u1_SRCH_NG_SMALL;
				break;
			
			case REGNG_LAT_LARGE:					// ���݈ܓx����
				u1t_ret = u1_SRCH_NG_LARGE;
				break;
			
			default:								// �ܓx�͈͓��Ȃ�
				u1t_ret = u1_SRCH_OK;				// �����T�[�`�Ƃ��Ă�OK
				// �����T�[�`�_�����L��
				u2s_inisrch_pos = u2s_srch_pos;		// �u���b�N���ʒu�L��
				break;
			}
		}
	}
	return	u1t_ret;
}

//--------------------------------------------------------------------------
//�y�֐��z�|�C���g�T�[�`����												
//�y�@�\�z���݂̃|�C���g����ܓx�㉺�̎w������Ƀ|�C���g�����o��			
//�y�����zu1h_romType �F�Ώ�ROM�^�C�v										
//		  u1h_dir�F���o������		SMALL / LARGE							
//�y���l�z																	
//--------------------------------------------------------------------------
static U1	u1_PntSrch(U1 u1h_dir, EM_TGT_DATA_TYPE type){
	
	U1	u1t_ret = u1_SRCH_OK;						// �ߒl
	U4	u4t_dataAddr;
	
	// ���̃f�[�^�����o�����߁A�u���b�N���ʒu�E�u���b�N�ԍ����X�V����
	// �ܓx����
	if(u1h_dir == SMALL){							// �ܓx��������
		if(u2s_srch_pos == 0){						// �ʒu����ԏ�H
			return	u1_SRCH_END;					// �I��
		}
		else{										// �ʒu����ԏ�łȂ�
			u2s_srch_pos--;							// �ʒu�ړ�
		}
	}
	else{											// �ܓx�������
		if(u2s_srch_pos >= (psts_index1->u2_elenum - (U2)1)){
													// �u���b�N�ŉ��_�ɓ��B�H
			return u1_SRCH_END;
		}
		else{
			u2s_srch_pos++;							// �ʒu�ړ�
		}
	}

	// �f�[�^��ǂݏo��
	u4t_dataAddr = psts_index1->u4_adr + ((U4)u2s_srch_pos << 4);
	if(u1_ReadGpsData(u4t_dataAddr, type) == NG){
		return	u1_SRCH_NG_DEV_BUSY;
	}

	// MAP�o�^�����݂�
	switch(u1_RegMap()){

	case REGNG_MAP_MAX:								// �o�^�����I�[�o�[�œo�^���ꂸ
		u1t_ret = u1_SRCH_MAX_ERR;					// �ő吔�G���[��ʒm
		break;

	case REGNG_LAT_SMALL:							// �ܓx���̂��ߓo�^���ꂸ
	case REGNG_LAT_LARGE:							// �ܓx��̂��ߓo�^���ꂸ
		u1t_ret = u1_SRCH_END;						// �ܓx�����ɂ͂����Ȃ����ߏI��
		break;

//	case REGOK:										// �͈͓��̂��ߓo�^���ꂽ
//	case REGNG_LON_SMALL:							// �o�x���̂��ߓo�^���ꂸ
//	case REGNG_LON_LARGE:							// �o�x��̂��ߓo�^���ꂸ
	default:
		u1t_ret = u1_SRCH_OK;						// �ܓx�����͓����Ă���̂Ōp��
		break;
	}
	
	return	u1t_ret;
}
//--------------------------------------------------------------------------
//�y�֐��z�}�b�v�o�^����													
//�y�@�\�z�|�C���g���͈͓��ł���΃}�b�v�ɓo�^����							
//�y�ߒl�zOK�F�o�^�\		NG�F�o�^�s�\									
//--------------------------------------------------------------------------
static U1	u1_RegMap(void){

	U4	u4t_diffLat;								// �ܓx����
	U4	u4t_diffLon;								// �o�x����
	U4	u4t_lat;									// �ܓx
	U4	u4t_lon;									// �o�x
	ST_GPSMAP	*pstt_mkmap;
	U1	u1t_highway;
	Bool in_range = FALSE;

	u4t_lat = u4_RomDeScrmble(&sts_gpsrom, TYPE_LAT);	// �ܓx�l�X�N�����u������

	if(u4t_lat < u4s_latDiffPrmMinus){
		return	REGNG_LAT_SMALL;					// �����l��艺�̂���NG
	}
	if(u4t_lat > u4s_latDiffPrmPlus){
		return	REGNG_LAT_LARGE;					// ����l����̂���NG
	}

	u4t_lon = u4_RomDeScrmble(&sts_gpsrom, TYPE_LON);	// �o�x�l�X�N�����u������

	if(u4t_lon < u4s_lonDiffPrmMinus){
		return	REGNG_LON_SMALL;					// �����l��艺�̂���NG
	}
	if(u4t_lon > u4s_lonDiffPrmPlus){
		return	REGNG_LON_LARGE;					// ����l����̂���NG
	}

	u4t_diffLat = ABS_SUB(sts_gpsinf.lat, u4t_lat);
	u4t_diffLon = ABS_SUB(sts_gpsinf.lon, u4t_lon);

	// POI�f�[�^����ʂɂ���̂ŁA���̑������͎̂��O�Ɏ̂Ă�悤�ɂ���
	if(Can_data_pre_expire(&sts_gpsrom) == TRUE){
		return REGOK;								// ���ɐi��
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

	// �͈͓��m��
	if(in_range){
		// �D��x����
		if(u1_TBL_MAPPRI[sts_gpsrom.u1_type] == LO){
			if(u2s_mkmap_num >= GPSMAP_MAX){		// ���łɒ�D�������]�n�Ȃ��H
				return	REGOK;						// �o�^�͂��Ȃ����A�܂����D�悪���邩������Ȃ��̂�
													// ���̍쐬�ɐi��
			}
			pstt_mkmap = &sts_gpsmap[(GPSMAP_MAX - 1) - u2s_mkmap_loPri_num];
													// ��D��}�b�v�쐬�ʒu
			u2s_mkmap_loPri_num++;					// ��D��}�b�v�쐬���X�V
		}
		else{
			if(u2s_mkmap_hiPri_num >= GPSMAP_MAX){	// ���łɂ��ׂč��D��œ����]�n�Ȃ��H
				return	REGNG_MAP_MAX;				// �ő���̂��ߓo�^�s��
			}
			pstt_mkmap = &sts_gpsmap[u2s_mkmap_hiPri_num];
													// ���D��}�b�v�ʒu
			u2s_mkmap_hiPri_num++;					// ���D��}�b�v�쐬���X�V
			if((u2s_mkmap_loPri_num != 0)			// ��D��}�b�v�P���ȏ゠�� and
			&& ((u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num) > GPSMAP_MAX)){
													// ���D�悪��D��̏㏑�������H
				u2s_mkmap_loPri_num--;				// ��D��}�b�v�P���폜
			}
		}
		u2s_mkmap_num = u2s_mkmap_hiPri_num + u2s_mkmap_loPri_num;
													// �}�b�v�쐬���X�V

		pstt_mkmap->u4_tgtLat = u4t_lat;			// �ܓx�ۑ�
		pstt_mkmap->u4_tgtLon = u4t_lon;			// �o�x�ۑ�
		pstt_mkmap->u4_dataAddr = sts_gpsrom.u4_addr;
													// ���A�h���X�ۑ�
		pstt_mkmap->u2_tgtDeg = (U2)sts_gpsrom.u1_deg * (U2)DEGDAT_LSB * (U2)DEG_LSB;
		pstt_mkmap->un_type.bit.b_code = sts_gpsrom.u1_type;
													// ���
		pstt_mkmap->un_type.bit.b_road = sts_gpsrom.road;
													// ���H����
		pstt_mkmap->un_type.bit.b_tunnel = sts_gpsrom.tunnel;
													// �g���l������
		pstt_mkmap->un_type.bit.b_dataArea = sts_gpsrom.area;
													// �G���A�����ۑ�
		pstt_mkmap->u1_wrnSts = 0;					// �x���ԏ�����
		pstt_mkmap->hys_ahead = FALSE;				// �O�����o������
		pstt_mkmap->ahead_hys_count = 0;			// �O���q�X�J�E���^������
		pstt_mkmap->u2_countdown_dist = U2_MAX;
		pstt_mkmap->u2_dst_old = U2_MAX;
		pstt_mkmap->u1_areaStsRd = AREA_STATUS_OUT;
		pstt_mkmap->u1_areaStsSc = AREA_STATUS_OUT;

		if(sts_gpsrom.u1_type == TGT_ICANCEL){		// I�L�����Z���̂Ƃ�
			if(sts_gpsrom.un_extra.dynamic.un_regday.hword == uns_today.hword){
				pstt_mkmap->u1_wrnSts = STS_ATCANCEL_REG1ST;
													// �L�����Z�������Ȃ�
			}
		}
		memcpy(&pstt_mkmap->un_extra.byte[0], &sts_gpsrom.un_extra.byte[0], EXTRA_DATA_SIZE);
													// EXTRA�ۑ�
		// �o�^����
	}
	// �͈͖͂����ł��o�^��OK�ɂ���(�����T�[�`�̏ꍇ�̓}�b�v���͂Ȃ��Ă��悢)

	return	REGOK;

}

//--------------------------------------------------------------------------
//�y�֐��zGPS�ܓx�E�o�x���X�N�����u����������								
//�y�@�\�z�����g�p�ł��鐔�l�ɕϊ�����									
//�y�����z�Ώۃ^�C�v  TYPE_LAT�F�ܓx  TYPE_LON�F�o�x						
//�y�ߒl�zU4	�ܓx�l or �o�x�l (LSB : 10^-3��)							
//�y���l�z																	
//--------------------------------------------------------------------------
static U4	u4_RomDeScrmble(ST_GPSROM *p_rom, U1 u1h_type){

	U4	u4t_calwork;								// �Z�o���[�N
	U4	u4t_addval;									// ���Z�l
	
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
	return FALSE;
}

//--------------------------------------------------------------------------//
//�y�֐��zGPS�f�[�^�ǂݏo������												//
//�y�@�\�z�w��u���b�N�A�w��ʒu�̃f�[�^��RAM�ɓW�J����						//
//�y�ߒl�zOK�F�ǂݏo������		NG�F�ǂݏo�����s							//
//�y���l�zdynamic��static�̐ؑ֎��͕K��cache disable�ɂ��邱��				//
//        ���ǂݏo�����������ɂȂ�A�����A�h���X���d�����邽��				//
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

	sts_gpsrom.u4_addr = u4h_dataAddr;	// �f�[�^�̕����A�h���X��ۑ�

	return	OK;
}

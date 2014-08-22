
#ifndef __GPSCTRL_H__
#define __GPSCTRL_H__

#include "comdat.h"

//--------------------------------------------------------------------------
//  �O�����J�}�N��
//--------------------------------------------------------------------------
#define	SPD_LSB				10							// ���xLSB�̋t��
#define	DEG_LSB				10							// �p�xLSB�̋t��

// �]�[���x���Ԓ�`
enum{
	STS_ZONE_NOALARM = 0,							// ��x����
	STS_ZONE_1100M,									// 1100m�x����
	STS_ZONE_600M,									// 600m�x����
	STS_ZONE_500M,									// 500m�x����
	STS_ZONE_100M,									// 100m�x����
	STS_ZONE_OUTWT_FAR								// ���S���E�҂����
};

#define	LATAREA_MAX			((U1)4)						// �ܓx�G���A�ő�l

//--------------------------------------------------------------------------
//  �^��`																	
//--------------------------------------------------------------------------
// GPS���^
typedef struct{
	U1 	b_sokui  :1; // ����
    U1 	b_spdAcc :1; // ���x�m���炵��(TRUE/FALSE)
    U1 	b_rx 	 :1; // ��M����(motion��clear)
    U1 	b_rsv 	 :5;
	U2 	u2_spd; // ���x[�P��:0.1km/h]
	U2 	u2_deg; // ����[�P��:0.1deg]
	S2 	s2_height; // ���x[�P��:1m]
	U4	lat; //�ܓx
	U4	lon; // �o�x
	U1  u1_vetAccEst; // ���x�m���炵��[0.1m] ��
}stGpsPos;

typedef struct{
	U1 	b_sokui       :1; // ����
	U1 	b_height_ok   :1; // ���x�L��
	U1 	b_spddeg_ok   :1; // ���x���ʗL��
	U1	b_tim_ok 	  :1; // ����OK
	U1 	b_mode		  :1; // GPS���샂�[�h
	U1 	b_rsv         :3;

	stGpsPos st_pos; // uart����pos�f�[�^���i�[

	U4	lat; // �ܓx
	U4	lon; // �o�x
	U2 	u2_spd; // ���x
	U2	u2_deg; // ����
	U1	u1_usesatnum; // ���ʎg�p�q����
	U1	u1_satnum; // �ǔ��q����
	S2 	s2_height; // ���x

	U1 	u1_latarea;	// �ܓx�G���A
	U2 	u2_lonarea;	// �o�x�G���A

	U1	u1_year; // �N
	U1	u1_month; // ��
	U1	u1_day;	// ��
	U1	u1_hour; // ��
	U1	u1_min; // ��
	U1	u1_sec; // �b
}ST_GPSINF;

#define	LATAREA_MAX			((U1)4)						// �ܓx�G���A�ő�l

// POI�ݒ�Z���N�g
enum{
	SEL_POI_NORMAL = 0,
	SEL_POI_TUNNEL,
};

void PoiSample(const char* fname, U2 dataSpec);

#endif


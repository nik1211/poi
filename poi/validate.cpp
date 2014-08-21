
#include "stdafx.h"
#include "validate.h"
#include "crypt.h"

// POIファイル正当性チェック
Bool validate_check_poi_file(const char *fname, U2 dataSpec)
{
#define kBuffSize	512
	FILE *fp;
	U4	rd_size;
	U4	header_sum = 0;
	int size_rem = 0;
	U4	sum = 0;
	U4	addr = 0;
	U4 tmp_buff[kBuffSize/4];

	if (fopen_s(&fp, fname, "rb") != 0) {
		return -1;
	}

	do
	{
		rd_size = fread(tmp_buff, sizeof(unsigned char), kBuffSize, fp);
		if(rd_size > 0)
		{
			int check_size = 0;

			if(addr == 0)
			{
				ST_HEADER *p_header = (ST_HEADER *)tmp_buff;
				if(p_header->mainVer.orbis.u2_orbisDataSpec != dataSpec){
					fclose(fp);
					return FALSE;
				}

				if(u1_ChkHeader(p_header) == OK)
				{
					header_sum = p_header->u4_allChksum;
					size_rem = p_header->u4_endAdr + 1 - p_header->u4_staAdr -(kBuffSize - sizeof(ST_HEADER));
					check_size = rd_size - sizeof(ST_HEADER);
					DataCryptPoi(check_size, sizeof(ST_HEADER), (U1 *)&tmp_buff[sizeof(ST_HEADER)/4], dataSpec);
					addr = kBuffSize;
					sum += u4_CalChkSum(&tmp_buff[sizeof(ST_HEADER)/4], check_size/4);
				}
				else
				{
					fclose(fp);
					return FALSE;
				}
			}
			else
			{
				check_size = rd_size;
				size_rem -= rd_size;
				DataCryptPoi(check_size, addr, (U1 *)tmp_buff, dataSpec);
				sum += u4_CalChkSum(tmp_buff, check_size/4);
				addr += kBuffSize;
			}

		}
		else
		{
			fclose(fp);
			return FALSE;
		}
	}
	while(size_rem > 0);

	fclose(fp);

	if(sum == header_sum)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

//--------------------------------------------------------------------------//
//【関数】ヘッダチェック処理												//
//【機能】ヘッダのサムをチェックする										//
//【備考】																	//
//--------------------------------------------------------------------------//
U1	u1_ChkHeader(ST_HEADER *psth_ptr){

	U1	u1t_ret = NG;
	U4	u4t_checkSum;

	if(psth_ptr->u4_headCode == 0x55AAFFFF){			// ヘッダコードチェック
		u4t_checkSum = u4_CalChkSum((U4 *)psth_ptr, sizeof(ST_HEADER)/sizeof(U4)-1);
		if(u4t_checkSum == psth_ptr->u4_headerChksum){	// ヘッダチェックサム一致なら
			u1t_ret = OK;
		}
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
U4	u4_CalChkSum(U4 *pu4h_ptr, U4 u4h_size){

	U4	u4t_sum = 0;									// チェックサム演算値
	U4	i;												// サイズループカウンタ
	U1	*pu1t_ptr;
	U4	u4t_long;

	if((int)pu4h_ptr & 0x3){							// ミスアライン？
		pu1t_ptr = (U1 *)pu4h_ptr;
		for(i=0; i<u4h_size; i++){
			u4t_long = 	(U4)*pu1t_ptr
				+	(U4)((U4)*(pu1t_ptr+1) << 8)
				+	(U4)((U4)*(pu1t_ptr+2) << 16)
				+	(U4)((U4)*(pu1t_ptr+3) << 24);
			u4t_sum += u4t_long;
			pu1t_ptr += 4;
		}
	}
	else{
		for(i=0; i<u4h_size; i++){
			u4t_sum += *pu4h_ptr;
			pu4h_ptr++;
		}
	}
	return	u4t_sum;
}

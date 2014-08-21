
#include "stdafx.h"
#include "crypt.h"

static Bool readHeader(const char *fname, ST_HEADER* pHeader);
static Bool readIndex1(const char *fname, ST_INDEX1* pIndex1, U4 sampleIdx, U2 dataSpec);
static Bool readPoiData(const char *fname, ST_ROMIMAGE* pRomImage, U4 topAdr, U4 sampleNum, U2 dataSpec);

Bool ReadTest(const char *fname, U2 dataSpec)
{
	ST_HEADER	stt_header;	
	ST_INDEX1	stt_index1;
	ST_ROMIMAGE stt_RomImage;
	U4	sampleIdx = 400;
	U4	i;

	readHeader(fname, &stt_header); 

	readIndex1(fname, &stt_index1, sampleIdx, dataSpec);

	for(i = 0; i < stt_index1.u2_elenum; i++){
		readPoiData(fname, &stt_RomImage, stt_index1.u4_adr, i, dataSpec);
	}

	return TRUE;
}

static Bool readHeader(const char *fname, ST_HEADER* pHeader){
	FILE *fp;

	if (fopen_s(&fp, fname, "rb") != 0) {
		return FALSE;
	}

	// header
	fread(pHeader, sizeof(unsigned char), sizeof(ST_HEADER), fp);

	fclose( fp );

	return TRUE;
}

static Bool readIndex1(const char *fname, ST_INDEX1* pIndex1, U4 sampleIdx, U2 dataSpec){
	FILE *fp;
	U4	u4t_adr;

	if (fopen_s(&fp, fname, "rb") != 0) {
		return FALSE;
	}

	u4t_adr = sizeof(ST_HEADER) + (U4)sizeof(ST_INDEX1) * sampleIdx;
	fseek(fp, u4t_adr, SEEK_CUR);
	fread(pIndex1, sizeof(unsigned char), sizeof(ST_INDEX1), fp);
	DataCryptPoi(sizeof(ST_INDEX1), u4t_adr, (U1*)pIndex1, dataSpec);

	fclose( fp );

	return TRUE;
}

static Bool readPoiData(const char *fname, ST_ROMIMAGE* pRomImage, U4 topAdr, U4 sampleNum, U2 dataSpec){
	FILE *fp;
	U4 u4t_adr = topAdr + sampleNum * sizeof(ST_ROMIMAGE);

	if (fopen_s(&fp, fname, "rb") != 0) {
		return FALSE;
	}

	fseek(fp, u4t_adr, SEEK_CUR);
	fread(pRomImage, sizeof(unsigned char), sizeof(ST_ROMIMAGE), fp);
	DataCryptPoi(sizeof(ST_ROMIMAGE), u4t_adr, (U1*)pRomImage, dataSpec);

	fclose( fp );

	return TRUE;
}


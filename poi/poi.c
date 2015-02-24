// poi.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "validate.h"
#include "gpsctrl.h"

extern Bool ReadTest(const char *fname, U2 dataSpec);

int _tmain(int argc, _TCHAR* argv[])
{
	const char *fname = "10061402.txt";
	U2 dataSpec = 6;	//1006xxxx.txt

	// POI�f�[�^�̑Ó����`�F�b�N�T���v��
	if(validate_check_poi_file(fname, dataSpec) == FALSE){
		return -1;
	}

	// POI�f�[�^�ǂݍ��݃T���v��
	if(ReadTest(fname, dataSpec) == FALSE){
		return -1;
	}

	// ���[�_�[��POI�f�[�^�����T���v��
	PoiSample(fname, dataSpec);

	return 0;
}







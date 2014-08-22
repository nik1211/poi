// poi.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "validate.h"
#include "gpsctrl.h"

extern Bool ReadTest(const char *fname, U2 dataSpec);

int _tmain(int argc, _TCHAR* argv[])
{
	const char *fname = "10061408.txt";
	U2 dataSpec = 6;	//1006xxxx.txt

	if(validate_check_poi_file(fname, dataSpec) == FALSE){
		return -1;
	}

	if(ReadTest(fname, dataSpec) == FALSE){
		return -1;
	}

	PoiSample(fname, dataSpec);

	return 0;
}







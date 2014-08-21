
#ifndef __VALIDATE_H__
#define __VALIDATE_H__

#include "comdat.h"

Bool validate_check_poi_file(const char *fname, U2 dataSpec);
U1	u1_ChkHeader(ST_HEADER *psth_ptr);
U4	u4_CalChkSum(U4 *pu4h_ptr, U4 u4h_size);

#endif


#ifndef DXLMEMMAP_H
#define DXLMEMMAP_H


#include "DxlMemMapConstants.h"

class DxlMemMap {
	private:
	
	char map[255];
	
	public:
	
	DxlMemMap();
	DxlMemMap(const char* jsonFileName);
};

#endif // DXLMEMMAP_H

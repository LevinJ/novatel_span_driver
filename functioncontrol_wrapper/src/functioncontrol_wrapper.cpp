#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <mutex>
#include "moduleControlMgr.h"

using namespace std;




extern "C" {
void (*g_functionPtr)(void *obj, int cmd) = NULL;

ModuleControl::ModuleControlMgr * g_moudleMgr;

// void moduleControlHandler(void* obj, int cmd)
// {
//     return;
    
// }

int init(void (*functionPtr)(void *obj, int cmd)){
	g_functionPtr = functionPtr;
	g_moudleMgr = new ModuleControl::ModuleControlMgr(ModuleControl::DRIVER_USB_LONG_CAM,
          g_functionPtr, NULL);
	// g_functionPtr(NULL, 2);
	return 0;
}



void verify(){
	cout<<"verify called"<<endl;
	// g_moudleMgr->verify();

}


}

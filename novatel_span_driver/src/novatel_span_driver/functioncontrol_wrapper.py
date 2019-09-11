#! /usr/bin/env python
# -*- coding: utf-8 -*-

from  ctypes import *
import time
import os
from enum import Enum 
import rospy



# obj = lib.Foo_new()
# lib.Foo_bar(obj)
# lib.Foo_bar_3(obj, 3)
# lib.Foo_bar_2(obj, b'this is incredible')


# name = b"Frank"
# c_name = ctypes.c_char_p(name)
# foo = lib.hello(c_name)
# # print (c_name.value )# this comes back fine
# print ("return value = {}".format(ctypes.c_char_p(foo).value ))# segfault
# 
# lib.free_mem()
# print('memory freeed!')


CALLBACKFUNCTYPE = CFUNCTYPE(None, c_void_p, c_int)
def callback(obj, cmd):
    global g_fcwrapper
    g_fcwrapper.module_status = ModuleStatus(cmd)
    rospy.logfatal("callback called {}, {}".format(obj, g_fcwrapper.module_status))
    print("callback called {}, {}".format(obj, cmd))
    if (g_fcwrapper.module_status == ModuleStatus.FINISH):
        rospy.logfatal("Module requested to be shut down.")
        rospy.signal_shutdown("Module requested to be shut down.") 
    return

class ModuleStatus(Enum):
    FINISH = 3
    RUNNING = 2
    PENDING = 1
    NOCONTROL = 100



class FunctionControlWrapper(object):
    def __init__(self):

        self.module_status = ModuleStatus.PENDING

        
        return
    def verify(self):
        if self.module_status == ModuleStatus.NOCONTROL:
            return
        self.lib.verify()
        return
    def start(self):
        if self.module_status == ModuleStatus.NOCONTROL:
            return
        lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 
                                                            "../../../../../../../../../devel/lib/libfunctioncontrol_wrapper.so"))
        print("library path={}".format(lib_path))
        lib = cdll.LoadLibrary(lib_path)

        lib.init.argtypes = [CALLBACKFUNCTYPE]
        lib.init.restype = c_int
        
        lib.verify.argtypes = None
        lib.verify.restype = None
        

        self.lib = lib
        print('module status = {}'.format(self.module_status))
        res = self.lib.init(CALLBACKFUNCTYPE(callback))
        print("output of init is {}".format(res))
        return
   
    
    def run(self):
        self.start()
        while True:
            print('module status = {}'.format(self.module_status))
            time.sleep(1)
            self.verify()
            
        return
    
g_fcwrapper = FunctionControlWrapper()
if __name__ == "__main__":   
    obj = g_fcwrapper
    obj.run()
    






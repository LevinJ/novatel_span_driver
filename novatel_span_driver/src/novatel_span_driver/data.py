#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @data.py
#  authors   Mike Purvis <mpurvis@clearpathrobotics.com>
#            NovAtel <novatel.com/support>
#  copyright Copyright (c) 2012, Clearpath Robotics, Inc., All rights reserved.
#            Copyright (c) 2014, NovAtel Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#    following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
# RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
# DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import novatel_msgs.msg

from port import Port
from novatel_span_driver.mapping import msgs
from handlers import MessageHandler
import translator

from cStringIO import StringIO
from threading import Lock
from publisher import g_CalibrateTime


class DataPort(Port):
    def run(self):
        # Set up handlers for translating different novatel messages as they arrive.
        handlers = {}
        pkt_counters = {}

        for msg_id in msgs.keys():
            handlers[msg_id] = MessageHandler(*msgs[msg_id])
            pkt_counters[msg_id] = 0

        bad_pkts = set()
        pkt_id = None
        init_timestamp = True

        while not self.finish.is_set():
            
            header, pkt_str = self.recv()
#             continue
            if header is None:
                continue
            
            rospy.loginfo("processing message {}".format(header.id))
            #initialize the timestamp gap between novatel device and main board
            if init_timestamp:
                gps_stamp = {'gps_week': header.gps_week, 'gps_week_seconds': header.gps_week_seconds}
                g_CalibrateTime.get_time("novatel", gps_stamp)
                init_timestamp = False
                
            if header.id not in handlers:
                rospy.loginfo("skip unexpected novatel message {}".format(header.id))
                continue
                
            if pkt_counters[header.id] >= 65535:
                pkt_counters[header.id] = 0
            pkt_counters[header.id] += 1
            header.sequence = pkt_counters[header.id]
            
            handlers[header.id].handle(StringIO(pkt_str), header)

            
                    
            
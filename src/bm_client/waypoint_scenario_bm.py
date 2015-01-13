#!/usr/bin/env python
"""
   waypoint_scenario_bm - reproduces scenario from bonnmotion traces
   
    Copyright (C) 2014 Eduardo Feo
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import lcm
from poselcm import pose_list_t
from poselcm import pose_t
import sys
import time
from xml.dom import minidom
from optparse import OptionParser
#############################################################
#############################################################
class WaypointScenarioBM():
#############################################################
#############################################################

    #############################################################
    def __init__(self, scenario, traces):
    #############################################################
        if not scenario:
            print "Error: Missing scenario file"
            print scenario
            exit(1)
        print "Reading scenario from ",scenario
        self.configureScenario(scenario)
        #self.x_shift = rospy.get_param("~x_shift", 0.0)
        #self.y_shift = rospy.get_param("~y_shift", 0.0)
        self.x_shift = 0
        self.y_shift = 0
        self.ticks_since_start=0
        self.rate = 1
        #traces = rospy.get_param("~bm_traces", None)
        if not traces:
            print "Error: Missing trace file"
            exit(1)
        print "reading traces from",traces
        self.readTraces(traces)
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1") 
        
    def shutdown(self):
        return False
    #############################################################
    def spin(self):
    #############################################################
        ###### main loop  ######
        while not self.shutdown():
            self.spinOnce()
            self.ticks_since_start+=1
            time.sleep(1)
             
    #############################################################
    def readTraces(self, tfile):
        f = open(tfile)
        self.wps = []
        for line in f.readlines():
            if len(self.wps) >= len(self.robots):
                break
            wplist=[]
            s = line.split()
            i=0
            while i<len(s):
                wplist.append( (float(s[i]), float(s[i+1]),float(s[i+2])) )
                i+=3
            print "read ",len(wplist)," traces for robot ",self.robots[len(self.wps)]
            self.wps.append(wplist)


    #############################################################
    def configureScenario(self, cfile):
        xmldoc = minidom.parse(cfile)
        itemlist = xmldoc.getElementsByTagName('robot') 
        print len(itemlist)
        self.goal_pub = []
        self.robot_to_ix = {}
        self.robots = []
        ix = 0
        for s in itemlist :
            rid = int(s.attributes['robotid'].value)
            self.robot_to_ix[rid] = ix
            self.robots.append(rid)
            ix+=1


    def sendWP(self,i,x,y):
        print "sending to ",i
        msg = pose_list_t()
        msg.timestamp = int(time.time() * 1000000)
        pose = pose_t()
        pose.robotid = i
        pose.position = [x,y,0]
        pose.orientation = [0,0,0,1]
        msg.poses = [pose]
        msg.n = 1
        self.lc.publish("TARGET", msg.encode())
    #############################################################

    #############################################################
    def spinOnce(self):
    #############################################################
        tt = self.ticks_since_start * (1.0/self.rate)
        for i in range(len(self.wps)):
            wpl = self.wps[i]
            if not len(wpl):
                continue
            (t,x,y) = wpl[0]
            if tt >= t:
                self.sendWP(i,x,y)
                #goalmsg = PoseStamped()
                #goalmsg.header.frame_id = "/world"
                #goalmsg.header.stamp = rospy.get_rostime()
                #goalmsg.pose.position.x = x + self.x_shift
                #goalmsg.pose.position.y = y + self.y_shift
                #self.goal_pub[i].publish(goalmsg)
                #print "wp ",goalmsg.pose.position.x," ",goalmsg.pose.position.y," sent to ",self.names[i]
                self.wps[i]=wpl[1:]
    
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    bmscene = WaypointScenarioBM(sys.argv[1], sys.argv[2])
    bmscene.spin()

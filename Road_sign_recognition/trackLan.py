#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

import numpy as np
#from home.env.opencv2.lib.python2_7.site_packages import cv2
import cv2
#import serial

# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import time
# import copy
import video
import motor
# from matplotlib import pyplot as plt

class App(object):
    def __init__(self, color):
        print "Step2: __init__"
        
        self.err_last = 0	#定义上一个偏差值
        self.Kp = 0.2		#定义比例系数
        self.Ki = 0.015		#定义积分系数
        self.Kd = 0.2		#定义微分系数
        self.integral = 0    #定义积分值

        self.F_err_last=0.0
        self.F_integral=0.0
        self.F_Kp=1.0
        self.F_Ki=0.0
        self.F_Kd=0.002
        
        self.cam = video.create_capture(0)
#         self.cam = PiCamera()
#         self.cam.resolution = (320,240)
#         self.cam.framerate = 32
#         self.rCa = PiRGBArray(self.cam, size=(320,240))
#         time.sleep(0.1) 
#        self.cam.capture(self.rCa, format='bgr')
#        self.frame = self.rCa.array
#        ret, self.frame = self.cam.read()
        cv2.namedWindow('camshift')
        if color == 0:
            print "Step3: __init__ if"
            
            self.roi = cv2.imread( 'hong.jpg' )
            self.flag = "Hong"
        else :
            print "Step4: __init__ else"
            
            self.flag = "Lan"
            self.roi = cv2.imread('lan.jpg')
            self.selection = None
            self.tracking_state = 0
            self.show_backproj = False
#         self.ser = serial.Serial('/dev/ttyAMA0',115200,timeout=0.5)

    def PID_init(self):
        print("PID_init begin")
	
        self.err_last=0.0
        self.integral=0.0
        self.Kp=0.002
        self.Ki=0.0
        self.Kd=0.002

        self.F_err_last=0.0
        self.F_integral=0.0
        self.F_Kp=0.010
        self.F_Ki=0.0
        self.F_Kd=0.002
	
        print("PID_init end")

    #PID转向控制算法的计算过程
    def PID_Center(self, actual, target):
        print "PID_Center start"
        #计算偏差
        err = target - actual
        print "err"
        #计算离散的积分项
        self.integral = self.integral + err
        print "integral"
        #离散的PID算法计算方法
        result = self.Kp * err + self.Ki * self.integral + self.Kd * (err - self.err_last)
        #更新差值
        self.err_last = err
        print "PID_Center end"
        return result

    #PID前进后退算法的计算过程
    def PID_Forward(self, actual, target):
        print "PID_Forward start"
        #计算偏差
        err = target - actual
        print "err"
        #计算离散的积分项
        self.F_integral = self.F_integral + err
        print "integral"
        #离散的PID算法计算方法
        result = self.F_Kp * err + self.F_Ki * self.F_integral + self.F_Kd * (err - self.F_err_last)
        #更新差值
        self.F_err_last = err
        print "PID_Forward end"
        return result

    def start(self):
        print "Step7: start"
        
        self.tracking_state = 0
        #x, y = np.int16([220, 110]) # BUG
        if self.flag == 'Hong':
            
            print "Step8: start if"
            
            self.selection = (4, 6, 407, 304)
        else:
            
            print "Step9: start else"
            
            self.selection = (40, 54, 296, 230)
            self.tracking_state = 1
    #        print "start"

    def show_hist(self):
        bin_count = self.hist.shape[0] #
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
#        cv2.imshow('hist', img)

    def run(self):
        print "Step6: run"
        roi = self.roi
        self.start()
        while True:
            print "Step10: run while"
            
#         for frame in self.cam.capture_continuous(self.rCa, format='bgr', use_video_port=True):
            ret, self.frame = self.cam.read()
#             self.frame = frame.array
            vis = self.frame.copy()
#             vis = copy.deepcopy(self.frame)
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.))) #创建一个遮罩mask
#             self.selection = 1

            if self.selection:
                print "Step10: run while if selection"
#                 x0, y0, x1, y1 = 220, 110, 358, 245
                x0, y0, x1, y1 = self.selection
                self.track_window = (x0, y0, x1-x0, y1-y0)
#                 hsv_roi = hsv[y0:y1, x0:x1]
#                 mask_roi = mask[y0:y1, x0:x1]
                hsv_roi = cv2. cvtColor(roi,cv2. COLOR_BGR2HSV)
                mask_roi = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
                #一维直方图
                hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                #二维直方图
#                 hist = cv2.calcHist( [hsv_roi], [0,2],None, [180,256], [0, 180,0 , 255] )
                
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
                self.hist = hist.reshape(-1)
                #二维直方图显示
#                 plt.imshow(hist,interpolation = 'nearest')
#                 plt.show()
#                self.show_hist()

                vis_roi = vis[y0:y1, x0:x1]
                cv2.bitwise_not(vis_roi, vis_roi)
                vis[mask == 0] = 0

            if self.tracking_state == 1:
                print "Step11: run while if tracking_state"
                
                self.selection = None
                prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
                prob &= mask
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
#                 if track_box[0][1] <= 240:
#             self.ser.write(str(int(track_box[0][0])-320) + " " + str(int(track_box[0][1])-240))
#             print str(int(track_box[0][0])-320) + " " + str(int(track_box[0][1])-240)
                if track_box[1][1] <= 1:
                    print "Step12: run while if tracking_state if"
                    
                    self.tracking_state = 0
                    self.start()
                    motor.walk(0,0)
                else:
                    print "Step13: run while if tracking_state else"
                    
                    if self.show_backproj:
                        vis[:] = prob[...,np.newaxis]
                    try:
                        print "Step14: run while if tracking_state try"
                        
                        cv2.ellipse(vis, track_box, (0, 0, 255), 2)
#                         print track_box
                        a = str(track_box[0][0])+" "+str(track_box[0][1])+" "+str(round(track_box[1][0],2))\
                                       +" "+str(round(track_box[1][1],2))+" "+str(round(track_box[2],2))+"\r\n"
                        print a
                        
                        ActualCenterX = track_box[0][0]
                        print "try1"
                        TargetCentorX = 160
                        print "try2"
                        SetX = self.PID_Center(ActualCenterX, TargetCentorX)
                        
                        ActualForward = (track_box[1][0] + track_box[1][1])/2
                        TargetForward = 80
                        if ActualForward < 80:
                            ActualForward = 1.732 * ActualForward - 58.56
                        
                        SetF = self.PID_Forward(ActualForward, TargetForward)
                        
                        print "try3"
                        
                        motor.walk(-SetX+SetF, SetX+SetF, 5)
                        #motor.walk(SetF, SetF)
                        
                        print "try4"
                        #b = "AcutualX = "+str(ActualCenterX)+" TargetX = "+str(TargetCentorX)+" SetX = "+str(SetX)
                        #print b
                        #print SetX
                        
#                         self.ser.write(a)
                    except:
                        print "Error"
                        #print track_box

            cv2.imshow('camshift', vis)

            ch = 0xFF & cv2.waitKey(5)
            if ch == 27:
                break
            if ch == ord('b'):
                self.show_backproj = not self.show_backproj
            if ch == ord('r'):
                self.tracking_state = 0
                self.start()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    try: color = sys.argv[1]
    except: color = 1
    print __doc__
    print "Step1: __name__ App"
    a = App(color)
    print "Step5: __init__ run"
    a.run()

    

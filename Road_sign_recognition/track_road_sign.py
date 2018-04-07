#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

import numpy as np
#from home.env.opencv2.lib.python2_7.site_packages import cv2
import cv2
import traceback
import tensorflow as tf
import time
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
        #print "Step2: __init__"
        
        self.err_last = 0    #定义上一个偏差值
        self.Kp = 0.2        #定义比例系数
        self.Ki = 0.015        #定义积分系数
        self.Kd = 0.2        #定义微分系数
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
        #cv2.namedWindow('camshift')
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
            self.label_list = []
#         self.ser = serial.Serial('/dev/ttyAMA0',115200,timeout=0.5)

    def PID_init(self):
        #print("PID_init begin")
    
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
    
        #print("PID_init end")

    #PID转向控制算法的计算过程
    def PID_Center(self, actual, target):
        #print "PID_Center start"
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
    
    def get_more(self, arr):  
        mode = [];  
        arr_appear = dict((a, arr.count(a)) for a in arr);  # 统计各个元素出现的次数  
        if max(arr_appear.values()) == 1:  # 如果最大的出现为1  
            return -1;  # 则没有众数  
        else:  
            for k, v in arr_appear.items():  # 否则，出现次数最大的数字，就是众数  
                if v == max(arr_appear.values()):  
                    mode.append(k);  
            return mode[-1]; 

    def start(self):
        #print "Step7: start"
        
        self.tracking_state = 0
        #x, y = np.int16([220, 110]) # BUG
        if self.flag == 'Hong':
            
            print "Step8: start if"
            
            self.selection = (4, 6, 407, 304)
        else:
            
            #print "Step9: start else"
            
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
        
        graph = tf.Graph()

        with graph.as_default():
            images_ph = tf.placeholder(tf.float32, [None, 32, 32, 3])
            labels_ph = tf.placeholder(tf.int32, [None])

            images_flat = tf.contrib.layers.flatten(images_ph)

            logits = tf.contrib.layers.fully_connected(images_flat, 100, tf.nn.relu)

            predicted_labels = tf.argmax(logits, 1)

            loss = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(logits=logits, labels=labels_ph))

            train = tf.train.AdamOptimizer(learning_rate=0.001).minimize(loss)
    
            saver = tf.train.Saver()

            init = tf.initialize_all_variables()
    
        print("images_flat: ", images_flat)
        print("logits: ", logits)
        print("loss: ", loss)
        print("predicted_labels: ", predicted_labels)
                        
        session = tf.Session(graph=graph)
        saver.restore(session, 'model/road_model.ckpt')
        
        roi = self.roi
        self.start()
        time.sleep(10)
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
                #print "Step10: run while if selection"
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
                #print "Step11: run while if tracking_state"
                
                self.selection = None
                prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
                prob &= mask
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
#                 if track_box[0][1] <= 240:
#             self.ser.write(str(int(track_box[0][0])-320) + " " + str(int(track_box[0][1])-240))
#             print str(int(track_box[0][0])-320) + " " + str(int(track_box[0][1])-240)
                if track_box[1][1] <= 200:
                    #print "Step12: run while if tracking_state if"
                    
                    self.tracking_state = 0
                    self.start()
                    motor.walk(10,10)
                else:
                    #print "Step13: run while if tracking_state else"
                    
                    if self.show_backproj:
                        vis[:] = prob[...,np.newaxis]
                    try:
                        #print "Step14: run while if tracking_state try"
                        
                        cv2.ellipse(vis, track_box, (0, 0, 255), 2)
#                         print track_box
                        imageX = track_box[0][0]
                        imageY = track_box[0][1]
                        imageR = (track_box[1][0] + track_box[1][1]) / 4
                        visCut = vis[int(imageY-imageR):int(imageY+imageR), int(imageX-imageR):int(imageX+imageR)]
                        imageSize = (int(100), int(100))
                        #visResize = cv2.resize(visCut, imageSize, interpolation=cv2.INTER_AREA)
                        visResize = cv2.resize(visCut, imageSize)
                        visResize = cv2.medianBlur(visResize,5)
                        
                        
                        print "Step15: shift image "
                        
                        image32 = []
                        #image = cv.imread('test.jpg')
                        image = cv2.resize(visResize, (32, 32))
                        image32.append(image)
                        np.array(image32).shape
                        
                        #print "Step16: run predict " 
                        pred_label = session.run([predicted_labels], 
                                           feed_dict={images_ph: image32})[0][0]
                                             
                        print("predicted:" + str(pred_label))
                        time.sleep(1)
                        self.label_list.append(pred_label)
                        print(self.label_list)
                        
                        if len(self.label_list) > -1:
                            pred_label = self.get_more(self.label_list)
                            print pred_label
                            if  pred_label == 0:         #left  road sign
                                motor.walk(-40, 40, 0)
                                time.sleep(0.8)
                            elif pred_label == 1:        #right  road sign
                                motor.walk(40, -40, 0)
                                time.sleep(0.8)
                            elif pred_label == 2:        #return  road sign
                                motor.walk(0, 40, 0)
                                time.sleep(1.5)  
                            elif pred_label == 3:        #people  road sign
                                motor.walk(20, 20, 0)
                                time.sleep(2) 
                            elif pred_label == 4:        #stop  road sign
                                motor.walk(0, 0, 0)
                                time.sleep(2)
                             elif pred_label == 4:       #circle  road sign
                                motor.walk(20, 10, 0)
                                time.sleep(0.5)
                                motor.walk(10, 15, 0)
                                time.sleep(1.5)
                                motor.walk(20, 10, 0)
                                time.sleep(0.5)                                 
                            else:
                                motor.walk(30, 30)
                            self.label_list = []
                            
                        
                        a = str(track_box[0][0])+" "+str(track_box[0][1])+" "+str(round(track_box[1][0],2))\
                                       +" "+str(round(track_box[1][1],2))+" "+str(round(track_box[2],2))+"\r\n"
                        
                        
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

                        
                        #motor.walk(-SetX+SetF, SetX+SetF, 5)
                        #motor.walk(SetF, SetF)
                        
                        #b = "AcutualX = "+str(ActualCenterX)+" TargetX = "+str(TargetCentorX)+" SetX = "+str(SetX)
                        #print b
                        #print SetX
                        
#                         self.ser.write(a)
                    except:
                        traceback.print_exc()

            #cv2.imshow('camshift', vis)

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

    

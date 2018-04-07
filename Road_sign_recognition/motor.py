#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#小车电机引脚定义
#左边引脚
IN1 = 20
IN2 = 21
#右边引脚
IN3 = 19
IN4 = 26

#小车PWM引脚定义
ENA = 16
ENB = 13

#初始化状态标志
MOTOR_INIT_FLAG = 0

#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

#忽略警告信息
GPIO.setwarnings(False)

#电机引脚初始化为输出模式
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    #设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

def cleanup():
    pwm_ENA.stop()
    pwm_ENB.stop()
    
#小车行走合成控制
def walk(leftspeed, rightspeed, thresholdspeed = 0):
    global MOTOR_INIT_FLAG
    #运行前自动初始化电机引脚
    if MOTOR_INIT_FLAG == 0:
        init()
        #延时2s    
        time.sleep(2)
        MOTOR_INIT_FLAG = 1
        
    if leftspeed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif leftspeed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else :
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    if rightspeed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif rightspeed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

    if leftspeed < 0:
        leftspeed = -leftspeed
    if rightspeed < 0:
        rightspeed = -rightspeed
        
    if leftspeed > 100:
        leftspeed = 100
    if rightspeed > 100:
        rightspeed = 100
        
    if leftspeed < thresholdspeed:
        leftspeed = 0
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    if rightspeed < thresholdspeed:
        rightspeed = 0
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

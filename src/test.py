#!/usr/bin/env python
from std_msgs.msg import Int32, Int32MultiArray, String
import rospy
from audio_common_msgs.msg import AudioData
import contextlib    
import wave
import json
from threading import Thread
import struct
from test_thomas.srv import * 

class ROSSide():
    def __init__(self):
        rospy.init_node('teleop', anonymous=True)

        self.buff = None
        self.counter = 0
        self.motors = self.load_json('../data/motors.json')
        self.init_motors_values()

        self.motor = [2, 0, 50]

        #self.motors = {'neck_h': 0, 'neck_v': 1, 'left_arm_h': 2, 'left_arm_v': 3,
        #'left_elbow': 4, 'right_arm_h': 5, 'right_arm_v ': 6, 'left_elbow': 7}

        self.pub_audio = rospy.Publisher('teleop/audio', AudioData, queue_size=10)
        self.pub_movement = rospy.Publisher('qt_movement/localMovement', Int32MultiArray, queue_size=10)
        #rospy.Service('increment_motor', IncrementMotor, self.increment_callback)
        self.listener()

    def init_motors_values(self):
        for key, values in self.motors.iteritems():
            values['value'] = values['home']

    def increment_motor_callback(self, msg):

        print msg.data

        tmp = msg.data.split()

        mode = tmp[0]

        motor =  self.motors.get(tmp[1])
        if motor is None:
            return

        value = float(tmp[2])

        if mode == 'p': #Percentage
            if value > 100 or value < 0:
                return
            motor['value'] = (value/100)*(motor['max']-motor['min'])+motor['min']
        elif mode == 'i': #Increment
            motor['value'] += value
            if motor['value'] > motor['max']:
                motor['value'] = motor['max']
            elif motor['value'] < motor['min']:
                motor['value'] = motor['min']
        self.move_motor(motor['id'], motor['value'])

    def move_motor(self, motor, value):
        print 'publishing {} to the motor {}'.format(value, motor)
        self.pub_movement.publish(Int32MultiArray(data=[motor, value, 50]))


    def load_json(self, path):
        with contextlib.closing(open(path)) as json_data:
            return json.loads(json_data.read())

    def set_motor_value(self, value):
        #if value > 30:
        #    value = 30
        #if value < -30:
        #    value = -30

        self.motor[1] = value

    def motor_callback(self, msg):
        self.set_motor_value(msg.data)

        self.pub_movement.publish(Int32MultiArray(data=self.motor))

    def audio_callback(self, msg):

        
        if self.buff is None:
            self.buff = msg.data
        else:
            self.buff += msg.data
            self.counter += 1

        if self.counter == 200:
            self.counter = 0
            self.tmp_buff = self.buff
            self.buff = None
            Thread(target=self.send_audio).start()

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    def send_audio(self):
    
        #print bytearray(buff)
        #print len(buff)
        #print len(buff)/200
        with contextlib.closing(wave.open("test.wav", 'wb')) as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(bytearray(self.tmp_buff))

        tmp = []
        with contextlib.closing(open("test.wav", 'rb')) as wf:
            ba = wf.read()
        for c in ba:
            #print c
            value = struct.unpack('B', c)[0]
            #print value
            tmp.append(value)
        self.pub_audio.publish(AudioData(tmp))
        
        self.tmp_buff = None
        print 'sent'
        
        
    def listener(self):

        rospy.Subscriber("teleop/motor", Int32, self.motor_callback)
        rospy.Subscriber("audio", AudioData, self.audio_callback)
        rospy.Subscriber("teleop/increment/motor", String, self.increment_motor_callback)
        rospy.spin()

if __name__ == '__main__':
    
    

    #import herkulex
    #from herkulex import servo

    #connect to the serial port
    #herkulex.connect("/dev/ttyUSB0", 115200)

    #scan for servos, it returns a tuple with servo id & model number
    #servos = herkulex.scan_servos()

    #print servos
    ROSSide()
    
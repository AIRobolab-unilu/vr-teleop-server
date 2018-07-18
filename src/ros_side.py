#!/usr/bin/env python
from std_msgs.msg import Int32, Int32MultiArray, String
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2 as pc2
import rospy
from audio_common_msgs.msg import AudioData
import contextlib    
import wave
import json
from threading import Thread
import struct
from vr_teleop_server.srv import * 
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ROSSide():
    def __init__(self):
        rospy.init_node('teleop', anonymous=True)

        self.buff = None
        self.counter = 0
        self.speed = 1000
        self.motors = self.load_json('../data/motors.json')
        self.init_motors_values()

        self.motor = [2, 0, 50]
        self.bridge = CvBridge()

        #self.motors = {'neck_h': 0, 'neck_v': 1, 'left_arm_h': 2, 'left_arm_v': 3,
        #'left_elbow': 4, 'right_arm_h': 5, 'right_arm_v ': 6, 'left_elbow': 7}

        self.pub_audio = rospy.Publisher('teleop/audio', AudioData, queue_size=10)
        self.pub_info = rospy.Publisher('teleop/motors/info', String, queue_size=10)
        self.pub_movement = rospy.Publisher('qt_movement/localMovement', Int32MultiArray, queue_size=10)
        #rospy.Service('increment_motor', IncrementMotor, self.increment_callback)
        self.listener()

        self.run()

    def run(self):
        rate = rospy.Rate(4) # 2z
        while not rospy.is_shutdown():


            #for key, values in self.motors.iteritems():

                #values['value'] = values['home'] 

            self.pub_info.publish('{}:{}:{}:{}:{}:{}:{}:{}'.format(
                self.motors['neck_h']['value'], self.motors['neck_v']['value'], self.motors['left_arm_h']['value'], self.motors['left_arm_v']['value'],
                self.motors['left_elbow']['value'], self.motors['right_arm_h']['value'], self.motors['right_arm_v']['value'], self.motors['left_elbow']['value']))


            rate.sleep()

    def process_cloud(self, msg):

        points = list(pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True))

        point = points[len(points)//2]
        x, y, z = point
        print '------'
        print x
        print y
        print z

        #for point in points:
            #x, y, z = point
            #print '------'
            #print x
            #print y
            #print z
            #break

        print 'got it'

    def process_depth(self, msg):
        #x : offset: 0 datatype: FLOAT32 count: 1
        #y : offset: 4 datatype: FLOAT32 count: 1
        #z : offset: 8 datatype: FLOAT32 count: 1
        #rgb : offset: 16 datatype: FLOAT32 count : 1

        #Length of a point in bytes : 32
        #Length of a row in bytes : 20480

        #height : 480
        #width : 640

        #points = pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)

        #for point in points:
            #x, y, z = point
            #print '------'
            #print x
            #print y
            #print z

        #print 'got it'
        #print msg.point_step
        #print msg.row_step
        #print msg.fields


        #Encoding of pixels : 16UC1
        #Full row length in bytes : 1280

        msg.encoding = 'mono16'
        print msg.step

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        #print msg.data
        print type(cv_image)
        print cv_image[:]
        cv2.imwrite('test.png', cv_image)

    def init_motors_values(self):
        for key, values in self.motors.iteritems():
            values['value'] = values['home']

    def increment_motor_callback(self, msg):

        print "motor received " + str(msg.data)

        tmp = msg.data.split()

        mode = tmp[0]

        motor =  self.motors.get(tmp[1])
        if motor is None:
            return

        value = float(tmp[2])
        new_value = motor['value']

        if mode == 'p': #Percentage
            if value > 100 or value < -100:
                return
            new_value = (value/100)*(motor['max']-motor['min'])/2
        elif mode == 'i': #Increment
            new_value += value
            if new_value > motor['max']:
                new_value= motor['max']
            elif new_value < motor['min']:
                new_value = motor['min']
        elif mode == 'a': #Absolue
            new_value = value
            if new_value > motor['max']:
                new_value = motor['max']
            elif new_value < motor['min']:
                new_value = motor['min']
        
        #Discard the value if the movemebt is too litlle, to avoid overflooding
        if abs(new_value-motor['value'])<5:
            #print 'Discard the command'
            return

        motor['value'] = new_value
        self.move_motor(motor['id'], motor['value'])

    def move_motor(self, motor, value):

        print 'publishing {} to the motor {}'.format(value, motor)
        self.pub_movement.publish(Int32MultiArray(data=[motor, value, self.speed]))


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
        #print 'sent'
        
        
    def listener(self):

        rospy.Subscriber("teleop/motor", Int32, self.motor_callback)
        rospy.Subscriber("audio", AudioData, self.audio_callback)
        rospy.Subscriber("teleop/increment/motor", String, self.increment_motor_callback)
        #rospy.Subscriber("camera/depth/image_rect_raw", Image, self.process_depth)
        rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.process_cloud)

        #print "ok"
        #rospy.spin()

if __name__ == '__main__':
    
    

    #import herkulex
    #from herkulex import servo

    #connect to the serial port
    #herkulex.connect("/dev/ttyUSB0", 115200)

    #scan for servos, it returns a tuple with servo id & model number
    #servos = herkulex.scan_servos()

    #print servos
    ROSSide()
    
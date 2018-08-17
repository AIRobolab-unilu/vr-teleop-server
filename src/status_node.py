#!/usr/bin/env python



from std_msgs.msg import String


import psutil
import os

import rospy


class Status():

    def __init__(self):

        rospy.init_node('status', anonymous=True)

        self.listener()

        self.status_dialog = ""
        self.status_motors = ""

        self.pub = rospy.Publisher('teleop/status', String, queue_size=10)
        
        self.service_error = False
        try:
            from motivational_component.srv import * 
        except ImportError:
            print 'Please install the package "motivational_component"'
            self.service_error = True

        if not self.service_error:
            try:
                self.variable_info = rospy.ServiceProxy('variable_info', VariableInfo)
                self.variable_info('pain')#test the call for error
            except:
                self.service_error = True

        self.run()

    def listener(self):
        rospy.Subscriber("speaker/info", String, self.status_dialog_callback)
        rospy.Subscriber("teleop/motors/info", String, self.status_motors_callback)
        #rospy.Subscriber("speaker/info", String, self.status_dialog_callback)
        #rospy.spin()

    def status_dialog_callback(self, msg):
        self.status_dialog = msg.data

    def status_motors_callback(self, msg):
        self.status_motors = msg.data

    def get_status_hardware(self):
        #print os.statvfs()

        #print "##########"

        return "{}:{}".format(psutil.cpu_percent(), psutil.virtual_memory().percent)

    def format(self, name):
        return '{}/{}/{}/{}/{}/{}'.format(name, self.variable_info(name).min, self.variable_info(name).low_limit,
                                            self.variable_info(name).value, self.variable_info(name).high_limit,
                                            self.variable_info(name).max)


    def get_status_emotions(self):
        if not self.service_error:

            status = '{}:{}:{}:{}'.format(self.format("pain"), self.format("curiosity"),
                                 self.format("frustration"), self.format("fatigue"))

        #print status

            return status
        return 'no package'

    def run(self):

        rate = rospy.Rate(2) # 2z
        print 'ok'
        while not rospy.is_shutdown():

            status_hardware = self.get_status_hardware()
            status_emotions = self.get_status_emotions()

            msg = "{}\n{}\n{}\n{}".format(status_emotions, status_hardware, self.status_motors, self.status_dialog)

            #msg = 'ok'

            self.pub.publish(msg)

            #print msg

            #print '###########'


            rate.sleep()


if __name__ == '__main__':
    Status()
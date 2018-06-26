/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include <sstream>

#include <yarp/os/all.h>
using namespace yarp::os;

#include <string>
#include <sstream>

#include <time.h>       /* time */

//////////////////////////////// 
//////////////////////////////// 

int v_moveMode;
int v_neckposition;


int V_MODE_RANDOM = 9;
int V_MODE_STOP = 3;
int V_MODE_CONTROLLED = 4;


int YAW_MOTOR = 0;
int PITCH_MOTOR = 1;
//////////////////////////////// 
////////////////////////////////

int v_moveBODY;


void movModeCallback(const std_msgs::Int8::ConstPtr& msg)
{
  
  v_moveMode = msg->data;
  ROS_INFO("v_moveMode: [%d]",v_moveMode );

}


void headPositionCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("Head Position: [%d]", msg->data);
  v_neckposition = msg->data;
}


void timerCallback(const ros::TimerEvent& event)
{
    ROS_INFO("Temporal Callback");
    v_moveBODY = 1;
    
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    int v_position;
    int v_show_face = 0;
    v_moveBODY = 0;
    v_moveMode = 4;
    
    /* initialize random seed: */
    srand (time(NULL));
        
    v_neckposition = -1;    
    yarp::os::Network yarpInst;
  
    ros::init(argc, argv, "game");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher face_pub = n.advertise<std_msgs::String>("/qt_face/setEmotion", 1000);
    ros::Subscriber sub_head_mov_mode = n.subscribe("head_mov_mode", 1000, movModeCallback);
    ros::Subscriber sub_head_position = n.subscribe("q_face_detected", 1000, headPositionCallback);
    
    ros::Publisher neck_pub = n.advertise<std_msgs::Int32MultiArray>("/qt_movement/localMovement", 1000);
    
    ros::Publisher neck_inc_pub = n.advertise<std_msgs::Int32MultiArray>("/qt_movement/incrementMotorCallback", 1000);
    
    ros::Publisher play_sound = n.advertise<std_msgs::String>("/qt_playsound/play_song", 1000);
	
    ros::Publisher voice_pub = n.advertise<std_msgs::String>("/qt_tts/say", 10);

    ros::Timer timer = n.createTimer(ros::Duration(30), timerCallback);
    
    
// %Tag(LOOP_RATE)%
    ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

   
   
    std_msgs::String msg;
 
  std::string action = "head_t_1";
  
  int v_num_pos=0;
  
  int position = 0, speed = 50;
  int neck_motor = YAW_MOTOR;
  int increment = 10;
  int new_movement = 0;
  int value = -1;
  int combined_neck_position [2];

  int last_human_position=0;
  int human_position=0;

  while (ros::ok())
  {
    
    ros::spinOnce();



    loop_rate.sleep();

    ROS_INFO("Mode %d", v_moveMode);
    
    
   
    if (v_moveMode == V_MODE_RANDOM)
    {
        /* generate secret number between 1 and 10: */
        v_position = rand() % 6;
        
        if( v_show_face == 5 )
        {   if(v_position %2 == 0)
                msg.data = "neutral_smile";
            else if(v_position %5 == 0)
                msg.data = "happy";
            else
                msg.data = "blinking";
                    
            face_pub.publish(msg);
            v_show_face = 0;
        }
        
        ros::Duration(1.0).sleep();
        v_num_pos = 0; 
        v_show_face ++;
    }
    else if (v_moveMode == V_MODE_CONTROLLED)
    {
        /*Yarp Command*/
        std_msgs::Int32MultiArray array;
        std_msgs::Int32MultiArray array2;
        //Clear array
        array.data.clear();
        array2.data.clear();

        array.data.push_back(neck_motor);
        array.data.push_back(position);
        array.data.push_back(speed);
        ROS_INFO("I publish: [%d] [%d] [%d]", array.data[0], array.data[1], array.data[2]);
        //Publish array
        neck_pub.publish(array);
           
          
   
//             AVA_QT ROBOT - Safe Mode

        if ((position == 30) and (new_movement == 0))
        {
            increment *= -1; 
            new_movement = 1;
        }
        else if ((position == -30) and (new_movement == 0))
        {
            //Publish array
            increment *= -1; 
            new_movement = 1;
        }
        else
        {
            position += increment;
            new_movement = 0;
        }

       
        ros::Duration(2.5).sleep();
       
    }
    else if (v_moveMode == V_MODE_STOP)
    {
       v_num_pos = 0; 
    }
  }


  return 0;
}

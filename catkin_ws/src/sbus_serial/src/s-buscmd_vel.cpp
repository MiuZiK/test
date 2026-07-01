/*
* Copyright 2018 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* SBUS -> cmd_vel node
*
* Subscribes to:
*	/sbus (Sbus)
* Publishes:
*	/output/sbus/cmd_vel (Twist)
*/

#include "ros/ros.h"
#include <sbus_serial/Sbus.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#define FORWARD_CHANNEL_INDX 1       // Channel 2 (elevator)
#define TURN_CHANNEL_INDX 0          // Channel 1 (ailerons)

// Publisher and parameters are in global scope so callback function can use them
ros::Publisher cmdVelPublisher;
int sbusMinValue;
int sbusMaxValue;
int sbusRange;          // Calculated when reading min/max
double maxSpeed;        // m/sec
double maxTurn;         // radians/sec
bool invertTurn;        // Whether to invert the turn channel
double deadzone;        // Deadzone for both channels

// Function to apply deadzone to a value
double applyDeadzone(double value, double deadzone) {
    if (fabs(value) < deadzone) {
        return 0.0;
    }
    return value;
}

void sbusCallback( const sbus_serial::Sbus::ConstPtr& msg )
{
    // 打印所有通道的值（每2秒一次）
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() > 2.0) {
        ROS_INFO("All channels: %d, %d, %d, %d, %d, %d, %d, %d",
                 msg->mappedChannels[0], msg->mappedChannels[1],
                 msg->mappedChannels[2], msg->mappedChannels[3],
                 msg->mappedChannels[4], msg->mappedChannels[5],
                 msg->mappedChannels[6], msg->mappedChannels[7]);
        last_print = ros::Time::now();
    }

    double proportional;
    geometry_msgs::Twist twist;

    // 前进通道处理
    proportional = static_cast<double>(msg->mappedChannels[ FORWARD_CHANNEL_INDX ] - sbusMinValue) / sbusRange;
    double fwdValue = (proportional * 2 - 1.0);  // 映射到[-1, 1]
    fwdValue = applyDeadzone(fwdValue, deadzone);  // 应用死区
    double fwdSpeed = maxSpeed * fwdValue;
    twist.linear.x = fwdSpeed;

    // 转向通道处理
    proportional = static_cast<double>(msg->mappedChannels[ TURN_CHANNEL_INDX ] - sbusMinValue) / sbusRange;
    double turnValue = (proportional * 2 - 1.0);  // 映射到[-1, 1]
    turnValue = applyDeadzone(turnValue, deadzone);  // 应用死区
    
    // Apply inversion if needed
    if (invertTurn) {
        turnValue = -turnValue;
    }
    
    double turn = maxTurn * turnValue;
    twist.angular.z = turn;

    // 添加调试输出（每0.5秒一次）
    static ros::Time last_turn_print = ros::Time::now();
    if ((ros::Time::now() - last_turn_print).toSec() > 0.5) {
        ROS_INFO("Forward: raw=%d, value=%.2f, speed=%.2f", 
                 msg->mappedChannels[FORWARD_CHANNEL_INDX], fwdValue, twist.linear.x);
        ROS_INFO("Turn: raw=%d, value=%.2f, turn=%.2f, invert=%s", 
                 msg->mappedChannels[TURN_CHANNEL_INDX], turnValue, twist.angular.z, invertTurn ? "true" : "false");
        last_turn_print = ros::Time::now();
    }
    
    cmdVelPublisher.publish( twist );
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "sbus_cmd_vel_node" );
    ros::NodeHandle nh;
    ros::NodeHandle param_nh( "~" );

    // Read/set parameters
    param_nh.param<int>( "sbusMinValue", sbusMinValue, -1 );
    param_nh.param<int>( "sbusMaxValue", sbusMaxValue, -1 );
    param_nh.param<double>( "maxSpeed", maxSpeed, -1 );
    param_nh.param<double>( "maxTurn", maxTurn, -1 );
    param_nh.param<bool>( "invertTurn", invertTurn, false );  // Default to not inverted
    param_nh.param<double>( "deadzone", deadzone, 0.1 );     // Default deadzone of 0.1

    // All parameters _must_ be explicitly specified
    if( sbusMaxValue == sbusMinValue && maxSpeed == -1 ) {
        ROS_ERROR( "Config error: sbusMinValue, sbusMaxValue and maxSpeed parameters must be specified!" );
        return 1;
    }

    sbusRange = sbusMaxValue - sbusMinValue;

    ros::Subscriber sbusSubscriber = nh.subscribe( "/sbus", 1, sbusCallback );
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>( "/output/sbus/cmd_vel", 1 );

    ROS_INFO( "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s, invertTurn = %s, deadzone = %.2f", 
        ros::this_node::getName().c_str(), sbusMinValue, sbusMaxValue, maxSpeed, maxTurn, invertTurn ? "true" : "false", deadzone );
    ros::spin();
    return 0;
}



/*913 shang mian de*/
/*9.10 ch1  turn direction right! 
* Copyright 2018 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* SBUS -> cmd_vel node
*
* Subscribes to:
*	/sbus (Sbus)
* Publishes:
*	/output/sbus/cmd_vel (Twist)
*/

#include "ros/ros.h"
#include <sbus_serial/Sbus.h>
#include <geometry_msgs/Twist.h>

#define FORWARD_CHANNEL_INDX 1       // Channel 2 (elevator)
#define TURN_CHANNEL_INDX 0          // Channel 1 (ailerons)

// Publisher and parameters are in global scope so callback function can use them
ros::Publisher cmdVelPublisher;
int sbusMinValue;
int sbusMaxValue;
int sbusRange;          // Calculated when reading min/max
double maxSpeed;        // m/sec
double maxTurn;         // radians/sec
bool invertTurn;        // Whether to invert the turn channel

void sbusCallback( const sbus_serial::Sbus::ConstPtr& msg )
{
    // 打印所有通道的值（每2秒一次）
    static ros::Time last_print = ros::Time::now();
    if ((ros::Time::now() - last_print).toSec() > 2.0) {
        ROS_INFO("All channels: %d, %d, %d, %d, %d, %d, %d, %d",
                 msg->mappedChannels[0], msg->mappedChannels[1],
                 msg->mappedChannels[2], msg->mappedChannels[3],
                 msg->mappedChannels[4], msg->mappedChannels[5],
                 msg->mappedChannels[6], msg->mappedChannels[7]);
        last_print = ros::Time::now();
    }

    double proportional;
    geometry_msgs::Twist twist;

    // 前进通道处理
    proportional = static_cast<double>(msg->mappedChannels[ FORWARD_CHANNEL_INDX ] - sbusMinValue) / sbusRange;
    double fwdSpeed = maxSpeed * (proportional * 2 - 1.0);
    twist.linear.x = fwdSpeed;

    // 转向通道处理
    proportional = static_cast<double>(msg->mappedChannels[ TURN_CHANNEL_INDX ] - sbusMinValue) / sbusRange;
    double turn = maxTurn * (proportional * 2 - 1.0);
    
    // 添加调试输出（每0.5秒一次）
    static ros::Time last_turn_print = ros::Time::now();
    if ((ros::Time::now() - last_turn_print).toSec() > 0.5) {
        ROS_INFO("Turn channel: raw=%d, proportional=%.2f, before_invert=%.2f, invert=%s", 
                 msg->mappedChannels[TURN_CHANNEL_INDX], proportional, turn, invertTurn ? "true" : "false");
        last_turn_print = ros::Time::now();
    }
    
    // Apply inversion if needed
    if (invertTurn) {
        turn = -turn;
    }
    
    twist.angular.z = turn;

    cmdVelPublisher.publish( twist );
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "sbus_cmd_vel_node" );
    ros::NodeHandle nh;
    ros::NodeHandle param_nh( "~" );

    // Read/set parameters
    param_nh.param<int>( "sbusMinValue", sbusMinValue, -1 );
    param_nh.param<int>( "sbusMaxValue", sbusMaxValue, -1 );
    param_nh.param<double>( "maxSpeed", maxSpeed, -1 );
    param_nh.param<double>( "maxTurn", maxTurn, -1 );
    param_nh.param<bool>( "invertTurn", invertTurn, false );  // Default to not inverted

    // All parameters _must_ be explicitly specified
    if( sbusMaxValue == sbusMinValue && maxSpeed == -1 ) {
        ROS_ERROR( "Config error: sbusMinValue, sbusMaxValue and maxSpeed parameters must be specified!" );
        return 1;
    }

    sbusRange = sbusMaxValue - sbusMinValue;

    ros::Subscriber sbusSubscriber = nh.subscribe( "/sbus", 1, sbusCallback );
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>( "/output/sbus/cmd_vel", 1 );

    ROS_INFO( "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s, invertTurn = %s", 
        ros::this_node::getName().c_str(), sbusMinValue, sbusMaxValue, maxSpeed, maxTurn, invertTurn ? "true" : "false" );
    ros::spin();
    return 0;
}

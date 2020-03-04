#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <thread>

// Time difference helper macro
#define DELTA(_nowTime, _thenTime) ((_thenTime>_nowTime)?((0xffffffff-_thenTime)+_nowTime):(_nowTime-_thenTime))

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

#include <geometry_msgs/Twist.h>

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information
#define _ODOM_SENSORS

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#ifdef _ODOM_SENSORS

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#endif

#include "roboteq_driver_ros.h"
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

static void mySigintHandler(int sig) {
    ROS_INFO_STREAM("Received SIGINT : " << sig << ", shutting down...");
    ros::shutdown();
}

static uint32_t millis() {
    ros::WallTime wallTime = ros::WallTime::now();
    return (uint32_t) (wallTime.toNSec() / 1000000);
}

int MainNode::run() {

    ROS_INFO("Beginning setup...");

    cmdvel_setup();
    cmd_ts_setup();
    cmd_jw_setup();
    odom_setup();

    ros::Rate loop_rate(10);

    startTime = millis();
    hsTimer = startTime;
    msTimer = startTime;
    lsTimer = startTime;

    ROS_INFO("Beginning looping...");
    
    volt_low_count=0;

    while (ros::ok()) {
        if (mainWheelController.IsConnected() && jockeyAndSecWheelController.IsConnected()) {
            uint32_t nowTime = millis();

            odom_loop();
            odom_loop2();
            //30hz publish
            if (DELTA(nowTime, hsTimer) >=33){
               hsTimer = nowTime;
               odom_hs_run();
            }
            //10hz publish
            if (DELTA(nowTime, msTimer) >=100){
               msTimer = nowTime;
               odom_ms_run();
            }
            //1hz publish
            if (DELTA(nowTime, lsTimer) >=1000){
               lsTimer = nowTime;
               odom_ls_run();
            }
            //Publish input 3 and 4
            jockeyAndSecWheelController.GetValue(_DIN,3,input3);
            jockeyAndSecWheelController.GetValue(_DIN,4,input4);
            input3_msg.data = input3;
            input4_msg.data = input4;
            input3_pub.publish(input3_msg);
            input4_pub.publish(input4_msg);
        } else {
            ROS_INFO_STREAM("Setting up controllers");
            controllers_setup();
            ROS_INFO_STREAM("Setting up Done");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (mainWheelController.IsConnected()) {
        mainWheelController.Disconnect();
    }

    if (jockeyAndSecWheelController.IsConnected()) {
        jockeyAndSecWheelController.Disconnect();
    }

    ROS_INFO("Exiting");

    return 0;
}

MainNode::MainNode() :
        nh("~"),
        port(""),
        port2(""),
#ifdef _ODOM_SENSORS
// Roboteq Tender Wheels
        voltage(0.0),
        PWM_right(0.0),
        PWM_left(0.0),
        RPM_right(0),
        RPM_left(0),
        current_right(0.0),
        current_left(0.0),
        energy(0.0),
        temperature(0.0),
// Roboteq J & S
        jockeyAndSecWheelVoltage(0.0),
        PWM_Jockey(0.0),
        PWM_Second(0.0),
        RPM_Jockey(0),
        RPM_Second(0),
        input3(0),
        input4(0),
        current_Jockey(0.0),
        current_Second(0.0)
#endif
{
#ifdef _ODOM_SENSORS
    voltage_msg.data = 0;

    vector<double> vec1 = { 0, 0, 0, 0 };
    current_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    current_msg.layout.dim[0].size = vec1.size();
    current_msg.layout.dim[0].stride = 1;
    current_msg.layout.dim[0].label = "current"; // or whatever name you typically use to index vec1
    current_msg.data.clear();
    current_msg.data.insert(current_msg.data.end(), vec1.begin(), vec1.end());
    
    vector<double> vec2 = { 0, 0, 0, 0 };
    RPM_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    RPM_msg.layout.dim[0].size = vec2.size();
    RPM_msg.layout.dim[0].stride = 1;
    RPM_msg.layout.dim[0].label = "RPM";
    RPM_msg.data.clear();
    RPM_msg.data.insert(RPM_msg.data.end(),vec2.begin(), vec2.end());

    energy_msg.data = 0;
    temperature_msg.data = 0;
#endif

    // CBA Read local params (from launch file)
    nh.getParam("cmdvel_topic", cmdvel_topic);
    ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
    nh.getParam("cmd_jw_topic", cmd_jw_topic);
    ROS_INFO_STREAM("cmd_jw_topic: " << cmd_jw_topic);
    nh.getParam("cmd_ts_topic", cmd_ts_topic);
    ROS_INFO_STREAM("cmd_ts_topic: " << cmd_ts_topic);
    nh.getParam("mode_motor_topic: ", mode_motor_topic);
    ROS_INFO_STREAM("mode_motor_topic: " << mode_motor_topic);
    nh.getParam("mode_ts_topic: ", mode_ts_topic);
    ROS_INFO_STREAM("mode_ts_topic: " << mode_ts_topic);
    nh.getParam("mode_jw_topic", mode_jw_topic);
    ROS_INFO_STREAM("mode_jw_topic << mode_jw_topic");
    nh.getParam("port", port);
    ROS_INFO_STREAM("port: " << port);
    nh.getParam("open_loop", open_loop);
    ROS_INFO_STREAM("open_loop: " << open_loop);
    nh.getParam("wheel_circumference", wheel_circumference);
    ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
    nh.getParam("track_width", track_width);
    ROS_INFO_STREAM("track_width: " << track_width);
    nh.getParam("encoder_ppr", encoder_ppr);
    ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
    nh.getParam("encoder_cpr", encoder_cpr);
    ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
    nh.getParam("port2", port2);
    ROS_INFO_STREAM("port2: " << port2);
    nh.getParam("open_loop2", open_loop2);
    ROS_INFO_STREAM("open_loop2: " << open_loop2);
    nh.getParam("encoder_ppr2", encoder_ppr2);
    ROS_INFO_STREAM("encoder_ppr2: " << encoder_ppr2);
    nh.getParam("encoder_cpr2", encoder_cpr2);
    ROS_INFO_STREAM("encoder_cpr2: " << encoder_cpr2);
}

MainNode::~MainNode() {
    mainWheelController.Disconnect();
    jockeyAndSecWheelController.Disconnect();
    std::terminate();
}

void MainNode::main_wheel_controller_setup() {
    // stop motors
    mainWheelController.SetCommand(_G, 1, 0);
    mainWheelController.SetCommand(_G, 2, 0);
    mainWheelController.SetCommand(_S, 1, 0);
    mainWheelController.SetCommand(_S, 2, 0);

    // disable echo
    // mainWheelController.write("^ECHOF 1\r");

    // enable watchdog timer (100 ms)
    mainWheelController.SetConfig(_RWD, 1000);

    // release emergency stop
    mainWheelController.SetCommand(_MG, 1);

    // set digital inupts action triggers
    // digital inupts action level
    mainWheelController.SetConfig(_DINL, 3, 0);
    mainWheelController.SetConfig(_DINL, 4, 0);

    // emergency stop
    mainWheelController.SetConfig(_DINA, 3, 0);
    mainWheelController.SetConfig(_DINA, 4, 0);

    // set  Roboteq Tender Wheels motor operating mode (1 for closed-loop speed)
    if (open_loop) {
        mainWheelController.SetConfig(_MMOD, 1, 0);
        mainWheelController.SetConfig(_MMOD, 2, 0);
    } else {
        mainWheelController.SetConfig(_MMOD, 1, 1);
        mainWheelController.SetConfig(_MMOD, 2, 1);
    }

    // set main motor amps limit (9 A)
    mainWheelController.SetConfig(_ALIM, 1, 90);
    mainWheelController.SetConfig(_ALIM, 2, 90);

    // set max voltage 55 V
    mainWheelController.SetConfig(_OVL, 550);
    mainWheelController.SetConfig(_OVH, 20);

    // set stall Detection
    mainWheelController.SetConfig(_BLSTD, 1, 2);
    mainWheelController.SetConfig(_BLSTD, 2, 2);

    // set Default command value
    mainWheelController.SetConfig(_DFC, 1, 0);
    mainWheelController.SetConfig(_DFC, 2, 0);

    // set max speed (rpm) for relative speed commands
    mainWheelController.SetConfig(_MXRPM, 1, 3350);
    mainWheelController.SetConfig(_MXRPM, 2, 3350);

    // set max acceleration rate 2000 rpm/s
    mainWheelController.SetConfig(_MAC, 1, 20000);
    mainWheelController.SetConfig(_MAC, 2, 20000);

    // set max deceleration rate 2000 rpm/s
    mainWheelController.SetConfig(_MDEC, 1, 20000);
    mainWheelController.SetConfig(_MDEC, 2, 20000);

    // set PID parameters
    mainWheelController.SetConfig(_KP, 1, 10);
    mainWheelController.SetConfig(_KP, 2, 10);
    mainWheelController.SetConfig(_KI, 1, 90);
    mainWheelController.SetConfig(_KI, 2, 85);
    mainWheelController.SetConfig(_KD, 1, 20);
    mainWheelController.SetConfig(_KD, 2, 20);

    // set encoder mode (encoder 1 for feedback on motor 1, encoder 2 for feedback on motor 2)
    mainWheelController.SetConfig(_EMOD, 1, 18);
    mainWheelController.SetConfig(_EMOD, 2, 34);

    // set encoder counts (ppr)
    mainWheelController.SetConfig(_EPPR, 1, encoder_ppr);
    mainWheelController.SetConfig(_EPPR, 2, encoder_ppr);
}

void MainNode::jockey_and_sec_wheel_controller_setup() {
    // stop motors
    jockeyAndSecWheelController.SetCommand(_G, 1, 0);
    jockeyAndSecWheelController.SetCommand(_G, 2, 0);
    jockeyAndSecWheelController.SetCommand(_S, 1, 0);
    jockeyAndSecWheelController.SetCommand(_S, 2, 0);

    // enable watchdog timer (100 ms)
    jockeyAndSecWheelController.SetConfig(_RWD, 1000);

    // release emergency stop
    jockeyAndSecWheelController.SetCommand(_MG, 1);

    // set digital inupts action triggers
    // digital inupts action level
    jockeyAndSecWheelController.SetConfig(_DINL, 3, 0);
    jockeyAndSecWheelController.SetConfig(_DINL, 4, 0);

    // emergency stop
    jockeyAndSecWheelController.SetConfig(_DINA, 3, (4+16));
    // forward limit switch for secondary train
    jockeyAndSecWheelController.SetConfig(_DINA, 4, (4+32));

    // set  Roboteq J & S motor operating mode (1 for closed-loop speed)
    if (open_loop2) {
        jockeyAndSecWheelController.SetConfig(_MMOD, 1, 3);
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 1);
    } else {
        jockeyAndSecWheelController.SetConfig(_MMOD, 1, 1);
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 1);
    }

    // set secondary wheel motor amps limit to 9 A and jockey wheel's to 4 A
    jockeyAndSecWheelController.SetConfig(_ALIM, 1, 200);
    jockeyAndSecWheelController.SetConfig(_ALIM, 2, 100);

    // set max voltage 65 V
    jockeyAndSecWheelController.SetConfig(_OVL, 530);
    jockeyAndSecWheelController.SetConfig(_OVH, 20);

    // set stall Detection
    jockeyAndSecWheelController.SetConfig(_BLSTD, 1, 2);
    jockeyAndSecWheelController.SetConfig(_BLSTD, 2, 2);

    // set Default command value
    jockeyAndSecWheelController.SetConfig(_DFC, 1, 0);
    jockeyAndSecWheelController.SetConfig(_DFC, 2, 0);

    // set max speed (rpm) for relative speed commands
    jockeyAndSecWheelController.SetConfig(_MXRPM, 1, 3350);
    jockeyAndSecWheelController.SetConfig(_MXRPM, 2, 3000);

    // set max acceleration rate 20000 rpm/s for secondary wheels and 20000 for jockey wheel
    jockeyAndSecWheelController.SetConfig(_MAC, 1, 200000);
    jockeyAndSecWheelController.SetConfig(_MAC, 2, 200000);

    // set max deceleration rate 2000 rpm/s for secondary wheels and 20000 for jockey wheel
    jockeyAndSecWheelController.SetConfig(_MDEC, 1, 200000);
    jockeyAndSecWheelController.SetConfig(_MDEC, 2, 200000);

    // set PID parameters
    //jockeyAndSecWheelController.SetConfig(_KP, 1, 12);
    jockeyAndSecWheelController.SetConfig(_KP, 1, 20);
    jockeyAndSecWheelController.SetConfig(_KP, 2, 5);
    //jockeyAndSecWheelController.SetConfig(_KI, 1, 20);
    jockeyAndSecWheelController.SetConfig(_KI, 1, 10);
    jockeyAndSecWheelController.SetConfig(_KI, 2, 23);
    //jockeyAndSecWheelController.SetConfig(_KD, 1, 10);
    jockeyAndSecWheelController.SetConfig(_KD, 1, 0);
    jockeyAndSecWheelController.SetConfig(_KD, 2, 20);

    // set encoder mode (encoder 1 for feedback on motor 1, encoder 2 for feedback on motor 2)
    jockeyAndSecWheelController.SetConfig(_EMOD, 1, 18);
    jockeyAndSecWheelController.SetConfig(_EMOD, 2, 34);

    // set encoder counts (ppr)
    jockeyAndSecWheelController.SetConfig(_EPPR, 1, encoder_ppr2);
    jockeyAndSecWheelController.SetConfig(_EPPR, 2, encoder_ppr2);

    int pin3 = 0;
    jockeyAndSecWheelController.GetValue(_DIN, 3, pin3);
    do{
        jockeyAndSecWheelController.SetCommand(_PR, 1, 28000);
        jockeyAndSecWheelController.GetValue(_DIN, 3, pin3);
    }while(pin3!=1);
     

}

void MainNode::controllers_setup() {
    int status;
    int v = 0;
    if ((!port.empty()) && (!mainWheelController.IsConnected())) {
        //ROS_INFO_STREAM("Opening serial port on " << port);
        status = mainWheelController.Connect(port);
        if (status == RQ_SUCCESS) {
            mainWheelController.GetValue(_V, 2, v);
            if (v > 300){
                ROS_INFO("Successfully connected to main wheels controller");
                main_wheel_controller_setup();}
            else {
                mainWheelController.Disconnect();}
        } else {
            //ROS_WARN_STREAM("Failed to connect to main wheels controller");
        }
    }

    if ((!port2.empty()) && (!jockeyAndSecWheelController.IsConnected())) {
        //ROS_INFO_STREAM("Opening serial port2 on " << port2);
        status = jockeyAndSecWheelController.Connect(port2);
        if (status == RQ_SUCCESS) {
            jockeyAndSecWheelController.GetValue(_V, 2, v);
            if (v > 300){
                ROS_INFO("Successfully connected to jockey wheel and secondary wheel controller");
                jockey_and_sec_wheel_controller_setup();}
            else {
                jockeyAndSecWheelController.Disconnect();}
        } else {
            //ROS_WARN_STREAM("Failed to connect to jockey wheel and secondary wheel controller");
        }
    }
}

//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback(const geometry_msgs::Twist &twist_msg) {
#ifdef _CMDVEL_DEBUG
    ROS_DEBUG_STREAM("callback of topic : " + cmdvel_sub.getTopic() + " start");
#endif
    // wheel speed (m/s)
    float right_speed = (twist_msg.linear.x - track_width * twist_msg.angular.z/ 2.0);
    float left_speed = -(twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0);

    // Roboteq Tender Wheels
    if (open_loop) {
        // motor power (scale 0-1000)
        auto right_power = (int32_t) (right_speed / wheel_circumference * 60.0 / 82.0 * 1000.0);
        auto left_power = (int32_t) (left_speed / wheel_circumference * 60.0 / 82.0 * 1000.0);
#ifdef _CMDVEL_DEBUG
        ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
#endif
        mainWheelController.SetCommand(_G, 1, right_power);
        mainWheelController.SetCommand(_G, 2, left_power);
    } else {
        auto right_rpm = (int32_t) (right_speed * 28 * 60.0/ wheel_circumference );
        auto left_rpm = (int32_t) (left_speed * 28 * 60.0/ wheel_circumference);
#ifdef _CMDVEL_DEBUG
        ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
#endif
        mainWheelController.SetCommand(_S, 1, right_rpm);
        mainWheelController.SetCommand(_S, 2, left_rpm);
    }

}


void MainNode::cmd_ts_callback(const std_msgs::Int32::ConstPtr& msg) {
    int32_t ts_position;
    ts_position = msg->data;
    if(ts_mode==3){
      jockeyAndSecWheelController.SetCommand(_P, 1, ts_position);}
}

void MainNode::cmd_jw_callback(const std_msgs::Int32::ConstPtr& msg) {
    int32_t jw_command = msg->data;
    if(jw_mode==1){
      jockeyAndSecWheelController.SetCommand(_S, 2, jw_command);}
    if(jw_mode==3){
      jockeyAndSecWheelController.SetCommand(_P, 2, jw_command);}
}

void MainNode::cmdvel_setup() {
    ROS_INFO_STREAM("Subscribing to cmdvel topic " << cmdvel_topic);
    cmdvel_sub = nh.subscribe(cmdvel_topic, 10, &MainNode::cmdvel_callback, this);
    mode_motor_sub = nh.subscribe(mode_motor_topic, 10, &MainNode::mode_motor_callback, this);
}

void MainNode::cmd_jw_setup() {
    ROS_INFO_STREAM("Subscribing to jockey wheel topic " << cmd_jw_topic);
    cmd_jw_sub = nh.subscribe(cmd_jw_topic, 10, &MainNode::cmd_jw_callback, this);
    mode_jw_sub = nh.subscribe(mode_jw_topic, 10, &MainNode::mode_jw_callback, this);
}

void MainNode::cmd_ts_setup() {
    ROS_INFO_STREAM("Subscribing to second train topic " << cmd_ts_topic);
    cmd_ts_sub = nh.subscribe(cmd_ts_topic, 10, &MainNode::cmd_ts_callback, this);
    mode_ts_sub = nh.subscribe(mode_ts_topic, 10, &MainNode::mode_ts_callback, this);
}


void MainNode::mode_jw_callback(const std_msgs::UInt8::ConstPtr& msg) {
    jw_mode = msg->data; 
    switch(jw_mode){
     //open loop 
     case 0:
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 0);
        break;
     
     //closed-loop speed 
     case 1:
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 1);
        jockeyAndSecWheelController.SetConfig(_KP, 2, 5);
        jockeyAndSecWheelController.SetConfig(_KI, 2, 23);
        jockeyAndSecWheelController.SetConfig(_KD, 2, 20);
        break;
    
     //closed-loop position 
     case 3:
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 3);
        jockeyAndSecWheelController.SetConfig(_KP, 2, 7);
        jockeyAndSecWheelController.SetConfig(_KI, 2, 1);
        jockeyAndSecWheelController.SetConfig(_KD, 2, 0);
        break;
     
     //closed-loop torque       
     case 5:
        break;
     
     default:
        break;
    }

}

void MainNode::mode_ts_callback(const std_msgs::UInt8::ConstPtr& msg) {
    ts_mode = msg->data;
    switch(ts_mode){
      //open loop
      case 0:
         jockeyAndSecWheelController.SetConfig(_MMOD, 1, 0);
         break;

      case 3:
         jockeyAndSecWheelController.SetConfig(_MMOD, 1, 3);
         jockeyAndSecWheelController.SetConfig(_KP, 1, 20);
         jockeyAndSecWheelController.SetConfig(_KI, 1, 10);
         jockeyAndSecWheelController.SetConfig(_KD, 1, 0);
         break;

      default:
         break;
    }
}

void MainNode::mode_motor_callback(const std_msgs::UInt8::ConstPtr& msg) {
    motor_mode = msg->data;
    switch(motor_mode){
      //open loop
      case 0:
         mainWheelController.SetConfig(_MMOD, 1, 0);
         mainWheelController.SetConfig(_MMOD, 2, 0);
         break;

      case 1:
         mainWheelController.SetConfig(_MMOD, 1, 1);
         mainWheelController.SetConfig(_MMOD, 2, 1);
         mainWheelController.SetConfig(_KP, 1, 10);
         mainWheelController.SetConfig(_KP, 2, 10);
         mainWheelController.SetConfig(_KI, 1, 90);
         mainWheelController.SetConfig(_KI, 2, 85);
         mainWheelController.SetConfig(_KD, 1, 20);
         mainWheelController.SetConfig(_KD, 2, 20);
         break;

      default:
         break;
    }
}

void MainNode::odom_setup() {
#ifdef _ODOM_SENSORS
    ROS_INFO("Publishing to topic roboteq/voltage");
    voltage_pub = nh.advertise<std_msgs::Float32>("/roboteq/voltage", 1000);
    ROS_INFO("Publishing to topic roboteq/current");
    current_pub = nh.advertise<std_msgs::Float32MultiArray>("/roboteq/current", 1000);
    ROS_INFO("Publishing to topic roboteq/RPM");
    RPM_pub = nh.advertise<std_msgs::Float32MultiArray>("/roboteq/RPM",1000);
    ROS_INFO("Publishing to topic roboteq/energy");
    energy_pub = nh.advertise<std_msgs::Float32>("/roboteq/energy", 1000);
    ROS_INFO("Publishing to topic roboteq/temperature");
    temperature_pub = nh.advertise<std_msgs::Float32>("/roboteq/temperature", 1000);
    ROS_INFO("Publishing to topic roboteq/input3");
    input3_pub = nh.advertise<std_msgs::UInt8>("/roboteq/input3", 1000);
    ROS_INFO("Publishing to topic roboteq/input4");
    input4_pub = nh.advertise<std_msgs::UInt8>("/roboteq/input4", 1000);
    encoder_jw_pub = nh.advertise<std_msgs::Int32>("/roboteq/encoder_jw", 1000);
    encoder_ts_pub = nh.advertise<std_msgs::Int32>("/roboteq/encoder_ts", 1000);   
#endif
}

void MainNode::odom_loop() {
    // read sensor data stream from motor controller
    // Roboteq Tender Wheels
    // CR : encoder counts
    mainWheelController.GetValue(_CR, 1, odom_encoder_right);
    mainWheelController.GetValue(_CR, 2, odom_encoder_left);
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("encoder right: " << odom_encoder_right << " left: " << odom_encoder_left);
#endif

    // V : voltage
    int volt;
    mainWheelController.GetValue(_V, 2, volt);
    voltage = volt / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("V: " << voltage);
#endif
    
    if(volt<=300){
        volt_low_count++;
    }else{

        volt_low_count=0;
    }

    if(volt_low_count>=10){
        mainWheelController.Disconnect();
        jockeyAndSecWheelController.Disconnect();
    }

    // P : PWM
    int pwmRight, pwmLeft;
    mainWheelController.GetValue(_P, 1, pwmRight);
    mainWheelController.GetValue(_P, 2, pwmLeft);
    PWM_right = pwmRight / 10.0;
    PWM_left = pwmLeft / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("PWM right: " << PWM_right);
    ROS_DEBUG_STREAM("PWM left: " << PWM_left);
#endif

    // S : RPM
    mainWheelController.GetValue(_S, 1, RPM_right);
    mainWheelController.GetValue(_S, 2, RPM_left);
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("RPM right: " << RPM_right);
    ROS_DEBUG_STREAM("RPM letf: " << RPM_left);
#endif

    // BA : motor currents
    int currentRight, currentLeft;
    mainWheelController.GetValue(_A, 1, currentRight);
    mainWheelController.GetValue(_A, 2, currentLeft);
    current_right = currentRight / 10.0;
    current_left = currentLeft / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("Current right: " << current_right);
    ROS_DEBUG_STREAM("Current left: " << current_left);
#endif

    //fault flag
    int faultFlag1 = 0, faultFlag2 = 0;

    mainWheelController.GetValue(_FF, 1, faultFlag1);
    faultFlag1 &= 0x00FF;
    if (faultFlag1 != 0) {
        ROS_WARN_STREAM("Main wheel 1 : " << faultFlag1);
    }

    mainWheelController.GetValue(_FF, 2, faultFlag2);
    faultFlag2 &= 0x00FF;
    if (faultFlag2 != 0) {
        ROS_WARN_STREAM("Main wheel 2 : " << faultFlag2);
    }
}

void MainNode::odom_loop2() {
    // read sensor data stream from motor controller
    // Roboteq J & S
    // CR : encoder counts
    jockeyAndSecWheelController.GetValue(_CR, 1, odom_encoder_Second);
    jockeyAndSecWheelController.GetValue(_CR, 2, odom_encoder_Jockey);
    encoder_jw_pub.publish(odom_encoder_Jockey);
    encoder_ts_pub.publish(odom_encoder_Second);
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("encoder Jockey: " << odom_encoder_Jockey << " Second: " << odom_encoder_Second);
#endif

    // V : voltage
    int volt;
    jockeyAndSecWheelController.GetValue(_V, 2, volt);
    jockeyAndSecWheelVoltage = volt / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("V2: " << jockeyAndSecWheelVoltage);
#endif

    if(volt<=300){
        volt_low_count++;
    }else{
        volt_low_count=0;
    }

    if(volt_low_count>=10){
        mainWheelController.Disconnect();
        jockeyAndSecWheelController.Disconnect();
    }

    // P : PWM
    int pwmSecond, pwmJockey;
    jockeyAndSecWheelController.GetValue(_P, 1, pwmSecond);
    jockeyAndSecWheelController.GetValue(_P, 2, pwmJockey);
    PWM_Second = pwmSecond / 10.0;
    PWM_Jockey = pwmJockey / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("PWM Second: " << PWM_Second);
    ROS_DEBUG_STREAM("PWM Jockey: " << PWM_Jockey);
#endif

    // S : RPM
    jockeyAndSecWheelController.GetValue(_S, 1, RPM_Second);
    jockeyAndSecWheelController.GetValue(_S, 2, RPM_Jockey);
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("RPM Second: " << RPM_Second);
    ROS_DEBUG_STREAM("RPM Jockey: " << RPM_Jockey);
#endif

    // BA : motor currents
    int currentSecond, currentJockey;
    jockeyAndSecWheelController.GetValue(_A, 1, currentSecond);
    jockeyAndSecWheelController.GetValue(_A, 2, currentJockey);
    current_Second = currentSecond / 10.0;
    current_Jockey = currentJockey / 10.0;
#ifdef _ODOM_DEBUG
    ROS_DEBUG_STREAM("Current Second: " << current_Second);
    ROS_DEBUG_STREAM("Current Jockey: " << current_Jockey);
#endif

    int input3=0;
    jockeyAndSecWheelController.GetValue(_DIN, 3, input3);
    if(input3==1){
      jockeyAndSecWheelController.SetCommand(_C, 1, 0);
    }
    input4 = 0;
    jockeyAndSecWheelController.GetValue(_DIN, 4, input4);
    if(input4==1){
      jockeyAndSecWheelController.SetCommand(_C, 2, 0);  
    }

    //fault flag
    int faultFlag1 = 0, faultFlag2 = 0;

    jockeyAndSecWheelController.GetValue(_FF, 1, faultFlag1);
    faultFlag1 &= 0x00FF;
    if (faultFlag1 != 0) {
        ROS_WARN_STREAM("Secondary Wheel : " << faultFlag1);
    }

    jockeyAndSecWheelController.GetValue(_FF, 2, faultFlag2);
    faultFlag2 &= 0x00FF;
    if (faultFlag2 != 0) {
        ROS_WARN_STREAM("Jockey wheel : " << faultFlag2);
    }
}

void MainNode::odom_hs_run() {
}

void MainNode::odom_ms_run() {

#ifdef _ODOM_SENSORS
    current_msg.data.at(0) = current_right;
    current_msg.data.at(1) = current_left;
    current_msg.data.at(2) = current_Second;
    current_msg.data.at(3) = current_Jockey;
    current_pub.publish(current_msg);
    RPM_msg.data.at(0) = float(RPM_right);
    RPM_msg.data.at(1) = float(RPM_left);
    RPM_msg.data.at(2) = float(RPM_Second);
    RPM_msg.data.at(3) = float(RPM_Jockey);
    RPM_pub.publish(RPM_msg);
#endif

}

void MainNode::odom_ls_run() {
#ifdef _ODOM_SENSORS
    voltage_msg.data = voltage;
    voltage_pub.publish(voltage_msg);
    energy_msg.data = energy;
    energy_pub.publish(energy_msg);
    temperature_msg.data = temperature;
    temperature_pub.publish(temperature_msg);
#endif
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "roboteq_node");

    MainNode node;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    return node.run();
}


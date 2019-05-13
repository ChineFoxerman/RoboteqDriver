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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information
#define _ODOM_SENSORS

// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#ifdef _ODOM_SENSORS

#include <std_msgs/Float32.h>
#include <roboteq_diff_msgs/Duplex.h>

#endif
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/OdometryCovariances.h"
#include "rogoteq_diff_msgs/RequestOdometryCovariances.h"
#endif

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

static void mySigintHandler(int sig) {
    ROS_INFO_STREAM("Received SIGINT : " << sig << ", shutting down...");
    ros::shutdown();
}

static uint32_t millis() {
    ros::WallTime wallTime = ros::WallTime::now();
//	return (uint32_t)((wallTime._sec*1000 + wallTime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(wallTime.toNSec()/1000000.0+0.5);
    return (uint32_t) (wallTime.toNSec() / 1000000);
}

class MainNode {

public:
    MainNode();
    ~MainNode();

public:

    //
    // cmd_vel subscriber
    //
    void cmdvel_callback(const geometry_msgs::Twist &twist_msg);
    void cmdvel_setup();

    //
    // odom publisher
    //
    void odom_setup();
    void odom_loop();
    void odom_loop2();
    void odom_hs_run();
    void odom_ms_run();
    void odom_ls_run();
    void odom_publish();

#ifdef _ODOM_COVAR_SERVER
    void odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res);
#endif

    int run();

protected:
    ros::NodeHandle nh;

    RoboteqDevice mainWheelController;
    RoboteqDevice jockeyAndSecWheelController;

    uint32_t startTime{};
    uint32_t hsTimer{};
    uint32_t msTimer{};
    uint32_t lsTimer{};

    //
    // cmd_vel subscriber
    //
    ros::Subscriber cmdvel_sub;

    //
    // odom publisher
    //
    geometry_msgs::TransformStamped tf_msg;
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom_msg;
    ros::Publisher odom_pub;

#ifdef _ODOM_SENSORS
    std_msgs::Float32 voltage_msg;
    ros::Publisher voltage_pub;
    roboteq_diff_msgs::Duplex current_msg;
    ros::Publisher current_pub;
    std_msgs::Float32 energy_msg;
    ros::Publisher energy_pub;
    std_msgs::Float32 temperature_msg;
    ros::Publisher temperature_pub;
#endif

    // toss out initial encoder readings
    int32_t odom_encoder_left{};
    int32_t odom_encoder_right{};
    int32_t odom_encoder_Jockey{};
    int32_t odom_encoder_Second{};

    float odom_x{};
    float odom_y{};
    float odom_yaw{};
    float odom_last_x{};
    float odom_last_y{};
    float odom_last_yaw{};

    uint32_t odom_last_time{};

#ifdef _ODOM_SENSORS
    // Roboteq Tender Wheels (deux roues)
    float PWM_right;
    float PWM_left;
    float voltage;
    int RPM_right;
    int RPM_left;
    float current_right;
    float current_left;
    float energy;
    float temperature;

    // Roboteq J & S (Roue jockey + secondaire)
    float jockeyAndSecWheelVoltage;
    float PWM_Jockey;
    float PWM_Second;
    int RPM_Jockey;
    int RPM_Second;
    float current_Jockey;
    float current_Second;
#endif

    // General settings
    bool pub_odom_tf;
    std::string odom_frame;
    std::string base_frame;
    std::string cmdvel_topic;
    std::string odom_topic;

    // settings  Roboteq Tender Wheels
    std::string port;
    int baud;
    bool open_loop;
    double wheel_circumference;
    double track_width;
    int encoder_ppr;
    int encoder_cpr;

    //settings  Roboteq J & S
    std::string port2;
    int baud2;
    bool open_loop2;
    int encoder_ppr2;
    int encoder_cpr2;
};

MainNode::MainNode() :
#ifdef _ODOM_SENSORS
        nh("roboteq_diff_driver"),
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
        current_Jockey(0.0),
        current_Second(0.0)
#endif
{
    // CBA Read local params (from launch file)
    nh.getParam("pub_odom_tf", pub_odom_tf);
    ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
    nh.getParam("odom_frame", odom_frame);
    ROS_INFO_STREAM("odom_frame: " << odom_frame);
    nh.getParam("base_frame", base_frame);
    ROS_INFO_STREAM("base_frame: " << base_frame);
    nh.getParam("cmdvel_topic", cmdvel_topic);
    ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
    nh.getParam("odom_topic", odom_topic);
    ROS_INFO_STREAM("odom_topic: " << odom_topic);
    nh.getParam("port", port);
    ROS_INFO_STREAM("port: " << port);
    nh.getParam("baud", baud);
    ROS_INFO_STREAM("baud: " << baud);
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
    nh.getParam("baud2", baud2);
    ROS_INFO_STREAM("baud2: " << baud2);
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

//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback(const geometry_msgs::Twist &twist_msg) {
#ifdef _CMDVEL_DEBUG
    ROS_DEBUG_STREAM("callback of topic : " + cmdvel_sub.getTopic() + " start");
#endif
    // wheel speed (m/s)
    float right_speed = (-twist_msg.angular.z + track_width * twist_msg.linear.x / 2.0);
    float left_speed = (-twist_msg.angular.z - track_width * twist_msg.linear.x / 2.0);
    //Jockey and second speed (pwm)
    float Jockey_speed = twist_msg.linear.z;
    float Second_speed = twist_msg.linear.y;

#ifdef _CMDVEL_DEBUG
    ROS_DEBUG_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);
    ROS_DEBUG_STREAM("cmdvel speed Jockey: " << Jockey_speed << " Second: " << Second_speed);
#endif

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
        auto right_rpm = (int32_t) (right_speed / wheel_circumference * 60.0);
        auto left_rpm = (int32_t) (left_speed / wheel_circumference * 60.0);
#ifdef _CMDVEL_DEBUG
        ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
#endif
        mainWheelController.SetCommand(_S, 1, right_rpm);
        mainWheelController.SetCommand(_S, 2, left_rpm);
    }
// Roboteq J & S
    if (open_loop2) {
        auto Jockey_power = (int32_t) (Jockey_speed * 10);
        auto Second_power = (int32_t) (Second_speed * 10);
#ifdef _CMDVEL_DEBUG
        ROS_DEBUG_STREAM("cmdvel power Jockey: " << Jockey_power << " Second: " << Second_power);
#endif
        jockeyAndSecWheelController.SetCommand(_G, 1, Second_power);
        jockeyAndSecWheelController.SetCommand(_G, 2, Jockey_power);
    } else {

    }

    ROS_INFO_STREAM("callback end");
}

void MainNode::cmdvel_setup() {
    // stop motors
    mainWheelController.SetCommand(_G, 1, 0);
    mainWheelController.SetCommand(_G, 2, 0);
    mainWheelController.SetCommand(_S, 1, 0);
    mainWheelController.SetCommand(_S, 2, 0);

    jockeyAndSecWheelController.SetCommand(_G, 1, 0);
    jockeyAndSecWheelController.SetCommand(_G, 2, 0);
    jockeyAndSecWheelController.SetCommand(_S, 1, 0);
    jockeyAndSecWheelController.SetCommand(_S, 2, 0);

    // disable echo
    // mainWheelController.write("^ECHOF 1\r");

    // enable watchdog timer (1000 ms)
    mainWheelController.SetConfig(_RWD, 1000);
    jockeyAndSecWheelController.SetConfig(_RWD, 1000);

    // set  Roboteq Tender Wheels motor operating mode (1 for closed-loop speed)
    if (open_loop) {
        mainWheelController.SetConfig(_MMOD, 1, 0);
        mainWheelController.SetConfig(_MMOD, 2, 0);
    } else {
        mainWheelController.SetConfig(_MMOD, 1, 1);
        mainWheelController.SetConfig(_MMOD, 2, 1);
    }

    // set  Roboteq J & S motor operating mode (1 for closed-loop speed)
    if (open_loop2) {
        jockeyAndSecWheelController.SetConfig(_MMOD, 1, 0);
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 0);
    } else {
        jockeyAndSecWheelController.SetConfig(_MMOD, 1, 1);
        jockeyAndSecWheelController.SetConfig(_MMOD, 2, 1);
    }

    // set motor amps limit (5 A * 10)
    mainWheelController.SetConfig(_ALIM, 1, 90);
    mainWheelController.SetConfig(_ALIM, 2, 90);
    jockeyAndSecWheelController.SetConfig(_ALIM, 1, 90);
    jockeyAndSecWheelController.SetConfig(_ALIM, 2, 90);

    // set max speed (rpm) for relative speed commands
    mainWheelController.SetConfig(_MXRPM, 1, 3350);
    mainWheelController.SetConfig(_MXRPM, 2, 3350);
    jockeyAndSecWheelController.SetConfig(_MXRPM, 1, 100);
    jockeyAndSecWheelController.SetConfig(_MXRPM, 2, 100);

    // set max acceleration rate (200 rpm/s * 10)
    mainWheelController.SetConfig(_MAC, 1, 20000);
    mainWheelController.SetConfig(_MAC, 2, 20000);
    jockeyAndSecWheelController.SetConfig(_MAC, 1, 2000);
    jockeyAndSecWheelController.SetConfig(_MAC, 2, 2000);

    // set max deceleration rate (2000 rpm/s * 10)
    mainWheelController.SetConfig(_MDEC, 1, 20000);
    mainWheelController.SetConfig(_MDEC, 2, 20000);
    jockeyAndSecWheelController.SetConfig(_MDEC, 1, 2000);
    jockeyAndSecWheelController.SetConfig(_MDEC, 2, 2000);

    // set PID parameters (gain * 10)
    mainWheelController.SetConfig(_KP, 1, 10);
    mainWheelController.SetConfig(_KP, 2, 10);
    mainWheelController.SetConfig(_KI, 1, 7);
    mainWheelController.SetConfig(_KI, 2, 7);
    mainWheelController.SetConfig(_KD, 1, 4);
    mainWheelController.SetConfig(_KD, 2, 4);
    jockeyAndSecWheelController.SetConfig(_KP, 1, 10);
    jockeyAndSecWheelController.SetConfig(_KP, 2, 10);
    jockeyAndSecWheelController.SetConfig(_KI, 1, 7);
    jockeyAndSecWheelController.SetConfig(_KI, 2, 7);
    jockeyAndSecWheelController.SetConfig(_KD, 1, 4);
    jockeyAndSecWheelController.SetConfig(_KD, 2, 4);

    // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
    mainWheelController.SetConfig(_EMOD, 1, 18);
    mainWheelController.SetConfig(_EMOD, 2, 34);
    jockeyAndSecWheelController.SetConfig(_EMOD, 1, 18);
    jockeyAndSecWheelController.SetConfig(_EMOD, 2, 34);

    // set encoder counts (ppr)
    mainWheelController.SetConfig(_EPPR, 1, encoder_ppr);
    mainWheelController.SetConfig(_EPPR, 2, encoder_ppr);
    jockeyAndSecWheelController.SetConfig(_EPPR, 1, encoder_ppr2);
    jockeyAndSecWheelController.SetConfig(_EPPR, 2, encoder_ppr2);

    ROS_INFO_STREAM("Before sub cmd");
    ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
    cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);
}

//
// odom publisher
//

#ifdef _ODOM_COVAR_SERVER
void MainNode::odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
#endif

void MainNode::odom_setup() {

    if (pub_odom_tf) {
        ROS_INFO("Broadcasting odom tf");
    }

    ROS_INFO_STREAM("Publishing to topic " << odom_topic);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

#ifdef _ODOM_COVAR_SERVER
    ROS_INFO("Advertising service on roboteq/odom_covar_srv");
    odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &MainNode::odom_covar_callback, this);
#endif

#ifdef _ODOM_SENSORS
    ROS_INFO("Publishing to topic roboteq/voltage");
    voltage_pub = nh.advertise<std_msgs::Float32>("roboteq/voltage", 1000);
    ROS_INFO("Publishing to topic roboteq/current");
    current_pub = nh.advertise<roboteq_diff_msgs::Duplex>("roboteq/current", 1000);
    ROS_INFO("Publishing to topic roboteq/energy");
    energy_pub = nh.advertise<std_msgs::Float32>("roboteq/energy", 1000);
    ROS_INFO("Publishing to topic roboteq/temperature");
    temperature_pub = nh.advertise<std_msgs::Float32>("roboteq/temperature", 1000);
#endif

    tf_msg.header.seq = 0;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;

    odom_msg.header.seq = 0;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    odom_msg.pose.covariance.assign(0);
    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1000000;
    odom_msg.pose.covariance[21] = 1000000;
    odom_msg.pose.covariance[28] = 1000000;
    odom_msg.pose.covariance[35] = 1000;

    odom_msg.twist.covariance.assign(0);
    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[14] = 1000000;
    odom_msg.twist.covariance[21] = 1000000;
    odom_msg.twist.covariance[28] = 1000000;
    odom_msg.twist.covariance[35] = 1000;

#ifdef _ODOM_SENSORS
//  voltage_msg.header.seq = 0;
//  voltage_msg.header.frame_id = 0;
//  current_msg.header.seq = 0;
//  current_msg.header.frame_id = 0;
//  energy_msg.header.seq = 0;
//  energy_msg.header.frame_id = 0;
//  temperature_msg.header.seq = 0;
//  temperature_msg.header.frame_id = 0;
#endif
}

void MainNode::odom_loop() {
    ROS_INFO_STREAM("enter odom loop1");

    // read sensor data stream from motor controller
    // Roboteq Tender Wheels
    if (mainWheelController.IsConnected()) {
        // Roboteq J & S

        // CR : encoder counts
        mainWheelController.GetValue(_CR, 1, odom_encoder_Second);
        mainWheelController.GetValue(_CR, 2, odom_encoder_Jockey);
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("encoder right: " << odom_encoder_right << " left: " << odom_encoder_left);
#endif

        // V : voltage
        int volt;
        mainWheelController.GetValue(_V, 1, volt);
        voltage = volt / 10.0;
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("V: " << voltage);
#endif

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
        mainWheelController.GetValue(_BA, 1, currentRight);
        mainWheelController.GetValue(_BA, 2, currentLeft);
        current_right = currentRight / 10.0;
        current_left = currentLeft / 10.0;
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("Current right: " << current_right);
        ROS_DEBUG_STREAM("Current left: " << current_left);
#endif
        ROS_INFO_STREAM("end odom loop1");
    } else {
        mainWheelController.Connect(port);
    }
}

void MainNode::odom_loop2() {
    ROS_INFO_STREAM("enter odom loop2");

    // read sensor data stream from motor controller
    // Roboteq Tender Wheels
    if (jockeyAndSecWheelController.IsConnected()) {
        // Roboteq J & S

        // CR : encoder counts
        jockeyAndSecWheelController.GetValue(_CR, 0, odom_encoder_Second);
        jockeyAndSecWheelController.GetValue(_CR, 1, odom_encoder_Jockey);
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("encoder Jockey: " << odom_encoder_Jockey << " Second: " << odom_encoder_Second);
#endif

        // V : voltage
        int volt;
        jockeyAndSecWheelController.GetValue(_V, 0, volt);
        jockeyAndSecWheelVoltage = volt / 10.0;
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("V2: " << jockeyAndSecWheelVoltage);
#endif

        // P : PWM
        int pwmSecond, pwmJockey;
        jockeyAndSecWheelController.GetValue(_P, 0, pwmSecond);
        jockeyAndSecWheelController.GetValue(_P, 1, pwmJockey);
        PWM_Second = pwmSecond / 10.0;
        PWM_Jockey = pwmJockey / 10.0;
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("PWM Second: " << PWM_Second);
        ROS_DEBUG_STREAM("PWM Jockey: " << PWM_Jockey);
#endif

        // S : RPM
        jockeyAndSecWheelController.GetValue(_S, 0, RPM_Second);
        jockeyAndSecWheelController.GetValue(_S, 1, RPM_Jockey);
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("RPM Second: " << RPM_Second);
        ROS_DEBUG_STREAM("RPM Jockey: " << RPM_Jockey);
#endif

        // BA : motor currents
        int currentSecond, currentJockey;
        jockeyAndSecWheelController.GetValue(_BA, 0, currentSecond);
        jockeyAndSecWheelController.GetValue(_BA, 1, currentJockey);
        current_Second = currentSecond / 10.0;
        current_Jockey = currentJockey / 10.0;
#ifdef _ODOM_DEBUG
        ROS_DEBUG_STREAM("Current Second: " << current_Second);
        ROS_DEBUG_STREAM("Current Jockey: " << current_Jockey);
#endif
        ROS_INFO_STREAM("end odom loop2");
    } else {
        jockeyAndSecWheelController.Connect(port2);
    }
}

void MainNode::odom_hs_run() {
}

void MainNode::odom_ms_run() {

#ifdef _ODOM_SENSORS
//  current_msg.header.seq++;
//  current_msg.header.stamp = ros::Time::now();
    current_msg.a = current_right;
    current_msg.b = current_left;
    current_pub.publish(current_msg);
#endif

}

void MainNode::odom_ls_run() {

#ifdef _ODOM_SENSORS
//  voltage_msg.header.seq++;
//  voltage_msg.header.stamp = ros::Time::now();
    voltage_msg.data = voltage;
    voltage_pub.publish(voltage_msg);
//  energy_msg.header.seq++;
//  energy_msg.header.stamp = ros::Time::now();
    energy_msg.data = energy;
    energy_pub.publish(energy_msg);
//  temperature_msg.header.seq++;
//  temperature_msg.header.stamp = ros::Time::now();
    temperature_msg.data = temperature;
    temperature_pub.publish(temperature_msg);
#endif

}

void MainNode::odom_publish() {

    // determine delta time in seconds
    uint32_t nowTime = millis();
    float dt = (float) DELTA(nowTime, odom_last_time) / 1000.0;
    odom_last_time = nowTime;

#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("right: ");
ROS_DEBUG(odom_encoder_right);
ROS_DEBUG(" left: ");
ROS_DEBUG(odom_encoder_left);
ROS_DEBUG(" dt: ");
ROS_DEBUG(dt);
ROS_DEBUG("");
*/
#endif

    // determine deltas of distance and angle
    float linear = ((float) odom_encoder_right / (float) encoder_cpr * wheel_circumference +
                    (float) odom_encoder_left / (float) encoder_cpr * wheel_circumference) / 2.0;
//  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width * -1.0;
    float angular = ((float) odom_encoder_right / (float) encoder_cpr * wheel_circumference -
                     (float) odom_encoder_left / (float) encoder_cpr * wheel_circumference) / track_width;
#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("linear: ");
ROS_DEBUG(linear);
ROS_DEBUG(" angular: ");
ROS_DEBUG(angular);
ROS_DEBUG("");
*/
#endif

    // Update odometry
    odom_x += linear * cos((double) odom_yaw);        // m
    odom_y += linear * sin((double) odom_yaw);        // m
    odom_yaw = NORMALIZE((double) odom_yaw + angular);  // rad
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "odom x: " << odom_x << " y: " << odom_y << " yaw: " << odom_yaw );
#endif

    // Calculate velocities
    float vx = (odom_x - odom_last_x) / dt;
    float vy = (odom_y - odom_last_y) / dt;
    float vyaw = (odom_yaw - odom_last_yaw) / dt;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "velocity vx: " << odom_x << " vy: " << odom_y << " vyaw: " << odom_yaw );
#endif
    odom_last_x = odom_x;
    odom_last_y = odom_y;
    odom_last_yaw = odom_yaw;
#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("vx: ");
ROS_DEBUG(vx);
ROS_DEBUG(" vy: ");
ROS_DEBUG(vy);
ROS_DEBUG(" vyaw: ");
ROS_DEBUG(vyaw);
ROS_DEBUG("");
*/
#endif

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);

    if (pub_odom_tf) {
        tf_msg.header.seq++;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.transform.translation.x = odom_x;
        tf_msg.transform.translation.y = odom_y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = quat;
        odom_broadcaster.sendTransform(tf_msg);
    }

    odom_msg.header.seq++;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vyaw;
    odom_pub.publish(odom_msg);

}


int MainNode::run() {

    ROS_INFO("Beginning setup...");

    // TODO: support automatic re-opening of port after disconnection
    while (ros::ok()) {
        int status;

        ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "...");
        status = mainWheelController.Connect(port);
        if (status == RQ_SUCCESS) {
            ROS_INFO("Successfully connected to main wheels controller");
        } else {
            ROS_WARN_STREAM("Failed to connect to main wheels controller");
        }

        ROS_INFO_STREAM("Opening serial port2 on " << port2 << " at " << baud2 << "...");
        status = jockeyAndSecWheelController.Connect(port2);
        if (status == RQ_SUCCESS) {
            ROS_INFO("Successfully connected to jockey wheel and secondary wheel controller");
        } else {
            ROS_WARN_STREAM("Failed to connect to jockey wheel and secondary wheel controller");
        }

        if (mainWheelController.IsConnected() && jockeyAndSecWheelController.IsConnected()) {
            break;
        }

        sleep(5);
    }

    cmdvel_setup();
    odom_setup();

    startTime = millis();
    hsTimer = startTime;
    msTimer = startTime;
    lsTimer = startTime;

    ros::Rate loop_rate(10);

    ROS_INFO("Beginning looping...");

    while (ros::ok()) {
        odom_loop();
        odom_loop2();
        uint32_t nowTime = millis();

        // Handle 30 Hz publishing
        if (DELTA(nowTime, hsTimer) >= 33) {
            hsTimer = nowTime;
            odom_hs_run();
        }

        // Handle 10 Hz publishing
        if (DELTA(nowTime, msTimer) >= 100) {
            msTimer = nowTime;
            odom_ms_run();
        }

        // Handle 1 Hz publishing
        if (DELTA(nowTime, lsTimer) >= 1000) {
            lsTimer = nowTime;
            odom_ls_run();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (mainWheelController.IsConnected())
        mainWheelController.Disconnect();

    if (jockeyAndSecWheelController.IsConnected())
        jockeyAndSecWheelController.Disconnect();

    ROS_INFO("Exiting");

    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");

    MainNode node;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    return node.run();
}


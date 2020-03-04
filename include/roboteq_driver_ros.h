#include <ros/ros.h>

#include "RoboteqDevice.h"

class MainNode {

public:
    int run();

    MainNode();
    ~MainNode();

private:

    //
    // roboteq controllers setup
    //
    void main_wheel_controller_setup();
    void jockey_and_sec_wheel_controller_setup();
    void controllers_setup();

    //
    // cmd_vel subscriber
    //
    void cmdvel_callback(const geometry_msgs::Twist &twist_msg);
    void cmd_jw_callback(const std_msgs::Int32::ConstPtr& msg);
    void cmd_ts_callback(const std_msgs::Int32::ConstPtr& msg);
    void cmdvel_setup();
    void cmd_jw_setup();
    void cmd_ts_setup();

    //
    //mode change subscriber
    //   
    void mode_jw_callback(const std_msgs::UInt8::ConstPtr& msg);   
    uint8_t jw_mode; 
    void mode_ts_callback(const std_msgs::UInt8::ConstPtr& msg);
    uint8_t ts_mode; 
    void mode_motor_callback(const std_msgs::UInt8::ConstPtr& msg);
    uint8_t motor_mode;
    //
    // odom publisher
    //
    void odom_setup();
    void odom_loop();
    void odom_loop2();
    void odom_hs_run();
    void odom_ms_run();
    void odom_ls_run();

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
    ros::Subscriber cmd_ts_sub;
    ros::Subscriber cmd_jw_sub;
    
    //
    // motor mode subscriber
    //
    ros::Subscriber mode_ts_sub;
    ros::Subscriber mode_jw_sub;
    ros::Subscriber mode_motor_sub;

#ifdef _ODOM_SENSORS
    std_msgs::Float32 voltage_msg;
    ros::Publisher voltage_pub;
    std_msgs::Float32MultiArray current_msg;
    ros::Publisher current_pub;
    std_msgs::Float32MultiArray RPM_msg;
    ros::Publisher RPM_pub;
    std_msgs::Float32 energy_msg;
    ros::Publisher energy_pub;
    std_msgs::Float32 temperature_msg;
    ros::Publisher temperature_pub;
    std_msgs::UInt8 input3_msg;
    ros::Publisher input3_pub;
    std_msgs::UInt8 input4_msg;
    ros::Publisher input4_pub;
    ros::Publisher encoder_ts_pub;
    ros::Publisher encoder_jw_pub;
#endif

    // toss out initial encoder readings
    int32_t odom_encoder_left{};
    int32_t odom_encoder_right{};
    int32_t odom_encoder_Jockey{};
    int32_t odom_encoder_Second{};

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
    int input3;
    int input4;

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
    std::string cmdvel_topic;
    std::string cmd_jw_topic;
    std::string cmd_ts_topic;
    std::string mode_jw_topic;
    std::string mode_ts_topic;
    std::string mode_motor_topic;

    // settings  Roboteq Tender Wheels
    std::string port;
    bool open_loop;
    double wheel_circumference;
    double track_width;
    int encoder_ppr;
    int encoder_cpr;

    //settings  Roboteq J & S
    std::string port2;
    bool open_loop2;
    int encoder_ppr2;
    int encoder_cpr2;

    int volt_low_count;
};

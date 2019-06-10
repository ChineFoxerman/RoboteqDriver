#include <ros/ros.h>

#include "RoboteqDevice.h"

class MainNode {

public:
    int run();

    MainNode();
    ~MainNode();

private:

    //
    // roboteq motors setup
    //
    void main_wheel_controller_setup();
    void jockey_and_sec_wheel_controller_setup();

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

#ifdef _ODOM_SENSORS
    std_msgs::Float32 voltage_msg;
    ros::Publisher voltage_pub;
    std_msgs::Float32MultiArray current_msg;
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
    std::string cmdvel_topic;

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
};

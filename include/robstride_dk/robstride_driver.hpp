#pragma once

#include <string>
#include <cstdint>
#include <atomic>
#include <iostream>
#include <cstring>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <optional>
#include <vector>
#include <map>
#include <bitset>
#include <iomanip>
#include <tuple>
#include <sys/time.h>
#include <stdexcept>   // runtime_error
#include <cstdio>      // printf, perror

#define Set_mode 	  'j' //设置控制模式
#define Set_parameter 'p' //设置参数
//各种控制模式
#define move_control_mode      0 //运控模式
#define PosPP_control_mode     1 //位置模式
#define Speed_control_mode     2 //速度模式
#define Elect_control_mode     3 //电流模式
#define Set_Zero_mode          4 //零点模式
#define PosCSP_control_mode    5 //位置模式CSP

#define SC_MAX    23.0f
#define SC_MIN     0.0f
#define SV_MAX    20.0f
#define SV_MIN   -20.0f
#define SCIQ_MIN -23.0f

//通信地址
#define Communication_Type_Get_ID            0x00
#define Communication_Type_MotionControl     0x01
#define Communication_Type_MotorRequest      0x02
#define Communication_Type_MotorEnable       0x03
#define Communication_Type_MotorStop         0x04
#define Communication_Type_SetPosZero        0x06
#define Communication_Type_Can_ID            0x07
#define Communication_Type_GetSingleParameter 0x11
#define Communication_Type_SetSingleParameter 0x12
#define Communication_Type_ErrorFeedback     0x15

using ReceiveResult =
  std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

typedef struct
{
    int   set_motor_mode;
    float set_current;
    float set_speed;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_Kp;
    float set_Ki;
    float set_Kd;
    float set_iq;
    float set_id;
    float set_acc;
} Motor_Set;

class data_read_write_one {
public:
    uint16_t index;
    float    data;
};

enum class ActuatorType {
    ROBSTRIDE_00 = 0,
    ROBSTRIDE_01 = 1,
    ROBSTRIDE_02 = 2,
    ROBSTRIDE_03 = 3,
    ROBSTRIDE_04 = 4,
    ROBSTRIDE_05 = 5,
    ROBSTRIDE_06 = 6
};

struct ActuatorOperation {
    double position;   // rad
    double velocity;   // rad/s
    double torque;     // Nm
    double kp;
    double kd;
};

static const std::map<ActuatorType, ActuatorOperation> ACTUATOR_OPERATION_MAPPING = {
    { ActuatorType::ROBSTRIDE_00, { 4 * M_PI, 50, 17,   500.0,  5.0 } },
    { ActuatorType::ROBSTRIDE_01, { 4 * M_PI, 44, 17,   500.0,  5.0 } },
    { ActuatorType::ROBSTRIDE_02, { 4 * M_PI, 44, 17,   500.0,  5.0 } },
    { ActuatorType::ROBSTRIDE_03, { 4 * M_PI, 50, 60,  5000.0, 100.0 } },
    { ActuatorType::ROBSTRIDE_04, { 4 * M_PI, 15, 120, 5000.0, 100.0 } },
    { ActuatorType::ROBSTRIDE_05, { 4 * M_PI, 33, 17,   500.0,  5.0 } },
    { ActuatorType::ROBSTRIDE_06, { 4 * M_PI, 20, 60,  5000.0, 100.0 } },
};

static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010,
    0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
    0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

class data_read_write
{
public:
    data_read_write_one run_mode;
    data_read_write_one iq_ref;
    data_read_write_one spd_ref;
    data_read_write_one imit_torque;
    data_read_write_one cur_kp;
    data_read_write_one cur_ki;
    data_read_write_one cur_filt_gain;
    data_read_write_one loc_ref;
    data_read_write_one limit_spd;
    data_read_write_one limit_cur;
    data_read_write_one mechPos;
    data_read_write_one iqf;
    data_read_write_one mechVel;
    data_read_write_one VBUS;
    data_read_write_one rotation;

    data_read_write(const uint16_t *index_list = Index_List)
    {
        run_mode.index      = index_list[0];
        iq_ref.index        = index_list[1];
        spd_ref.index       = index_list[2];
        imit_torque.index   = index_list[3];
        cur_kp.index        = index_list[4];
        cur_ki.index        = index_list[5];
        cur_filt_gain.index = index_list[6];
        loc_ref.index       = index_list[7];
        limit_spd.index     = index_list[8];
        limit_cur.index     = index_list[9];
        mechPos.index       = index_list[10];
        iqf.index           = index_list[11];
        mechVel.index       = index_list[12];
        VBUS.index          = index_list[13];
        rotation.index      = index_list[14];
    }
};

class RobStrideMotor
{
public:
 RobStrideMotor(const std::string& can_interface,
                   uint8_t master_id,
                   uint8_t motor_id,
                   int actuator_type);

    ~RobStrideMotor();

    ReceiveResult receive(double timeout_sec = 0.0);

    std::tuple<float, float, float, float> return_data_pvtt();

    bool receive_status_frame();
    void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);
    std::tuple<float, float, float, float> send_velocity_mode_command(float velocity_rad_s);
    std::tuple<float, float, float, float> enable_motor();
    float read_initial_position();
    bool init_socket();

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    std::tuple<float, float, float, float> send_motion_command(
        float torque,
        float position_rad,
        float velocity_rad_s,
        float kp = 0.5f,
        float kd = 0.1f);

    float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        return static_cast<float>(x_int) * span / ((1 << bits) - 1) + x_min;
    }

    void Get_RobStrite_Motor_parameter(uint16_t Index);
    std::tuple<float, float, float, float> RobStrite_Motor_PosPP_control(
        float Speed, float Acceleration, float Angle);
    std::tuple<float, float, float, float> RobStrite_Motor_PosCSP_control(
        float Speed, float Angle);
    std::tuple<float, float, float, float> RobStrite_Motor_Current_control(
        float IqCommand, float IdCommand);
    void RobStrite_Motor_Set_Zero_control();
    void Disenable_Motor(uint8_t clear_error);
    void Set_CAN_ID(uint8_t Set_CAN_ID);
    void Set_ZeroPos();

    float Byte_to_float(uint8_t* bytedata)
    {
        uint32_t data = (bytedata[7] << 24) |
                        (bytedata[6] << 16) |
                        (bytedata[5] << 8)  |
                         bytedata[4];
        float data_float;
        std::memcpy(&data_float, &data, sizeof(float));
        return data_float;
    }

    float Byte_to_float(const std::vector<uint8_t>& bytedata)
    {
        if (bytedata.size() < 8) return 0.0f;
        uint32_t data = (bytedata[7] << 24) |
                        (bytedata[6] << 16) |
                        (bytedata[5] << 8)  |
                         bytedata[4];
        float data_float;
        std::memcpy(&data_float, &data, sizeof(float));
        return data_float;
    }

public:
    std::string iface;
    uint8_t master_id;
    uint8_t motor_id;
    int socket_fd = -1;

    Motor_Set           Motor_Set_All;
    data_read_write_one params;
    data_read_write     drw;

    float position_   = 0.0f;
    float velocity_   = 0.0f;
    float torque_     = 0.0f;
    float temperature_= 0.0f;

    uint8_t error_code = 0;
    uint8_t pattern    = 0;
    std::atomic<bool> is_move_control_first { true };
    int actuator_type;
};

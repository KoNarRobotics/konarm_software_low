#pragma once

#include "config.hpp"
#include "can_messages.h"
//************************************************************
// CODE/DECODE CLASS
class Config_code {
public:
  Config_code() {};
  // uint8_t ID;
  // IdConfig current_config;

  void encode(uint8_t ID, uint8_t *tab, IdConfig &current_config) {
    uint32_t val;
    if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_STEPS_PER_REV_CHOICE)
      val = current_config.stepper_motor_steps_per_rev;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_GEAR_RATIO_CHOICE)
      val = current_config.stepper_motor_gear_ratio;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_MAX_VELOCITY_CHOICE)
      val = current_config.stepper_motor_max_velocity;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_MIN_VELOCITY_CHOICE)
      val = current_config.stepper_motor_min_velocity;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_REVERSE_CHOICE)
      val = current_config.stepper_motor_reverse;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_ENABLE_REVERSED_CHOICE)
      val = current_config.stepper_motor_enable_reversed;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_TIMER_PRESCALER_CHOICE)
      val = current_config.stepper_motor_timer_prescaler;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_OFFSET_CHOICE)
      val = current_config.encoder_arm_offset;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_REVERSE_CHOICE)
      val = current_config.encoder_arm_reverse;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_DEAD_ZONE_CORRECTION_ANGLE_CHOICE)
      val = current_config.encoder_arm_dead_zone_correction_angle;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_VELOCITY_SAMPLE_AMOUNT_CHOICE)
      val = current_config.encoder_arm_velocity_sample_amount;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_OFFSET_CHOICE)
      val = current_config.encoder_motor_offset;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_REVERSE_CHOICE)
      val = current_config.encoder_motor_reverse;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_DEAD_ZONE_CORRECTION_ANGLE_CHOICE)
      val = current_config.encoder_motor_dead_zone_correction_angle;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_VELOCITY_SAMPLE_AMOUNT_CHOICE)
      val = current_config.encoder_motor_velocity_sample_amount;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_ENABLE_CHOICE)
      val = current_config.encoder_motor_enable;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_P_CHOICE)
      val = current_config.pid_p;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_I_CHOICE)
      val = current_config.pid_i;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_D_CHOICE)
      val = current_config.pid_d;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_MAX_VELOCITY_CHOICE)
      val = current_config.movement_max_velocity;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_LIMIT_LOWER_CHOICE)
      val = current_config.movement_limit_lower;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_LIMIT_UPPER_CHOICE)
      val = current_config.movement_limit_upper;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_CONTROL_MODE_CHOICE)
      val = current_config.movement_control_mode;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_MAX_ACCELERATION_CHOICE)
      val = current_config.movement_max_acceleration;
    tab[0] = ID;
    tab[1] = val >> 24;
    tab[2] = (val & 0x00ff0000) >> 16;
    tab[3] = (val & 0x0000ff00) >> 8;
    tab[4] = (val & 0x000000ff);
  }

  void decode(uint8_t *tab, IdConfig &current_config) {

    uint8_t ID   = tab[0];
    uint32_t val = tab[1] << 24 | tab[2] << 16 | tab[3] << 8 | tab[4];

    if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_STEPS_PER_REV_CHOICE)
      current_config.stepper_motor_steps_per_rev = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_GEAR_RATIO_CHOICE)
      current_config.stepper_motor_gear_ratio = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_MAX_VELOCITY_CHOICE)
      current_config.stepper_motor_max_velocity = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_MIN_VELOCITY_CHOICE)
      current_config.stepper_motor_min_velocity = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_REVERSE_CHOICE)
      current_config.stepper_motor_reverse = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_ENABLE_REVERSED_CHOICE)
      current_config.stepper_motor_enable_reversed = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_STEPPER_MOTOR_TIMER_PRESCALER_CHOICE)
      current_config.stepper_motor_timer_prescaler = val;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_OFFSET_CHOICE)
      current_config.encoder_arm_offset = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_REVERSE_CHOICE)
      current_config.encoder_arm_reverse = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_DEAD_ZONE_CORRECTION_ANGLE_CHOICE)
      current_config.encoder_arm_dead_zone_correction_angle = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_ARM_VELOCITY_SAMPLE_AMOUNT_CHOICE)
      current_config.encoder_arm_velocity_sample_amount = val;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_OFFSET_CHOICE)
      current_config.encoder_motor_offset = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_REVERSE_CHOICE)
      current_config.encoder_motor_reverse = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_DEAD_ZONE_CORRECTION_ANGLE_CHOICE)
      current_config.encoder_motor_dead_zone_correction_angle = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_VELOCITY_SAMPLE_AMOUNT_CHOICE)
      current_config.encoder_motor_velocity_sample_amount = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_ENCODER_MOTOR_ENABLE_CHOICE)
      current_config.encoder_motor_enable = val;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_P_CHOICE)
      current_config.pid_p = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_I_CHOICE)
      current_config.pid_i = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_PID_D_CHOICE)
      current_config.pid_d = val;

    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_MAX_VELOCITY_CHOICE)
      current_config.movement_max_velocity = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_LIMIT_LOWER_CHOICE)
      current_config.movement_limit_lower = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_LIMIT_UPPER_CHOICE)
      current_config.movement_limit_upper = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_CONTROL_MODE_CHOICE)
      current_config.movement_control_mode = val;
    else if(ID == CAN_KONARM_1_GET_CONFIG_ASK_FOR_CONFIG_MOVEMENT_MAX_ACCELERATION_CHOICE)
      current_config.movement_max_acceleration = val;
    // }
  }
};
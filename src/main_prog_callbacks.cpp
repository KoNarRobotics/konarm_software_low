#include "Timing.hpp"
#include "can_messages.h"
// #include "can_control.hpp"
#include "config.hpp"
#include "main.h"
#include "main_prog.hpp"
#include "stm32f4xx_hal.h"
#include "can_config.hpp"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM10) {
    stmepic::Ticker::get_instance().irq_update_ticker();
  }

  if(htim->Instance == TIM14) {
    HAL_IncTick();
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if(hadc->Instance == ADC1) {
    pin_temp_steper_board.analog_value = adc_dma_buffer[0];
    pin_temp_board.analog_value        = adc_dma_buffer[1];
    pin_temp_motor.analog_value        = adc_dma_buffer[2];
    pin_vsense.analog_value            = adc_dma_buffer[3];
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if(hadc->Instance == ADC1) {
    pin_poz_zero_sensor.analog_value = adc_dma_buffer[4];
    pin_inout_ca1.analog_value       = adc_dma_buffer[5];
    pin_inout_ca2.analog_value       = adc_dma_buffer[6];
    pin_inout_crx.analog_value       = adc_dma_buffer[7];
  }
}

void can_callback_get_config(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)args;

  Config_code config_to_get;
  config_to_get.current_config = config_temp;

  config_to_get.decode(recived_msg.data);

  stmepic::CanDataFrame send_msg;
  send_msg.frame_id  = config_temp.can_konarm_get_config_frame_id;
  send_msg.data_size = 5;

  config_to_get.encode(send_msg.data);

  can.write(send_msg);
}

void can_callback_send_config(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)args;

  Config_code config_to_send;
  config_to_send.current_config = config_temp;

  config_to_send.decode(recived_msg.data);

  config_temp = config_to_send.current_config;
}

void can_callback_set_and_reset(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)recived_msg;
  (void)args;
  (void)fram->writeStruct(FRAM_CONFIG_ADDRESS, config_temp);
  HAL_NVIC_SystemReset;
}

void can_callback_set_pos(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)args;
  can_disconnect_timeout_reset();
  can_konarm_1_set_pos_t signals;
  (void)can_konarm_1_set_pos_unpack(&signals, recived_msg.data, recived_msg.data_size);
  // float targetPosition = can_konarm_1_set_pos_position_decode(signals.position);
  // float targetVelocity = can_konarm_1_set_pos_velocity_decode(signals.velocity);
  movement_controler.set_velocity(signals.velocity);
  movement_controler.set_position(signals.position);
  movement_controler.set_enable(true);
  log_debug("Set position:" + std::to_string(signals.position) + "velocity:" + std::to_string(signals.velocity));
}

void can_callback_get_pos(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)recived_msg;
  (void)args;

  can_disconnect_timeout_reset();
  stmepic::CanDataFrame send_msg;
  can_konarm_1_get_pos_t src_p;
  send_msg.frame_id  = config.can_konarm_get_pos_frame_id;
  src_p.position     = movement_controler.get_current_position();
  src_p.velocity     = movement_controler.get_current_velocity();
  send_msg.data_size = CAN_KONARM_1_GET_POS_LENGTH;
  (void)can_konarm_1_get_pos_pack(send_msg.data, &src_p, send_msg.data_size);
  can.write(send_msg);
}

void can_callback_status(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)recived_msg;
  (void)args;
  can_disconnect_timeout_reset();
  stmepic::CanDataFrame send_msg;
  can_konarm_1_status_t src_p;
  send_msg.frame_id  = config.can_konarm_status_frame_id;
  src_p.status       = can_konarm_1_status_status_encode(CAN_KONARM_1_STATUS_STATUS_OK_CHOICE);
  send_msg.data_size = CAN_KONARM_1_STATUS_LENGTH;
  can_konarm_1_status_pack(send_msg.data, &src_p, send_msg.data_size);
  can.write(send_msg);
}

void can_callback_clear_errors(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)recived_msg;
  (void)args;
  can_disconnect_timeout_reset();
}

void can_callback_get_errors(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)recived_msg;
  (void)args;
  can_disconnect_timeout_reset();
  stmepic::CanDataFrame send_msg;
  can_konarm_1_get_errors_t src_p;
  send_msg.frame_id                    = config.can_konarm_get_errors_frame_id;
  src_p.temp_engine_overheating        = error_data.temp_engine_overheating;
  src_p.temp_driver_overheating        = error_data.temp_driver_overheating;
  src_p.temp_board_overheating         = error_data.temp_board_overheating;
  src_p.temp_engine_sensor_disconnect  = error_data.temp_engine_sensor_disconnect;
  src_p.temp_driver_sensor_disconnect  = error_data.temp_driver_sensor_disconnect;
  src_p.temp_board_sensor_disconnect   = error_data.temp_board_sensor_disconnect;
  src_p.encoder_arm_disconnect         = error_data.encoder_arm_disconnect;
  src_p.encoder_motor_disconnect       = error_data.encoder_motor_disconnect;
  src_p.board_overvoltage              = error_data.baord_overvoltage;
  src_p.board_undervoltage             = error_data.baord_undervoltage;
  src_p.can_disconnected               = error_data.can_disconnected;
  src_p.can_error                      = error_data.can_error;
  src_p.controler_motor_limit_position = error_data.controler_motor_limit_position;
  send_msg.data_size                   = CAN_KONARM_1_GET_ERRORS_LENGTH;
  can_konarm_1_get_errors_pack(send_msg.data, &src_p, send_msg.data_size);
  can.write(send_msg);
  error_data.can_error = false;
}

void can_callback_default(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)recived_msg;
  (void)args;
  can_disconnect_timeout_reset();
  error_data.can_error = true;
}

void can_callback_set_control_mode(stmepic::CanBase &can, stmepic::CanDataFrame &recived_msg, void *args) {
  (void)can;
  (void)args;
  can_disconnect_timeout_reset();
  can_konarm_1_set_control_mode_t signals;
  can_konarm_1_set_control_mode_unpack(&signals, recived_msg.data, recived_msg.data_size);
  if(!can_konarm_1_set_control_mode_control_mode_is_in_range(signals.control_mode)) {
    error_data.can_error = true;
    log_error("Control mode is out of range");
    return;
  }
  init_and_set_movement_controler_mode(signals.control_mode);
  log_debug("Set control mode:" + std::to_string(signals.control_mode));
}
#ifndef _T_RAX_BOARD_CONFIG_H_
#define _T_RAX_BOARD_CONFIG_H_

#include <driver/gpio.h>

// Audio sample rates
#define AUDIO_INPUT_SAMPLE_RATE 16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// I2S Simplex mode for separate INMP441 Mic and MAX98357A Amp
#define AUDIO_I2S_METHOD_SIMPLEX

// INMP441 I2S Microphone pins
#define AUDIO_I2S_MIC_GPIO_WS GPIO_NUM_1
#define AUDIO_I2S_MIC_GPIO_SCK GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_DIN GPIO_NUM_3

// MAX98357A I2S Amplifier pins
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_4
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_6

// Buttons & Status LED
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// WS2812 RGB LED (Eye Effect)
#define WS2812_RGB_LED_GPIO GPIO_NUM_48

// I2C Pins for VL53L0X ToF Distance Sensor
#define TOF_I2C_SDA_GPIO GPIO_NUM_NC
#define TOF_I2C_SCL_GPIO GPIO_NUM_NC

// DRV8833 Motor Driver Pins (Track DC Motors)
#define MOTOR_LEFT_AIN1_GPIO GPIO_NUM_10
#define MOTOR_LEFT_AIN2_GPIO GPIO_NUM_11
#define MOTOR_RIGHT_BIN1_GPIO GPIO_NUM_12
#define MOTOR_RIGHT_BIN2_GPIO GPIO_NUM_13

// Head Movement Servo Pins (PWM)
#define SERVO_PAN_PAN_GPIO GPIO_NUM_8  // Trục xoay ngang (Yaw)
#define SERVO_TILT_GPIO GPIO_NUM_9     // Trục gật lên xuống (Pitch)

// Driver Enable Pin (DRV8833 nSLEEP / Amp Enable)
#define DRIVER_ENABLE_GPIO GPIO_NUM_7

#endif  // _T_RAX_BOARD_CONFIG_H_

#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "wifi_board.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <math.h>
#include <atomic>
#include <mutex>

#define TAG "TRaxBoard"

// Standard VL53L0X I2C Address
#define VL53L0X_I2C_ADDR 0x29

// 23 Robot States Enum
enum TRaxState {
    kStateCurious = 1,     // 1. Tò mò
    kStateFocused,         // 2. Tập trung
    kStateAlertWarning,    // 3. Cảnh báo
    kStateAngry,           // 4. Tức giận
    kStateScared,          // 5. Sợ hãi
    kStateHappy,           // 6. Vui vẻ
    kStateDisappointed,    // 7. Thất vọng
    kStateTargetDetected,  // 8. Phát hiện mục tiêu
    kStateConfused,        // 9. Bối rối
    kStateSurprised,       // 10. Ngạc nhiên
    kStateSuspicious,      // 11. Nghi ngờ
    kStateLoving,          // 12. Yêu thương
    kStateVictorious,      // 13. Chiến thắng
    kStateShy,             // 14. E ngại
    kStateBored,           // 15. Chán nản
    kStateArrogant,        // 16. Kiêu ngạo
    kStateSearching,       // 17. Tìm kiếm
    kStateSystemError,     // 18. Lỗi hệ thống
    kStateLowBattery,      // 19. Sắp hết pin
    kStateCharging,        // 20. Đang sạc pin
    kStateBooting,         // 21. Khởi động
    kStateSleeping,        // 22. Ngủ
    kStateIdle             // 23. Chế độ IDLE
};

enum EyeLedMode { kEyeModeBreathing, kEyeModeStrobe, kEyeModeSolid, kEyeModeBlink, kEyeModeOff };

// Organic Easing Functions Engine
class OrganicEasing {
public:
    static float CubicEaseInOut(float t) {
        if (t < 0.5f) {
            return 4.0f * t * t * t;
        } else {
            float f = (-2.0f * t + 2.0f);
            return 1.0f - (f * f * f) / 2.0f;
        }
    }

    static float BackEaseOut(float t, float s = 1.70158f) {
        float f = t - 1.0f;
        return f * f * ((s + 1.0f) * f + s) + 1.0f;
    }
};

class TRaxBoard : public WifiBoard {
private:
    Button boot_button_;
    NoDisplay display_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    i2c_master_dev_handle_t vl53l0x_dev_ = nullptr;

    TaskHandle_t tof_task_handle_ = nullptr;
    TaskHandle_t led_task_handle_ = nullptr;
    TaskHandle_t idle_task_handle_ = nullptr;

    led_strip_handle_t eye_led_strip_ = nullptr;
    std::mutex led_mutex_;    // Thread safety lock for RMT LED strip
    std::mutex servo_mutex_;  // Thread safety lock for Servos & LEDC PWM
    TickType_t last_tof_trigger_time_ = 0;

    TRaxState current_state_ = kStateIdle;
    EyeLedMode current_led_mode_ = kEyeModeBreathing;
    uint8_t target_r_ = 0, target_g_ = 200, target_b_ = 255;
    std::atomic<bool> is_performing_action_{
        false};  // Pauses idle micro-movements during emotion execution

    // Debounce: prevent LLM from spamming identical set_state calls during the same turn burst
    TickType_t last_state_change_ticks_ = 0;
    static constexpr uint32_t STATE_DEBOUNCE_MS =
        1200;  // 1.2-second cooldown: spans single-turn animation & prevents duplicate spam without
               // delaying next user turn

    // Current Servo Physical Positions
    float current_pan_ = 90.0f;
    float current_tilt_ = 90.0f;

    // Motor PWM Low-Pass Filter Output State
    float pwm_out_left_ = 0.0f;
    float pwm_out_right_ = 0.0f;

    void InitializeI2c() {
        if (TOF_I2C_SDA_GPIO == GPIO_NUM_NC || TOF_I2C_SCL_GPIO == GPIO_NUM_NC) {
            ESP_LOGW(TAG, "VL53L0X ToF sensor I2C pins are GPIO_NUM_NC. ToF disabled.");
            return;
        }
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = TOF_I2C_SDA_GPIO,
            .scl_io_num = TOF_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_));

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L0X_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &vl53l0x_dev_);
        ESP_LOGI(TAG, "I2C Bus & VL53L0X Device initialized on SDA=%d, SCL=%d", TOF_I2C_SDA_GPIO,
                 TOF_I2C_SCL_GPIO);
    }

    void InitializeWs2812Led() {
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = WS2812_RGB_LED_GPIO;
        strip_config.max_leds = 1;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
        rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10MHz

        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &eye_led_strip_);
        if (err == ESP_OK) {
            std::lock_guard<std::mutex> lock(led_mutex_);
            led_strip_clear(eye_led_strip_);
            ESP_LOGI(TAG, "WS2812 RGB Eye LED initialized on GPIO %d", WS2812_RGB_LED_GPIO);
        } else {
            ESP_LOGE(TAG, "Failed to initialize WS2812 LED: %s", esp_err_to_name(err));
        }
    }

    void InitializeMotorPwm() {
        // LEDC Hardware PWM for DRV8833 DC Motor Driver (Timer 1, 5kHz, 8-bit resolution)
        ledc_timer_config_t motor_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                           .duty_resolution = LEDC_TIMER_8_BIT,  // 0 to 255
                                           .timer_num = LEDC_TIMER_1,
                                           .freq_hz = 5000,  // 5kHz PWM frequency
                                           .clk_cfg = LEDC_AUTO_CLK};
        ledc_timer_config(&motor_timer);

        // Channel 2: Left AIN1
        ledc_channel_config_t ch_ain1 = {.gpio_num = MOTOR_LEFT_AIN1_GPIO,
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = LEDC_CHANNEL_2,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = LEDC_TIMER_1,
                                         .duty = 0,
                                         .hpoint = 0};
        ledc_channel_config(&ch_ain1);

        // Channel 3: Left AIN2
        ledc_channel_config_t ch_ain2 = {.gpio_num = MOTOR_LEFT_AIN2_GPIO,
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = LEDC_CHANNEL_3,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = LEDC_TIMER_1,
                                         .duty = 0,
                                         .hpoint = 0};
        ledc_channel_config(&ch_ain2);

        // Channel 4: Right BIN1
        ledc_channel_config_t ch_bin1 = {.gpio_num = MOTOR_RIGHT_BIN1_GPIO,
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = LEDC_CHANNEL_4,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = LEDC_TIMER_1,
                                         .duty = 0,
                                         .hpoint = 0};
        ledc_channel_config(&ch_bin1);

        // Channel 5: Right BIN2
        ledc_channel_config_t ch_bin2 = {.gpio_num = MOTOR_RIGHT_BIN2_GPIO,
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = LEDC_CHANNEL_5,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = LEDC_TIMER_1,
                                         .duty = 0,
                                         .hpoint = 0};
        ledc_channel_config(&ch_bin2);

        StopMotors();
        ESP_LOGI(
            TAG,
            "DRV8833 Motor LEDC PWM Hardware initialized on AIN1=%d, AIN2=%d, BIN1=%d, BIN2=%d",
            MOTOR_LEFT_AIN1_GPIO, MOTOR_LEFT_AIN2_GPIO, MOTOR_RIGHT_BIN1_GPIO,
            MOTOR_RIGHT_BIN2_GPIO);
    }

    void InitializeServos() {
        ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                          .duty_resolution = LEDC_TIMER_13_BIT,
                                          .timer_num = LEDC_TIMER_0,
                                          .freq_hz = 50,
                                          .clk_cfg = LEDC_AUTO_CLK};
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel_pan = {.gpio_num = SERVO_PAN_PAN_GPIO,
                                                  .speed_mode = LEDC_LOW_SPEED_MODE,
                                                  .channel = LEDC_CHANNEL_0,
                                                  .intr_type = LEDC_INTR_DISABLE,
                                                  .timer_sel = LEDC_TIMER_0,
                                                  .duty = 410,
                                                  .hpoint = 0};
        ledc_channel_config(&ledc_channel_pan);

        ledc_channel_config_t ledc_channel_tilt = {.gpio_num = SERVO_TILT_GPIO,
                                                   .speed_mode = LEDC_LOW_SPEED_MODE,
                                                   .channel = LEDC_CHANNEL_1,
                                                   .intr_type = LEDC_INTR_DISABLE,
                                                   .timer_sel = LEDC_TIMER_0,
                                                   .duty = 410,
                                                   .hpoint = 0};
        ledc_channel_config(&ledc_channel_tilt);
    }

    void SetRawServoAngle(float pan, float tilt) {
        std::lock_guard<std::mutex> lock(servo_mutex_);
        // Expanded mechanical safety range (15..165 for Pan, 10..160 for Tilt)
        if (pan < 15.0f)
            pan = 15.0f;
        if (pan > 165.0f)
            pan = 165.0f;
        if (tilt < 10.0f)
            tilt = 10.0f;
        if (tilt > 160.0f)
            tilt = 160.0f;

        current_pan_ = pan;
        current_tilt_ = tilt;
        uint32_t pan_duty = 205 + (uint32_t)((pan * 410.0f) / 180.0f);
        uint32_t tilt_duty = 205 + (uint32_t)((tilt * 410.0f) / 180.0f);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pan_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, tilt_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }

    // Organic Servo Motion with Cubic Easing & Continuous Trajectory
    void OrganicMoveHead(float target_pan, float target_tilt, int duration_ms,
                         bool use_overshoot = false) {
        float start_pan, start_tilt;
        {
            std::lock_guard<std::mutex> lock(servo_mutex_);
            start_pan = current_pan_;
            start_tilt = current_tilt_;
        }
        int steps = duration_ms / 20;  // 50 FPS (20ms step)
        if (steps < 1)
            steps = 1;

        for (int i = 1; i <= steps; i++) {
            float t = (float)i / (float)steps;
            float ease =
                use_overshoot ? OrganicEasing::BackEaseOut(t) : OrganicEasing::CubicEaseInOut(t);

            float current_p = start_pan + ease * (target_pan - start_pan);
            float current_t = start_tilt + ease * (target_tilt - start_tilt);

            SetRawServoAngle(current_p, current_t);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // DRV8833 LEDC Hardware PWM Smoothing Motor Controller
    void SmoothDriveMotors(float target_left, float target_right, int duration_ms,
                           float alpha = 0.15f) {
        std::lock_guard<std::mutex> lock(servo_mutex_);

        // Cap maximum motor power to 0.45f to prevent electrical current brownout sags on weak power rails
        float boosted_left = fminf(0.45f, fmaxf(-0.45f, target_left * 0.75f));
        float boosted_right = fminf(0.45f, fmaxf(-0.45f, target_right * 0.75f));

        int scaled_duration = (int)(duration_ms * 1.25f);
        if (scaled_duration < 100)
            scaled_duration = 100;

        int steps = scaled_duration / 20;
        if (steps < 1)
            steps = 1;

        uint32_t prev_l2 = 999, prev_l3 = 999, prev_r4 = 999, prev_r5 = 999;

        for (int i = 0; i < steps; i++) {
            // Low-Pass Filter: PWM_out(k) = PWM_out(k-1) + alpha * (PWM_target - PWM_out(k-1))
            pwm_out_left_ = pwm_out_left_ + alpha * (boosted_left - pwm_out_left_);
            pwm_out_right_ = pwm_out_right_ + alpha * (boosted_right - pwm_out_right_);

            // Left Motor Duty (LEDC Channels 2 & 3)
            uint32_t duty_left = (uint32_t)(fabsf(pwm_out_left_) * 255.0f);
            uint32_t duty_l2 = (pwm_out_left_ > 0.05f) ? duty_left : 0;
            uint32_t duty_l3 = (pwm_out_left_ < -0.05f) ? duty_left : 0;

            if (duty_l2 != prev_l2) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_l2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                prev_l2 = duty_l2;
            }
            if (duty_l3 != prev_l3) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty_l3);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                prev_l3 = duty_l3;
            }

            // Right Motor Duty (LEDC Channels 4 & 5)
            uint32_t duty_right = (uint32_t)(fabsf(pwm_out_right_) * 255.0f);
            uint32_t duty_r4 = (pwm_out_right_ > 0.05f) ? duty_right : 0;
            uint32_t duty_r5 = (pwm_out_right_ < -0.05f) ? duty_right : 0;

            if (duty_r4 != prev_r4) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty_r4);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
                prev_r4 = duty_r4;
            }
            if (duty_r5 != prev_r5) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, duty_r5);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
                prev_r5 = duty_r5;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void StopMotors() {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
        pwm_out_left_ = 0.0f;
        pwm_out_right_ = 0.0f;
    }

    void SetEyeColor(uint8_t r, uint8_t g, uint8_t b, EyeLedMode mode = kEyeModeBreathing) {
        std::lock_guard<std::mutex> lock(led_mutex_);
        target_r_ = r;
        target_g_ = g;
        target_b_ = b;
        current_led_mode_ = mode;
    }

    void GenerateFrequencySweepPCM(std::vector<int16_t>& pcm_buffer, float start_freq,
                                   float end_freq, int duration_ms, float amplitude = 12000.0f) {
        int sample_rate = AUDIO_OUTPUT_SAMPLE_RATE;  // 24000 Hz
        int samples = (sample_rate * duration_ms) / 1000;
        pcm_buffer.reserve(pcm_buffer.size() + samples);

        float phase = 0.0f;
        for (int i = 0; i < samples; i++) {
            float t = (float)i / (float)samples;
            float freq = start_freq + (end_freq - start_freq) * t;
            phase += 2.0f * M_PI * freq / sample_rate;
            if (phase >= 2.0f * M_PI)
                phase -= 2.0f * M_PI;

            // Envelope to avoid pop/clicks at start and end
            float env = 1.0f;
            if (i < 100)
                env = (float)i / 100.0f;
            else if (i > samples - 100)
                env = (float)(samples - i) / 100.0f;

            int16_t sample = (int16_t)(sinf(phase) * amplitude * env);
            pcm_buffer.push_back(sample);
        }
    }

    void PlayR2D2Chirp(const char* sound_name) {
        ESP_LOGI(TAG, "R2D2 Audio Synthesizer: Playing [%s]", sound_name);
        std::vector<int16_t> pcm_data;

        std::string sound_str(sound_name);
        if (sound_str == "CURIOUS_WHISTLE") {
            GenerateFrequencySweepPCM(pcm_data, 800, 1800, 180);
            GenerateFrequencySweepPCM(pcm_data, 1800, 1200, 120);
        } else if (sound_str == "FOCUSED_BEEP") {
            GenerateFrequencySweepPCM(pcm_data, 1500, 1500, 80);
            GenerateFrequencySweepPCM(pcm_data, 2000, 2000, 80);
        } else if (sound_str == "ALERT_SWEEP") {
            GenerateFrequencySweepPCM(pcm_data, 600, 2800, 120);
            GenerateFrequencySweepPCM(pcm_data, 2800, 1000, 100);
        } else if (sound_str == "ANGRY_BUZZ") {
            GenerateFrequencySweepPCM(pcm_data, 400, 200, 250, 15000.0f);
        } else if (sound_str == "SCARED_SCREAM") {
            GenerateFrequencySweepPCM(pcm_data, 3200, 1000, 300);
        } else if (sound_str == "HAPPY_ARPEGGIO") {
            GenerateFrequencySweepPCM(pcm_data, 1000, 1000, 40);
            GenerateFrequencySweepPCM(pcm_data, 1400, 1400, 40);
            GenerateFrequencySweepPCM(pcm_data, 1800, 1800, 40);
            GenerateFrequencySweepPCM(pcm_data, 2200, 2200, 40);
            GenerateFrequencySweepPCM(pcm_data, 2600, 2600, 60);
        } else if (sound_str == "SAD_SLIDE_DOWN") {
            GenerateFrequencySweepPCM(pcm_data, 1600, 400, 400);
        } else if (sound_str == "TARGET_LOCK_BEEP") {
            GenerateFrequencySweepPCM(pcm_data, 2400, 2400, 40);
            GenerateFrequencySweepPCM(pcm_data, 2400, 2400, 40);
            GenerateFrequencySweepPCM(pcm_data, 2400, 2400, 40);
        } else if (sound_str == "CONFUSED_QUESTION") {
            GenerateFrequencySweepPCM(pcm_data, 900, 900, 80);
            GenerateFrequencySweepPCM(pcm_data, 900, 2100, 180);
        } else if (sound_str == "SURPRISED_HIGH") {
            GenerateFrequencySweepPCM(pcm_data, 1200, 3200, 150);
        } else if (sound_str == "SUSPICIOUS_LOW") {
            GenerateFrequencySweepPCM(pcm_data, 700, 350, 350);
        } else if (sound_str == "LOVING_PURR") {
            GenerateFrequencySweepPCM(pcm_data, 600, 900, 120);
            GenerateFrequencySweepPCM(pcm_data, 900, 700, 120);
            GenerateFrequencySweepPCM(pcm_data, 700, 1000, 150);
        } else if (sound_str == "FANFARE_CHIRP") {
            GenerateFrequencySweepPCM(pcm_data, 800, 1200, 80);
            GenerateFrequencySweepPCM(pcm_data, 1200, 1600, 80);
            GenerateFrequencySweepPCM(pcm_data, 1600, 2400, 140);
        } else if (sound_str == "SHY_WHIMPER") {
            GenerateFrequencySweepPCM(pcm_data, 2400, 1600, 250);
        } else if (sound_str == "BORED_SIGH") {
            GenerateFrequencySweepPCM(pcm_data, 1000, 350, 500);
        } else if (sound_str == "PROUD_TUNE") {
            GenerateFrequencySweepPCM(pcm_data, 1200, 1800, 90);
            GenerateFrequencySweepPCM(pcm_data, 1800, 2600, 150);
        } else if (sound_str == "SCANNING_RADAR") {
            GenerateFrequencySweepPCM(pcm_data, 2600, 2600, 50);
            GenerateFrequencySweepPCM(pcm_data, 2800, 2800, 50);
            GenerateFrequencySweepPCM(pcm_data, 3000, 3000, 50);
        } else if (sound_str == "GLITCH_NOISE") {
            GenerateFrequencySweepPCM(pcm_data, 300, 2400, 60);
            GenerateFrequencySweepPCM(pcm_data, 2400, 400, 60);
            GenerateFrequencySweepPCM(pcm_data, 500, 2000, 60);
        } else if (sound_str == "LOW_POWER_BEEP") {
            GenerateFrequencySweepPCM(pcm_data, 500, 150, 400);
        } else if (sound_str == "CHARGING_HUM") {
            GenerateFrequencySweepPCM(pcm_data, 250, 750, 400);
        } else if (sound_str == "SNIFF_CHIRP") {
            GenerateFrequencySweepPCM(pcm_data, 1200, 1600, 60);
            GenerateFrequencySweepPCM(pcm_data, 1600, 1400, 60);
        } else if (sound_str == "YAWN_TUNE") {
            GenerateFrequencySweepPCM(pcm_data, 450, 1100, 350);
            GenerateFrequencySweepPCM(pcm_data, 1100, 350, 450);
        } else if (sound_str == "ALARM_SCREAM") {
            GenerateFrequencySweepPCM(pcm_data, 3400, 1000, 120);
            GenerateFrequencySweepPCM(pcm_data, 3400, 1000, 120);
            GenerateFrequencySweepPCM(pcm_data, 3400, 1000, 150);
        } else if (sound_str == "LOW_POWER_DROOP") {
            GenerateFrequencySweepPCM(pcm_data, 800, 200, 600);
        } else if (sound_str == "CHASER_BEEPS") {
            GenerateFrequencySweepPCM(pcm_data, 2400, 2400, 40);
            GenerateFrequencySweepPCM(pcm_data, 2600, 2600, 40);
            GenerateFrequencySweepPCM(pcm_data, 2800, 2800, 40);
            GenerateFrequencySweepPCM(pcm_data, 3000, 3000, 60);
        } else if (sound_str == "DIZZY_WHIMPER") {
            GenerateFrequencySweepPCM(pcm_data, 700, 1300, 120);
            GenerateFrequencySweepPCM(pcm_data, 1300, 600, 120);
            GenerateFrequencySweepPCM(pcm_data, 600, 1100, 150);
        } else if (sound_str == "HERO_TRIUMPH") {
            GenerateFrequencySweepPCM(pcm_data, 900, 1400, 80);
            GenerateFrequencySweepPCM(pcm_data, 1400, 2000, 80);
            GenerateFrequencySweepPCM(pcm_data, 2000, 2800, 160);
        } else if (sound_str == "RADAR_SWEEP_PING") {
            GenerateFrequencySweepPCM(pcm_data, 2800, 2800, 60);
            GenerateFrequencySweepPCM(pcm_data, 3200, 3200, 80);
        } else if (sound_str == "CAUTIOUS_PROBE") {
            GenerateFrequencySweepPCM(pcm_data, 1400, 1000, 180);
            GenerateFrequencySweepPCM(pcm_data, 1000, 1200, 150);
        } else if (sound_str == "ECHO_PULSE_CHIRP") {
            GenerateFrequencySweepPCM(pcm_data, 1800, 1800, 50);
            GenerateFrequencySweepPCM(pcm_data, 2200, 2200, 70);
        } else if (sound_str == "AWE_WONDER_WHISTLE") {
            GenerateFrequencySweepPCM(pcm_data, 600, 1800, 450);
        } else if (sound_str == "TRAIL_HUNTER_CLICK") {
            GenerateFrequencySweepPCM(pcm_data, 2200, 2200, 35);
            GenerateFrequencySweepPCM(pcm_data, 2400, 2400, 35);
            GenerateFrequencySweepPCM(pcm_data, 2600, 2600, 45);
        } else if (sound_str == "GIGGLE_CHIRP") {
            GenerateFrequencySweepPCM(pcm_data, 1800, 2400, 60);
            GenerateFrequencySweepPCM(pcm_data, 2400, 2000, 60);
            GenerateFrequencySweepPCM(pcm_data, 2000, 2600, 80);
        } else if (sound_str == "PRANK_ALARM_GIGGLE") {
            GenerateFrequencySweepPCM(pcm_data, 3200, 1000, 150);
            GenerateFrequencySweepPCM(pcm_data, 1200, 2400, 80);
            GenerateFrequencySweepPCM(pcm_data, 2400, 1800, 80);
        } else if (sound_str == "RUNAWAY_CHICKEN") {
            GenerateFrequencySweepPCM(pcm_data, 2800, 1200, 70);
            GenerateFrequencySweepPCM(pcm_data, 1200, 2200, 70);
            GenerateFrequencySweepPCM(pcm_data, 2200, 1000, 90);
        } else if (sound_str == "STUBBORN_RASPBERRY") {
            GenerateFrequencySweepPCM(pcm_data, 450, 250, 250, 14000.0f);
        } else if (sound_str == "PLAYFUL_BARK_CHIRP") {
            GenerateFrequencySweepPCM(pcm_data, 2000, 2800, 40);
            GenerateFrequencySweepPCM(pcm_data, 2800, 2000, 40);
            GenerateFrequencySweepPCM(pcm_data, 2200, 3000, 60);
        } else {  // Default Chirp
            GenerateFrequencySweepPCM(pcm_data, 1200, 2200, 120);
        }

        if (!pcm_data.empty()) {
            auto codec = GetAudioCodec();
            if (codec != nullptr) {
                codec->EnableOutput(true);
                codec->OutputData(pcm_data);
                // Allow hardware I2S DMA playback to finish and acoustic room echo to decay
                vTaskDelay(pdMS_TO_TICKS(200));
                codec->EnableOutput(false);
            }
        }
    }

    // 28 Improvised Spontaneous Choreography Scenarios (0 to 27)
    void PerformImprovisedScenario(int scenario_id) {
        is_performing_action_.store(true);
        ESP_LOGI(TAG, "🎭 Executing Improvised Scenario #%d", scenario_id);

        switch (scenario_id) {
            case 0:  // 👃 Scenario 0: "Sniff & Explore" (Đánh hơi & Khám phá môi trường)
                SetEyeColor(0, 220, 255, kEyeModeBreathing);
                OrganicMoveHead(60, 40, 450);  // Sniff down-left
                PlayR2D2Chirp("SNIFF_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(250));
                OrganicMoveHead(120, 40, 500);  // Sniff down-right
                PlayR2D2Chirp("SNIFF_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(250));
                OrganicMoveHead(90, 115, 350, true);  // Look up proud
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 90, 300);
                break;

            case 1:  // 💃 Scenario 1: "Playful Wiggle Dance" (Vũ điệu lắc hông vui nhộn)
                SetEyeColor(255, 215, 0, kEyeModeStrobe);
                OrganicMoveHead(75, 110, 250);
                SmoothDriveMotors(0.75f, -0.75f, 180);
                OrganicMoveHead(105, 110, 250);
                SmoothDriveMotors(-0.75f, 0.75f, 180);
                StopMotors();
                OrganicMoveHead(90, 125, 200, true);
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                OrganicMoveHead(90, 90, 300);
                break;

            case 2:  // 🐾 Scenario 2: "Sneaky Dino Prowl" (Rình rập chậm rãi, bước lén)
                SetEyeColor(255, 100, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 50, 400);                // Head low
                SmoothDriveMotors(0.45f, 0.45f, 250, 0.3f);  // Creep forward
                StopMotors();
                OrganicMoveHead(135, 70, 250, true);  // Snap look left
                vTaskDelay(pdMS_TO_TICKS(350));
                OrganicMoveHead(45, 70, 250, true);  // Snap look right
                vTaskDelay(pdMS_TO_TICKS(350));
                OrganicMoveHead(90, 90, 350);
                break;

            case 3:  // 🦜 Scenario 3: "Curious Bird Tilt" (Nghiêng đầu ngơ ngác)
                SetEyeColor(0, 255, 180, kEyeModeSolid);
                OrganicMoveHead(115, 115, 300, true);  // Curious cocked head
                PlayR2D2Chirp("CONFUSED_QUESTION");
                vTaskDelay(pdMS_TO_TICKS(500));
                OrganicMoveHead(65, 115, 350, true);  // Cock other side
                vTaskDelay(pdMS_TO_TICKS(400));
                OrganicMoveHead(90, 90, 300);
                break;

            case 4:  // 👀 Scenario 4: "Surprise Look Behind" (Ngoái đầu kiểm tra sau lưng)
                SetEyeColor(255, 255, 255, kEyeModeStrobe);
                OrganicMoveHead(145, 95, 220, true);  // Snap full left-back
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(35, 95, 280, true);  // Snap full right-back
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 90, 250);
                break;

            case 5:  // 🥱 Scenario 5: "Lazy Stretch & Yawn" (Vươn vai ngáp lười biếng)
                SetEyeColor(120, 80, 200, kEyeModeBreathing);
                OrganicMoveHead(90, 140, 700);  // High stretch
                PlayR2D2Chirp("YAWN_TUNE");
                vTaskDelay(pdMS_TO_TICKS(350));
                OrganicMoveHead(90, 75, 500);  // Relax down
                vTaskDelay(pdMS_TO_TICKS(200));
                OrganicMoveHead(90, 90, 400);
                break;

            case 6:  // 🏆 Scenario 6: "Victory Spin & Nod" (Xoay 1 vòng ăn mừng & gật đầu)
                SetEyeColor(0, 255, 255, kEyeModeStrobe);
                SmoothDriveMotors(0.80f, -0.80f, 320);  // Quick pivot turn
                StopMotors();
                OrganicMoveHead(90, 130, 200, true);  // Nod up
                PlayR2D2Chirp("FANFARE_CHIRP");
                OrganicMoveHead(90, 70, 200);   // Nod down
                OrganicMoveHead(90, 110, 200);  // Nod up
                OrganicMoveHead(90, 90, 250);   // Center
                break;

            case 7:  // 🦖 Scenario 7: "Affectionate Nudge" (Húc đầu làm nũng)
                SetEyeColor(255, 105, 180, kEyeModeBreathing);
                SmoothDriveMotors(0.50f, 0.50f, 150);    // Step forward
                SmoothDriveMotors(-0.50f, -0.50f, 150);  // Step back
                StopMotors();
                OrganicMoveHead(90, 110, 350);  // Tilt chin up affectionately
                PlayR2D2Chirp("LOVING_PURR");
                vTaskDelay(pdMS_TO_TICKS(250));
                OrganicMoveHead(90, 90, 300);
                break;

            case 8:  // 🚨 Scenario 8: "Startled Reflex" (Giật mình phòng thủ)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 135, 180, true);     // Snap head back
                SmoothDriveMotors(-0.80f, -0.80f, 250);  // Jump back
                StopMotors();
                PlayR2D2Chirp("ALARM_SCREAM");
                OrganicMoveHead(120, 110, 180, true);  // Quick glance left
                OrganicMoveHead(60, 110, 180, true);   // Quick glance right
                OrganicMoveHead(90, 90, 300);
                break;

            case 9:  // 🪫 Scenario 9: "Sleepy Low Battery" (Ngủ gật kiệt sức)
                SetEyeColor(255, 60, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 100, 400);
                OrganicMoveHead(90, 40, 800);  // Head droops slowly
                PlayR2D2Chirp("LOW_POWER_DROOP");
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 65, 200, true);  // Sudden nod awake
                vTaskDelay(pdMS_TO_TICKS(200));
                OrganicMoveHead(90, 30, 900);  // Falls back to sleep
                break;

            case 10:  // 🐛 Scenario 10: "Curious Bug Hunt" (Săn bọ tò mò)
                SetEyeColor(0, 255, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 45, 350);          // Lower head to ground
                SmoothDriveMotors(0.35f, 0.35f, 150);  // Micro step forward
                StopMotors();
                PlayR2D2Chirp("CHASER_BEEPS");
                OrganicMoveHead(60, 40, 200);          // Zig-zag left
                OrganicMoveHead(120, 40, 200);         // Zig-zag right
                SmoothDriveMotors(0.35f, 0.35f, 150);  // Another micro step
                StopMotors();
                OrganicMoveHead(90, 90, 300);
                break;

            case 11:  // 💫 Scenario 11: "Dizzy Confused" (Chóng mặt ngơ ngác)
                SetEyeColor(255, 200, 0, kEyeModeStrobe);
                PlayR2D2Chirp("DIZZY_WHIMPER");
                SmoothDriveMotors(-0.35f, 0.15f, 200);  // Wobble backwards left
                OrganicMoveHead(60, 110, 300);
                SmoothDriveMotors(0.15f, -0.35f, 200);  // Wobble backwards right
                OrganicMoveHead(120, 80, 300);
                StopMotors();
                OrganicMoveHead(90, 90, 400);
                break;

            case 12:  // 🦸 Scenario 12: "Proud Superhero" (Tư thế anh hùng)
                SetEyeColor(0, 150, 255, kEyeModeSolid);
                SmoothDriveMotors(0.65f, 0.65f, 220);  // Bold hero stride
                StopMotors();
                OrganicMoveHead(90, 130, 300, true);  // Head high proud
                PlayR2D2Chirp("HERO_TRIUMPH");
                vTaskDelay(pdMS_TO_TICKS(400));
                OrganicMoveHead(105, 130, 300);  // Look left proudly
                OrganicMoveHead(75, 130, 300);   // Look right proudly
                OrganicMoveHead(90, 90, 350);
                break;

            case 13:  // 📡 Scenario 13: "Long-Range Scan" (Thám sát tầm xa)
                SetEyeColor(0, 100, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 135, 300);  // Head high
                OrganicMoveHead(30, 135, 700);  // Slow sweep left
                PlayR2D2Chirp("RADAR_SWEEP_PING");
                OrganicMoveHead(150, 135, 900);  // Slow sweep right
                PlayR2D2Chirp("RADAR_SWEEP_PING");
                OrganicMoveHead(90, 90, 400);
                break;

            case 14:  // 🚪 Scenario 14: "Narrow Gap Probe" (Lách khe hẹp)
                SetEyeColor(255, 200, 0, kEyeModeStrobe);
                OrganicMoveHead(45, 60, 350);          // Peer left low
                SmoothDriveMotors(0.20f, 0.20f, 120);  // Step forward cautiously
                StopMotors();
                PlayR2D2Chirp("CAUTIOUS_PROBE");
                OrganicMoveHead(135, 60, 400);           // Peer right low
                SmoothDriveMotors(-0.20f, -0.20f, 100);  // Hesitant step back
                StopMotors();
                OrganicMoveHead(90, 90, 300);
                break;

            case 15:  // 🧱 Scenario 15: "Wall Tracker" (Dò vách địa hình)
                SetEyeColor(0, 255, 120, kEyeModeBreathing);
                OrganicMoveHead(150, 85, 300);  // Head turned sideways to "listen to wall"
                PlayR2D2Chirp("ECHO_PULSE_CHIRP");
                SmoothDriveMotors(0.40f, 0.25f, 300);  // Parallel wall curve
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));
                OrganicMoveHead(90, 90, 300);
                break;

            case 16:  // 🌌 Scenario 16: "Ceiling Recon" (Thám hiểm tầm cao)
                SetEyeColor(160, 30, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 140, 500);  // Look straight up at ceiling
                PlayR2D2Chirp("AWE_WONDER_WHISTLE");
                SmoothDriveMotors(0.25f, -0.25f, 400);  // Slow pivot turn looking up
                StopMotors();
                OrganicMoveHead(80, 140, 200);
                OrganicMoveHead(100, 140, 200);
                OrganicMoveHead(90, 90, 400);
                break;

            case 17:  // 🐾 Scenario 17: "Trail Hunter" (Dò vết mặt đất)
                SetEyeColor(255, 120, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 30, 300);  // Nose down to ground
                PlayR2D2Chirp("TRAIL_HUNTER_CLICK");
                SmoothDriveMotors(0.35f, -0.15f, 150);  // Zig-zag left
                OrganicMoveHead(80, 30, 150);
                SmoothDriveMotors(-0.15f, 0.35f, 150);  // Zig-zag right
                OrganicMoveHead(100, 30, 150);
                StopMotors();
                OrganicMoveHead(90, 90, 300);
                break;

            case 18:  // 🙈 Scenario 18: "Peek-a-Boo Peek" (Trốn tìm lén lút)
                SetEyeColor(0, 255, 100, kEyeModeStrobe);
                OrganicMoveHead(40, 60, 300);          // Hide sideways low
                SmoothDriveMotors(0.30f, 0.30f, 150);  // Micro step
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));
                OrganicMoveHead(90, 100, 200, true);  // Snap head out!
                PlayR2D2Chirp("GIGGLE_CHIRP");
                SmoothDriveMotors(-0.30f, -0.30f, 100);  // Quick retreat
                StopMotors();
                OrganicMoveHead(40, 60, 200);  // Duck back
                vTaskDelay(pdMS_TO_TICKS(250));
                OrganicMoveHead(90, 90, 300);
                break;

            case 19:  // 😜 Scenario 19: "Prank Scare & Laugh" (Báo động giả trêu đùa)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Alarm red
                OrganicMoveHead(90, 135, 180, true);     // Fake surprise head up
                SmoothDriveMotors(-0.70f, -0.70f, 200);  // Jump back
                StopMotors();
                PlayR2D2Chirp("PRANK_ALARM_GIGGLE");
                SetEyeColor(255, 220, 0, kEyeModeBreathing);  // Switch to bright yellow laugh
                OrganicMoveHead(45, 100, 250, true);          // Confused glance left
                OrganicMoveHead(135, 100, 250, true);         // Confused glance right
                OrganicMoveHead(90, 90, 300);
                break;

            case 20:  // 🐔 Scenario 20: "Fake Charge & Escape" (Dọa húc rồi bỏ chạy)
                SetEyeColor(255, 100, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 40, 250, true);    // Head low charge pose
                SmoothDriveMotors(0.80f, 0.80f, 400);  // Fast forward rush
                StopMotors();
                PlayR2D2Chirp("RUNAWAY_CHICKEN");
                SetEyeColor(180, 0, 255, kEyeModeStrobe);  // Switch to purple panic
                OrganicMoveHead(150, 110, 200, true);      // Turn head back
                SmoothDriveMotors(-0.80f, 0.80f, 350);     // 180-degree spin escape
                StopMotors();
                OrganicMoveHead(90, 90, 300);
                break;

            case 21:  // 😤 Scenario 21: "Stubborn Refusal" (Bướng bỉnh chống đối)
                SetEyeColor(255, 20, 147, kEyeModeStrobe);  // Deep pink huff
                OrganicMoveHead(60, 80, 150, true);         // Shake left
                OrganicMoveHead(120, 80, 150, true);        // Shake right
                OrganicMoveHead(60, 80, 150, true);         // Shake left
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                SmoothDriveMotors(-0.30f, 0.30f, 120);  // Stubborn wheel twitch
                StopMotors();
                OrganicMoveHead(150, 90, 350);  // Turn head away stubborn
                vTaskDelay(pdMS_TO_TICKS(500));
                OrganicMoveHead(90, 90, 350);
                break;

            case 22:  // 🌀 Scenario 22: "Tail Chasing Craze" (Đuổi đuôi cuồng nhiệt)
                SetEyeColor(255, 255, 0, kEyeModeStrobe);  // Rainbow golden flash
                OrganicMoveHead(150, 50, 300, true);       // Look back at tail
                PlayR2D2Chirp("PLAYFUL_BARK_CHIRP");
                SmoothDriveMotors(0.90f, -0.90f, 600);  // High-speed spin chasing tail
                StopMotors();
                OrganicMoveHead(90, 120, 250, true);  // Dizzy happy head up
                OrganicMoveHead(90, 90, 350);
                break;

            case 23:  // 🛸 Story Scenario 23: "Alien Contact Ritual" (Nghi thức tiếp xúc ngoài hành
                      // tinh - ~7.5s)
                ESP_LOGI(TAG, "🛸 Scenario #23 Step 1: Raising head to cosmos...");
                SetEyeColor(0, 255, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 140, 700);  // Raise head to cosmos
                PlayR2D2Chirp("SCANNING_RADAR");
                vTaskDelay(pdMS_TO_TICKS(400));

                ESP_LOGI(TAG, "🛸 Scenario #23 Step 2: 360-degree alignment turn...");
                SetEyeColor(160, 30, 255, kEyeModeStrobe);  // Violet pulse
                SmoothDriveMotors(0.60f, -0.60f, 600);      // 360-degree alignment turn
                StopMotors();

                ESP_LOGI(TAG, "🛸 Scenario #23 Step 3: Panoramic space scan...");
                OrganicMoveHead(60, 120, 400);  // Scan space left
                PlayR2D2Chirp("AWE_WONDER_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(120, 120, 400);  // Scan space right
                vTaskDelay(pdMS_TO_TICKS(300));

                ESP_LOGI(TAG, "🛸 Scenario #23 Step 4: Contact signal & cosmos bow...");
                SetEyeColor(255, 215, 0, kEyeModeSolid);  // Golden contact!
                PlayR2D2Chirp("FANFARE_CHIRP");
                OrganicMoveHead(90, 130, 300, true);  // Head nod signal
                vTaskDelay(pdMS_TO_TICKS(400));
                OrganicMoveHead(90, 70, 500);  // Respectful cosmos bow
                OrganicMoveHead(90, 90, 400);
                break;

            case 24:  // 🦟 Story Scenario 24: "The Great Mosquito Battle" (Cuộc điền dại săn muỗi -
                      // ~8.0s)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 1: Tracking annoying bug...");
                SetEyeColor(255, 140, 0, kEyeModeStrobe);
                OrganicMoveHead(120, 110, 150, true);  // Fast twitch left
                PlayR2D2Chirp("ANGRY_BUZZ");
                OrganicMoveHead(60, 90, 150, true);  // Fast twitch right
                vTaskDelay(pdMS_TO_TICKS(200));

                ESP_LOGI(TAG, "🦟 Scenario #24 Step 2: Snap bite & miss...");
                OrganicMoveHead(90, 35, 180, true);  // Snap bite down!
                PlayR2D2Chirp("CONFUSED_QUESTION");  // Missed!
                OrganicMoveHead(90, 120, 300);       // Look back frustrated
                vTaskDelay(pdMS_TO_TICKS(300));

                ESP_LOGI(TAG, "🦟 Scenario #24 Step 3: Spin pursuit & lunging bite...");
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                SmoothDriveMotors(0.85f, -0.85f, 300);  // Spin 180 chasing bug
                StopMotors();
                SmoothDriveMotors(0.70f, 0.70f, 250);  // Final lunging charge!
                OrganicMoveHead(90, 40, 150, true);    // Got it!
                StopMotors();

                ESP_LOGI(TAG, "🦟 Scenario #24 Step 4: Victory celebration...");
                SetEyeColor(0, 255, 0, kEyeModeStrobe);
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                SmoothDriveMotors(-0.60f, 0.60f, 180);  // Victory wiggle
                StopMotors();
                OrganicMoveHead(90, 90, 350);
                break;

            case 25:  // 🦴 Story Scenario 25: "Archaeologist Fossil Dig" (Nhà khảo cổ đào hóa thạch
                      // - ~8.5s)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 1: Sniffing ground...");
                SetEyeColor(255, 165, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 25, 500);  // Nose to floor
                PlayR2D2Chirp("SNIFF_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(300));

                ESP_LOGI(TAG, "🦴 Scenario #25 Step 2: Excavation digging...");
                for (int dig = 0; dig < 3; dig++) {
                    OrganicMoveHead(80, 25, 120);
                    SmoothDriveMotors(0.50f, -0.20f, 120);
                    OrganicMoveHead(100, 25, 120);
                    SmoothDriveMotors(-0.20f, 0.50f, 120);
                }
                StopMotors();

                ESP_LOGI(TAG, "🦴 Scenario #25 Step 3: Blowing dust off fossil...");
                OrganicMoveHead(90, 50, 350);  // Pause digging, blow dust off
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                vTaskDelay(pdMS_TO_TICKS(400));

                ESP_LOGI(TAG, "🦴 Scenario #25 Step 4: Discovery & proud stride...");
                SetEyeColor(255, 215, 0, kEyeModeSolid);  // Discovery! Gold flash
                OrganicMoveHead(90, 135, 250, true);      // Snap head high in awe
                PlayR2D2Chirp("HERO_TRIUMPH");
                SmoothDriveMotors(0.40f, 0.40f, 200);  // Proud stride
                StopMotors();
                OrganicMoveHead(90, 90, 400);
                break;

            case 26:  // ⚡ Story Scenario 26: "Thunderstorm Terror & Courage" (Cơn dông đáng sợ &
                      // Lòng dũng cảm - ~9.0s)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 1: Thunderclap shock...");
                SetEyeColor(180, 0, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 140, 150, true);  // Thunderclap shock!
                PlayR2D2Chirp("SCARED_SCREAM");
                SmoothDriveMotors(-0.85f, -0.85f, 400);  // Panic reverse
                StopMotors();

                ESP_LOGI(TAG, "⚡ Scenario #26 Step 2: Trembling in fear...");
                SetEyeColor(255, 0, 0, kEyeModeBreathing);  // Trembling in fear
                OrganicMoveHead(90, 35, 300);
                PlayR2D2Chirp("SHY_WHIMPER");
                for (int quake = 0; quake < 4; quake++) {
                    SmoothDriveMotors(0.20f, -0.20f, 80);
                    SmoothDriveMotors(-0.20f, 0.20f, 80);
                }
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(400));

                ESP_LOGI(TAG, "⚡ Scenario #26 Step 3: Roar of inner courage...");
                OrganicMoveHead(90, 110, 700);  // Head slowly rises
                SetEyeColor(0, 255, 255, kEyeModeSolid);
                vTaskDelay(pdMS_TO_TICKS(300));
                SmoothDriveMotors(0.70f, 0.70f, 250);  // Stand ground charge step!
                StopMotors();
                OrganicMoveHead(90, 130, 250, true);
                PlayR2D2Chirp("PROUD_TUNE");
                OrganicMoveHead(90, 90, 400);
                break;

            case 27:  // 🤖 Story Scenario 27: "Robot System Reboot & Diagnostic" (Tự khởi động lại
                      // & Kiểm tra phần cứng - ~9.5s)
                ESP_LOGI(TAG, "🤖 Scenario #27 Step 1: Power shutdown...");
                SetEyeColor(0, 0, 0, kEyeModeOff);  // Power shut down
                OrganicMoveHead(90, 20, 700);       // Head drops limp
                PlayR2D2Chirp("LOW_POWER_DROOP");
                vTaskDelay(pdMS_TO_TICKS(400));

                ESP_LOGI(TAG, "🤖 Scenario #27 Step 2: Reboot & Servo calibration...");
                SetEyeColor(255, 255, 255, kEyeModeBreathing);
                PlayR2D2Chirp("CHARGING_HUM");
                OrganicMoveHead(90, 90, 500);  // Head powers up to level
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(30, 90, 350);   // Pan left limit
                OrganicMoveHead(150, 90, 400);  // Pan right limit
                OrganicMoveHead(90, 90, 300);
                PlayR2D2Chirp("SCANNING_RADAR");

                ESP_LOGI(TAG, "🤖 Scenario #27 Step 3: Motor drive test & system ready...");
                SetEyeColor(0, 255, 0, kEyeModeStrobe);
                SmoothDriveMotors(0.40f, 0.0f, 150);  // Left motor pulse
                SmoothDriveMotors(0.0f, 0.40f, 150);  // Right motor pulse
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(300));
                PlayR2D2Chirp("BOOT_POWER_UP");
                OrganicMoveHead(90, 115, 250, true);
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;
        }

        SetEyeColor(0, 200, 255, kEyeModeBreathing);
        is_performing_action_.store(false);
    }

public:
    // Multi-Sensory Sequencer (Synchronized Head -> Audio -> Motor)
    void SetRobotState(TRaxState state) {
        current_state_ = state;
        is_performing_action_.store(true);

        ESP_LOGI(TAG, "========== Organic Sequencer State: %d ==========", (int)state);

        switch (state) {
            case kStateCurious:  // 1. Tò mò
                SetEyeColor(0, 220, 255, kEyeModeBreathing);
                OrganicMoveHead(120, 110, 500);
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                break;

            case kStateFocused:  // 2. Tập trung
                SetEyeColor(0, 255, 120, kEyeModeSolid);
                OrganicMoveHead(90, 100, 300);
                PlayR2D2Chirp("FOCUSED_BEEP");
                break;

            case kStateAlertWarning:  // 3. Cảnh báo
                SetEyeColor(255, 140, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 130, 200, true);
                PlayR2D2Chirp("ALERT_SWEEP");
                break;

            case kStateAngry:  // 4. Tức giận
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                OrganicMoveHead(60, 80, 200, true);
                PlayR2D2Chirp("ANGRY_BUZZ");
                SmoothDriveMotors(0.9f, -0.9f, 250);  // Wiggle
                SmoothDriveMotors(-0.9f, 0.9f, 250);
                StopMotors();
                break;

            case kStateScared:  // 5. Sợ hãi
                SetEyeColor(150, 0, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 40, 200, true);
                PlayR2D2Chirp("SCARED_SCREAM");
                vTaskDelay(pdMS_TO_TICKS(50));
                SmoothDriveMotors(-1.0f, -1.0f, 400);  // Backward ramp
                StopMotors();
                break;

            case kStateHappy:  // 6. Vui vẻ
                SetEyeColor(0, 255, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 120, 400);
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                SmoothDriveMotors(0.8f, -0.8f, 200);
                SmoothDriveMotors(-0.8f, 0.8f, 200);
                StopMotors();
                break;

            case kStateDisappointed:  // 7. Thất vọng
                SetEyeColor(50, 50, 150, kEyeModeBreathing);
                OrganicMoveHead(90, 30, 800);
                PlayR2D2Chirp("SAD_SLIDE_DOWN");
                break;

            case kStateTargetDetected:  // 8. Phát hiện mục tiêu
                SetEyeColor(255, 255, 0, kEyeModeSolid);
                OrganicMoveHead(90, 110, 250, true);
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                break;

            case kStateConfused:  // 9. Bối rối
                SetEyeColor(200, 0, 200, kEyeModeBreathing);
                OrganicMoveHead(135, 80, 600);
                PlayR2D2Chirp("CONFUSED_QUESTION");
                break;

            case kStateSurprised:  // 10. Ngạc nhiên
                SetEyeColor(255, 255, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 140, 200, true);
                PlayR2D2Chirp("SURPRISED_HIGH");
                break;

            case kStateSuspicious:  // 11. Nghi ngờ
                SetEyeColor(255, 100, 0, kEyeModeBreathing);
                OrganicMoveHead(45, 90, 700);
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                break;

            case kStateLoving:  // 12. Yêu thương
                SetEyeColor(255, 105, 180, kEyeModeBreathing);
                OrganicMoveHead(90, 105, 600);
                PlayR2D2Chirp("LOVING_PURR");
                break;

            case kStateVictorious:  // 13. Chiến thắng
                SetEyeColor(0, 255, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 135, 300);
                PlayR2D2Chirp("FANFARE_CHIRP");
                SmoothDriveMotors(0.9f, -0.9f, 300);
                SmoothDriveMotors(-0.9f, 0.9f, 300);
                StopMotors();
                break;

            case kStateShy:  // 14. E ngại
                SetEyeColor(255, 180, 200, kEyeModeBreathing);
                OrganicMoveHead(120, 40, 700);
                PlayR2D2Chirp("SHY_WHIMPER");
                break;

            case kStateBored:  // 15. Chán nản
                SetEyeColor(100, 100, 100, kEyeModeBreathing);
                OrganicMoveHead(90, 50, 900);
                PlayR2D2Chirp("BORED_SIGH");
                break;

            case kStateArrogant:  // 16. Kiêu ngạo
                SetEyeColor(255, 215, 0, kEyeModeSolid);
                OrganicMoveHead(90, 150, 500);
                PlayR2D2Chirp("PROUD_TUNE");
                break;

            case kStateSearching:  // 17. Tìm kiếm
                SetEyeColor(0, 150, 255, kEyeModeStrobe);
                OrganicMoveHead(45, 90, 600);
                OrganicMoveHead(135, 90, 600);
                OrganicMoveHead(90, 90, 400);
                PlayR2D2Chirp("SCANNING_RADAR");
                break;

            case kStateSystemError:  // 18. Lỗi hệ thống
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 30, 200);
                PlayR2D2Chirp("GLITCH_NOISE");
                break;

            case kStateLowBattery:  // 19. Sắp hết pin
                SetEyeColor(255, 50, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 20, 1000);
                PlayR2D2Chirp("LOW_POWER_BEEP");
                break;

            case kStateCharging:  // 20. Đang sạc pin
                SetEyeColor(0, 255, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 500);
                PlayR2D2Chirp("CHARGING_HUM");
                break;

            case kStateBooting:  // 21. Khởi động
                SetEyeColor(255, 255, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 500);
                PlayR2D2Chirp("BOOT_POWER_UP");
                break;

            case kStateSleeping:  // 22. Ngủ
                SetEyeColor(0, 0, 0, kEyeModeOff);
                OrganicMoveHead(90, 20, 1000);
                break;

            case kStateIdle:  // 23. Chế độ IDLE
            default:
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 400);
                break;
        }

        // Action completed, resume gentle organic breathing & micro-saccades seamlessly
        is_performing_action_.store(false);
    }

private:
    void TriggerSurpriseReaction() {
        SetRobotState(kStateSurprised);
        vTaskDelay(pdMS_TO_TICKS(1200));
        SetRobotState(kStateIdle);
    }

    void StartTofTask() {
        if (vl53l0x_dev_ == nullptr)
            return;
        xTaskCreate(
            [](void* arg) {
                auto board = static_cast<TRaxBoard*>(arg);
                uint8_t read_buf[2];

                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(100));

                    if (board->vl53l0x_dev_ == nullptr)
                        continue;

                    uint8_t reg = 0x14;
                    esp_err_t err =
                        i2c_master_transmit_receive(board->vl53l0x_dev_, &reg, 1, read_buf, 2, 50);
                    if (err == ESP_OK) {
                        uint16_t distance_mm = (read_buf[0] << 8) | read_buf[1];

                        if (distance_mm > 20 && distance_mm < 150) {
                            TickType_t now = xTaskGetTickCount();
                            if ((now - board->last_tof_trigger_time_) > pdMS_TO_TICKS(2000)) {
                                board->last_tof_trigger_time_ = now;
                                board->TriggerSurpriseReaction();
                            }
                        }
                    }
                }
            },
            "tof_sensor_task", 4096, this, 5, &tof_task_handle_);
    }

    // Dedicated Thread-Safe WS2812 RMT Refresher Task
    void StartEyeLedBreathingTask() {
        xTaskCreate(
            [](void* arg) {
                auto board = static_cast<TRaxBoard*>(arg);
                float step = 0.0f;

                while (true) {
                    if (board->eye_led_strip_ == nullptr) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        continue;
                    }

                    uint8_t r = 0, g = 0, b = 0;
                    EyeLedMode mode = kEyeModeOff;

                    {
                        std::lock_guard<std::mutex> lock(board->led_mutex_);
                        r = board->target_r_;
                        g = board->target_g_;
                        b = board->target_b_;
                        mode = board->current_led_mode_;
                    }

                    if (mode == kEyeModeBreathing) {
                        step += 0.05f;
                        if (step >= 2.0f * M_PI)
                            step = 0.0f;

                        float factor = 0.1f + 0.9f * (0.5f * (sinf(step) + 1.0f));

                        uint8_t out_r = static_cast<uint8_t>(r * factor);
                        uint8_t out_g = static_cast<uint8_t>(g * factor);
                        uint8_t out_b = static_cast<uint8_t>(b * factor);

                        {
                            std::lock_guard<std::mutex> lock(board->led_mutex_);
                            led_strip_set_pixel(board->eye_led_strip_, 0, out_r, out_g, out_b);
                            led_strip_refresh(board->eye_led_strip_);
                        }
                        vTaskDelay(pdMS_TO_TICKS(30));
                    } else if (mode == kEyeModeStrobe) {
                        {
                            std::lock_guard<std::mutex> lock(board->led_mutex_);
                            led_strip_set_pixel(board->eye_led_strip_, 0, r, g, b);
                            led_strip_refresh(board->eye_led_strip_);
                        }
                        vTaskDelay(pdMS_TO_TICKS(60));

                        {
                            std::lock_guard<std::mutex> lock(board->led_mutex_);
                            led_strip_clear(board->eye_led_strip_);
                        }
                        vTaskDelay(pdMS_TO_TICKS(60));
                    } else if (mode == kEyeModeOff) {
                        {
                            std::lock_guard<std::mutex> lock(board->led_mutex_);
                            led_strip_clear(board->eye_led_strip_);
                        }
                        vTaskDelay(pdMS_TO_TICKS(200));
                    } else {
                        {
                            std::lock_guard<std::mutex> lock(board->led_mutex_);
                            led_strip_set_pixel(board->eye_led_strip_, 0, r, g, b);
                            led_strip_refresh(board->eye_led_strip_);
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
            },
            "eye_led_task", 3072, this, 3, &led_task_handle_);
    }

    // Living Biomimetic Organism Simulator: Dual-Harmonic Breathing & Attentive Gaze Tracking &
    // Spontaneous Scenarios
    void StartIdleSequenceTask() {
        xTaskCreate(
            [](void* arg) {
                auto board = static_cast<TRaxBoard*>(arg);
                float sim_time = 0.0f;
                float target_gaze_pan = 90.0f;
                float target_gaze_tilt = 90.0f;
                float current_base_pan = 90.0f;
                float current_base_tilt = 90.0f;
                TickType_t next_saccade_tick = xTaskGetTickCount() + pdMS_TO_TICKS(2500);
                TickType_t next_scenario_tick =
                    xTaskGetTickCount() + pdMS_TO_TICKS(6000 + (esp_random() % 6000));

                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(25));  // 40 FPS silky-smooth animation loop

                    // When an emotion/tool call action is running, sync base coordinates and yield
                    if (board->is_performing_action_.load()) {
                        {
                            std::lock_guard<std::mutex> lock(board->servo_mutex_);
                            current_base_pan = board->current_pan_;
                            current_base_tilt = board->current_tilt_;
                            target_gaze_pan = current_base_pan;
                            target_gaze_tilt = current_base_tilt;
                        }
                        sim_time = 0.0f;
                        continue;
                    }

                    sim_time += 0.025f;
                    TickType_t now = xTaskGetTickCount();

                    // Device State Awareness (Attentive Listening vs Speaking vs Relaxed Idle)
                    auto dev_state = Application::GetInstance().GetDeviceState();
                    bool is_listening = (dev_state == kDeviceStateListening);
                    bool is_speaking = (dev_state == kDeviceStateSpeaking);

                    // If in active dialog, postpone spontaneous choreography
                    if (is_listening || is_speaking) {
                        next_scenario_tick = now + pdMS_TO_TICKS(14000 + (esp_random() % 6000));
                    }

                    // Trigger Improvised Scenario periodically during undisturbed idle
                    if (!is_listening && !is_speaking && (now >= next_scenario_tick)) {
                        int scenario_idx = esp_random() % 28;
                        board->PerformImprovisedScenario(scenario_idx);
                        next_scenario_tick =
                            xTaskGetTickCount() + pdMS_TO_TICKS(15000 + (esp_random() % 12000));
                        continue;
                    }

                    // Natural Gaze Saccade Planner (expressive wide curious head shifts every 3-6s)
                    if (now >= next_saccade_tick) {
                        if (is_listening) {
                            // Attentive posture: look slightly upward towards human with subtle
                            // curious tilt
                            target_gaze_pan = 80.0f + (float)(esp_random() % 21);  // 80..100 deg
                            target_gaze_tilt =
                                100.0f +
                                (float)(esp_random() % 16);  // 100..115 deg (attentive upward gaze)
                            next_saccade_tick = now + pdMS_TO_TICKS(3000 + (esp_random() % 2500));
                        } else {
                            // Wide curious exploration: full-range wandering glances across room
                            target_gaze_pan =
                                40.0f +
                                (float)(esp_random() % 101);  // 40..140 deg wide horizontal sweep
                            target_gaze_tilt =
                                45.0f +
                                (float)(esp_random() % 81);  // 45..125 deg wide vertical sweep
                            next_saccade_tick = now + pdMS_TO_TICKS(3500 + (esp_random() % 3500));
                        }
                    }

                    // Smooth exponential low-pass filter to smoothly glide base position toward
                    // gaze target
                    float lerp_rate = is_listening ? 0.07f : 0.05f;
                    current_base_pan += (target_gaze_pan - current_base_pan) * lerp_rate;
                    current_base_tilt += (target_gaze_tilt - current_base_tilt) * lerp_rate;

                    // Full-Range Dual-Harmonic Biomimetic Breathing Wave
                    float pan_breath = 8.0f * sinf(0.6f * sim_time) +
                                       3.0f * cosf(1.1f * sim_time);  // Max ~11.0 deg amplitude
                    float tilt_breath = 10.0f * sinf(0.85f * sim_time) +
                                        4.0f * sinf(1.7f * sim_time);  // Max ~14.0 deg amplitude

                    if (is_listening) {
                        // Attentive, medium-amplitude breathing while listening to user
                        pan_breath *= 0.4f;
                        tilt_breath = 4.0f * sinf(1.3f * sim_time);
                    }

                    float final_pan = current_base_pan + pan_breath;
                    float final_tilt = current_base_tilt + tilt_breath;

                    board->SetRawServoAngle(final_pan, final_tilt);
                }
            },
            "idle_sequence_task", 8192, this, 2, &idle_task_handle_);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool(
            "self.trax.set_state",
            "Đặt 1 trong 23 trạng thái cảm xúc cho Robot T-Rax (1..23). CHỈ GỌI 1 LẦN DUY NHẤT.",
            PropertyList({Property("state_id", kPropertyTypeInteger, 1, 23)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int state_id = properties["state_id"].value<int>();

                // === RATE-LIMITING: Reject duplicate calls within 3-second cooldown ===
                TickType_t now = xTaskGetTickCount();
                TickType_t elapsed_ms = (now - last_state_change_ticks_) * portTICK_PERIOD_MS;
                if (elapsed_ms < STATE_DEBOUNCE_MS) {
                    ESP_LOGW(TAG, "set_state(%d) REJECTED: debounce cooldown (%lu ms < %lu ms)",
                             state_id, (unsigned long)elapsed_ms, (unsigned long)STATE_DEBOUNCE_MS);
                    return std::string(
                        "ALREADY EXECUTED. State change is rate-limited. Do NOT call this tool "
                        "again.");
                }
                last_state_change_ticks_ = now;

                SetRobotState(static_cast<TRaxState>(state_id));
                return std::string("State ") + std::to_string(state_id) +
                       " applied. Do NOT call this tool again. STOP.";
            });
    }

    void InitializeDriverEnable() {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << DRIVER_ENABLE_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
        gpio_set_level(DRIVER_ENABLE_GPIO, 1);
        ESP_LOGI(TAG, "Driver Enable GPIO %d set to HIGH (Active)", DRIVER_ENABLE_GPIO);
    }

public:
    TRaxBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeDriverEnable();
        InitializeI2c();
        InitializeWs2812Led();
        InitializeMotorPwm();
        InitializeServos();
        InitializeButtons();
        InitializeTools();
        StartTofTask();
        StartEyeLedBreathingTask();
        StartIdleSequenceTask();

        // Cap Wi-Fi TX Power to 15dBm (60 * 0.25dBm) to prevent Brownout Reset during Wi-Fi
        // connection bursts
        esp_wifi_set_max_tx_power(60);

        SetRobotState(kStateBooting);
        vTaskDelay(pdMS_TO_TICKS(1000));
        SetRobotState(kStateIdle);
    }

    virtual Led* GetLed() override { return nullptr; }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
                                               AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return &display_; }
};

DECLARE_BOARD(TRaxBoard);

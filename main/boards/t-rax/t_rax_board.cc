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
    kStateCurious = 1,              // 1. Tò mò
    kStateFocused,                  // 2. Tập trung
    kStateAlertWarning,             // 3. Cảnh báo
    kStateAngry,                    // 4. Tức giận
    kStateScared,                   // 5. Sợ hãi
    kStateHappy,                    // 6. Vui vẻ
    kStateDisappointed,             // 7. Thất vọng
    kStateTargetDetected,           // 8. Phát hiện mục tiêu
    kStateConfused,                 // 9. Bối rối
    kStateSurprised,                // 10. Ngạc nhiên
    kStateSuspicious,               // 11. Nghi ngờ
    kStateLoving,                   // 12. Yêu thương
    kStateVictorious,               // 13. Chiến thắng
    kStateShy,                      // 14. E ngại
    kStateBored,                    // 15. Chán nản
    kStateArrogant,                 // 16. Kiêu ngạo
    kStateSearching,                // 17. Tìm kiếm
    kStateSystemError,              // 18. Lỗi hệ thống
    kStateLowBattery,               // 19. Sắp hết pin
    kStateCharging,                 // 20. Đang sạc pin
    kStateBooting,                  // 21. Khởi động
    kStateSleeping,                 // 22. Ngủ
    kStateIdle,                     // 23. Chế độ IDLE
    kStateWakeGreetingEnergetic,    // 24. Chào mừng Bừng Tỉnh Phấn Khởi (Energetic Pop-Up)
    kStateWakeGreetingCurious,      // 25. Chào mừng Tò Mò Nghiêng Đầu (Curious Cocked Head)
    kStateWakeGreetingFriendlyNod,  // 26. Chào mừng Gật Đầu Thân Thiện (Friendly Double Nod)
    kStateWakeGreetingSleepyWake,   // 27. Chào mừng Vươn Vai Tỉnh Giấc (Sleepy Stretch to Alert)
    kStateWakeGreetingScoutScan     // 28. Chào mừng Quét Radar Sẵn Sàng (Scout Radar Snap & Lock)
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

class TRaxBoard;
class TRaxLed : public Led {
private:
    TRaxBoard& board_;
    DeviceState prev_state_ = kDeviceStateUnknown;

public:
    TRaxLed(TRaxBoard& board) : board_(board) {}
    virtual void OnStateChanged() override;
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

        // Hardware mechanical horn mapping for Tilt:
        // Invert tilt so that logical tilt < 90° points head down (sleep/ground scan),
        // and logical tilt > 90° points head up (stargazing/roaring/stretching)
        float physical_tilt = 180.0f - tilt;

        uint32_t pan_duty = 205 + (uint32_t)((pan * 410.0f) / 180.0f);
        uint32_t tilt_duty = 205 + (uint32_t)((physical_tilt * 410.0f) / 180.0f);

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
        // Cap maximum motor power to 0.45f to prevent electrical current brownout sags on weak
        // power rails
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

            // Right Motor Duty (LEDC Channels 4 & 5)
            uint32_t duty_right = (uint32_t)(fabsf(pwm_out_right_) * 255.0f);
            uint32_t duty_r4 = (pwm_out_right_ > 0.05f) ? duty_right : 0;
            uint32_t duty_r5 = (pwm_out_right_ < -0.05f) ? duty_right : 0;

            // Lock ONLY for the microsecond hardware register updates
            {
                std::lock_guard<std::mutex> lock(servo_mutex_);

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
            }

            // Sleep UNLOCKED so other tasks (VAD, servos, audio) can run freely
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
        // Prevent acoustic mic self-feedback loops during active voice listening state
        auto dev_state = Application::GetInstance().GetDeviceState();
        if (dev_state == kDeviceStateListening) {
            ESP_LOGW(TAG, "PlayR2D2Chirp[%s] suppressed during active listening state", sound_name);
            return;
        }

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
            case 0:  // 👃 Scenario 0: "Sniff & Explore" (Đánh hơi & Khám phá môi trường - 5 Phase
                     // Arc)
                // Phase 1: Initial Scent Catching (Cúi đầu bên trái đánh hơi sàn)
                SetEyeColor(0, 220, 255, kEyeModeBreathing);
                OrganicMoveHead(55, 35, 450);  // Sniff down-left
                PlayR2D2Chirp("SNIFF_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(150));
                // Micro-sniffing vertical twitches (mô phỏng nhấp nhô mũi khi hít thở mùi)
                OrganicMoveHead(55, 45, 120);
                OrganicMoveHead(55, 35, 120);
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 2: Following the Scent Trail (Nhích xích nhẹ & quét mùi sang phải)
                SmoothDriveMotors(0.30f, 0.30f, 180, 0.2f);  // Micro creep step forward
                StopMotors();
                OrganicMoveHead(125, 35, 450);  // Sweep scent trail to the right
                PlayR2D2Chirp("TRAIL_HUNTER_CLICK");
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(125, 45, 120);
                OrganicMoveHead(125, 35, 120);
                PlayR2D2Chirp("SNIFF_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 3: Scent Analysis & Suspicious Discovery (Tập trung phân tích điểm khả
                // nghi)
                SetEyeColor(255, 180, 0, kEyeModeSolid);  // Focused Amber
                OrganicMoveHead(90, 30, 350);             // Sniff center close to ground
                PlayR2D2Chirp("CAUTIOUS_PROBE");
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 50, 200, true);  // Quick recoil double-take

                // Phase 4: Eureka Moment & Proud Look-around (Hân hoan ngẩng cao đầu & lắc hông)
                SetEyeColor(0, 255, 120, kEyeModeStrobe);  // Emerald celebration strobe
                OrganicMoveHead(90, 125, 300, true);       // Snap head high with proud bounce
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                SmoothDriveMotors(0.40f, -0.40f, 100);  // Happy track twitch
                SmoothDriveMotors(-0.40f, 0.40f, 100);
                StopMotors();
                OrganicMoveHead(75, 115, 200);   // Gaze left
                OrganicMoveHead(105, 115, 250);  // Gaze right
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Return to Calm Center (Dịu dàng trở về trung tâm)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 1:  // 💃 Scenario 1: "Playful Wiggle Dance" (Vũ điệu lắc hông vui nhộn - 4 Phase
                     // Dance Routine)
                // Phase 1: Intro Groove & Beat Drop (Khởi động lắc hông 2 nhịp)
                SetEyeColor(255, 215, 0, kEyeModeStrobe);  // Gold Disco Strobe
                PlayR2D2Chirp("FANFARE_CHIRP");
                OrganicMoveHead(70, 110, 180);
                SmoothDriveMotors(0.40f, -0.40f, 140);  // Quick hip swing left
                OrganicMoveHead(110, 110, 180);
                SmoothDriveMotors(-0.40f, 0.40f, 140);  // Quick hip swing right
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Rhythmic 4-Step Wiggle Strobe (Lắc hông 4 thì tốc độ cao liên hoàn)
                SetEyeColor(255, 0, 150, kEyeModeStrobe);  // Magenta Party Strobe
                PlayR2D2Chirp("GIGGLE_CHIRP");
                OrganicMoveHead(65, 95, 120);
                SmoothDriveMotors(0.45f, -0.45f, 100);
                OrganicMoveHead(115, 95, 120);
                SmoothDriveMotors(-0.45f, 0.45f, 100);
                OrganicMoveHead(75, 105, 120);
                SmoothDriveMotors(0.45f, -0.45f, 100);
                OrganicMoveHead(105, 105, 120);
                SmoothDriveMotors(-0.45f, 0.45f, 100);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Victory Spin & High Pitch Climax (Xoay tròn nghệ thuật & Bật ngửa ăn
                // mừng)
                SetEyeColor(0, 255, 255, kEyeModeStrobe);  // Electric Cyan Strobe
                SmoothDriveMotors(0.45f, -0.45f, 320);     // 360-degree celebratory pivot spin
                StopMotors();
                OrganicMoveHead(90, 130, 220, true);  // Proud bounce look up
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                vTaskDelay(pdMS_TO_TICKS(250));

                // Phase 4: Polite Bow & Smooth Return to Center (Cúi chào duyên dáng & Trở về IDLE)
                SetEyeColor(0, 255, 120, kEyeModeSolid);
                OrganicMoveHead(90, 70, 220);  // Gentle thank-you bow
                vTaskDelay(pdMS_TO_TICKS(150));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 2:  // 🐾 Scenario 2: "The Apex Prowler: Stealth Dino Stalk & Pounce" (Khủng long
                     // rình mồi lén lút & Cú vồ dã thú - 5 Phase Arc)
                // Phase 1: Low-Crouch Stealth Creep (Hạ thấp trọng tâm - Bò lén từng bước ngắn ngắt
                // quãng)
                SetEyeColor(255, 40, 0, kEyeModeBreathing);  // Predator Crimson Breathing
                PlayR2D2Chirp("SNIFF_CHIRP");
                OrganicMoveHead(90, 35, 450);  // Lower head predatory low
                // 2-stage cautious stealth creep steps
                SmoothDriveMotors(0.25f, 0.25f, 180, 0.15f);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                SmoothDriveMotors(0.25f, 0.25f, 180, 0.15f);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 2: Freeze & Peripheral Threat Scan (Đóng băng bất động - Quét mắt ngoại vi
                // kiểm tra địa hình)
                SetEyeColor(255, 140, 0, kEyeModeSolid);  // Icy Amber Solid
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                OrganicMoveHead(135, 55, 220, true);  // Snap glance left
                vTaskDelay(pdMS_TO_TICKS(250));
                OrganicMoveHead(45, 55, 250, true);  // Snap glance right
                vTaskDelay(pdMS_TO_TICKS(250));

                // Phase 3: Wiggling Tail Pounce Preparation (Căng cơ chuẩn bị vồ - Lắc hông lấy đà
                // & Khóa mục tiêu)
                SetEyeColor(255, 255, 0, kEyeModeStrobe);  // Target Lock Laser Strobe
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                OrganicMoveHead(90, 30, 200);  // Lock head dead center low
                PlayR2D2Chirp("CHASER_BEEPS");
                // 3-step rapid butt/track wiggle charging up jump energy (lấy đà như mèo săn mồi)
                SmoothDriveMotors(0.35f, -0.35f, 100);
                SmoothDriveMotors(-0.35f, 0.35f, 100);
                SmoothDriveMotors(0.35f, -0.35f, 100);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 4: Explosive Strike Pounce & Predator Roar (Cú phóng vồ chớp nhoáng & Gầm
                // vang chiến thắng)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Blood Strike Strobe!
                SmoothDriveMotors(0.50f, 0.50f, 220);    // Sudden explosive forward sprint!
                OrganicMoveHead(90, 20, 120);            // Snap head down (Pounce gotcha!)
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));

                // Roar & Victorious Head Thrash (Gầm vang & Lắc đầu khoe chiến tích)
                PlayR2D2Chirp("ANGRY_BUZZ");
                SetEyeColor(0, 255, 120, kEyeModeStrobe);  // Emerald Triumph Strobe
                OrganicMoveHead(90, 140, 250, true);       // Roar facing high sky!
                PlayR2D2Chirp("HERO_TRIUMPH");
                OrganicMoveHead(70, 125, 180);   // Proud head thrash left
                OrganicMoveHead(110, 125, 200);  // Proud head thrash right
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Satisfied Savoring & Return to IDLE (Thỏa mãn & Lướt êm về tâm)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 3:  // 🦜 Scenario 3: "The Inquisitive Fledgling: Multi-Angle Bird Tilt" (Chim non
                     // hiếu kỳ: Nghiêng đầu ngơ ngác & Gật đầu hiểu ra - 5 Phase Arc)
                // Phase 1: Deep Right Cocked Head & Questioning Chirp (Nghiêng đầu sang phải 45°
                // ngơ ngác)
                SetEyeColor(0, 255, 180, kEyeModeSolid);  // Curious Turquoise Solid
                PlayR2D2Chirp("CONFUSED_QUESTION");
                OrganicMoveHead(120, 120, 280, true);  // Cock head deep right
                vTaskDelay(pdMS_TO_TICKS(150));
                // Micro inquisitive twitches
                OrganicMoveHead(120, 110, 100);
                OrganicMoveHead(120, 125, 100);
                vTaskDelay(pdMS_TO_TICKS(300));

                // Phase 2: Sudden Flip to Left Cocked Head (Lật phắt đầu sang vai trái & Nhảy xích
                // nhẹ)
                SetEyeColor(255, 230, 0, kEyeModeSolid);  // Lemon Yellow Curious Solid
                PlayR2D2Chirp("CAUTIOUS_PROBE");
                OrganicMoveHead(60, 120, 250, true);   // Flip head deep left!
                SmoothDriveMotors(0.25f, 0.10f, 100);  // Little bird hop step
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                // Micro twitches left side
                OrganicMoveHead(60, 110, 100);
                OrganicMoveHead(60, 125, 100);
                vTaskDelay(pdMS_TO_TICKS(300));

                // Phase 3: Peering Forward & Back (Rướn cổ lại gần soi kỹ & Rụt cổ đánh giá)
                SetEyeColor(255, 140, 20, kEyeModeBreathing);  // Focused Amber Glow
                PlayR2D2Chirp("SNIFF_CHIRP");
                OrganicMoveHead(90, 75, 220);  // Peer forward low
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 110, 200, true);  // Recoil back high
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 4: Eureka Understanding & Joyful Nods (Aha! Đã hiểu ra rồi! & Gật đầu lia
                // lịa)
                SetEyeColor(0, 255, 120, kEyeModeStrobe);  // Emerald Joy Strobe!
                OrganicMoveHead(90, 130, 220, true);       // Head high pop
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // 3 enthusiastic understanding nods
                OrganicMoveHead(90, 90, 120);
                OrganicMoveHead(90, 120, 120);
                OrganicMoveHead(90, 90, 120);
                OrganicMoveHead(90, 120, 120);
                // Happy little track wiggle
                SmoothDriveMotors(0.35f, -0.35f, 80);
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Contented Return to IDLE (Thỏa mãn hiểu biết & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);  // Soft Cyan Breathing
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 90, 350);
                break;

            case 4:  // 👀 Scenario 4: "Surprise Look Behind" (Ngoái đầu & Xoay thân kiểm tra sau
                     // lưng - 5 Phase Arc)
                // Phase 1: Sudden Startle / Spook (Giật mình cảnh giác vì tiếng động lạ)
                SetEyeColor(255, 255, 255, kEyeModeStrobe);  // Pure White Shock Strobe
                PlayR2D2Chirp("SURPRISED_HIGH");
                OrganicMoveHead(90, 135, 140, true);  // Snap head high with shock recoil
                vTaskDelay(pdMS_TO_TICKS(120));

                // Phase 2: Deep Left-Rear Shoulder Check (Ngoái vai trái cực đại & Xoay nhẹ thân
                // quét góc chết)
                SetEyeColor(255, 140, 0, kEyeModeSolid);  // Alert Amber Solid
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                OrganicMoveHead(150, 95, 200, true);    // Snap full left-back
                SmoothDriveMotors(0.40f, -0.40f, 120);  // Body pivot turn to look fully behind!
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                // Inquisitive head tilt checking shadows
                OrganicMoveHead(150, 75, 160);
                OrganicMoveHead(150, 110, 180);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 3: Rapid 180° Snap Right-Rear Double-Take (Giật mình đảo phắt sang vai
                // phải)
                SetEyeColor(255, 220, 0, kEyeModeStrobe);  // Yellow Warning Strobe
                PlayR2D2Chirp("CAUTIOUS_PROBE");
                OrganicMoveHead(30, 95, 250, true);     // Rapid snap to full right-back
                SmoothDriveMotors(-0.40f, 0.40f, 220);  // Reverse pivot turn right!
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                // Low angle squint check
                OrganicMoveHead(30, 70, 180);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 4: Relief & Reassurance (Thở phào nhẹ nhõm - Không có gì nguy hiểm)
                SetEyeColor(0, 220, 255, kEyeModeBreathing);  // Calming Sky Blue Breathing
                PlayR2D2Chirp("SNIFF_CHIRP");
                OrganicMoveHead(90, 80, 250);  // Return to facing front
                vTaskDelay(pdMS_TO_TICKS(150));
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                // Reassuring little head shake "Just the wind"
                OrganicMoveHead(80, 95, 120);
                OrganicMoveHead(100, 95, 120);
                OrganicMoveHead(90, 95, 120);
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Re-center & Relaxed Return to IDLE (Định thần an tâm & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 5:  // 🥱 Scenario 5: "Lazy Stretch & Yawn" (Vươn vai ngáp lười biếng - 5 Phase
                     // Biomimetic Arc)
                // Phase 1: Drowsy Droop (Cơn buồn ngủ ập đến - Đầu trĩu nặng)
                SetEyeColor(140, 90, 220, kEyeModeBreathing);  // Soft Lavender Breathing
                PlayR2D2Chirp("BORED_SIGH");
                OrganicMoveHead(90, 35, 650);  // Head droops heavy and sleepy
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 2: Epic Full-Body High Stretch & Yawn (Vươn vai hết cỡ & Ngáp dài)
                SetEyeColor(255, 140, 30, kEyeModeBreathing);  // Warm Sunrise Amber
                PlayR2D2Chirp("YAWN_TUNE");
                OrganicMoveHead(90, 145, 800);  // High full-body stretch
                vTaskDelay(pdMS_TO_TICKS(150));
                // Neck stretching roll left & right
                OrganicMoveHead(75, 140, 220);
                OrganicMoveHead(105, 140, 250);
                OrganicMoveHead(90, 145, 180);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 3: Body Shiver & Track Shake (Rũ mình sảng khoái sau khi thức giấc)
                SetEyeColor(255, 200, 50, kEyeModeStrobe);  // Warm Honey Strobe
                PlayR2D2Chirp("SNIFF_CHIRP");
                // 3-step rapid track & head shiver (mô phỏng động tác rũ lông sảng khoái)
                OrganicMoveHead(80, 100, 90);
                SmoothDriveMotors(0.35f, -0.35f, 90);
                OrganicMoveHead(100, 100, 90);
                SmoothDriveMotors(-0.35f, 0.35f, 90);
                OrganicMoveHead(85, 100, 90);
                SmoothDriveMotors(0.35f, -0.35f, 90);
                OrganicMoveHead(95, 100, 90);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 4: Blinking Awake & Looking Around (Chớp mắt tỉnh táo & Nhìn quanh ngơ
                // ngác)
                SetEyeColor(0, 255, 150, kEyeModeSolid);  // Fresh Mint Green Solid
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 80, 180);   // Little blink nod
                OrganicMoveHead(90, 105, 200);  // Attentive head lift
                OrganicMoveHead(70, 95, 220);   // Look left curious
                OrganicMoveHead(110, 95, 250);  // Look right curious
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Cozy Settle & Smooth Return to Center (Thư thái định vị & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 400);
                break;

            case 6:  // 🏆 Scenario 6: "The King of Pop: MJ Billie Jean & Moonwalk Tribute" (Vũ điệu
                     // Michael Jackson: Trượt Moonwalk, Xoay 360° & Cúi chào - 5 Phase Arc)
                // Phase 1: Fedora Hat Tip & Beat Drop Intro (Giật vai & Kéo vành mũ Fedora vào nhịp
                // nhạc)
                SetEyeColor(255, 215, 0, kEyeModeStrobe);  // King of Pop Golden Strobe
                PlayR2D2Chirp("FANFARE_CHIRP");
                // Sharp snap hat tilts left and right on the beat
                OrganicMoveHead(60, 125, 150, true);
                OrganicMoveHead(120, 125, 150, true);
                OrganicMoveHead(90, 110, 120);
                // Beat-drop track twitches
                SmoothDriveMotors(0.35f, -0.35f, 70);
                SmoothDriveMotors(-0.35f, 0.35f, 70);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(120));

                // Phase 2: The Signature Moonwalk Glide (Cú trượt lùi Moonwalk thần sầu)
                SetEyeColor(230, 230, 255, kEyeModeStrobe);  // Silver Billie Jean Strobe
                PlayR2D2Chirp("DISCO_BEAT_PULSE");
                OrganicMoveHead(90, 95, 200);  // Head locked gazing forward coolly
                // Silky smooth S-curve reverse moonwalk glide
                SmoothDriveMotors(-0.40f, -0.25f, 130);
                SmoothDriveMotors(-0.25f, -0.40f, 130);
                SmoothDriveMotors(-0.45f, -0.45f, 150);
                StopMotors();
                // Rhythmic head bobs on beat
                OrganicMoveHead(90, 80, 90);
                OrganicMoveHead(90, 105, 90);
                OrganicMoveHead(90, 90, 90);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: 360° Spin & Toe Freeze "Hee-Hee!" (Xoay tròn 360° tốc độ cao & Kiễng
                // chân "Hee-Hee!")
                SetEyeColor(255, 0, 50, kEyeModeStrobe);  // Thriller Crimson Strobe!
                PlayR2D2Chirp("SURPRISED_HIGH");          // "Hee-Hee!" vocal scream
                SmoothDriveMotors(0.60f, -0.60f, 320);    // High-speed 360 spin
                StopMotors();
                // Instant freeze on tiptoes with head raised high
                OrganicMoveHead(90, 140, 120, true);
                PlayR2D2Chirp("CHASER_BEEPS");
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 4: Anti-Gravity Lean & Pop-Locking (Nghiêng người chống trọng lực & Giật
                // Pop-Locking)
                SetEyeColor(0, 240, 255, kEyeModeStrobe);  // Electro Blue Strobe
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // Smooth Criminal Forward Lean
                OrganicMoveHead(90, 50, 250);
                vTaskDelay(pdMS_TO_TICKS(100));
                // Rapid pop-locking head snaps
                OrganicMoveHead(75, 65, 80);
                OrganicMoveHead(105, 65, 80);
                OrganicMoveHead(90, 65, 80);
                // Step forward reclaiming center stage
                SmoothDriveMotors(0.40f, 0.40f, 180);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 5: Iconic Final Pose, Hat Drop Bow & IDLE (Tạo dáng kết bài, Cúi chào hạ
                // màn & Về IDLE)
                SetEyeColor(255, 215, 0, kEyeModeSolid);  // Golden Finale
                OrganicMoveHead(90, 130, 200, true);      // Proud chin pop
                PlayR2D2Chirp("HERO_TRIUMPH");
                vTaskDelay(pdMS_TO_TICKS(180));
                // Respectful theatre bow
                OrganicMoveHead(90, 60, 400);
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(250));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 7:  // 🦖 Scenario 7: "The Cuddly Companion: Affectionate Nudge & Purr" (Húc đầu
                     // làm nũng & Xin xoa cằm - 5 Phase Arc)
                // Phase 1: Shy Approach & Chin Lift (Tiến lại gần ngập ngừng & Ngước cằm xin xoa
                // đầu)
                SetEyeColor(255, 105, 180, kEyeModeBreathing);  // Blushing Rose Pink Breathing
                PlayR2D2Chirp("SHY_WHIMPER");
                SmoothDriveMotors(0.28f, 0.28f, 180, 0.15f);  // Gentle step forward
                StopMotors();
                OrganicMoveHead(90, 120, 400);  // Lift chin up inviting pets
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 2: Double Cheek Nudge (Húc má trái rồi húc má phải vào tay chủ nhân)
                SetEyeColor(255, 60, 150, kEyeModeSolid);  // Sweet Peach Pink Solid
                PlayR2D2Chirp("LOVING_PURR");
                // Left cheek cuddle nudge
                OrganicMoveHead(65, 110, 250);
                SmoothDriveMotors(0.30f, 0.15f, 120);  // Nudge body gently to the left
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(180));
                // Right cheek cuddle nudge
                OrganicMoveHead(115, 110, 280);
                SmoothDriveMotors(0.15f, 0.30f, 120);  // Nudge body gently to the right
                StopMotors();
                PlayR2D2Chirp("LOVING_PURR");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 3: Blissful Chin Scratch (Tận hưởng cảm giác được xoa cằm - Mắt lim dim &
                // Ngửa cổ)
                SetEyeColor(255, 130, 200, kEyeModeBreathing);  // Cotton Candy Glow
                OrganicMoveHead(90, 135, 450);                  // Blissful high chin tilt
                PlayR2D2Chirp("GIGGLE_CHIRP");
                // Gentle chin rubbing micro-movements
                OrganicMoveHead(90, 125, 150);
                OrganicMoveHead(90, 135, 150);
                // Happy little butt wiggle (vẫy đuôi vui sướng)
                SmoothDriveMotors(0.35f, -0.35f, 80);
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(220));

                // Phase 4: Grateful & Loving Look (Ngước nhìn trìu mến & Chớp mắt cảm ơn)
                SetEyeColor(200, 80, 255, kEyeModeSolid);  // Loving Violet Glow
                OrganicMoveHead(90, 105, 250);             // Gaze lovingly at owner
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                // 2 appreciative gentle nods
                OrganicMoveHead(90, 95, 140);
                OrganicMoveHead(90, 110, 140);
                OrganicMoveHead(90, 100, 140);
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Cozy Settle & Smooth Return to Center (Ấm áp thỏa mãn & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 8:  // 🚨 Scenario 8: "The Combat Sentinel: Startled Reflex & Defensive Guard"
                     // (Giật mình phòng thủ & Cảnh giác cao độ - 5 Phase Arc)
                // Phase 1: Panic Startle & Jump Back (Giật nảy mình hoảng hốt & Bật lùi né đòn)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Emergency Red Strobe
                PlayR2D2Chirp("ALARM_SCREAM");
                OrganicMoveHead(90, 140, 120, true);     // Snap head high with shock recoil
                SmoothDriveMotors(-0.45f, -0.45f, 200);  // Rapid emergency reverse jump!
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(120));

                // Phase 2: Defensive Bunker Crouch (Hạ thấp thủ thế boong-ke & Gầm gừ đe dọa)
                SetEyeColor(255, 80, 0, kEyeModeSolid);  // Flaming Orange Threat Solid
                PlayR2D2Chirp("ANGRY_BUZZ");
                OrganicMoveHead(90, 45, 250);  // Low combat bunker stance
                // Defensive body wiggle stance
                SmoothDriveMotors(0.40f, -0.40f, 90);
                SmoothDriveMotors(-0.40f, 0.40f, 90);
                StopMotors();
                OrganicMoveHead(90, 35, 150);  // Glower menacingly
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 3: Tactical Threat Perimeter Scan (Quét radar 180° dò tìm mối đe dọa)
                SetEyeColor(255, 200, 0, kEyeModeStrobe);  // Hazard Yellow Strobe
                PlayR2D2Chirp("ALERT_SWEEP");
                OrganicMoveHead(140, 90, 180, true);  // Snap check left flank
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(40, 90, 220, true);  // Snap check right flank
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 100, 160);  // Lock front
                // Tactical micro retreat
                SmoothDriveMotors(-0.30f, -0.30f, 120);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 4: Stand Ground & Proud Warning (Khẳng định bản lĩnh "Đừng lại gần!" & Tiến
                // chiếm lại vị trí)
                SetEyeColor(180, 0, 255, kEyeModeSolid);  // Warrior Violet Solid
                OrganicMoveHead(90, 125, 300, true);      // Proud chin lift
                PlayR2D2Chirp("PROUD_TUNE");
                // Reclaim ground step
                SmoothDriveMotors(0.35f, 0.35f, 150);
                StopMotors();
                // 2 authoritative nod shakes
                OrganicMoveHead(90, 110, 140);
                OrganicMoveHead(90, 130, 140);
                OrganicMoveHead(90, 120, 140);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: De-escalation & Calm Return to IDLE (Hạ nhiệt an toàn & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
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

            case 10:  // 🕵️‍♂️ Scenario 10: "The Mini Sleuth: Bug Investigation" (Thám
                      // tử Khủng long phá án / Săn bọ trinh thám)
                // Phase 1: Crime Scene Recon & Radar Scan (Vào hiện trường & Soi kính lúp quét
                // radar)
                SetEyeColor(60, 0, 220, kEyeModeSolid);  // Detective Deep Indigo
                PlayR2D2Chirp("SCANNING_RADAR");
                OrganicMoveHead(45, 50, 350);  // Scan left ground
                vTaskDelay(pdMS_TO_TICKS(120));
                OrganicMoveHead(135, 50, 450);  // Slow magnifying sweep to right ground
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 2: Suspicious Clue Discovery (Phát hiện manh mối khả nghi & Bước lén tiếp
                // cận)
                SetEyeColor(255, 120, 0, kEyeModeBreathing);  // Suspicious Amber Breathing
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                OrganicMoveHead(70, 30, 300);  // Low angle inspection
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(70, 45, 150, true);           // Suspicious twitch recoil
                SmoothDriveMotors(0.25f, 0.25f, 150, 0.15f);  // Stealthy creep step forward
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Tactical Stalking & Target Lock (Rình rập chiến thuật & Khóa mục tiêu)
                SetEyeColor(255, 255, 0, kEyeModeStrobe);  // Target Lock Alert Strobe
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                // Zig-zag stalking wheel maneuvers
                OrganicMoveHead(60, 35, 140);
                SmoothDriveMotors(0.35f, 0.10f, 120);  // Tactical arc step left
                OrganicMoveHead(120, 35, 140);
                SmoothDriveMotors(0.10f, 0.35f, 120);  // Tactical arc step right
                StopMotors();
                OrganicMoveHead(90, 25, 220);  // Pounce posture: locked onto prey!
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 4: The Strike & Case Solved (Vồ bắt chớp nhoáng & Phá án thành công)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Red Strike Flash!
                SmoothDriveMotors(0.50f, 0.50f, 120);    // Sudden forward snap strike!
                OrganicMoveHead(90, 20, 100);            // Snap head down (Gotcha!)
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(120));

                // Victory celebration (Ăn mừng chiến tích)
                SetEyeColor(0, 255, 120, kEyeModeStrobe);  // Emerald Triumph Strobe
                OrganicMoveHead(90, 130, 250, true);       // Snap head high with proud bounce
                PlayR2D2Chirp("HERO_TRIUMPH");
                vTaskDelay(pdMS_TO_TICKS(250));

                // Phase 5: Case Closed & Relaxed Return (Khép lại hồ sơ & Trở về IDLE)
                OrganicMoveHead(90, 100, 180);  // Satisfied nod
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 11:  // 💫 Scenario 11: "The Dizzy Spinner: 720° Whirl & Drunken Wobble" (Xoay tít
                      // mù, hoa mắt chóng mặt & Đi lảo đảo - 5 Phase Arc)
                // Phase 1: Rapid 720° Multi-Spin Whirl (Xoay tròn tít mù 2 vòng liên tiếp tại chỗ)
                SetEyeColor(255, 230, 0, kEyeModeStrobe);  // Dizzy Gold Strobe
                PlayR2D2Chirp("CHASER_BEEPS");
                OrganicMoveHead(90, 115, 200);
                SmoothDriveMotors(0.55f, -0.55f, 650);  // High-speed 720-degree pivot spin
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Sudden Halt & Head Swirl Disorientation (Phanh gấp - Đầu óc quay cuồng
                // đảo mắt tròn)
                SetEyeColor(255, 120, 0, kEyeModeStrobe);  // Disoriented Amber Strobe
                PlayR2D2Chirp("DIZZY_WHIMPER");
                // Circular head swirl motion (mô phỏng đảo mắt và đầu hoa mắt chóng mặt)
                OrganicMoveHead(60, 120, 160);
                OrganicMoveHead(120, 120, 160);
                OrganicMoveHead(120, 60, 160);
                OrganicMoveHead(60, 60, 160);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Drunken Wobble & Stumbling Tracks (Đi đứng lảo đảo ngơ ngác như say
                // rượu)
                SetEyeColor(255, 60, 180, kEyeModeBreathing);  // Confused Magenta Breathing
                PlayR2D2Chirp("CONFUSED_QUESTION");
                // Drunken stumble backwards left
                OrganicMoveHead(60, 75, 200);
                SmoothDriveMotors(-0.35f, 0.15f, 180);
                // Drunken stumble backwards right
                OrganicMoveHead(120, 75, 200);
                SmoothDriveMotors(0.15f, -0.35f, 180);
                // Stumble slip recovery
                SmoothDriveMotors(-0.25f, -0.25f, 120);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 4: Rapid Head Shake Reset (Lắc mạnh đầu dồn dập xua cơn choáng & Bừng tỉnh)
                SetEyeColor(0, 255, 150, kEyeModeSolid);  // Fresh Mint Green Recovery Solid
                PlayR2D2Chirp("SNIFF_CHIRP");
                // 4-step rapid head shake to snap out of dizziness
                OrganicMoveHead(75, 95, 90);
                OrganicMoveHead(105, 95, 90);
                OrganicMoveHead(80, 95, 90);
                OrganicMoveHead(100, 95, 90);
                // "Aha, I'm back!" sudden proud head snap
                OrganicMoveHead(90, 120, 200, true);
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Steady Balance & Return to IDLE (Lấy lại thăng bằng & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
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

            case 19:  // 😜 Scenario 19: "The Fake Meltdown Prank: Critical Alarm to Troll Giggle"
                      // (Báo động đỏ giả vờ & Trò đùa tinh quái - 5 Phase Arc)
                // Phase 1: Hyper-Dramatic Fake Critical Emergency (Báo động đỏ khẩn cấp giả vờ -
                // Hoảng loạn tột độ)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Critical Red Alarm Strobe
                PlayR2D2Chirp("ALARM_SCREAM");
                OrganicMoveHead(90, 140, 120, true);  // Snap head high in simulated panic
                // Jittery panic head twitches
                OrganicMoveHead(80, 135, 70);
                OrganicMoveHead(100, 135, 70);
                OrganicMoveHead(85, 135, 70);
                SmoothDriveMotors(-0.45f, -0.45f, 180);  // Panic reverse jump
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Total Fake System Breakdown (Giả vờ sập nguồn hoàn toàn - Tắt ngóm mắt &
                // Gục đầu)
                SetEyeColor(0, 0, 0, kEyeModeOff);  // Sudden screen blackout!
                PlayR2D2Chirp("GLITCH_NOISE");
                OrganicMoveHead(90, 25, 500);  // Head droops limp as if "dead"
                PlayR2D2Chirp("LOW_POWER_DROOP");
                vTaskDelay(pdMS_TO_TICKS(400));  // Suspenseful pause (làm người xem thót tim!)

                // Phase 3: The Big Reveal Troll & Laugh (Bừng tỉnh trêu ngươi "Lừa được bạn rồi
                // nhé!" & Cười khúc khích)
                SetEyeColor(255, 220, 0, kEyeModeStrobe);  // Troll Yellow Laugh Strobe!
                OrganicMoveHead(90, 125, 200, true);       // Pop head up energetically
                PlayR2D2Chirp("PRANK_ALARM_GIGGLE");
                // Rapid laughing head wobbles (cười rung bần bật)
                OrganicMoveHead(75, 120, 100);
                OrganicMoveHead(105, 120, 100);
                OrganicMoveHead(75, 120, 100);
                OrganicMoveHead(105, 120, 100);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 4: Playful Hip Wiggle & Victory Tease (Lắc hông trêu ngươi & Nháy mắt đắc
                // thắng)
                SetEyeColor(255, 0, 150, kEyeModeStrobe);  // Playful Magenta Strobe
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                // 3-step cheeky butt wiggle dance
                SmoothDriveMotors(0.40f, -0.40f, 90);
                SmoothDriveMotors(-0.40f, 0.40f, 90);
                SmoothDriveMotors(0.40f, -0.40f, 90);
                StopMotors();
                PlayR2D2Chirp("GIGGLE_CHIRP");
                // Reclaim ground step
                SmoothDriveMotors(0.35f, 0.35f, 150);
                StopMotors();
                // Cheeky cocked head wink left & right
                OrganicMoveHead(65, 110, 180);
                OrganicMoveHead(115, 110, 220);
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Self-Satisfied Settle & Return to IDLE (Khoái chí tự mãn & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);  // Relaxed Cyan Breathing
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 85, 200);  // Cheeky little nod
                vTaskDelay(pdMS_TO_TICKS(120));
                OrganicMoveHead(90, 90, 350);
                break;

            case 20:  // 🐔 Scenario 20: "The Cowardly Bull: Charge to Panic Retreat" (Dọa húc dũng
                      // mãnh & Quay đầu 180° tháo chạy - 5 Phase Arc)
                // Phase 1: Bull Stomp & Threat Stance (Thủ thế bò tót - Dậm chân cào đất & Cúi gằm
                // đầu đe dọa)
                SetEyeColor(255, 60, 0, kEyeModeSolid);  // Flaming Orange Threat Solid
                PlayR2D2Chirp("ANGRY_BUZZ");
                OrganicMoveHead(90, 35, 350);  // Low menacing head charge posture
                // 3-step rapid ground scraping foot stomps
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                SmoothDriveMotors(0.35f, -0.35f, 80);
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                StopMotors();
                // Menacing horn bobbing
                OrganicMoveHead(90, 25, 120);
                OrganicMoveHead(90, 45, 120);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 2: Fierce Sprint Forward Charge (Lao thẳng tới trước như tên bắn)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Blood Red Charge Strobe!
                PlayR2D2Chirp("CHASER_BEEPS");
                OrganicMoveHead(90, 30, 150);
                SmoothDriveMotors(0.50f, 0.50f, 250);  // Fast aggressive forward sprint!
                StopMotors();  // Hard screeching halt right in front of target!
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 3: Sudden Regret & Chicken Panic (Bất ngờ hối hận giật thót tim "Thôi chết
                // rồi!")
                SetEyeColor(180, 0, 255, kEyeModeStrobe);  // Panic Purple Strobe
                PlayR2D2Chirp("SURPRISED_HIGH");
                OrganicMoveHead(90, 135, 120, true);  // Snap head high with sudden panic recoil
                PlayR2D2Chirp("RUNAWAY_CHICKEN");
                SmoothDriveMotors(-0.35f, -0.35f, 100);  // Little terror slip back
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(120));

                // Phase 4: 180° Emergency Spin & Scram Escape (Quay ngoắt 180° cắm đầu chạy thục
                // mạng)
                SetEyeColor(255, 200, 0, kEyeModeStrobe);  // Hazard Yellow Strobe
                OrganicMoveHead(150, 105, 180, true);      // Look back at escape route
                SmoothDriveMotors(-0.60f, 0.60f, 320);     // Rapid 180-degree pivot flip
                SmoothDriveMotors(0.50f, 0.50f, 220);      // High-speed escape sprint away!
                StopMotors();
                // Paranoid backward glances "Is it chasing me?!"
                OrganicMoveHead(35, 110, 180, true);
                OrganicMoveHead(145, 110, 200, true);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Safe Distance Sigh & Re-center (Thở phào may quá thoát rồi & Trở về
                // IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);  // Calming Cyan Breathing
                PlayR2D2Chirp("SNIFF_CHIRP");
                // Turn around back to face user
                SmoothDriveMotors(0.40f, -0.40f, 240);
                StopMotors();
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 80, 200);  // Relieved breath
                vTaskDelay(pdMS_TO_TICKS(120));
                OrganicMoveHead(90, 90, 350);
                break;

            case 21:  // 😤 Scenario 21: "The Defiant Brat: Stubborn Tantrum & Pouting Sulk" (Bướng
                      // bỉnh chống đối, dậm chân & Dỗi hờn - 5 Phase Arc)
                // Phase 1: Emphatic 'NO!' Head Shakes (Lắc đầu quầy quậy dứt khoát "Không bao
                // giờ!")
                SetEyeColor(255, 0, 100, kEyeModeStrobe);  // Hot Pink Fury Strobe
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                // 5 rapid defiant head shakes with snappy ease
                OrganicMoveHead(50, 85, 110, true);
                OrganicMoveHead(130, 85, 110, true);
                OrganicMoveHead(50, 85, 110, true);
                OrganicMoveHead(130, 85, 110, true);
                OrganicMoveHead(90, 85, 100);
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Tantrum Track Stomping (Dậm chân đành đạch chống đối & Lùi phắt ra xa)
                SetEyeColor(255, 30, 0, kEyeModeSolid);  // Fiery Red Fury Solid
                PlayR2D2Chirp("ANGRY_BUZZ");
                // Rapid 3-step foot stomping / track shudder
                SmoothDriveMotors(-0.40f, 0.40f, 90);
                SmoothDriveMotors(0.40f, -0.40f, 90);
                SmoothDriveMotors(-0.40f, 0.40f, 90);
                // Angry step back "Stay away from me!"
                SmoothDriveMotors(-0.35f, -0.35f, 140);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: The Cold Shoulder Sulk (Quay ngoắt mặt đi giận dỗi & Hất cằm kiêu kỳ)
                SetEyeColor(180, 0, 180, kEyeModeBreathing);  // Pouting Purple Glow
                PlayR2D2Chirp("BORED_SIGH");
                // Snap turn head all the way to 150 deg, nose in the air!
                OrganicMoveHead(150, 125, 250, true);
                vTaskDelay(pdMS_TO_TICKS(400));  // Stubborn freeze

                // Phase 4: Sneaky Side-Eye Peek (Liếc trộm xem chủ nhân có dỗ không & Ngượng ngùng)
                SetEyeColor(255, 140, 0, kEyeModeSolid);  // Guilty Amber Solid
                PlayR2D2Chirp("SHY_WHIMPER");
                // Sneaky side glance back towards user
                OrganicMoveHead(110, 85, 280);
                vTaskDelay(pdMS_TO_TICKS(200));
                // Caught! Snap head back away pouting
                OrganicMoveHead(145, 110, 180, true);
                vTaskDelay(pdMS_TO_TICKS(220));

                // Phase 5: Grudging Acceptance & Return to IDLE (Miễn cưỡng làm hòa & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);  // Soft Cyan Breathing
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 80, 250);  // Drop chin sheepishly
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 90, 350);
                break;

            case 22:  // 🕺 Scenario 22: "The Smooth Moonwalker: Slick Reverse Glide" (Đi lùi điệu
                      // nghệ, lướt xích đảo hông & Cúi chào quý ông - 5 Phase Arc)
                // Phase 1: Cool Pose & Beat Drop (Tạo dáng quý ông & Nhún chân lấy trớn)
                SetEyeColor(180, 0, 255, kEyeModeStrobe);  // Electric Violet Strobe
                PlayR2D2Chirp("FANFARE_CHIRP");
                OrganicMoveHead(65, 115, 220, true);  // Cool tilted chin snap
                // Quick rhythmic pre-glide tap
                SmoothDriveMotors(0.20f, 0.20f, 80);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(120));

                // Phase 2: S-Curve Moonwalk Reverse Glide (Đi lùi lượn sóng chữ S điệu nghệ)
                SetEyeColor(0, 255, 255, kEyeModeStrobe);  // Neon Cyan Strobe
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // Arc 1: Reverse curve left + Head leans right
                OrganicMoveHead(115, 105, 180);
                SmoothDriveMotors(-0.45f, -0.20f, 180);
                // Arc 2: Reverse curve right + Head leans left
                OrganicMoveHead(65, 105, 180);
                SmoothDriveMotors(-0.20f, -0.45f, 180);
                // Arc 3: Straight smooth slide back
                OrganicMoveHead(90, 110, 160);
                SmoothDriveMotors(-0.40f, -0.40f, 160);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: The 180° Reverse Spin & Head Snap Flare (Xoay đảo 180° phanh gấp & Hất
                // đầu "Hee-hee!")
                SetEyeColor(255, 215, 0, kEyeModeStrobe);  // Gold Disco Strobe
                PlayR2D2Chirp("RADAR_SWEEP_PING");
                SmoothDriveMotors(-0.55f, 0.55f, 260);  // Snappy 180-degree pivot flip
                StopMotors();
                OrganicMoveHead(90, 135, 180, true);  // High energetic head pop
                PlayR2D2Chirp("GIGGLE_CHIRP");
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 4: Gentleman's Tip-of-the-Hat Bow (Nghiêng đầu cúi chào phong thái quý ông)
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Emerald Gentleman Solid
                PlayR2D2Chirp("PROUD_TUNE");
                // Stylish diagonal polite bow
                OrganicMoveHead(75, 70, 250);
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(75, 85, 140);
                OrganicMoveHead(75, 70, 140);
                // Reclaim ground step
                SmoothDriveMotors(0.35f, 0.35f, 160);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 5: Smooth Return to Center & IDLE (Lướt êm về tâm & Trở về IDLE)
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
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

            case 24:  // 🦟 Story Scenario 24: "The Great Mosquito Battle" (Đại chiến diệt muỗi: Vo
                      // ve quanh tai & Cú đớp quyết định - 5 Phase Arc)
                // Phase 1: Tracking Annoying Buzzing Bug (Muỗi vo ve quanh tai - Đảo đầu giật cục
                // theo đường bay)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 1: Mosquito buzzing around head...");
                SetEyeColor(255, 150, 0, kEyeModeStrobe);  // Annoyed Amber Strobe
                PlayR2D2Chirp("ANGRY_BUZZ");
                // Rapid twitch tracking mosquito around head
                OrganicMoveHead(130, 115, 120, true);  // Buzzing near left ear!
                vTaskDelay(pdMS_TO_TICKS(80));
                OrganicMoveHead(50, 115, 120, true);  // Flew to right ear!
                vTaskDelay(pdMS_TO_TICKS(80));
                OrganicMoveHead(90, 140, 140, true);  // Flew right above eyes!
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Desperate Snap Bite & Frustrated Miss (Cú đớp hụt đầu tiên & Cay cú bực
                // tức)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 2: Snap bite & frustrated miss!");
                SetEyeColor(255, 40, 0, kEyeModeSolid);  // Frustrated Crimson
                OrganicMoveHead(90, 25, 100, true);      // Snap bite right in front! "Chomp!"
                PlayR2D2Chirp("CONFUSED_QUESTION");      // Missed!
                vTaskDelay(pdMS_TO_TICKS(120));
                // Look up frustrated & stamp tracks
                OrganicMoveHead(90, 125, 220);
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                // Angry track stomps
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                SmoothDriveMotors(0.35f, -0.35f, 80);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Frantic 360° Spin Pursuit (Xoay tròn 360° đuổi theo & Khóa tọa độ khi
                // muỗi hạ cánh)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 3: 360-degree pursuit & target lock...");
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Enraged Red Strobe!
                PlayR2D2Chirp("CHASER_BEEPS");
                SmoothDriveMotors(0.55f, -0.55f, 320);  // Full 360 spin following bug!
                StopMotors();
                // Target landed on floor! Lock on!
                OrganicMoveHead(90, 35, 180);
                SetEyeColor(255, 255, 0, kEyeModeStrobe);  // Yellow Target Laser
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 4: Ultimate Airborne Pounce & Crunch (Lao tới đớp tử thần - Tiêu diệt con
                // muỗi!)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 4: Lunging death snap!");
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                SmoothDriveMotors(0.50f, 0.50f, 180);  // Explosive forward leap!
                OrganicMoveHead(90, 20, 100, true);    // Gotcha! Chomp down!
                StopMotors();
                PlayR2D2Chirp("HERO_TRIUMPH");
                vTaskDelay(pdMS_TO_TICKS(100));
                // Chewing head bobs (nhai nhai đắc thắng)
                OrganicMoveHead(90, 35, 100);
                OrganicMoveHead(90, 20, 100);
                OrganicMoveHead(90, 35, 100);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 5: Victory Celebration & Peaceful Return (Vũ điệu ăn mừng lắc hông & Trở về
                // IDLE)
                ESP_LOGI(TAG, "🦟 Scenario #24 Step 5: Victory dance & peaceful IDLE!");
                SetEyeColor(0, 255, 120, kEyeModeStrobe);  // Emerald Celebration Strobe
                OrganicMoveHead(90, 135, 220, true);       // Proud head pop facing sky
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // 3-step celebratory wiggle
                SmoothDriveMotors(0.40f, -0.40f, 90);
                SmoothDriveMotors(-0.40f, 0.40f, 90);
                SmoothDriveMotors(0.40f, -0.40f, 90);
                StopMotors();
                // Reclaim ground step
                SmoothDriveMotors(-0.35f, -0.35f, 120);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 25:  // 🦴 Story Scenario 25: "The Little Paleontologist: Fossil Dig" (Nhà khảo cổ
                      // đào hóa thạch: Cào bới, thổi bụi & Khám phá - 5 Phase Arc)
                // Phase 1: Ground Radar Scan & Scent Tracking (Dò tìm địa tầng & Đánh hơi phát hiện
                // di chỉ)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 1: Scanning ground for fossils...");
                SetEyeColor(255, 140, 20, kEyeModeBreathing);  // Warm Earth Amber Breathing
                PlayR2D2Chirp("SCANNING_RADAR");
                OrganicMoveHead(90, 25, 450);  // Nose right to the floor
                vTaskDelay(pdMS_TO_TICKS(120));
                // Scanning left and right ground
                OrganicMoveHead(60, 25, 200);
                OrganicMoveHead(120, 25, 250);
                OrganicMoveHead(90, 25, 180);
                PlayR2D2Chirp("SNIFF_CHIRP");
                // Micro-sniffing twitches
                OrganicMoveHead(90, 35, 100);
                OrganicMoveHead(90, 25, 100);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 2: Vigorous Excavation & Sand Digging (Đào đất bới cát hăng say - Cào xích
                // 4 nhịp)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 2: Vigorous excavation digging...");
                SetEyeColor(255, 90, 0, kEyeModeSolid);  // Excavation Orange Solid
                PlayR2D2Chirp("TRAIL_HUNTER_CLICK");
                // 4-step rapid dirt-digging track and head coordination
                for (int dig = 0; dig < 2; dig++) {
                    OrganicMoveHead(75, 20, 100);
                    SmoothDriveMotors(0.45f, -0.20f, 100);
                    OrganicMoveHead(105, 20, 100);
                    SmoothDriveMotors(-0.20f, 0.45f, 100);
                }
                StopMotors();
                // Step back from the dirt pile
                SmoothDriveMotors(-0.25f, -0.25f, 120);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Blowing Dust & Careful Magnifying Inspection (Thổi bụi làm sạch & Soi
                // kính lúp kiểm tra)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 3: Blowing dust off fossil & inspection...");
                SetEyeColor(255, 200, 0, kEyeModeSolid);  // Focused Amber
                // Blow dust off "Phù phù!"
                OrganicMoveHead(90, 45, 200);
                OrganicMoveHead(90, 25, 120);
                PlayR2D2Chirp("STUBBORN_RASPBERRY");
                vTaskDelay(pdMS_TO_TICKS(200));
                // Inspecting fossil carefully with head tilt
                OrganicMoveHead(70, 30, 200);
                PlayR2D2Chirp("CAUTIOUS_PROBE");
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(110, 30, 200);
                vTaskDelay(pdMS_TO_TICKS(180));

                // Phase 4: Eureka! Priceless Fossil Discovery (Bảo vật vô giá! Bừng sáng vàng kim
                // khoe hóa thạch)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 4: Priceless fossil discovery!");
                SetEyeColor(255, 215, 0, kEyeModeStrobe);  // Golden Relic Strobe!
                OrganicMoveHead(90, 135, 250, true);       // Snap head high in glorious awe
                PlayR2D2Chirp("HERO_TRIUMPH");
                // Proud forward stride
                SmoothDriveMotors(0.40f, 0.40f, 180);
                StopMotors();
                // Proud nod to audience
                OrganicMoveHead(90, 115, 140);
                OrganicMoveHead(90, 135, 140);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Joyful Celebration & Return to IDLE (Hân hoan ăn mừng & Trở về IDLE)
                ESP_LOGI(TAG, "🦴 Scenario #25 Step 5: Joyful celebration & IDLE...");
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Emerald Triumph
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // 2-step celebratory wiggle
                SmoothDriveMotors(0.35f, -0.35f, 90);
                SmoothDriveMotors(-0.35f, 0.35f, 90);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(150));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 26:  // ⚡ Story Scenario 26: "The Brave Dinosaur: Thunderstorm to Heroic Courage"
                      // (Sấm sét hoảng sợ & Thức tỉnh lòng can đảm - 5 Phase Arc)
                // Phase 1: Blinding Lightning & Thunderclap Shock (Chớp lóe sáng & Sét đánh đì đùng
                // giật thót tim)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 1: Lightning flash & thunderclap shock...");
                SetEyeColor(255, 255, 255, kEyeModeStrobe);  // Pure Lightning White Strobe!
                PlayR2D2Chirp("SURPRISED_HIGH");
                OrganicMoveHead(90, 145, 120, true);  // Snap head high in terror
                PlayR2D2Chirp("ALARM_SCREAM");
                SmoothDriveMotors(-0.45f, -0.45f, 200);  // Emergency panic reverse jump!
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));

                // Phase 2: Terrified Cowering & Full-Body Trembling (Co ro sợ hãi - Toàn thân run
                // rẩy bần bật)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 2: Cowering and shivering in fear...");
                SetEyeColor(40, 20, 160, kEyeModeBreathing);  // Cold Shivering Indigo
                OrganicMoveHead(90, 25, 400);                 // Cower low to ground
                PlayR2D2Chirp("SHY_WHIMPER");
                // 4-step rapid full-body shivering
                for (int shiver = 0; shiver < 4; shiver++) {
                    SmoothDriveMotors(0.22f, -0.22f, 70);
                    SmoothDriveMotors(-0.22f, 0.22f, 70);
                }
                StopMotors();
                PlayR2D2Chirp("DIZZY_WHIMPER");
                vTaskDelay(pdMS_TO_TICKS(250));

                // Phase 3: Awakening of Inner Courage (Tia sáng dũng khí - Chậm rãi ngẩng đầu tìm
                // lại niềm tin)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 3: Gathering inner courage...");
                SetEyeColor(255, 140, 0, kEyeModeBreathing);  // Awakening Flame Amber
                PlayR2D2Chirp("CHARGING_HUM");
                // Slow heroic head rise (vươn thẳng đầu kiên định)
                OrganicMoveHead(90, 105, 750);
                vTaskDelay(pdMS_TO_TICKS(180));
                // Firm head shake "I am not afraid!"
                OrganicMoveHead(75, 105, 140);
                OrganicMoveHead(105, 105, 140);
                OrganicMoveHead(90, 110, 120);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 4: Heroic Roar & Stride Against the Storm (Tiếng gầm anh hùng & Bước tiến
                // hiên ngang nghênh bão)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 4: Heroic roar & standing ground!");
                SetEyeColor(0, 255, 255, kEyeModeStrobe);  // Electric Cyan Hero Strobe!
                PlayR2D2Chirp("ANGRY_BUZZ");
                // Bold forward stride taking ground
                SmoothDriveMotors(0.40f, 0.40f, 220);
                StopMotors();
                OrganicMoveHead(90, 140, 250, true);  // Roar facing high sky!
                PlayR2D2Chirp("HERO_TRIUMPH");
                // 2 authoritative triumphant nods
                OrganicMoveHead(90, 120, 140);
                OrganicMoveHead(90, 140, 140);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Peaceful Rainbow & Calm Return (Cầu vồng sau mưa & Bình yên trở về IDLE)
                ESP_LOGI(TAG, "⚡ Scenario #26 Step 5: Peaceful rainbow & calm return...");
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Emerald Peace Solid
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 85, 220);  // Gentle relieved nod
                vTaskDelay(pdMS_TO_TICKS(150));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;

            case 27:  // 🌠 Story Scenario 27: "The Stargazer: Wishing Upon a Shooting Star" (Ngắm
                      // sao băng & Ước nguyện ước mơ - 5 Phase Arc)
                // Phase 1: Gazing at the Night Cosmos (Ngước nhìn bầu trời đêm huyền diệu - Chiêm
                // ngưỡng dải ngân hà)
                ESP_LOGI(TAG, "🌠 Scenario #27 Step 1: Gazing into the cosmic night sky...");
                SetEyeColor(20, 50, 200, kEyeModeBreathing);  // Deep Cosmic Blue Breathing
                PlayR2D2Chirp("AWE_WONDER_WHISTLE");
                OrganicMoveHead(90, 140, 650);  // Slowly raise head high to the stars
                vTaskDelay(pdMS_TO_TICKS(200));
                // Panoramic stargazing sweep across the constellation
                OrganicMoveHead(55, 135, 350);
                OrganicMoveHead(125, 135, 450);
                OrganicMoveHead(90, 140, 250);
                vTaskDelay(pdMS_TO_TICKS(250));

                // Phase 2: A Shooting Star Streaks Across the Sky! (Vệt sao băng rạch ngang bầu
                // trời đêm)
                ESP_LOGI(TAG, "🌠 Scenario #27 Step 2: A shooting star streaks across the sky!");
                SetEyeColor(255, 230, 80, kEyeModeStrobe);  // Shooting Star Golden Strobe!
                PlayR2D2Chirp("RADAR_SWEEP_PING");
                // Rapid eye tracking the shooting star arc across the sky
                OrganicMoveHead(40, 145, 140, true);
                OrganicMoveHead(140, 140, 200, true);
                vTaskDelay(pdMS_TO_TICKS(150));

                // Phase 3: Making a Silent Heartfelt Wish (Nhắm mắt cúi đầu thành tâm ước nguyện)
                ESP_LOGI(TAG, "🌠 Scenario #27 Step 3: Making a silent heartfelt wish...");
                SetEyeColor(180, 80, 220, kEyeModeBreathing);  // Dreamy Twilight Violet Breathing
                PlayR2D2Chirp("LOVING_PURR");
                OrganicMoveHead(90, 65, 450);    // Bow head in reverent, heartfelt prayer
                vTaskDelay(pdMS_TO_TICKS(500));  // Sacred moment of silence

                // Phase 4: Joyful Hope & Celebration (Niềm vui bừng nở - Lời ước nguyện gửi đến các
                // vì sao)
                ESP_LOGI(TAG, "🌠 Scenario #27 Step 4: Joyful hope & celebration!");
                SetEyeColor(0, 255, 180, kEyeModeStrobe);  // Emerald Hope Strobe!
                OrganicMoveHead(90, 135, 250, true);       // Raise head high with radiant joy
                PlayR2D2Chirp("HERO_TRIUMPH");
                // Celebratory happy spin
                SmoothDriveMotors(0.45f, -0.45f, 280);
                StopMotors();
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Phase 5: Warm Blessing to Companion & Return to IDLE (Tri ân người đồng hành &
                // Trở về IDLE)
                ESP_LOGI(TAG, "🌠 Scenario #27 Step 5: Warm blessing & return to IDLE...");
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Gentle Emerald Blessing Solid
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                OrganicMoveHead(90, 80, 250);  // Respectful gratitude nod to human companion
                vTaskDelay(pdMS_TO_TICKS(180));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 350);
                break;
        }

        // Gracefully glide head back to natural neutral center (90, 90) before yielding to IDLE
        OrganicMoveHead(90.0f, 90.0f, 400);
        SetEyeColor(0, 200, 255, kEyeModeBreathing);
        vTaskDelay(pdMS_TO_TICKS(100));
        is_performing_action_.store(false);
    }

public:
    // Multi-Sensory Sequencer (Synchronized Head -> Audio -> Motor)
    void SetRobotState(TRaxState state) {
        current_state_ = state;
        is_performing_action_.store(true);

        ESP_LOGI(TAG, "========== Organic Sequencer State: %d ==========", (int)state);

        switch (state) {
            case kStateCurious:  // 1. Tò mò (Nghiêng đầu 2 bên, rướn cổ quan sát)
                SetEyeColor(0, 220, 255, kEyeModeBreathing);  // Curious Cyan
                OrganicMoveHead(120, 115, 300, true);         // Cock head right
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(120));
                OrganicMoveHead(60, 115, 280, true);  // Flip head left
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(90, 95, 220);  // Peer forward
                break;

            case kStateFocused:  // 2. Tập trung (Khóa mục tiêu & Tiến nhẹ 1 bước)
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Focused Emerald
                OrganicMoveHead(90, 95, 200);             // Lock gaze level
                PlayR2D2Chirp("FOCUSED_BEEP");
                SmoothDriveMotors(0.25f, 0.25f, 90);  // Small attentive forward step
                StopMotors();
                // Attentive micro head bob
                OrganicMoveHead(90, 85, 100);
                OrganicMoveHead(90, 95, 100);
                break;

            case kStateAlertWarning:  // 3. Cảnh báo (Bật ngửa đầu & Quét radar cảnh giác)
                SetEyeColor(255, 140, 0, kEyeModeStrobe);  // Alert Amber Strobe
                OrganicMoveHead(90, 135, 150, true);       // Snap head high alert
                PlayR2D2Chirp("ALERT_SWEEP");
                // Rapid threat sweep
                OrganicMoveHead(55, 120, 150);
                OrganicMoveHead(125, 120, 180);
                OrganicMoveHead(90, 110, 140);
                SmoothDriveMotors(-0.30f, -0.30f, 100);  // Cautious micro retreat
                StopMotors();
                break;

            case kStateAngry:  // 4. Tức giận (Cúi gằm gầm gừ, dậm chân cào đất & Lắc đầu)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Angry Blood Red Strobe
                OrganicMoveHead(90, 40, 180, true);      // Menacing low posture
                PlayR2D2Chirp("ANGRY_BUZZ");
                // 3-step rapid ground scraping foot stomps
                SmoothDriveMotors(-0.35f, 0.35f, 75);
                SmoothDriveMotors(0.35f, -0.35f, 75);
                SmoothDriveMotors(-0.35f, 0.35f, 75);
                StopMotors();
                // Defiant head thrash
                OrganicMoveHead(65, 65, 100);
                OrganicMoveHead(115, 65, 100);
                OrganicMoveHead(90, 75, 120);
                break;

            case kStateScared:  // 5. Sợ hãi (Bật ngửa giật thót, nhảy lùi & Rụt cổ run rẩy)
                SetEyeColor(150, 0, 255, kEyeModeStrobe);  // Panic Purple Strobe
                OrganicMoveHead(90, 140, 120, true);       // Recoil in terror
                PlayR2D2Chirp("SCARED_SCREAM");
                SmoothDriveMotors(-0.40f, -0.40f, 180);  // Panic jump back
                StopMotors();
                // Cower down shivering
                OrganicMoveHead(90, 30, 250);
                for (int sh = 0; sh < 2; sh++) {
                    SmoothDriveMotors(0.20f, -0.20f, 60);
                    SmoothDriveMotors(-0.20f, 0.20f, 60);
                }
                StopMotors();
                break;

            case kStateHappy:  // 6. Vui vẻ (Bật ngửa đầu hân hoan, lắc hông 3 nhịp & Gật đầu)
                SetEyeColor(0, 255, 100, kEyeModeStrobe);  // Happy Green Strobe
                OrganicMoveHead(90, 130, 200, true);       // Happy head pop
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // 3 celebratory butt wiggles
                SmoothDriveMotors(0.35f, -0.35f, 80);
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                SmoothDriveMotors(0.35f, -0.35f, 80);
                StopMotors();
                // Joyful nods
                OrganicMoveHead(90, 100, 120);
                OrganicMoveHead(90, 125, 120);
                break;

            case kStateDisappointed:  // 7. Thất vọng (Đầu rũ gục sát đất, thở dài & Lùi chậm)
                SetEyeColor(50, 50, 150, kEyeModeBreathing);  // Melancholy Blue
                PlayR2D2Chirp("SAD_SLIDE_DOWN");
                OrganicMoveHead(90, 25, 700);  // Head droops slowly to ground
                vTaskDelay(pdMS_TO_TICKS(150));
                // Disappointed subtle head shakes
                OrganicMoveHead(80, 25, 150);
                OrganicMoveHead(100, 25, 150);
                OrganicMoveHead(90, 25, 150);
                SmoothDriveMotors(-0.20f, -0.20f, 120);  // Slow dejected step back
                StopMotors();
                break;

            case kStateTargetDetected:  // 8. Khóa mục tiêu (Hạ thấp đầu khóa tọa độ & Bước tới)
                SetEyeColor(255, 255, 0, kEyeModeStrobe);  // Target Laser Yellow Strobe
                OrganicMoveHead(90, 50, 180, true);        // Lock onto target
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                SmoothDriveMotors(0.35f, 0.35f, 100);  // Confident step forward
                StopMotors();
                OrganicMoveHead(90, 95, 150);
                break;

            case kStateConfused:  // 9. Bối rối (Nghiêng đầu sâu 2 bên & Lắc đầu hỏi chấm)
                SetEyeColor(200, 0, 200, kEyeModeBreathing);  // Confused Magenta
                OrganicMoveHead(125, 115, 250, true);         // Deep cock right
                PlayR2D2Chirp("CONFUSED_QUESTION");
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(55, 115, 250, true);  // Deep cock left
                vTaskDelay(pdMS_TO_TICKS(150));
                // Confused little wobble
                OrganicMoveHead(90, 80, 150);
                OrganicMoveHead(90, 105, 150);
                break;

            case kStateSurprised:  // 10. Ngạc nhiên (Bật ngửa đầu, chớp mắt trắng & Nhìn quanh)
                SetEyeColor(255, 255, 255, kEyeModeStrobe);  // Pure White Shock Strobe
                OrganicMoveHead(90, 145, 120, true);         // Sudden shock snap high
                PlayR2D2Chirp("SURPRISED_HIGH");
                SmoothDriveMotors(-0.30f, -0.30f, 100);  // Micro flinch back
                StopMotors();
                // Wide-eyed left/right glance
                OrganicMoveHead(65, 120, 150);
                OrganicMoveHead(115, 120, 180);
                OrganicMoveHead(90, 100, 150);
                break;

            case kStateSuspicious:  // 11. Nghi ngờ (Liếc mắt góc hẹp & Đi rón rén)
                SetEyeColor(255, 100, 0, kEyeModeBreathing);  // Suspicious Amber
                OrganicMoveHead(45, 65, 350);                 // Sneaky look left
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                SmoothDriveMotors(0.25f, 0.15f, 120);  // Creeping curve step
                StopMotors();
                OrganicMoveHead(135, 65, 400);  // Sneaky look right
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 80, 250);
                break;

            case kStateLoving:  // 12. Yêu thương (Tiến lại gần, ngửa cằm & Húc má âu yếm)
                SetEyeColor(255, 105, 180, kEyeModeBreathing);  // Romantic Rose Pink
                SmoothDriveMotors(0.25f, 0.25f, 120, 0.15f);    // Gentle affectionate approach
                StopMotors();
                OrganicMoveHead(90, 125, 350);  // Chin up for pets
                PlayR2D2Chirp("LOVING_PURR");
                vTaskDelay(pdMS_TO_TICKS(120));
                // Double cheek nuzzle
                OrganicMoveHead(75, 115, 180);
                OrganicMoveHead(105, 115, 200);
                OrganicMoveHead(90, 100, 200);
                break;

            case kStateVictorious:  // 13. Chiến thắng (Xoay tròn 360°, ngửa đầu gầm & Kèn khải
                                    // hoàn)
                SetEyeColor(0, 255, 255, kEyeModeStrobe);  // Cyan Victory Strobe
                OrganicMoveHead(90, 140, 200, true);       // High proud roar pose
                PlayR2D2Chirp("FANFARE_CHIRP");
                // 360-degree celebratory pivot
                SmoothDriveMotors(0.55f, -0.55f, 300);
                StopMotors();
                PlayR2D2Chirp("HERO_TRIUMPH");
                // 2 authoritative triumphant nods
                OrganicMoveHead(90, 110, 120);
                OrganicMoveHead(90, 130, 120);
                OrganicMoveHead(90, 95, 150);
                break;

            case kStateSystemError:  // 18. Lỗi hệ thống (Rung giật chập cheng, rè rè & Rũ đầu)
                SetEyeColor(255, 0, 0, kEyeModeStrobe);  // Error Red Strobe
                PlayR2D2Chirp("GLITCH_NOISE");
                // 4 rapid glitch twitches
                OrganicMoveHead(75, 90, 60);
                OrganicMoveHead(105, 90, 60);
                OrganicMoveHead(80, 90, 60);
                OrganicMoveHead(100, 90, 60);
                OrganicMoveHead(90, 25, 250);  // Power droop
                break;

            case kStateLowBattery:  // 19. Sắp hết pin (Rên rỉ sụt nguồn, đầu trĩu nặng sát đất)
                SetEyeColor(255, 50, 0, kEyeModeBreathing);  // Fading Amber
                PlayR2D2Chirp("LOW_POWER_DROOP");
                OrganicMoveHead(90, 20, 850);  // Exhausted slump
                break;

            case kStateCharging:  // 20. Đang sạc pin (Hơi thở xanh lục ấm áp, nâng đầu thư thái)
                SetEyeColor(0, 255, 100, kEyeModeBreathing);  // Gentle Charging Green
                PlayR2D2Chirp("CHARGING_HUM");
                OrganicMoveHead(90, 105, 450);
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 90, 300);
                break;

            case kStateBooting:  // 21. Khởi động (Mắt trắng bừng sáng, vươn cao đầu sẵn sàng)
                SetEyeColor(255, 255, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 30, 200);
                PlayR2D2Chirp("BOOT_POWER_UP");
                OrganicMoveHead(90, 125, 400, true);
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 95, 200);
                break;

            case kStateSleeping:  // 22. Ngủ (Tắt mắt, chậm rãi gục đầu yên bình)
                SetEyeColor(60, 20, 140, kEyeModeBreathing);
                OrganicMoveHead(90, 25, 750);
                PlayR2D2Chirp("BORED_SIGH");
                SetEyeColor(0, 0, 0, kEyeModeOff);
                break;

            case kStateWakeGreetingEnergetic:  // 24. Chào Bừng Tỉnh Phấn Khởi (Energetic Pop-Up)
                SetEyeColor(255, 215, 0, kEyeModeStrobe);  // Golden Energy Strobe
                OrganicMoveHead(90, 130, 180, true);       // Snappy pop up
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                // 2 joyful track hops
                SmoothDriveMotors(0.35f, -0.35f, 80);
                SmoothDriveMotors(-0.35f, 0.35f, 80);
                StopMotors();
                OrganicMoveHead(90, 100, 150);
                break;

            case kStateWakeGreetingCurious:  // 25. Chào Tò Mò Nghiêng Đầu (Curious Cocked Head)
                SetEyeColor(0, 220, 255, kEyeModeBreathing);  // Turquoise Curious
                OrganicMoveHead(120, 120, 220, true);         // Cock deep right
                PlayR2D2Chirp("CURIOUS_WHISTLE");
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(60, 120, 220, true);  // Cock deep left
                vTaskDelay(pdMS_TO_TICKS(80));
                OrganicMoveHead(90, 95, 180);
                break;

            case kStateWakeGreetingFriendlyNod:  // 26. Chào Gật Đầu Thân Thiện (Friendly Double
                                                 // Nod)
                SetEyeColor(0, 255, 120, kEyeModeSolid);  // Friendly Mint Solid
                PlayR2D2Chirp("FOCUSED_BEEP");
                // 2 respectful crisp nods
                OrganicMoveHead(90, 120, 130);
                OrganicMoveHead(90, 75, 130);
                OrganicMoveHead(90, 115, 130);
                OrganicMoveHead(90, 95, 150);
                SmoothDriveMotors(0.25f, 0.25f, 90);  // Welcoming micro-step
                StopMotors();
                break;

            case kStateWakeGreetingSleepyWake:  // 27. Chào Vươn Vai Tỉnh Giấc (Sleepy Stretch to
                                                // Alert)
                SetEyeColor(255, 140, 40, kEyeModeBreathing);  // Warm Amber
                OrganicMoveHead(90, 35, 200);                  // Drowsy nod
                SetEyeColor(255, 255, 255, kEyeModeSolid);
                OrganicMoveHead(90, 140, 400, true);  // Big yawn stretch!
                PlayR2D2Chirp("YAWN_TUNE");
                vTaskDelay(pdMS_TO_TICKS(150));
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 95, 220);
                break;

            case kStateWakeGreetingScoutScan:  // 28. Chào Quét Radar Sẵn Sàng (Scout Radar Snap &
                                               // Lock)
                SetEyeColor(180, 0, 255, kEyeModeStrobe);  // Tactical Violet Strobe
                // Quick tactical perimeter sweep
                OrganicMoveHead(50, 105, 140, true);
                OrganicMoveHead(130, 105, 180, true);
                OrganicMoveHead(90, 105, 140, true);  // Snap lock center!
                PlayR2D2Chirp("RADAR_SWEEP_PING");
                break;

            case kStateIdle:  // 23. Chế độ IDLE
            default:
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                break;
        }

        // Action completed, resume gentle organic breathing & micro-saccades seamlessly
        vTaskDelay(pdMS_TO_TICKS(120));
        is_performing_action_.store(false);
    }

    void TriggerRandomWakeGreeting() {
        xTaskCreate(
            [](void* arg) {
                auto board = static_cast<TRaxBoard*>(arg);
                int greeting_style = 24 + (esp_random() % 5);
                ESP_LOGI(TAG, "🌟 Triggering Wake-up Organic Sequencer Greeting Style #%d",
                         greeting_style);
                board->SetRobotState(static_cast<TRaxState>(greeting_style));
                vTaskDelete(NULL);
            },
            "wake_greeting_task", 4096, this, 4, NULL);
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
                uint32_t sample_count = 0;

                ESP_LOGI(TAG, "📡 ToF Task Started - Monitoring VL53L0X sensor");

                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(100));

                    if (board->vl53l0x_dev_ == nullptr)
                        continue;

                    uint8_t reg = 0x14;
                    esp_err_t err =
                        i2c_master_transmit_receive(board->vl53l0x_dev_, &reg, 1, read_buf, 2, 50);
                    if (err == ESP_OK) {
                        uint16_t distance_mm = (read_buf[0] << 8) | read_buf[1];

                        // Periodic distance log (Every 1 second / 10 samples)
                        if (++sample_count % 10 == 0) {
                            ESP_LOGI(TAG, "📏 ToF Distance: %u mm", distance_mm);
                        }

                        if (distance_mm > 20 && distance_mm < 150) {
                            TickType_t now = xTaskGetTickCount();
                            if ((now - board->last_tof_trigger_time_) > pdMS_TO_TICKS(2000)) {
                                ESP_LOGI(TAG,
                                         "🎯 ToF Triggered! Distance: %u mm (Threshold: 20-150mm)",
                                         distance_mm);
                                board->last_tof_trigger_time_ = now;
                                board->TriggerSurpriseReaction();
                            }
                        }
                    } else {
                        // Rate-limited error logging (Every 2 seconds)
                        if (++sample_count % 20 == 0) {
                            ESP_LOGE(TAG, "⚠️ ToF I2C Read Error: %s (%d)", esp_err_to_name(err),
                                     err);
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
    // Spontaneous Scenarios & Prolonged-Idle Dozing Slumber Engine
    void StartIdleSequenceTask() {
        xTaskCreate(
            [](void* arg) {
                auto board = static_cast<TRaxBoard*>(arg);
                float sim_time = 0.0f;
                float target_gaze_pan = 90.0f;
                float target_gaze_tilt = 90.0f;
                float current_base_pan = 90.0f;
                float current_base_tilt = 90.0f;
                bool is_dozing = false;
                TickType_t last_active_tick = xTaskGetTickCount();
                TickType_t next_saccade_tick = xTaskGetTickCount() + pdMS_TO_TICKS(2500);
                TickType_t next_scenario_tick =
                    xTaskGetTickCount() + pdMS_TO_TICKS(8000 + (esp_random() % 6000));
                TickType_t next_drowse_nod_tick = 0;

                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(25));  // 40 FPS silky-smooth animation loop

                    TickType_t now = xTaskGetTickCount();

                    // When an emotion/tool call action is running, sync base coordinates and yield
                    if (board->is_performing_action_.load()) {
                        {
                            std::lock_guard<std::mutex> lock(board->servo_mutex_);
                            current_base_pan = board->current_pan_;
                            current_base_tilt = board->current_tilt_;
                            target_gaze_pan =
                                90.0f;  // Set gaze target to natural center for smooth exit
                            target_gaze_tilt = 90.0f;
                        }
                        if (is_dozing) {
                            is_dozing = false;
                            board->SetEyeColor(0, 200, 255, kEyeModeBreathing);
                        }
                        last_active_tick = now;
                        sim_time = 0.0f;
                        next_saccade_tick = now + pdMS_TO_TICKS(2000 + (esp_random() % 1500));
                        continue;
                    }

                    sim_time += 0.025f;

                    // Device State Awareness (Attentive Listening vs Speaking vs Relaxed Idle)
                    auto dev_state = Application::GetInstance().GetDeviceState();
                    bool is_listening = (dev_state == kDeviceStateListening);
                    bool is_speaking = (dev_state == kDeviceStateSpeaking);

                    // If in active dialog, reset activity timer and wake up if dozing
                    if (is_listening || is_speaking) {
                        if (is_dozing) {
                            is_dozing = false;
                            board->SetEyeColor(0, 200, 255, kEyeModeBreathing);
                            ESP_LOGI(
                                TAG,
                                "☀️ T-Rax woke up from dozing sleep due to active interaction!");
                        }
                        last_active_tick = now;
                        next_scenario_tick = now + pdMS_TO_TICKS(14000 + (esp_random() % 6000));
                    }

                    // Check for Prolonged Idle -> Enter Dozing Slumber Mode (>5 minutes / 300s of
                    // undisturbed idle)
                    uint32_t idle_duration_ms = (now - last_active_tick) * portTICK_PERIOD_MS;
                    if (!is_listening && !is_speaking && !is_dozing &&
                        (idle_duration_ms > 300000)) {
                        is_dozing = true;
                        board->SetEyeColor(60, 20, 140,
                                           kEyeModeBreathing);  // Deep Night Purple Breathing
                        target_gaze_pan = 90.0f;
                        target_gaze_tilt = 30.0f;  // Droop head low to sleep
                        next_drowse_nod_tick = now + pdMS_TO_TICKS(12000 + (esp_random() % 10000));
                        ESP_LOGI(TAG,
                                 "😴 T-Rax is feeling drowsy... entering Dozing Slumber Mode "
                                 "(>5min idle).");
                    }

                    // --- DOZING SLUMBER BEHAVIOR (Ngủ gà ngủ gật) ---
                    if (is_dozing) {
                        // Periodic "Nodding Awake" reflex (Thỉnh thoảng giật mình thức giấc rồi ngủ
                        // tiếp)
                        if (now >= next_drowse_nod_tick) {
                            int reflex_type = esp_random() % 3;
                            if (reflex_type == 0) {
                                // 🥱 Reflex 0: Sudden Hypnic Jerk & Drowsy Sigh (Giật mình ngửa đầu
                                // thức giấc rồi gục xuống thở dài)
                                ESP_LOGI(TAG, "🥱 Drowse Reflex: Sudden hypnic jerk awake!");
                                target_gaze_tilt = 75.0f;  // Snap head up awake
                                current_base_tilt = 70.0f;
                                board->SetEyeColor(255, 140, 40,
                                                   kEyeModeSolid);  // Warm amber eye flash
                                board->PlayR2D2Chirp("BORED_SIGH");
                                vTaskDelay(pdMS_TO_TICKS(300));
                                board->SetEyeColor(60, 20, 140,
                                                   kEyeModeBreathing);  // Back to sleepy purple
                                target_gaze_tilt = 28.0f;  // Head slowly droops back to sleep
                                next_drowse_nod_tick =
                                    now + pdMS_TO_TICKS(15000 + (esp_random() % 12000));
                            } else if (reflex_type == 1) {
                                // 🥱 Reflex 1: Drowsy Yawn & Sleepy Stretch (Ngáp dài vươn nhẹ cổ
                                // rồi ngủ tiếp)
                                ESP_LOGI(TAG, "🥱 Drowse Reflex: Sleepy yawn & stretch!");
                                target_gaze_tilt = 95.0f;
                                board->SetEyeColor(255, 200, 80, kEyeModeBreathing);
                                board->PlayR2D2Chirp("YAWN_TUNE");
                                vTaskDelay(pdMS_TO_TICKS(400));
                                board->SetEyeColor(60, 20, 140, kEyeModeBreathing);
                                target_gaze_tilt = 30.0f;  // Falls back to sleep
                                next_drowse_nod_tick =
                                    now + pdMS_TO_TICKS(18000 + (esp_random() % 14000));
                            } else {
                                // 🥱 Reflex 2: Sleepy Glance & Droop (Mở mắt ngó nghiêng lơ mơ rồi
                                // gục tiếp)
                                ESP_LOGI(TAG, "🥱 Drowse Reflex: Sleepy glance & droop!");
                                target_gaze_tilt = 65.0f;
                                target_gaze_pan = (esp_random() % 2 == 0) ? 65.0f : 115.0f;
                                board->PlayR2D2Chirp("LOW_POWER_DROOP");
                                board->SetEyeColor(120, 60, 200, kEyeModeSolid);
                                vTaskDelay(pdMS_TO_TICKS(350));
                                target_gaze_pan = 90.0f;
                                target_gaze_tilt = 25.0f;
                                board->SetEyeColor(60, 20, 140, kEyeModeBreathing);
                                next_drowse_nod_tick =
                                    now + pdMS_TO_TICKS(14000 + (esp_random() % 10000));
                            }
                        }

                        // Slow, heavy head drooping with ultra-smooth low-pass filter
                        current_base_pan += (target_gaze_pan - current_base_pan) * 0.03f;
                        current_base_tilt += (target_gaze_tilt - current_base_tilt) * 0.025f;

                        // Deep, ultra-slow slumber breathing (nhịp thở ngủ say chậm và sâu)
                        float breath_fade = fminf(1.0f, sim_time / 2.0f);
                        float pan_breath = (2.0f * sinf(0.35f * sim_time)) * breath_fade;
                        float tilt_breath = (3.5f * sinf(0.45f * sim_time)) * breath_fade;

                        float final_pan = current_base_pan + pan_breath;
                        float final_tilt = current_base_tilt + tilt_breath;

                        board->SetRawServoAngle(final_pan, final_tilt);
                        continue;
                    }

                    // --- ACTIVE IDLE BEHAVIOR (Trạng thái thức bình thường) ---
                    // Trigger Improvised Scenario periodically during undisturbed active idle
                    if (!is_listening && !is_speaking && (now >= next_scenario_tick)) {
                        int scenario_idx = esp_random() % 28;
                        board->PerformImprovisedScenario(scenario_idx);
                        next_scenario_tick =
                            xTaskGetTickCount() + pdMS_TO_TICKS(5000 + (esp_random() % 5000));
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
                                45.0f + (float)(esp_random() % 91);  // 45..135 deg horizontal sweep
                            target_gaze_tilt =
                                55.0f + (float)(esp_random() % 61);  // 55..115 deg vertical sweep
                            next_saccade_tick = now + pdMS_TO_TICKS(3500 + (esp_random() % 3000));
                        }
                    }

                    // Smooth exponential low-pass filter to smoothly glide base position toward
                    // gaze target (zero jerk)
                    float lerp_rate = is_listening ? 0.06f : 0.04f;
                    current_base_pan += (target_gaze_pan - current_base_pan) * lerp_rate;
                    current_base_tilt += (target_gaze_tilt - current_base_tilt) * lerp_rate;

                    // Smooth fade-in of breathing amplitude over 2.0s after an action completes
                    // (guarantees C0 and C1 continuity)
                    float breath_fade = fminf(1.0f, sim_time / 2.0f);

                    // Dual-Harmonic Biomimetic Breathing Wave (starts precisely at 0 offset at
                    // sim_time=0)
                    float pan_breath =
                        (8.0f * sinf(0.6f * sim_time) + 3.0f * sinf(1.1f * sim_time)) * breath_fade;
                    float tilt_breath =
                        (10.0f * sinf(0.85f * sim_time) + 4.0f * sinf(1.7f * sim_time)) *
                        breath_fade;

                    if (is_listening) {
                        // Attentive, medium-amplitude breathing while listening to user
                        pan_breath *= 0.35f;
                        tilt_breath = (4.0f * sinf(1.3f * sim_time)) * breath_fade;
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

        // 1. Set Emotion State Tool
        mcp_server.AddTool(
            "self.trax.set_state",
            "Đặt trạng thái cảm xúc / cử chỉ cho Robot T-Rax (1..28: 1=Tò mò, 2=Tập trung, 3=Cảnh "
            "báo, 4=Tức giận, 5=Sợ hãi, 6=Vui vẻ, 7=Thất vọng, 8=Phát hiện mục tiêu, 9=Bối rối, "
            "10=Ngạc nhiên, 11=Nghi ngờ, 12=Yêu thương, 13=Chiến thắng, 14=E ngại, 15=Chán nản, "
            "16=Kiêu ngạo, 17=Tìm kiếm, 24=Chào bừng tỉnh, 25=Chào tò mò, 26=Chào gật đầu, 27=Chào "
            "vươn vai, 28=Chào quét radar). CHỈ GỌI 1 LẦN DUY NHẤT.",
            PropertyList({Property("state_id", kPropertyTypeInteger, 1, 28)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int state_id = properties["state_id"].value<int>();
                if (state_id < 1 || state_id > 28) {
                    ESP_LOGW(TAG, "set_state(%d) out of range (1..28), clamping to Happy (6)",
                             state_id);
                    state_id = 6;
                }

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
                return std::string("State ") + std::to_string(state_id) + " applied.";
            });

        // 2. Move Forward Tool
        mcp_server.AddTool("self.trax.move_forward",
                           "Di chuyển Robot T-Rax TIẾN tới phía trước một khoảng thời gian "
                           "(duration_ms từ 200 đến 2000 ms).",
                           PropertyList({Property("duration_ms", kPropertyTypeInteger, 200, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int duration_ms = properties["duration_ms"].value<int>();
                               if (duration_ms < 100)
                                   duration_ms = 500;
                               if (duration_ms > 2000)
                                   duration_ms = 2000;
                               is_performing_action_.store(true);
                               SmoothDriveMotors(0.35f, 0.35f, duration_ms);
                               StopMotors();
                               is_performing_action_.store(false);
                               return std::string("Moved forward for ") +
                                      std::to_string(duration_ms) + " ms.";
                           });

        // 3. Move Backward Tool
        mcp_server.AddTool("self.trax.move_backward",
                           "Di chuyển Robot T-Rax LÙI về phía sau một khoảng thời gian "
                           "(duration_ms từ 200 đến 2000 ms).",
                           PropertyList({Property("duration_ms", kPropertyTypeInteger, 200, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int duration_ms = properties["duration_ms"].value<int>();
                               if (duration_ms < 100)
                                   duration_ms = 500;
                               if (duration_ms > 2000)
                                   duration_ms = 2000;
                               is_performing_action_.store(true);
                               SmoothDriveMotors(-0.35f, -0.35f, duration_ms);
                               StopMotors();
                               is_performing_action_.store(false);
                               return std::string("Moved backward for ") +
                                      std::to_string(duration_ms) + " ms.";
                           });

        // 4. Turn Left Tool
        mcp_server.AddTool(
            "self.trax.turn_left",
            "Xoay Robot T-Rax SANG TRÁI tại chỗ trên xích (duration_ms từ 200 đến 1500 ms).",
            PropertyList({Property("duration_ms", kPropertyTypeInteger, 200, 1500)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int duration_ms = properties["duration_ms"].value<int>();
                if (duration_ms < 100)
                    duration_ms = 350;
                if (duration_ms > 1500)
                    duration_ms = 1500;
                is_performing_action_.store(true);
                SmoothDriveMotors(-0.40f, 0.40f, duration_ms);
                StopMotors();
                is_performing_action_.store(false);
                return std::string("Turned left for ") + std::to_string(duration_ms) + " ms.";
            });

        // 5. Turn Right Tool
        mcp_server.AddTool(
            "self.trax.turn_right",
            "Xoay Robot T-Rax SANG PHẢI tại chỗ trên xích (duration_ms từ 200 đến 1500 ms).",
            PropertyList({Property("duration_ms", kPropertyTypeInteger, 200, 1500)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int duration_ms = properties["duration_ms"].value<int>();
                if (duration_ms < 100)
                    duration_ms = 350;
                if (duration_ms > 1500)
                    duration_ms = 1500;
                is_performing_action_.store(true);
                SmoothDriveMotors(0.40f, -0.40f, duration_ms);
                StopMotors();
                is_performing_action_.store(false);
                return std::string("Turned right for ") + std::to_string(duration_ms) + " ms.";
            });

        // 6. Nod Head Base Tool
        mcp_server.AddTool("self.trax.nod_head",
                           "Điều khiển đầu Robot T-Rax GẬT ĐẦU cơ bản thể hiện đồng ý, hài lòng.",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               is_performing_action_.store(true);
                               SetEyeColor(0, 255, 120, kEyeModeBreathing);
                               OrganicMoveHead(90, 120, 200);
                               OrganicMoveHead(90, 70, 200);
                               OrganicMoveHead(90, 110, 200);
                               OrganicMoveHead(90, 90, 200);
                               is_performing_action_.store(false);
                               return std::string("Nodded head.");
                           });

        // 6b. Nod Head Enthusiastic (Multi-Sensory: Head + Step Forward + Happy Sound)
        mcp_server.AddTool("self.trax.nod_enthusiastic",
                           "Biến thể GẬT ĐẦU HÀO HỨNG: Gật đầu hào hứng kết hợp nhún nhẹ tiến bước, mắt chớp xanh lục & âm thanh vui vẻ.",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               is_performing_action_.store(true);
                               SetEyeColor(0, 255, 120, kEyeModeStrobe);
                               PlayR2D2Chirp("HAPPY_ARPEGGIO");
                               SmoothDriveMotors(0.30f, 0.30f, 100);
                               StopMotors();
                               OrganicMoveHead(90, 130, 150);
                               OrganicMoveHead(90, 75, 150);
                               OrganicMoveHead(90, 120, 150);
                               OrganicMoveHead(90, 95, 150);
                               is_performing_action_.store(false);
                               return std::string("Executed enthusiastic nod with forward step.");
                           });

        // 6c. Nod Head Respectful (Multi-Sensory: Slow Bow + Micro Retreat + Focused Sound)
        mcp_server.AddTool("self.trax.nod_respectful",
                           "Biến thể GẬT ĐẦU TRÂN TRỌNG: Cúi đầu lịch sự, lùi rón rén 1 bước nhỏ, mắt xanh lam ấm áp.",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               is_performing_action_.store(true);
                               SetEyeColor(0, 200, 255, kEyeModeBreathing);
                               PlayR2D2Chirp("FOCUSED_BEEP");
                               OrganicMoveHead(90, 55, 250);
                               SmoothDriveMotors(-0.20f, -0.20f, 90);
                               StopMotors();
                               OrganicMoveHead(90, 95, 200);
                               is_performing_action_.store(false);
                               return std::string("Executed respectful bow nod.");
                           });

        // 7. Shake Head Base Tool
        mcp_server.AddTool(
            "self.trax.shake_head",
            "Điều khiển đầu Robot T-Rax LẮC ĐẦU cơ bản thể hiện phản đối, bối rối hoặc từ chối.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(255, 100, 0, kEyeModeStrobe);
                OrganicMoveHead(60, 90, 180);
                OrganicMoveHead(120, 90, 180);
                OrganicMoveHead(60, 90, 180);
                OrganicMoveHead(90, 90, 180);
                is_performing_action_.store(false);
                return std::string("Shook head.");
            });

        // 7b. Shake Head Emphatic (Multi-Sensory: Defiant Shake + Flinch Back + Angry Sound)
        mcp_server.AddTool(
            "self.trax.shake_emphatic",
            "Biến thể LẮC ĐẦU KIÊN QUYẾT: Lắc đầu mạnh mẽ dứt khoát, giật lùi động cơ phòng thủ & mắt đỏ chớp cảnh báo.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(255, 40, 0, kEyeModeStrobe);
                PlayR2D2Chirp("ANGRY_BUZZ");
                OrganicMoveHead(55, 80, 120, true);
                OrganicMoveHead(125, 80, 120, true);
                SmoothDriveMotors(-0.35f, -0.35f, 120);
                StopMotors();
                OrganicMoveHead(90, 90, 150);
                is_performing_action_.store(false);
                return std::string("Executed emphatic head shake with retreat.");
            });

        // 7c. Shake Head Confused (Multi-Sensory: Cocked Head + Track Wiggle + Confused Sound)
        mcp_server.AddTool(
            "self.trax.shake_confused",
            "Biến thể LẮC ĐẦU BỐI RỐI: Nghiêng lắc đầu thắc mắc kết hợp lắc hông xoay xích bối rối & mắt tím mộng mơ.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(200, 0, 255, kEyeModeBreathing);
                PlayR2D2Chirp("CONFUSED_QUESTION");
                OrganicMoveHead(125, 115, 220, true);
                OrganicMoveHead(55, 115, 220, true);
                SmoothDriveMotors(0.25f, -0.25f, 70);
                SmoothDriveMotors(-0.25f, 0.25f, 70);
                StopMotors();
                OrganicMoveHead(90, 95, 180);
                is_performing_action_.store(false);
                return std::string("Executed confused head shake with hip wiggle.");
            });

        // 8. Play R2D2 Sound Tool
        mcp_server.AddTool("self.trax.play_sound",
                           "Phát âm thanh R2D2 đặc trưng (sound_id: 1=Vui, 2=Tức giận, 3=Bối rối, "
                           "4=Cảnh báo, 5=Sợ hãi, 6=Ngạc nhiên, 7=Chiến thắng).",
                           PropertyList({Property("sound_id", kPropertyTypeInteger, 1, 7)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int sound_id = properties["sound_id"].value<int>();
                               const char* sounds[] = {"HAPPY_ARPEGGIO",    "ANGRY_BUZZ",
                                                       "CONFUSED_QUESTION", "ALERT_SWEEP",
                                                       "SCARED_SCREAM",     "SURPRISED_HIGH",
                                                       "HERO_TRIUMPH"};
                               if (sound_id < 1 || sound_id > 7)
                                   sound_id = 1;
                               is_performing_action_.store(true);
                               PlayR2D2Chirp(sounds[sound_id - 1]);
                               is_performing_action_.store(false);
                               return std::string("Played sound ") + std::to_string(sound_id);
                           });

        // 9. Hardware Self-Test Diagnostic Tool
        mcp_server.AddTool(
            "self.trax.test_hardware",
            "Chạy quy trình tự kiểm tra chuẩn đoán phần cứng (Servo Pan/Tilt, Động cơ Trái/Phải).",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                ESP_LOGI(TAG, "Running Hardware Diagnostic Self-Test...");

                // Step 1: Pan Servo Test (Left -> Center -> Right -> Center)
                ESP_LOGI(TAG, "1/4 Testing Pan Servo (GPIO 5): Left 45 -> Center 90 -> Right 135");
                OrganicMoveHead(45, 90, 400);
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(135, 90, 400);
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 90, 400);
                vTaskDelay(pdMS_TO_TICKS(300));

                // Step 2: Tilt Servo Test (Down -> Center -> Up -> Center)
                ESP_LOGI(TAG, "2/4 Testing Tilt Servo (GPIO 6): Down 60 -> Center 90 -> Up 120");
                OrganicMoveHead(90, 60, 400);
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 120, 400);
                vTaskDelay(pdMS_TO_TICKS(300));
                OrganicMoveHead(90, 90, 400);
                vTaskDelay(pdMS_TO_TICKS(300));

                // Step 3: Left Motor Test (Forward -> Backward -> Stop)
                ESP_LOGI(TAG, "3/4 Testing Left Motor (GPIO 1 & 2): Forward -> Backward");
                SmoothDriveMotors(0.35f, 0.0f, 400);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));
                SmoothDriveMotors(-0.35f, 0.0f, 400);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(300));

                // Step 4: Right Motor Test (Forward -> Backward -> Stop)
                ESP_LOGI(TAG, "4/4 Testing Right Motor (GPIO 3 & 4): Forward -> Backward");
                SmoothDriveMotors(0.0f, 0.35f, 400);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(200));
                SmoothDriveMotors(0.0f, -0.35f, 400);
                StopMotors();
                vTaskDelay(pdMS_TO_TICKS(300));

                is_performing_action_.store(false);
                return std::string("Hardware self-test completed successfully.");
            });

        // 10. Look Around Tool
        mcp_server.AddTool(
            "self.trax.look_around",
            "Cho Robot T-Rax liếc mắt nhìn quanh quét môi trường (Quét từ trái 45° sang phải 135°, ngửa cao rồi trả về trung tâm).",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(0, 220, 255, kEyeModeStrobe);
                OrganicMoveHead(45, 100, 250);
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(135, 100, 300);
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(90, 125, 250);
                OrganicMoveHead(90, 95, 200);
                is_performing_action_.store(false);
                return std::string("Looked around environment.");
            });

        // 11. Direct Head Angle Adjustment Tool
        mcp_server.AddTool(
            "self.trax.set_head_position",
            "Điều chỉnh vị trí góc nhìn của đầu Robot T-Rax trực tiếp (pan_angle từ 45 đến 135 độ, tilt_angle từ 30 đến 140 độ, duration_ms từ 100 đến 1000 ms).",
            PropertyList({Property("pan_angle", kPropertyTypeInteger, 45, 135),
                          Property("tilt_angle", kPropertyTypeInteger, 30, 140),
                          Property("duration_ms", kPropertyTypeInteger, 100, 1000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int pan = properties["pan_angle"].value<int>();
                int tilt = properties["tilt_angle"].value<int>();
                int duration = properties["duration_ms"].value<int>();

                if (pan < 45) pan = 45;
                if (pan > 135) pan = 135;
                if (tilt < 30) tilt = 30;
                if (tilt > 140) tilt = 140;
                if (duration < 100) duration = 100;
                if (duration > 1000) duration = 1000;

                is_performing_action_.store(true);
                OrganicMoveHead(pan, tilt, duration);
                is_performing_action_.store(false);
                return std::string("Set head position to Pan=") + std::to_string(pan) +
                       ", Tilt=" + std::to_string(tilt);
            });

        // 12. Set WS2812 Eye Color & Animation Tool
        mcp_server.AddTool(
            "self.trax.set_eye_color",
            "Đổi màu sắc và chế độ hiệu ứng mắt LED RGB WS2812 của T-Rax (red, green, blue từ 0..255; mode: 1=Breathing, 2=Solid, 3=Strobe, 4=Off).",
            PropertyList({Property("red", kPropertyTypeInteger, 0, 255),
                          Property("green", kPropertyTypeInteger, 0, 255),
                          Property("blue", kPropertyTypeInteger, 0, 255),
                          Property("mode", kPropertyTypeInteger, 1, 4)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int r = properties["red"].value<int>();
                int g = properties["green"].value<int>();
                int b = properties["blue"].value<int>();
                int mode_val = properties["mode"].value<int>();

                if (r < 0) r = 0;
                if (r > 255) r = 255;
                if (g < 0) g = 0;
                if (g > 255) g = 255;
                if (b < 0) b = 0;
                if (b > 255) b = 255;

                EyeLedMode mode = kEyeModeBreathing;
                if (mode_val == 2) mode = kEyeModeSolid;
                else if (mode_val == 3) mode = kEyeModeStrobe;
                else if (mode_val == 4) mode = kEyeModeOff;

                SetEyeColor(r, g, b, mode);
                return std::string("Set eye color to RGB(") + std::to_string(r) + "," +
                       std::to_string(g) + "," + std::to_string(b) + ") mode " + std::to_string(mode_val);
            });

        // 13. Celebration Wiggle Dance Tool
        mcp_server.AddTool(
            "self.trax.dance",
            "Cho Robot T-Rax nhảy múa ăn mừng vui nhộn (Lắc hông bằng động cơ xích, gật đầu liên hồi, mắt chớp nháy & phát kèn khải hoàn).",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(0, 255, 200, kEyeModeStrobe);
                PlayR2D2Chirp("HERO_TRIUMPH");
                SmoothDriveMotors(0.40f, -0.40f, 150);
                SmoothDriveMotors(-0.40f, 0.40f, 150);
                SmoothDriveMotors(0.40f, -0.40f, 150);
                StopMotors();
                OrganicMoveHead(60, 120, 150);
                OrganicMoveHead(120, 70, 150);
                OrganicMoveHead(90, 95, 200);
                is_performing_action_.store(false);
                return std::string("Executed victory dance.");
            });

        // 14. Emergency Stop All Tool
        mcp_server.AddTool(
            "self.trax.stop_all",
            "Dừng khẩn cấp mọi chuyển động của động cơ, đưa đầu về trung tâm và reset mắt về trạng thái nghỉ.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                StopMotors();
                OrganicMoveHead(90, 95, 200);
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                is_performing_action_.store(false);
                return std::string("Stopped all motor movements and reset to Idle.");
            });

        // 15. Listen Attentively Conversational Gesture Tool
        mcp_server.AddTool(
            "self.trax.listen_attentively",
            "Cử chỉ LẮNG NGHE CHÚ Ý: Robot nghiêng nhẹ đầu lắng nghe người dùng nói, mắt phát sáng xanh ngọc & phát tiếng bip tập trung.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(0, 255, 180, kEyeModeSolid);
                PlayR2D2Chirp("FOCUSED_BEEP");
                OrganicMoveHead(105, 110, 200, true);
                vTaskDelay(pdMS_TO_TICKS(100));
                OrganicMoveHead(90, 95, 150);
                is_performing_action_.store(false);
                return std::string("Attentively listening gesture executed.");
            });

        // 16. Think & Ponder Conversational Gesture Tool
        mcp_server.AddTool(
            "self.trax.think_ponder",
            "Cử chỉ SUY NGHĨ TÌM CÂU TRẢ LỜI: Robot ngửa đầu nghiêng góc tư duy, mắt sáng vàng hổ phách nhịp thở & phát âm thanh bối rối tư duy.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(255, 180, 0, kEyeModeBreathing);
                PlayR2D2Chirp("CONFUSED_QUESTION");
                OrganicMoveHead(65, 125, 250, true);
                vTaskDelay(pdMS_TO_TICKS(150));
                OrganicMoveHead(90, 95, 200);
                is_performing_action_.store(false);
                return std::string("Pondering thinking gesture executed.");
            });

        // 17. Express Empathy Conversational Gesture Tool
        mcp_server.AddTool(
            "self.trax.express_empathy",
            "Cử chỉ AN ỦI THẤU HIỂU: Robot gật đầu nhẹ nhàng trân trọng, mắt hồng ấm áp & tiến nhẹ 1 bước lại gần người dùng.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(255, 120, 180, kEyeModeBreathing);
                PlayR2D2Chirp("LOVING_PURR");
                SmoothDriveMotors(0.20f, 0.20f, 90);
                StopMotors();
                OrganicMoveHead(90, 70, 250);
                OrganicMoveHead(90, 100, 200);
                is_performing_action_.store(false);
                return std::string("Empathy comforting gesture executed.");
            });

        // 18. Express Excitement Conversational Gesture Tool
        mcp_server.AddTool(
            "self.trax.express_excitement",
            "Cử chỉ HÀO HỨNG PHẤN KHỞI: Robot bật vươn cao đầu, nhún 2 nhịp xích ăn mừng, mắt vàng chớp nháy & phát chuỗi âm thanh reo vui.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(255, 215, 0, kEyeModeStrobe);
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                OrganicMoveHead(90, 130, 150, true);
                SmoothDriveMotors(0.30f, -0.30f, 60);
                SmoothDriveMotors(-0.30f, 0.30f, 60);
                StopMotors();
                OrganicMoveHead(90, 95, 150);
                is_performing_action_.store(false);
                return std::string("Excitement gesture executed.");
            });

        // 19. Wave & Warm Greeting Gesture Tool
        mcp_server.AddTool(
            "self.trax.wave_greeting",
            "Cử chỉ CHÀO ĐÓN THÂN THIỆN: Robot gật đầu đôi chào mừng, nhún bước nhẹ tiến tới & mắt chớp xanh lam chào bạn.",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                is_performing_action_.store(true);
                SetEyeColor(0, 255, 120, kEyeModeStrobe);
                PlayR2D2Chirp("BOOT_POWER_UP");
                SmoothDriveMotors(0.25f, 0.25f, 90);
                StopMotors();
                OrganicMoveHead(90, 120, 150);
                OrganicMoveHead(90, 80, 150);
                OrganicMoveHead(90, 105, 150);
                OrganicMoveHead(90, 95, 150);
                is_performing_action_.store(false);
                return std::string("Warm greeting gesture executed.");
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

    TRaxLed led_{*this};

    virtual Led* GetLed() override { return &led_; }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
                                               AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return &display_; }
};

void TRaxLed::OnStateChanged() {
    auto dev_state = Application::GetInstance().GetDeviceState();
    if (dev_state == kDeviceStateConnecting &&
        (prev_state_ == kDeviceStateIdle || prev_state_ == kDeviceStateUnknown)) {
        board_.TriggerRandomWakeGreeting();
    }
    prev_state_ = dev_state;
}

DECLARE_BOARD(TRaxBoard);

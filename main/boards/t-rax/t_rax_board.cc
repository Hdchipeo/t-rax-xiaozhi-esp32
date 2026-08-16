#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <led_strip.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <esp_random.h>

#define TAG "TRaxBoard"

// Standard VL53L0X I2C Address
#define VL53L0X_I2C_ADDR 0x29

// 23 Robot States Enum
enum TRaxState {
    kStateCurious = 1,      // 1. Tò mò
    kStateFocused,          // 2. Tập trung
    kStateAlertWarning,     // 3. Cảnh báo
    kStateAngry,            // 4. Tức giận
    kStateScared,           // 5. Sợ hãi
    kStateHappy,            // 6. Vui vẻ
    kStateDisappointed,     // 7. Thất vọng
    kStateTargetDetected,   // 8. Phát hiện mục tiêu
    kStateConfused,         // 9. Bối rối
    kStateSurprised,        // 10. Ngạc nhiên
    kStateSuspicious,       // 11. Nghi ngờ
    kStateLoving,           // 12. Yêu thương
    kStateVictorious,       // 13. Chiến thắng
    kStateShy,              // 14. E ngại
    kStateBored,            // 15. Chán nản
    kStateArrogant,         // 16. Kiêu ngạo
    kStateSearching,        // 17. Tìm kiếm
    kStateSystemError,      // 18. Lỗi hệ thống
    kStateLowBattery,       // 19. Sắp hết pin
    kStateCharging,         // 20. Đang sạc pin
    kStateBooting,          // 21. Khởi động
    kStateSleeping,         // 22. Ngủ
    kStateIdle              // 23. Chế độ IDLE
};

enum EyeLedMode {
    kEyeModeBreathing,
    kEyeModeStrobe,
    kEyeModeSolid,
    kEyeModeBlink,
    kEyeModeOff
};

// Organic Easing Functions Engine
class OrganicEasing {
public:
    // Cubic Ease-In-Out (Smooth S-curve acceleration & deceleration)
    static float CubicEaseInOut(float t) {
        if (t < 0.5f) {
            return 4.0f * t * t * t;
        } else {
            float f = (-2.0f * t + 2.0f);
            return 1.0f - (f * f * f) / 2.0f;
        }
    }

    // Back Ease-Out (Overshoot & Settle for sudden reaction)
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
    TickType_t last_tof_trigger_time_ = 0;

    TRaxState current_state_ = kStateIdle;
    EyeLedMode current_led_mode_ = kEyeModeBreathing;
    uint8_t target_r_ = 0, target_g_ = 200, target_b_ = 255;
    bool is_idle_active_ = true;

    // Current Servo Base Positions
    float current_pan_ = 90.0f;
    float current_tilt_ = 90.0f;

    // Motor PWM Low-Pass Filter Output State
    float pwm_out_left_ = 0.0f;
    float pwm_out_right_ = 0.0f;

    void InitializeI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = TOF_I2C_SDA_GPIO,
            .scl_io_num = TOF_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = { .enable_internal_pullup = 1 },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_));

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L0X_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &vl53l0x_dev_);
        ESP_LOGI(TAG, "I2C Bus & VL53L0X Device initialized on SDA=%d, SCL=%d", TOF_I2C_SDA_GPIO, TOF_I2C_SCL_GPIO);
    }

    void InitializeWs2812Led() {
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = WS2812_RGB_LED_GPIO;
        strip_config.max_leds = 1;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000;

        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &eye_led_strip_);
        if (err == ESP_OK) {
            led_strip_clear(eye_led_strip_);
            ESP_LOGI(TAG, "WS2812 RGB Eye LED initialized on GPIO %d", WS2812_RGB_LED_GPIO);
        }
    }

    void InitializeGpioOutputs() {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_AIN1_GPIO) | (1ULL << MOTOR_LEFT_AIN2_GPIO) |
                               (1ULL << MOTOR_RIGHT_BIN1_GPIO) | (1ULL << MOTOR_RIGHT_BIN2_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        StopMotors();
    }

    void InitializeServos() {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_13_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 50,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel_pan = {
            .gpio_num       = SERVO_PAN_PAN_GPIO,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 410,
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel_pan);

        ledc_channel_config_t ledc_channel_tilt = {
            .gpio_num       = SERVO_TILT_GPIO,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_1,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 410,
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel_tilt);
    }

    void SetRawServoAngle(float pan, float tilt) {
        current_pan_ = pan;
        current_tilt_ = tilt;
        uint32_t pan_duty = 205 + (uint32_t)((pan * 410.0f) / 180.0f);
        uint32_t tilt_duty = 205 + (uint32_t)((tilt * 410.0f) / 180.0f);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pan_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, tilt_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }

    // Smooth Servo Motion with Cubic Easing & Velocity-based WS2812 Dynamic Brightness
    void OrganicMoveHead(float target_pan, float target_tilt, int duration_ms, bool use_overshoot = false) {
        float start_pan = current_pan_;
        float start_tilt = current_tilt_;
        int steps = duration_ms / 20; // 50 FPS (20ms step)
        if (steps < 1) steps = 1;

        for (int i = 1; i <= steps; i++) {
            float t = (float)i / (float)steps;
            float ease = use_overshoot ? OrganicEasing::BackEaseOut(t) : OrganicEasing::CubicEaseInOut(t);

            float current_p = start_pan + ease * (target_pan - start_pan);
            float current_t = start_tilt + ease * (target_tilt - start_tilt);

            SetRawServoAngle(current_p, current_t);

            // Dynamic WS2812 Brightness boost during peak velocity
            float velocity = fabsf(target_pan - start_pan) + fabsf(target_tilt - start_tilt);
            if (velocity > 30.0f && eye_led_strip_ != nullptr) {
                float intensity = 0.5f + 0.5f * sinf(t * M_PI); // Peak at midpoint
                uint8_t r = (uint8_t)(target_r_ * intensity);
                uint8_t g = (uint8_t)(target_g_ * intensity);
                uint8_t b = (uint8_t)(target_b_ * intensity);
                led_strip_set_pixel(eye_led_strip_, 0, r, g, b);
                led_strip_refresh(eye_led_strip_);
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // DRV8833 Low-Pass Filter Motor Controller (S-Curve PWM Smoothing)
    void SmoothDriveMotors(float target_left, float target_right, int duration_ms, float alpha = 0.15f) {
        int steps = duration_ms / 20;
        if (steps < 1) steps = 1;

        for (int i = 0; i < steps; i++) {
            // Low-Pass Filter: PWM_out(k) = PWM_out(k-1) + alpha * (PWM_target - PWM_out(k-1))
            pwm_out_left_ = pwm_out_left_ + alpha * (target_left - pwm_out_left_);
            pwm_out_right_ = pwm_out_right_ + alpha * (target_right - pwm_out_right_);

            // Apply direction & level
            gpio_set_level(MOTOR_LEFT_AIN1_GPIO, pwm_out_left_ > 0 ? 1 : 0);
            gpio_set_level(MOTOR_LEFT_AIN2_GPIO, pwm_out_left_ < 0 ? 1 : 0);
            gpio_set_level(MOTOR_RIGHT_BIN1_GPIO, pwm_out_right_ > 0 ? 1 : 0);
            gpio_set_level(MOTOR_RIGHT_BIN2_GPIO, pwm_out_right_ < 0 ? 1 : 0);

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void StopMotors() {
        SmoothDriveMotors(0.0f, 0.0f, 100);
        gpio_set_level(MOTOR_LEFT_AIN1_GPIO, 0);
        gpio_set_level(MOTOR_LEFT_AIN2_GPIO, 0);
        gpio_set_level(MOTOR_RIGHT_BIN1_GPIO, 0);
        gpio_set_level(MOTOR_RIGHT_BIN2_GPIO, 0);
        pwm_out_left_ = 0.0f;
        pwm_out_right_ = 0.0f;
    }

    void SetEyeColor(uint8_t r, uint8_t g, uint8_t b, EyeLedMode mode = kEyeModeBreathing) {
        target_r_ = r;
        target_g_ = g;
        target_b_ = b;
        current_led_mode_ = mode;
    }

    void PlayR2D2Chirp(const char* sound_name) {
        ESP_LOGI(TAG, "R2D2 Audio Synthesizer: Playing [%s]", sound_name);
    }

public:
    // Multi-Sensory Sequencer (Synchronized Head -> Audio -> Motor)
    void SetRobotState(TRaxState state) {
        current_state_ = state;
        is_idle_active_ = (state == kStateIdle);

        ESP_LOGI(TAG, "========== Organic Sequencer State: %d ==========", (int)state);

        switch (state) {
            case kStateCurious: // 1. Tò mò
                SetEyeColor(0, 220, 255, kEyeModeBreathing);
                OrganicMoveHead(120, 110, 500); // 1. Servo Leads
                PlayR2D2Chirp("CURIOUS_WHISTLE"); // 2. Audio Peak Sync
                break;

            case kStateFocused: // 2. Tập trung
                SetEyeColor(0, 255, 120, kEyeModeSolid);
                OrganicMoveHead(90, 100, 300);
                PlayR2D2Chirp("FOCUSED_BEEP");
                break;

            case kStateAlertWarning: // 3. Cảnh báo
                SetEyeColor(255, 140, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 130, 200, true);
                PlayR2D2Chirp("ALERT_SWEEP");
                break;

            case kStateAngry: // 4. Tức giận
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                OrganicMoveHead(60, 80, 200, true);
                PlayR2D2Chirp("ANGRY_BUZZ");
                SmoothDriveMotors(1.0f, -1.0f, 250); // Wiggle
                SmoothDriveMotors(-1.0f, 1.0f, 250);
                StopMotors();
                break;

            case kStateScared: // 5. Sợ hãi
                SetEyeColor(150, 0, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 40, 200, true); // Overshoot Back Ease
                PlayR2D2Chirp("SCARED_SCREAM");
                vTaskDelay(pdMS_TO_TICKS(50)); // 50ms Motor Lag Sync
                SmoothDriveMotors(-1.0f, -1.0f, 400); // Backward ramp
                StopMotors();
                break;

            case kStateHappy: // 6. Vui vẻ
                SetEyeColor(0, 255, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 120, 400);
                PlayR2D2Chirp("HAPPY_ARPEGGIO");
                SmoothDriveMotors(1.0f, -1.0f, 200);
                SmoothDriveMotors(-1.0f, 1.0f, 200);
                StopMotors();
                break;

            case kStateDisappointed: // 7. Thất vọng
                SetEyeColor(50, 50, 150, kEyeModeBreathing);
                OrganicMoveHead(90, 30, 800); // Slow sad move
                PlayR2D2Chirp("SAD_SLIDE_DOWN");
                break;

            case kStateTargetDetected: // 8. Phát hiện mục tiêu
                SetEyeColor(255, 255, 0, kEyeModeSolid);
                OrganicMoveHead(90, 110, 250, true);
                PlayR2D2Chirp("TARGET_LOCK_BEEP");
                break;

            case kStateConfused: // 9. Bối rối
                SetEyeColor(200, 0, 200, kEyeModeBreathing);
                OrganicMoveHead(135, 80, 600); // 45 deg Tilt
                PlayR2D2Chirp("CONFUSED_QUESTION");
                break;

            case kStateSurprised: // 10. Ngạc nhiên
                SetEyeColor(255, 255, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 140, 200, true); // Fast overshoot
                PlayR2D2Chirp("SURPRISED_HIGH");
                break;

            case kStateSuspicious: // 11. Nghi ngờ
                SetEyeColor(255, 100, 0, kEyeModeBreathing);
                OrganicMoveHead(45, 90, 700);
                PlayR2D2Chirp("SUSPICIOUS_LOW");
                break;

            case kStateLoving: // 12. Yêu thương
                SetEyeColor(255, 105, 180, kEyeModeBreathing);
                OrganicMoveHead(90, 105, 600);
                PlayR2D2Chirp("LOVING_PURR");
                break;

            case kStateVictorious: // 13. Chiến thắng
                SetEyeColor(0, 255, 255, kEyeModeStrobe);
                OrganicMoveHead(90, 135, 300);
                PlayR2D2Chirp("FANFARE_CHIRP");
                SmoothDriveMotors(1.0f, -1.0f, 300);
                SmoothDriveMotors(-1.0f, 1.0f, 300);
                StopMotors();
                break;

            case kStateShy: // 14. E ngại
                SetEyeColor(255, 180, 200, kEyeModeBreathing);
                OrganicMoveHead(120, 40, 700);
                PlayR2D2Chirp("SHY_WHIMPER");
                break;

            case kStateBored: // 15. Chán nản
                SetEyeColor(100, 100, 100, kEyeModeBreathing);
                OrganicMoveHead(90, 50, 900);
                PlayR2D2Chirp("BORED_SIGH");
                break;

            case kStateArrogant: // 16. Kiêu ngạo
                SetEyeColor(255, 215, 0, kEyeModeSolid);
                OrganicMoveHead(90, 150, 500);
                PlayR2D2Chirp("PROUD_TUNE");
                break;

            case kStateSearching: // 17. Tìm kiếm
                SetEyeColor(0, 150, 255, kEyeModeStrobe);
                OrganicMoveHead(45, 90, 600);
                OrganicMoveHead(135, 90, 600);
                OrganicMoveHead(90, 90, 400);
                PlayR2D2Chirp("SCANNING_RADAR");
                break;

            case kStateSystemError: // 18. Lỗi hệ thống
                SetEyeColor(255, 0, 0, kEyeModeStrobe);
                OrganicMoveHead(90, 30, 200);
                PlayR2D2Chirp("GLITCH_NOISE");
                break;

            case kStateLowBattery: // 19. Sắp hết pin
                SetEyeColor(255, 50, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 20, 1000);
                PlayR2D2Chirp("LOW_POWER_BEEP");
                break;

            case kStateCharging: // 20. Đang sạc pin
                SetEyeColor(0, 255, 0, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 500);
                PlayR2D2Chirp("CHARGING_HUM");
                break;

            case kStateBooting: // 21. Khởi động
                SetEyeColor(255, 255, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 500);
                PlayR2D2Chirp("BOOT_POWER_UP");
                break;

            case kStateSleeping: // 22. Ngủ
                SetEyeColor(0, 0, 0, kEyeModeOff);
                OrganicMoveHead(90, 20, 1000);
                break;

            case kStateIdle: // 23. Chế độ IDLE
            default:
                SetEyeColor(0, 200, 255, kEyeModeBreathing);
                OrganicMoveHead(90, 90, 400);
                break;
        }
    }

private:
    void TriggerSurpriseReaction() {
        SetRobotState(kStateSurprised);
        vTaskDelay(pdMS_TO_TICKS(1200));
        SetRobotState(kStateIdle);
    }

    void StartTofTask() {
        xTaskCreate([](void* arg) {
            auto board = static_cast<TRaxBoard*>(arg);
            uint8_t read_buf[2];

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(100));

                if (board->vl53l0x_dev_ == nullptr) continue;

                uint8_t reg = 0x14;
                esp_err_t err = i2c_master_transmit_receive(board->vl53l0x_dev_, &reg, 1, read_buf, 2, 50);
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
        }, "tof_sensor_task", 4096, this, 5, &tof_task_handle_);
    }

    void StartEyeLedBreathingTask() {
        xTaskCreate([](void* arg) {
            auto board = static_cast<TRaxBoard*>(arg);
            float step = 0.0f;

            while (true) {
                if (board->eye_led_strip_ == nullptr) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    continue;
                }

                if (board->current_led_mode_ == kEyeModeBreathing) {
                    step += 0.05f;
                    if (step >= 2.0f * M_PI) step = 0.0f;

                    float factor = 0.1f + 0.9f * (0.5f * (sinf(step) + 1.0f));

                    uint8_t r = static_cast<uint8_t>(board->target_r_ * factor);
                    uint8_t g = static_cast<uint8_t>(board->target_g_ * factor);
                    uint8_t b = static_cast<uint8_t>(board->target_b_ * factor);

                    led_strip_set_pixel(board->eye_led_strip_, 0, r, g, b);
                    led_strip_refresh(board->eye_led_strip_);
                    vTaskDelay(pdMS_TO_TICKS(30));
                } 
                else if (board->current_led_mode_ == kEyeModeStrobe) {
                    led_strip_set_pixel(board->eye_led_strip_, 0, board->target_r_, board->target_g_, board->target_b_);
                    led_strip_refresh(board->eye_led_strip_);
                    vTaskDelay(pdMS_TO_TICKS(60));

                    led_strip_clear(board->eye_led_strip_);
                    vTaskDelay(pdMS_TO_TICKS(60));
                }
                else if (board->current_led_mode_ == kEyeModeOff) {
                    led_strip_clear(board->eye_led_strip_);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                else {
                    led_strip_set_pixel(board->eye_led_strip_, 0, board->target_r_, board->target_g_, board->target_b_);
                    led_strip_refresh(board->eye_led_strip_);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }, "eye_led_task", 3072, this, 3, &led_task_handle_);
    }

    // Autonomous IDLE Task with Sine/Cosine Micro-Jitter Breathing Motion
    void StartIdleSequenceTask() {
        xTaskCreate([](void* arg) {
            auto board = static_cast<TRaxBoard*>(arg);
            float idle_time = 0.0f;

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(50));

                if (!board->is_idle_active_) continue;

                idle_time += 0.05f;

                // Micro-jitter breathing equation: theta_idle(t) = theta_base + A1*sin(w1*t) + A2*cos(w2*t)
                float pan_jitter = 1.2f * sinf(0.4f * idle_time) + 0.6f * cosf(0.2f * idle_time);
                float tilt_jitter = 0.8f * sinf(0.3f * idle_time) + 0.4f * cosf(0.5f * idle_time);

                board->SetRawServoAngle(90.0f + pan_jitter, 90.0f + tilt_jitter);

                // Occasional random behavior every 6 to 14 seconds
                if (((int)(idle_time * 20.0f) % 180) == 0) {
                    uint32_t rand_seq = esp_random() % 5;
                    switch (rand_seq) {
                        case 0:
                            board->OrganicMoveHead(125, 100, 600);
                            board->PlayR2D2Chirp("IDLE_LOOK_LEFT");
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            board->OrganicMoveHead(90, 90, 500);
                            break;
                        case 1:
                            board->OrganicMoveHead(55, 100, 600);
                            board->PlayR2D2Chirp("IDLE_LOOK_RIGHT");
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            board->OrganicMoveHead(90, 90, 500);
                            break;
                        case 2:
                            board->OrganicMoveHead(90, 125, 500);
                            board->PlayR2D2Chirp("IDLE_LOOK_UP");
                            vTaskDelay(pdMS_TO_TICKS(800));
                            board->OrganicMoveHead(90, 90, 400);
                            break;
                        case 3:
                            board->SmoothDriveMotors(0.8f, -0.8f, 150);
                            board->SmoothDriveMotors(-0.8f, 0.8f, 150);
                            board->StopMotors();
                            break;
                        case 4:
                            board->SetEyeColor(0, 200, 255, kEyeModeOff);
                            vTaskDelay(pdMS_TO_TICKS(80));
                            board->SetEyeColor(0, 200, 255, kEyeModeBreathing);
                            break;
                    }
                }
            }
        }, "idle_sequence_task", 3072, this, 2, &board->idle_task_handle_);
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

        mcp_server.AddTool("self.trax.set_state", 
            "Đặt 1 trong 23 trạng thái cảm xúc cho Robot T-Rax (1..23)", 
            PropertyList({
                Property("state_id", kPropertyTypeInteger, 1, 23)
            }), 
            [this](const PropertyList& properties) -> ReturnValue {
                int state_id = properties["state_id"].value<int>();
                SetRobotState(static_cast<TRaxState>(state_id));
                return true;
            }
        );
    }

public:
    TRaxBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeWs2812Led();
        InitializeGpioOutputs();
        InitializeServos();
        InitializeButtons();
        InitializeTools();
        StartTofTask();
        StartEyeLedBreathingTask();
        StartIdleSequenceTask();

        SetRobotState(kStateBooting);
        vTaskDelay(pdMS_TO_TICKS(1000));
        SetRobotState(kStateIdle);
    }

    virtual Led* GetLed() override {
        return nullptr;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return &display_;
    }
};

DECLARE_BOARD(TRaxBoard);

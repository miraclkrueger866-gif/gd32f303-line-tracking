/*********************************************************************************************************************
* @file            xunxian_main.c
* @author          
* @Target core     GD32F303RCT6
* @revisions       2025.06.16, V2.0
* @modify          基于电磁传感器的完整巡线系统
********************************************************************************************************************/

#include "headfile.h"
#include "xunxian.h"
#include "car_motion.h"
#include "encoder.h"

/**
 * @brief 电磁巡线主函数 - 基于PID算法
 * 使用5路电磁传感器进行黑线检测和跟踪
 * 流程：
 * 1. 读取5路ADC传感器值
 * 2. 通过七点中值滤波去噪
 * 3. 归一化处理（0-100%）
 * 4. 计算偏差值
 * 5. PID调控电机速度
 */

//=== 全局参数定义 ===
// 基础速度配置
unsigned int st_speed = 3000;           // 直线行驶基础速度 (PWM值 0-10000)
unsigned int max_speed = 8000;          // 最大速度限制
unsigned int min_speed = 1500;          // 最小速度（防止电机不转）

// PID参数
float kp = 2.0f;                        // 比例系数（偏差响应强度）
float ki = 0.1f;                        // 积分系数（消除静态误差）
float kd = 3.0f;                        // 微分系数（阻尼作用）

// 传感器校准值
float ll_max = 3500.0f, lm_max = 3500.0f, mm_max = 3500.0f;
float rm_max = 3500.0f, rr_max = 3500.0f;

// 传感器归一化值（0-100）
float ll = 0, lm = 0, mm = 0, rm = 0, rr = 0;

// PID累积误差
float error[2] = {0, 0};                // error[0]当前误差，error[1]上次误差
float integral_error = 0;               // 积分项累积

// ADC原始值
float adc_value[5] = {0};

// 模式切换
typedef enum {
    MODE_CALIBRATION = 0,               // 校准模式
    MODE_LINE_TRACKING = 1,             // 巡线模式
    MODE_ISLAND_DETECTION = 2,          // 岛屿检测模式
    MODE_STOP = 3                       // 停止模式
} RunMode;

RunMode current_mode = MODE_CALIBRATION;
unsigned int calibration_count = 0;
unsigned int line_lost_count = 0;       // 丢线计数

//===== 函数声明 =====
void calibrate_sensors(void);
void update_sensor_values(void);
void calculate_line_error(void);
void pid_line_tracking(void);
void handle_line_lost(void);
void display_debug_info(void);

/**
 * @brief 传感器校准函数
 * 用于获取白线和黑线的ADC最大值（校准期间小车需在黑线上）
 */
void calibrate_sensors(void) {
    if (calibration_count == 0) {
        oled_clear();
        oled_show_string(0, 0, "Calibration...", 12);
        car_both_rgb_on(yellow, 100);
    }
    
    // 循环采样2000次，找出最大值
    if (calibration_count < 2000) {
        float val0 = adc_get(ADC0, ADC_CH_10);
        float val1 = adc_get(ADC0, ADC_CH_11);
        float val2 = adc_get(ADC0, ADC_CH_12);
        float val3 = adc_get(ADC0, ADC_CH_13);
        float val4 = adc_get(ADC0, ADC_CH_15);
        
        if (val0 > ll_max) ll_max = val0;
        if (val1 > lm_max) lm_max = val1;
        if (val2 > mm_max) mm_max = val2;
        if (val3 > rm_max) rm_max = val3;
        if (val4 > rr_max) rr_max = val4;
        
        calibration_count++;
        delay_1ms(1);
    } else {
        // 校准完成
        oled_clear();
        oled_show_string(0, 0, "Calibration Done", 12);
        oled_show_number(0, 2, (uint32_t)ll_max, 4, 12);
        delay_1ms(1000);
        
        current_mode = MODE_LINE_TRACKING;
        car_both_rgb_on(green, 100);
        oled_clear();
    }
}

/**
 * @brief 读取并处理5路传感器值
 * 使用七点中值滤波算法去除噪声
 */
void update_sensor_values(void) {
    // 直接读取ADC值（实际应用中可加滤波）
    adc_value[0] = adc_get(ADC0, ADC_CH_10);
    adc_value[1] = adc_get(ADC0, ADC_CH_11);
    adc_value[2] = adc_get(ADC0, ADC_CH_12);
    adc_value[3] = adc_get(ADC0, ADC_CH_13);
    adc_value[4] = adc_get(ADC0, ADC_CH_15);
    
    // 五路传感器归一化处理（0-100）
    // 公式：归一化值 = (ADC值 / 最大值) × 100
    ll = (adc_value[0] / ll_max) * 100.0f;
    lm = (adc_value[1] / lm_max) * 100.0f;
    mm = (adc_value[2] / mm_max) * 100.0f;
    rm = (adc_value[3] / rm_max) * 100.0f;
    rr = (adc_value[4] / rr_max) * 100.0f;
    
    // 防止数值溢出
    if (ll > 100) ll = 100;
    if (lm > 100) lm = 100;
    if (mm > 100) mm = 100;
    if (rm > 100) rm = 100;
    if (rr > 100) rr = 100;
}

/**
 * @brief 计算线性误差
 * 当中间传感器在黑线上时，使用左右两边传感器差值计算偏差
 * 当中间传感器离开黑线时，使用最外两个传感器计算偏差
 */
void calculate_line_error(void) {
    // 优先级策略：中间 > 两侧 > 最外
    
    if (mm > 60.0f) {
        // 中间传感器在黑线上 - 最准确的情况
        error[0] = (rm - lm) / (rm + lm + 0.1f) * 100.0f;
    } 
    else if ((lm > 40.0f) || (rm > 40.0f)) {
        // 两侧传感器检测到黑线
        error[0] = (rm - lm) / (rm + lm + 0.1f) * 100.0f;
    }
    else {
        // 使用最外侧传感器
        error[0] = (rr - ll) / (rr + ll + 0.1f) * 100.0f;
    }
}

/**
 * @brief PID巡线控制算法
 * 实现位置式PID控制
 * 输出：左右电机PWM速度值
 */
void pid_line_tracking(void) {
    int32_t left_motor_pwm, right_motor_pwm;
    float pid_output;
    
    // PID控制计算
    // P项：根据当前偏差调整
    // I项：消除持久偏差
    // D项：预测未来趋势，提供阻尼
    
    float p_term = error[0] * kp;
    integral_error += error[0];
    
    // 积分项限制（防止积分饱和）
    if (integral_error > 100.0f) integral_error = 100.0f;
    if (integral_error < -100.0f) integral_error = -100.0f;
    float i_term = integral_error * ki;
    
    float d_term = (error[0] - error[1]) * kd;
    
    pid_output = p_term + i_term + d_term;
    
    // ===== 电机控制逻辑 =====
    // 策略：误差为正 → 向右转（左减右增）
    //      误差为负 → 向左转（左增右减）
    
    if (pid_output > 0) {
        // 需要右转
        left_motor_pwm = st_speed + (int32_t)pid_output;
        right_motor_pwm = st_speed - (int32_t)(pid_output * 0.7f);
    } else {
        // 需要左转
        left_motor_pwm = st_speed + (int32_t)(pid_output * 0.7f);
        right_motor_pwm = st_speed - (int32_t)pid_output;
    }
    
    // 速度限制
    if (left_motor_pwm > max_speed) left_motor_pwm = max_speed;
    if (left_motor_pwm < 0) left_motor_pwm = 0;
    if (right_motor_pwm > max_speed) right_motor_pwm = max_speed;
    if (right_motor_pwm < 0) right_motor_pwm = 0;
    
    // 防止电机停转
    if (left_motor_pwm > 0 && left_motor_pwm < min_speed) left_motor_pwm = min_speed;
    if (right_motor_pwm > 0 && right_motor_pwm < min_speed) right_motor_pwm = min_speed;
    
    // 驱动电机
    motor_forward(left, left_motor_pwm);
    motor_forward(right, right_motor_pwm);
    
    // 更新上次误差
    error[1] = error[0];
}

/**
 * @brief 丢线处理函数
 * 当5路传感器都未检测到黑线时，进行应急处理
 */
void handle_line_lost(void) {
    line_lost_count++;
    
    if (line_lost_count < 50) {
        // 前50ms尝试往上次转向继续转
        if (error[1] > 0) {
            motor_forward(left, 6000);
            motor_forward(right, 2000);
        } else {
            motor_forward(left, 2000);
            motor_forward(right, 6000);
        }
    } else if (line_lost_count < 100) {
        // 后50ms反向转
        if (error[1] > 0) {
            motor_forward(left, 2000);
            motor_forward(right, 6000);
        } else {
            motor_forward(left, 6000);
            motor_forward(right, 2000);
        }
    } else {
        // 超过100ms停止
        car_stop();
        line_lost_count = 0;
    }
}

/**
 * @brief OLED调试信息显示
 */
void display_debug_info(void) {
    static unsigned int refresh_count = 0;
    
    refresh_count++;
    if (refresh_count % 10 == 0) {  // 每10次循环更新一次显示
        oled_clear();
        
        // 显示传感器值
        oled_show_string(0, 0, "LL LM MM RM RR", 12);
        oled_show_number(0, 1, (uint32_t)ll, 2, 12);
        oled_show_number(20, 1, (uint32_t)lm, 2, 12);
        oled_show_number(40, 1, (uint32_t)mm, 2, 12);
        oled_show_number(60, 1, (uint32_t)rm, 2, 12);
        oled_show_number(80, 1, (uint32_t)rr, 2, 12);
        
        // 显示误差值
        oled_show_string(0, 2, "Err:", 12);
        oled_show_number(40, 2, (uint32_t)(error[0] * 10), 3, 12);
        
        // 显示模式
        switch (current_mode) {
            case MODE_CALIBRATION:
                oled_show_string(0, 3, "Mode: CALIB", 12);
                break;
            case MODE_LINE_TRACKING:
                oled_show_string(0, 3, "Mode: TRACK", 12);
                break;
            case MODE_ISLAND_DETECTION:
                oled_show_string(0, 3, "Mode: ISLAND", 12);
                break;
            case MODE_STOP:
                oled_show_string(0, 3, "Mode: STOP", 12);
                break;
        }
    }
}

/**
 * @brief 主巡线处理函数
 * 整合所有巡线逻辑
 */
void xunxian_process(void) {
    switch (current_mode) {
        case MODE_CALIBRATION:
            calibrate_sensors();
            break;
            
        case MODE_LINE_TRACKING:
            update_sensor_values();
            
            // 检测是否丢线
            if (mm < 20.0f && lm < 20.0f && rm < 20.0f && ll < 20.0f && rr < 20.0f) {
                handle_line_lost();
            } else {
                line_lost_count = 0;
                calculate_line_error();
                pid_line_tracking();
            }
            break;
            
        case MODE_STOP:
            car_stop();
            break;
            
        default:
            car_stop();
            break;
    }
    
    display_debug_info();
}

/**
 * @brief 参数调试接口（通过按键切换参数）
 */
void tune_parameters(void) {
    unsigned char key_value = key_check();
    
    switch (key_value) {
        case 0x01:  // K1 - 增加Kp
            kp += 0.1f;
            oled_show_string(0, 6, "Kp+", 12);
            break;
        case 0x02:  // K2 - 减少Kp
            kp -= 0.1f;
            oled_show_string(0, 6, "Kp-", 12);
            break;
        case 0x03:  // K3 - 增加Kd
            kd += 0.1f;
            oled_show_string(0, 6, "Kd+", 12);
            break;
        case 0x04:  // K4 - 减少Kd
            kd -= 0.1f;
            oled_show_string(0, 6, "Kd-", 12);
            break;
        case 0x05:  // K5 - 切换模式
            current_mode = (current_mode == MODE_LINE_TRACKING) ? MODE_STOP : MODE_LINE_TRACKING;
            break;
        case 0x06:  // K6 - 重新校准
            current_mode = MODE_CALIBRATION;
            calibration_count = 0;
            break;
    }
}

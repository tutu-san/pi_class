#include "pi_ctrl.hpp"
#include "main.h"


//単位は(rad/s)
float pid_class::pi_calc_rad(float current_rad){
    float result_p = 0, result_i = 0;
    float calc_p = 0;
    
    //計算
    calc_p = pid_class::current_target_rad - current_rad;
    pid_class::integral += (calc_p + pid_class::before_p) / 2.0f * pid_class::DELTA_T;

    //ゲインとかけてみる
    result_p = pid_class::gain_p * calc_p;
    result_i = pid_class::gain_i * pid_class::integral;

    //次回計算用に今回のエラー値を保存
    pid_class::before_p = calc_p;

    return result_p + result_i;
}

//エンコーダーちゃんからのデータはここに入れて、(rad/s)にする
float pid_class::re_convert_rad(float current_data){
    float rad_per_sec = 0;

    rad_per_sec = ((current_data * pid_class::pi) / 1024.0f) / pid_class::DELTA_T;
    
    return rad_per_sec;
}

//ゲインを変えたいときに
void pid_class::gain_setup(float new_p_gain, float new_i_gain, float new_d_gain){
    pid_class::gain_p = new_p_gain;
    pid_class::gain_i = new_i_gain;
    pid_class::gain_d = new_d_gain;

    return;
}

//delta_t(ms)を変えたいときに
void pid_class::delta_t_setup(float new_delta_t){
    pid_class::DELTA_T = new_delta_t;

    return;
}

//目標速度設定を変えたいときに
void pid_class::terget_setup(float new_terget_rad){
    pid_class::max_terget_rad = new_terget_rad;

    return;
}

//現在の目標速度設定(コントローラーの入力をここで目標速度に変換する)
void pid_class::update_target_spd(float spd_rate){
    pid_class::current_target_rad = pid_class::max_terget_rad * spd_rate; //spd_rateは、コントローラーから得た入力を計算して、最高速度との割合で出す。
}

//エンコーダーの入力から、PWMの値を求める(この前に、delayを入れておくこととする)
float pid_class::motor_calc(float current_data){
    float rad_per_sec = 0.0f, result_motor_pwm = 0.0f;

    rad_per_sec = pid_class::re_convert_rad(current_data);

    result_motor_pwm = pid_class::pi_calc_rad(rad_per_sec);

    if(result_motor_pwm < 0){
        result_motor_pwm = 0;
    }else if(result_motor_pwm > 65535){
        result_motor_pwm = 65535;
    }

    return result_motor_pwm;
}

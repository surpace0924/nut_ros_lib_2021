/**
 * @file PID.h
 * @brief PIDの計算
**/
#pragma once

#include "./../nut_generic.h"
#include "./feedback_controller.h"

namespace nut_ros
{
/**
 * @brief PIDの計算
**/
template <typename T>
class PID : FeedbackController
{
public:
    /**
     * @brief モードリスト
     */
    enum class Mode
    {
        pPID = 0, /**< 位置型PID */
        sPID,     /**< 速度型PID */
        PI_D,     /**< 微分先行型PID */
        I_PD      /**< 比例微分先行型PID */
    };

    /**
     * @brief ゲイン構造体
     */
    struct gain_t
    {
        T Kp; /**< 比例ゲイン */
        T Ki; /**< 積分ゲイン */
        T Kd; /**< 微分ゲイン */
    };

    /**
     * @brief パラメータ構造体
     */
    struct param_t
    {
        Mode mode;            /**< PIDモード */
        gain_t gain;          /**< PIDゲイン */
        bool need_saturation; /**< 出力制限を行うか */
        T output_min;         /**< 出力制限時の最小値 */
        T output_max;         /**< 出力制限時の最大値 */
    };

    /**
     * @brief コンストラクタ
     */
    PID() = default;

    /**
     * @brief コンストラクタ PIDゲインで初期化
     * @param kp: 比例ゲイン
     * @param ki: 微分ゲイン
     * @param kd: 積分ゲイン
     */
    PID(T kp, T ki, T kd)
    {
        _param.gain.Kp = kp;
        _param.gain.Ki = ki;
        _param.gain.Kd = kd;
    }

    /**
     * @brief コンストラクタ パラメータ構造体で初期化
     * @param param: パラメータ構造体
     */
    PID(param_t param) : _param(param) {}

    /**
     * @brief リセット
     */
    void reset();

    /**
     * @brief パラメータの設定
     * @param param: パラメータ構造体
     */
    inline void setParam(const param_t param) { _param = param; }

    /**
     * @brief ゲインの設定
     * @param gain: ゲイン構造体
     */
    inline void setGain(const gain_t gain) { _param.gain = gain; }

    /**
     * @brief PIDモードの設定
     * @param mode: PIDモードenum
     */
    inline void setMode(const Mode mode) { _param.mode = mode; }

    /**
     * @brief 出力の最小，最大値の設定
     * @param min_v: 最小値
     * @param min_v: 最大値
     */
    inline void setSaturation(T min_v, T max_v);

    /**
     * @brief 値の更新
     * @param target: 目標値
     * @param now_val: 現在値
     * @param dt: 前回この関数をコールしてからの経過時間
     */
    inline void update(T target, T now_val, T dt);

    /**
     * @brief 制御量（PIDの計算結果）の取得
     * @return 制御量（PIDの計算結果）
     * @attention update()を呼び出さないと値は更新されない
     */
    inline T getControlValue() { return _output; };

private:
    param_t _param;
    inline T calculate_pPID(T target, T now_val, T dt);
    inline T calculate_sPID(T target, T now_val, T dt);
    inline T calculate_PI_D(T target, T now_val, T dt);
    inline T calculate_I_PD(T target, T now_val, T dt);

    // リセットするやつ
    std::array<T, 3> _diff; // 0: 現在, 1: 過去, 2: 大過去
    T _prev_val, _prev_target;
    T _integral;
    T _output;
};

template <typename T>
void PID<T>::reset()
{
    _diff.fill(0.0);
    _prev_val = _prev_target = 0.0;
    _integral = 0.0;
    _output = 0.0;
}

template <typename T>
inline void PID<T>::setSaturation(T min_v, T max_v)
{
    _param.need_saturation = true;
    _param.output_min = min_v;
    _param.output_max = max_v;
}

template <typename T>
inline void PID<T>::update(T target, T now_val, T dt)
{
    _diff[0] = target - now_val;                     // 最新の偏差
    _integral += (_diff[0] + _diff[1]) * (dt / 2.0); // 積分
    switch (_param.mode)
    {
    case Mode::pPID:
        _output = calculate_pPID(target, now_val, dt);
        break;
    case Mode::sPID:
        _output = calculate_sPID(target, now_val, dt);
        break;
    case Mode::PI_D:
        _output = calculate_PI_D(target, now_val, dt);
        break;
    case Mode::I_PD:
        _output = calculate_I_PD(target, now_val, dt);
        break;
    }

    // 次回ループのために今回の値を前回の値にする
    _diff[2] = _diff[1];
    _diff[1] = _diff[0];
    _prev_target = target;
    _prev_val = now_val;

    // ガード処理
    if (_param.need_saturation)
        _output = (_param.need_saturation) ? Generic::guard<double>(_output, _param.output_min, _param.output_max) : _output;
}

template <typename T>
inline T PID<T>::calculate_pPID(T target, T now_val, T dt)
{
    T p = _param.gain.Kp * _diff[0];
    T i = _param.gain.Ki * _integral;
    T d = _param.gain.Kd * ((_diff[0] - _diff[1]) / dt);
    return p + i + d;
}

// 速度型PID
template <typename T>
inline T PID<T>::calculate_sPID(T target, T now_val, T dt)
{
    T p = _param.gain.Kp * _diff[0] - _diff[1];
    T i = _param.gain.Ki * _diff[0] * dt;
    T d = _param.gain.Kd * (_diff[0] - 2 * _diff[1] + _diff[2]) / dt;
    return _prev_val + p + i + d;
}

// 微分先行型PID
template <typename T>
inline T PID<T>::calculate_PI_D(T target, T now_val, T dt)
{
    T p =  _param.gain.Kp * _diff[0];
    T i =  _param.gain.Ki * _integral;
    T d = -_param.gain.Kd * ((now_val - _prev_val) / dt);
    return p + i + d;
}

// 比例微分先行型PID
template <typename T>
inline T PID<T>::calculate_I_PD(T target, T now_val, T dt)
{
    T p = -_param.gain.Kp * now_val;
    T i =  _param.gain.Ki * _integral;
    T d = -_param.gain.Kd * ((now_val - _prev_val) / dt);
    return p + i + d;
}
}

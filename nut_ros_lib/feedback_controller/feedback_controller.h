/**
 * @file feedback_controller.h
 * @brief フィードバックコントローラの基底クラス
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros
{
/**
 * @brief フィードバックコントローラの基底クラス
**/
class FeedbackController
{
public:
    /**
     * @brief リセット
     */
    void reset();

    /**
     * @brief パラメータの設定
     */
    template <class... Args>
    void setParam(Args... args);

    /**
     * @brief ゲインの設定
     */
    template <class... Args>
    void setGain(Args... args);

    /**
     * @brief モードの設定
     */
    template <class... Args>
    void setMode(Args... args);

    /**
     * @brief 出力の最小，最大値の設定
     */
    template <class... Args>
    void setSaturation(Args... args);

    /**
     * @brief 値の更新
     */
    void update(double target, double now_val, double dt);

    /**
     * @brief 制御量（計算結果）の取得
     */
    template <typename T>
    T getControlValue();
};
}

/**
 * @file angles.h
 * @brief 角度計算ライブラリ
 * @author Ryoga Sato
 * @date 2020/12/01
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros
{
    /**
     * @brief 角度計算クラス
    **/
    class Angles
    {
    public:
        /**
         * @brief [deg]から[rad]へ
         * @param degrees: 入力角度[deg]
         * @return 出力角度[rad]
         *
        **/
        static inline double radians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }

        /**
         * @brief [rad]から[deg]へ
         * @param radians: 入力角度[rad]
         * @return 出力角度[deg]
        **/
        static inline double degrees(double radians)
        {
            return radians * 180.0 / M_PI;
        }

        /**
         * @brief 角度を0～2*piに正規化
         * @param angle: 入力角度[rad]
         * @return 0～2*piに正規化された角度[rad]
        **/
        static inline double normalizePositive(double angle)
        {
            const double result = fmod(angle, 2.0 * M_PI);
            if (result < 0)
                return result + 2.0 * M_PI;
            return result;
        }

        /**
         * @brief 角度を-piから+piに正規化する
         * @param angle: 入力角度[rad]
         * @return -pi～+piに正規化された角度[rad]
        **/
        static inline double normalize(double angle)
        {
            const double result = fmod(angle + M_PI, 2.0 * M_PI);
            if (result <= 0.0)
                return result + M_PI;
            return result - M_PI;
        }

        /**
         * @brief 2つの角度の最短角度を返す 範囲は-pi <= result <= pi
         * @param from: 始点角度[rad]
         * @param to: 終点角度[rad]
         * @return 2つの角度の最短角度[rad]
        **/
        static inline double getShortestAngle(double from, double to)
        {
            return normalize(to - from);
        }

        /**
         * @brief 単位円に沿って逆方向への角度を返す
         * @param angle: 入力角度[rad]
         * @return 単位円に沿って逆方向への角度[rad]
        **/
        static inline double complement(double angle)
        {
            //check input conditions
            if (angle > 2 * M_PI || angle < -2.0 * M_PI)
                angle = fmod(angle, 2.0 * M_PI);
            if (angle < 0)
                return (2 * M_PI + angle);
            else if (angle > 0)
                return (-2 * M_PI + angle);

            return (2 * M_PI);
        }

    private:
    };
} // namespace nut_ros

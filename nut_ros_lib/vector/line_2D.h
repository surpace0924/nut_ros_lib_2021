/**
 * @file line_2D.h
 * @brief 直線を扱う
 * @author Ryoga Sato
 * @date 2020/12/12
**/
#pragma once

#include "./../nut_generic.h"
#include "./pose_2D.h"

namespace nut_ros {
/**
 * @brief 直線を扱う
**/
template <typename T>
class Line2D
{
private:
public:
    Pose2D<T> start;
    Pose2D<T> end;
    constexpr static T eps = 1e-10;  // 許容誤差

    /**
     * @brief コンストラクタ
     */
    Line2D() = default;

    /**
     * @brief コンストラクタ 直交座標で初期化
     * @param x1: 始点のx座標
     * @param y1: 始点のy座標
     * @param theta1: 始点のtheta座標
     * @param x2: 終点のx座標
     * @param y2: 終点のy座標
     * @param theta2: 終点のtheta座標
     */
    Line2D(T x1, T y1, T theta1, T x2, T y2, T theta2) {
        set(x1, y1, theta1, x2, y2, theta2);
        start.x = x1;
        start.y = y1;
        start.theta = theta1;
        end.x = x2;
        end.y = y2;
        end.theta = theta2;
    }

    /**
     * @brief コンストラクタ 直交座標で初期化
     * @param x1: 始点のx座標
     * @param y1: 始点のy座標
     * @param x2: 終点のx座標
     * @param y2: 終点のy座標
     */
    Line2D(T x1, T y1, T x2, T y2) {
        set(x1, y1, x2, y2);
        // start.x = x1;
        // start.y = y1;
        // end.x = x2;
        // end.y = y2;
    }

    /**
     * @brief コンストラクタ Pose2Dでこの直線を設定
     * @param _start: Pose2Dでの始点座標
     * @param _end: Pose2Dでの終点座標
     */
    Line2D(Pose2D<T> _start, Pose2D<T> _end) {
        set(_start, _end);
    }

    /**
     * @brief コンストラクタ Vector2でこの直線を設定
     * @param _start: Vector2での始点座標
     * @param _end: Vector2での終点座標
     */
    Line2D(Vector2<T> _start, Vector2<T> _end) {
        set(_start, _end);
    }

    /**
     * @brief 直交座標形式でこの直線を設定
     * @param x1: 始点のx座標
     * @param y1: 始点のy座標
     * @param theta1: 始点のtheta座標
     * @param x2: 終点のx座標
     * @param y2: 終点のy座標
     * @param theta2: 終点のtheta座標
     */
    void set(T x1, T y1, T theta1, T x2, T y2, T theta2) {
        start.x = x1;
        start.y = y1;
        start.theta = theta1;
        end.x = x2;
        end.y = y2;
        end.theta = theta2;
    }

    /**
     * @brief 直交座標形式でこの直線を設定
     * @param x1: 始点のx座標
     * @param y1: 始点のy座標
     * @param x2: 終点のx座標
     * @param y2: 終点のy座標
     */
    void set(T x1, T y1, T x2, T y2) {
        start.x = x1;
        start.y = y1;
        end.x = x2;
        end.y = y2;
    }

    /**
     * @brief Pose2Dでこの直線を設定
     * @param _start: Pose2Dでの始点座標
     * @param _end: Pose2Dでの終点座標
     */
    void set(Pose2D<T> _start, Pose2D<T> _end) {
        start = _start;
        end = _end;
    }

    /**
     * @brief Vector2でこの直線を設定
     * @param _start: Vector2での始点座標
     * @param _end: Vector2での終点座標
     */
    void set(Vector2<T> _start, Vector2<T> _end) {
        start.x = _start.x;
        start.y = _start.y;
        end.x = _end.x;
        end.y = _end.y;
    }

    /**
     * @brief この直線の長さを返す
     * @return この直線の長さ
     */
    T getLength()
    {
        return Pose2D<T>::getDistance(start, end);
    }

    /**
     * @brief この直線の角度を返す
     * @return この直線の角度
     */
    T getAngle()
    {
        return Pose2D<T>::getAngle(start, end);
    }

    /**
     * @brief この直線上に指定点が存在するかどうか
     * @param p: 指定点
     * @return この直線上に指定点が存在するかどうか
     */
    bool isPointOnLine(Pose2D<T> p)
    {
        return std::abs(Pose2D<T>::getCross(end-start, p-start)) < eps;
    }

    /**
     * @brief この線分上に指定点が存在するかどうか
     * @param p: 指定点
     * @return この線分上に指定点が存在するかどうか
     */
    bool isPointOnLineWithinRange(Pose2D<T> p)
    {
        if(isPointOnLine(p))
        {
            // 存在する
            bool result = true;
            T x_min = std::min(start.x, end.x);
            T x_max = std::max(start.x, end.x);
            T y_min = std::min(start.y, end.y);
            T y_max = std::max(start.y, end.y);

            if (std::abs(x_max - x_min) < eps)
            {
                // 範囲が狭いとき
                if (std::abs(x_max - p.x) > eps)
                    result = false;
            }
            else
            {
                // 範囲が広い
                if (!(x_min < p.x && p.x < x_max))
                    result = false;
            }

            if (std::abs(y_max - y_min) < eps)
            {
                // 範囲が狭いとき
                if (std::abs(y_max - p.y) > eps)
                    result = false;
            }
            else
            {
                // 範囲が広い
                if (!(y_min < p.y && p.y < y_max))
                    result = false;
            }

            return result;
        }
        else
        {
            // 存在しない
            return false;
        }
    }

    /**
     * @brief 2直線の交点を求める
     * @param line1: 直線1
     * @param line2: 直線2
     * @return 2直線が交差するか, 2直線の交点
     */
    static std::tuple<bool, Pose2D<T>> getIntersection(Line2D line1, Line2D line2)
    {
        if (std::abs(Pose2D<T>::getCross(line1.end - line1.start, line2.end - line2.start)) > eps)
        {
            // 交差する
            Pose2D<T> a = line1.end - line1.start;
            Pose2D<T> b = line2.end - line2.start;
            Pose2D<T> intersection = line1.start + a * Pose2D<T>::getCross(b, line2.start-line1.start) / Pose2D<T>::getCross(b, a);
            return std::forward_as_tuple(true, intersection);
        }
        else
        {
            // 交差しない
            return std::forward_as_tuple(false, Pose2D<T>(0, 0, 0));
        }
    }

    /**
     * @brief 2線分の交点を求める
     * @param line1: 線分1
     * @param line2: 線分2
     * @return 2線分が交差するか, 2線分の交点
     */
    static std::tuple<bool, Pose2D<T>> getIntersectionWithinRange(Line2D line1, Line2D line2)
    {
        bool is_intersected;    // 交差判定
        Pose2D<T> intersection; // 交点
        std::tie(is_intersected, intersection) = getIntersection(line1, line2);

        if (is_intersected)
        {
            // 交差する
            is_intersected = (line1.isPointOnLineWithinRange(intersection) && line2.isPointOnLineWithinRange(intersection));
            return std::forward_as_tuple(is_intersected, intersection);
        }
        else
        {
            // 交差しない
            return std::forward_as_tuple(false, intersection);
        }
    }

    /**
     * @brief 点と直線との距離
     * @param pose: 点
     * @param line: 直線
     * @return 点と直線との距離
     */
    static T getDistanceFromPointToLine(Pose2D<T> pose, Line2D line)
    {
        // ax + by + c = 0 の形にする
        T a, b, c;
        if (std::abs(line.start.x - line.end.x) > eps)
        {
            a =  (line.end.y - line.start.y) / (line.end.x - line.start.x);
            b = -1;
            c = -(line.end.y - line.start.y) / (line.end.x - line.start.x) * line.start.x + line.start.y;
        }
        else
        {
            a = 1;
            b = 0;
            c = -line.start.x;
        }
        std::cout << "a:" << a <<  " b:" << b <<  " c:" << c << std::endl;

        T distance = std::abs(a * pose.x + b * pose.y + c) / std::sqrt(a * a + b * b);
        return distance;
    }

    /**
     * @brief 点と線分との距離
     * @param pose: 点
     * @param line: 線分
     * @return 点と線分との距離
     */
    static T getDistanceFromPointToLineWithinRange(Pose2D<T> pose, Line2D line)
    {
        // ax + by + c = 0 の形にする
        T a, b, c;
        if (std::abs(line.start.x - line.end.x) > eps)
        {
            a =  (line.end.y - line.start.y) / (line.end.x - line.start.x);
            b = -1;
            c = -(line.end.y - line.start.y) / (line.end.x - line.start.x) * line.start.x + line.start.y;
        }
        else
        {
            a = 1;
            b = 0;
            c = -line.start.x;
        }

        // 点から線分に垂線をおろしたときの交点の座標を求める
        T p = pose.x;
        T q = pose.y;
        Pose2D<T> h;
        h.x = (p*(a*a+b*b) - a*(a*p+b*q+c)) / (a*a+b*b);
        h.y = (q*(a*a+b*b) - b*(a*p+b*q+c)) / (a*a+b*b);

        if (line.isPointOnLineWithinRange(h))
        {
            // 垂線の交点が線分上に存在するとき
            // そのまま距離を返す
            return std::abs(a * pose.x + b * pose.y + c) / std::sqrt(a * a + b * b);
        }
        else
        {
            // 垂線の交点が線分上に存在しないとき
            // 線分の境界のうち近い方を返す
            return std::min(Pose2D<T>::getDistance(pose, line.start), Pose2D<T>::getDistance(pose, line.end));
        }
    }
};
}

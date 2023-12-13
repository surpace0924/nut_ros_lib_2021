/**
 * @file pose_2D.h
 * @brief 時間計測を行う
 * @author Ryoga Sato
 * @date 2020/11/27
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros
{
/**
 * @brief 時間計測を行う
 * @details start()を実行したタイミングからの経過秒をgetDuration()で取得できる
**/
class Stopwatch
{
private:
    ros::Time start_time;

public:
    /**
     * @brief コンストラクタ （計測開始）
     */
    Stopwatch()
    {
        start();
    }

    /**
     * @brief 計測開始
     */
    void start()
    {
        start_time = ros::Time::now();
    }

    /**
     * @brief start()を呼び出してからの経過時間[sec]を返す
     * @return start()を呼び出してからの経過時間[sec]
     */
    double getDuration()
    {
        ros::Duration ros_duration = ros::Time::now() - start_time;
        return (double)ros_duration.sec + (double)ros_duration.nsec * std::pow(10, -9);
    }

    /**
     * @brief start()を呼び出してからの経過時間[sec]を表示
     */
    void displayDuration()
    {
        double dt = getDuration();
        ROS_INFO("dt: %lf[sec]\tf: %lf[Hz]", dt, 1 / dt);
    }
};
}

/**
 * @file msg_calculator.h
 * @brief さまざまな図形のgeometry_msgs::Polygon型を簡単に生成できるようにする
 * @author Ryoga Sato
 * @date 2020/09/06
**/
#pragma once

#include "./../nut_generic.h"
#include "./msg_generator.h"

namespace nut_ros {

/**
 * @brief さまざまな図形のgeometry_msgs::Polygon型を簡単に生成できるようにする
**/
class MsgCalculator{
private:
public:

    /**
     * @brief 2点間の距離を取得
     * @param point1: geometry_msgs::Point型
     * @param point2: geometry_msgs::Point型
     * @return 2点間の距離
    **/
    static double getDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
    {
        return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2) + std::pow(point2.z - point1.z, 2));
    }

    /**
     * @brief 2点間の距離を取得
     * @param point1: geometry_msgs::Point32型
     * @param point2: geometry_msgs::Point32型
     * @return 2点間の距離
    **/
    static float getDistance(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2)
    {
        return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2) + std::pow(point2.z - point1.z, 2));
    }
    
    /**
     * @brief 2点間の距離を取得
     * @param vector1: geometry_msgs::Vector3型
     * @param vector2: geometry_msgs::Vector3型
     * @return 2点間の距離
    **/
    static double getDistance(geometry_msgs::Vector3 vector1, geometry_msgs::Vector3 vector2)
    {
        return std::sqrt(std::pow(vector2.x - vector1.x, 2) + std::pow(vector2.y - vector1.y, 2) + std::pow(vector2.z - vector1.z, 2));
    }

    /**
     * @brief 2つの座標間の距離を取得
     * @param pose1: geometry_msgs::Pose型
     * @param pose2: geometry_msgs::Pose型
     * @return 2つの座標間の距離
    **/
    static double getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {
        return getDistance(pose1.position, pose2.position);
    }

    /**
     * @brief 2つの座標間の距離を取得
     * @param pose1: geometry_msgs::Pose型
     * @param pose2: geometry_msgs::Pose型
     * @return 2つの座標間の距離
    **/
    static double getDistance(geometry_msgs::PoseStamped pose_stamped1, geometry_msgs::PoseStamped pose_stamped2)
    {
        return getDistance(pose_stamped1.pose, pose_stamped2.pose);
    }

    /**
     * @brief geometry_msgs::Twistの並進速度を取得
     * @param twist: geometry_msgs::Twist
     * @return geometry_msgs::Twistの並進速度
    **/
    static double getLinearVelocity(geometry_msgs::Twist twist)
    {
        return getDistance(twist.linear, MsgGenerator::toVector3(0.0, 0.0, 0.0));
    }
};
}

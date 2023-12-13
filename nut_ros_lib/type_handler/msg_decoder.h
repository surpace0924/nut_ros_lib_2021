/**
 * @file msg_decoder.h
 * @brief ROSのさまざまなメッセージ型の中身を簡単に取り出せるようにする
 * @author Ryoga Sato
 * @date 2020/09/03
**/
#pragma once

#include "./../nut_generic.h"
#include "./../vector/vector2.h"
#include "./../vector/pose_2D.h"

namespace nut_ros {

/**
 * @brief ROSのさまざまなメッセージ型の中身を簡単に取り出せるようにする
**/
class MsgDecoder{
private:
public:
    ///////////////// vector /////////////////
    /**
     * @brief geometry_msgs::Vector3型のメッセージをstd::vector（x, y, z）として取得する
     * @param vector3: geometry_msgs::Vector3のメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::Vector3 vector3)
    {
        std::vector<double> vector3_vector = {vector3.x, vector3.y, vector3.z};
        return vector3_vector;
    }

    /**
     * @brief geometry_msgs::Point型のメッセージをstd::vector（x, y, z）として取得する
     * @param point: geometry_msgs::Pointのメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::Point point)
    {
        std::vector<double> point_vector = {point.x, point.y, point.z};
        return point_vector;
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::vector（x, y, z）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::Pose pose)
    {
        return getLinearVector(pose.position);
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::vector（x, y, z）として取得する
     * @param pose: geometry_msgs::PoseStampedのメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::PoseStamped pose_stamped)
    {
        return getLinearVector(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::vector（x, y, z）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::Twist twist)
    {
        return getLinearVector(twist.linear);
    }

    /**
     * @brief geometry_msgs::Accel型のメッセージをstd::vector（x, y, z）として取得する
     * @param accel: geometry_msgs::Accelのメッセージ
     * @return std::vector（x, y, z）
    **/
    static std::vector<double> getLinearVector(geometry_msgs::Accel accel)
    {
        return getLinearVector(accel.linear);
    }

    /**
     * @brief geometry_msgs::Quaternion型のメッセージをstd::vector（Roll, Pitch, Yaw）として取得する
     * @param quaternion: geometry_msgs::Quaternionのメッセージ
     * @return std::vector（Roll, Pitch, Yaw）
    **/
    static std::vector<double> getAngularVector(geometry_msgs::Quaternion quaternion)
    {
        double roll, pitch, yaw;
        tf::Quaternion tf_quaternion;
        quaternionMsgToTF(quaternion, tf_quaternion);
        tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        std::vector<double> quaternion_vector = {roll, pitch, yaw};
        return quaternion_vector;
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::vector（Roll, Pitch, Yaw）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::vector（Roll, Pitch, Yaw）
    **/
    static std::vector<double> getAngularVector(geometry_msgs::Pose pose)
    {
        return getAngularVector(pose.orientation);
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::vector（Roll, Pitch, Yaw）として取得する
     * @param pose_stamped: geometry_msgs::PoseStampedのメッセージ
     * @return std::vector（Roll, Pitch, Yaw）
    **/
    static std::vector<double> getAngularVector(geometry_msgs::PoseStamped pose_stamped)
    {
        return getAngularVector(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::vector（Roll, Pitch, Yaw）として取得する
     * @param twist: geometry_msgs::Accelのメッセージ
     * @return std::vector（Roll, Pitch, Yaw）
    **/
    static std::vector<double> getAngularVector(geometry_msgs::Twist twist)
    {
        return getLinearVector(twist.angular);
    }

    /**
     * @brief geometry_msgs::Accel型のメッセージをstd::vector（Roll, Pitch, Yaw）として取得する
     * @param accel: geometry_msgs::Twistのメッセージ
     * @return std::vector（Roll, Pitch, Yaw）
    **/
    static std::vector<double> getAngularVector(geometry_msgs::Accel accel)
    {
        return getLinearVector(accel.angular);
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::vector（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::vector（x, y, Yaw）
    **/
    static std::vector<double> get2DVector(geometry_msgs::Pose pose)
    {
        auto linear_vector  = getLinearVector(pose);
        auto angular_vector = getAngularVector(pose);
        std::vector<double> return_vector = {linear_vector[0], linear_vector[1], angular_vector[2]};
        return return_vector;
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::vector（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::PoseStampedのメッセージ
     * @return std::vector（x, y, Yaw）
    **/
    static std::vector<double> get2DVector(geometry_msgs::PoseStamped pose_stamped)
    {
        return get2DVector(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::vector（x, y, Yaw）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return std::vector（x, y, Yaw）
    **/
    static std::vector<double> get2DVector(geometry_msgs::Twist twist)
    {
        auto linear_vector  = getLinearVector(twist);
        auto angular_vector = getAngularVector(twist);
        std::vector<double> return_vector = {linear_vector[0], linear_vector[1], angular_vector[2]};
        return return_vector;
    }

    ///////////////// nut_ros::Vector2 /////////////////
    /**
     * @brief geometry_msgs::Vector3型のメッセージをnut_ros::Vector2（x, y）として取得する
     * @param vector3: geometry_msgs::Vector3のメッセージ
     * @return nut_ros::Vector2（x, y）
    **/
    static nut_ros::Vector2<double> getLinearVector2(geometry_msgs::Vector3 vector3)
    {
        nut_ros::Vector2<double> vector3_vector2(vector3.x, vector3.y);
        return vector3_vector2;
    }

    /**
     * @brief geometry_msgs::Point型のメッセージをnut_ros::Vector2（x, y）として取得する
     * @param point: geometry_msgs::Pointのメッセージ
     * @return nut_ros::Vector2（x, y）
    **/
    static nut_ros::Vector2<double> getLinearVector2(geometry_msgs::Point point)
    {
        nut_ros::Vector2<double> point_vector2(point.x, point.y);
        return point_vector2;
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをnut_ros::Vector2（x, y）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return nut_ros::Vector2（x, y）
    **/
    static nut_ros::Vector2<double> getLinearVector2(geometry_msgs::Pose pose)
    {
        return getLinearVector2(pose.position);
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをnut_ros::Vector2（x, y）として取得する
     * @param pose: geometry_msgs::PoseStampedのメッセージ
     * @return nut_ros::Vector2（x, y）
    **/
    static nut_ros::Vector2<double> getLinearVector2(geometry_msgs::PoseStamped pose_stamped)
    {
        return getLinearVector2(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをnut_ros::Vector2（x, y）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return nut_ros::Vector2（x, y）
    **/
    static nut_ros::Vector2<double> getLinearVector2(geometry_msgs::Twist twist)
    {
        return getLinearVector2(twist.linear);
    }


    ///////////////// nut_ros::Pose2D /////////////////
    /**
     * @brief geometry_msgs::Pose型のメッセージをnut_ros::Pose2D（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return nut_ros::Pose2D（x, y, Yaw）
    **/
    static nut_ros::Pose2D<double> getPose2D(geometry_msgs::Pose pose)
    {
        nut_ros::Pose2D<double> pose_2d(pose.position.x, pose.position.y, getAngularVector(pose)[2]);
        return pose_2d;
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをnut_ros::Pose2D（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::PoseStampedのメッセージ
     * @return nut_ros::Pose2D（x, y, Yaw）
    **/
    static nut_ros::Pose2D<double> getPose2D(geometry_msgs::PoseStamped pose_stamped)
    {
        return getPose2D(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Accel型のメッセージをnut_ros::Pose2D（x, y, Yaw）として取得する
     * @param accel: geometry_msgs::Accelのメッセージ
     * @return nut_ros::Pose2D（x, y, Yaw）
    **/
    static nut_ros::Pose2D<double> getPose2D(geometry_msgs::Accel accel)
    {
        nut_ros::Pose2D<double> pose_2d(getLinearVector(accel)[0], getLinearVector(accel)[1], getAngularVector(accel)[2]);
        return pose_2d;
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをnut_ros::Pose2D（x, y, Yaw）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return nut_ros::Pose2D（x, y, Yaw）
    **/
    static nut_ros::Pose2D<double> getPose2D(geometry_msgs::Twist twist)
    {
        nut_ros::Pose2D<double> pose_2d(getLinearVector(twist)[0], getLinearVector(twist)[1], getAngularVector(twist)[2]);
        return pose_2d;
    }


    ///////////////// array /////////////////
    /**
     * @brief geometry_msgs::Vector3型のメッセージをstd::array（x, y, z）として取得する
     * @param vector3: geometry_msgs::Vector3のメッセージ
     * @return std::array（x, y, z）
    **/
    static std::array<double, 3> getLinearArray(geometry_msgs::Vector3 vector3)
    {
        std::array<double, 3> vector3_array = {vector3.x, vector3.y, vector3.z};
        return vector3_array;
    }

    /**
     * @brief geometry_msgs::Point型のメッセージをstd::array（x, y, z）として取得する
     * @param point: geometry_msgs::Pointのメッセージ
     * @return std::array（x, y, z）
    **/
    static std::array<double, 3> getLinearArray(geometry_msgs::Point point)
    {
        std::array<double, 3> point_array = {point.x, point.y, point.z};
        return point_array;
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::array（x, y, z）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::array（x, y, z）
    **/
    static std::array<double, 3> getLinearArray(geometry_msgs::Pose pose)
    {
        return getLinearArray(pose.position);
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::array（x, y, z）として取得する
     * @param pose_stamped: geometry_msgs::PoseStampedのメッセージ
     * @return std::array（x, y, z）
    **/
    static std::array<double, 3> getLinearArray(geometry_msgs::PoseStamped pose_stamped)
    {
        return getLinearArray(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::array（x, y, z）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return std::array（x, y, z）
    **/
    static std::array<double, 3> getLinearArray(geometry_msgs::Twist twist)
    {
        return getLinearArray(twist.linear);
    }

    /**
     * @brief geometry_msgs::Quaternion型のメッセージをstd::array（Roll, Pitch, Yaw）として取得する
     * @param quaternion: geometry_msgs::Quaternionのメッセージ
     * @return std::array（Roll, Pitch, Yaw）
    **/
    static std::array<double, 3> getAngularArray(geometry_msgs::Quaternion quaternion)
    {
        double roll, pitch, yaw;
        tf::Quaternion tf_quaternion;
        quaternionMsgToTF(quaternion, tf_quaternion);
        tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
        std::array<double, 3> quaternion_vector = {roll, pitch, yaw};
        return quaternion_vector;
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::array（Roll, Pitch, Yaw）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::array（Roll, Pitch, Yaw）
    **/
    static std::array<double, 3> getAngularArray(geometry_msgs::Pose pose)
    {
        return getAngularArray(pose.orientation);
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::array（Roll, Pitch, Yaw）として取得する
     * @param pose_stamped: geometry_msgs::PoseStampedのメッセージ
     * @return std::array（Roll, Pitch, Yaw）
    **/
    static std::array<double, 3> getAngularArray(geometry_msgs::PoseStamped pose_stamped)
    {
        return getAngularArray(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::array（Roll, Pitch, Yaw）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return std::array（Roll, Pitch, Yaw）
    **/
    static std::array<double, 3> getAngularArray(geometry_msgs::Twist twist)
    {
        return getLinearArray(twist.angular);
    }

    /**
     * @brief geometry_msgs::Pose型のメッセージをstd::array（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::Poseのメッセージ
     * @return std::array（x, y, Yaw）
    **/
    static std::array<double, 3> get2DArray(geometry_msgs::Pose pose)
    {
        auto linear_array  = getLinearArray(pose);
        auto angular_array = getAngularArray(pose);
        std::array<double, 3> return_array = {linear_array[0], linear_array[1], angular_array[2]};
        return return_array;
    }

    /**
     * @brief geometry_msgs::PoseStamped型のメッセージをstd::array（x, y, Yaw）として取得する
     * @param pose: geometry_msgs::PoseStampedのメッセージ
     * @return std::array（x, y, Yaw）
    **/
    static std::array<double, 3> get2DArray(geometry_msgs::PoseStamped pose_stamped)
    {
        return get2DArray(pose_stamped.pose);
    }

    /**
     * @brief geometry_msgs::Twist型のメッセージをstd::array（x, y, Yaw）として取得する
     * @param twist: geometry_msgs::Twistのメッセージ
     * @return std::array（x, y, Yaw）
    **/
    static std::array<double, 3> get2DArray(geometry_msgs::Twist twist)
    {
        auto linear_array  = getLinearArray(twist);
        auto angular_array = getAngularArray(twist);
        std::array<double, 3> return_array = {linear_array[0], linear_array[1], angular_array[2]};
        return return_array;
    }

};
}


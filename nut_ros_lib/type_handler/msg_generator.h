/**
 * @file msg_generator.h
 * @brief ROSのさまざまなメッセージ型を簡単に生成できるようにする
 * @author Ryoga Sato
 * @date 2020/09/03
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros
{

    /**
 * @brief ROSのさまざまなメッセージ型を簡単に生成できるようにする
**/
    class MsgGenerator
    {
    private:
    public:
        /**
     * @brief 引数に応じたstd_msgs::Header型のメッセージを返す
     * @param frame_id: 基準フレームのID
     * @param seq: シーケンス番号（任意）
     * @return std_msgs::Header型のメッセージ
    **/
        static std_msgs::Header toHeader(std::string frame_id, const uint32_t seq = 0)
        {
            std_msgs::Header header;
            header.seq = seq;
            header.stamp = ros::Time::now();
            header.frame_id = frame_id;
            return header;
        }

        /**
     * @brief 引数に応じたstd_msgs::ColorRGBA型のメッセージを返す
     * @param r: 赤色値（0.0 to 1.0）
     * @param g: 緑色値（0.0 to 1.0）
     * @param b: 青色値（0.0 to 1.0）
     * @param a: 透明度（0.0 to 1.0）
     * @return std_msgs::ColorRGBA型のメッセージ
    **/
        static std_msgs::ColorRGBA toColorRGBA(double r, double g, double b, double a)
        {
            std_msgs::ColorRGBA color;
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = a;
            return color;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Point型のメッセージを返す
     * @param x: x[m]
     * @param y: y[m]
     * @param z: z[m]
     * @return geometry_msgs::Point型のメッセージ
    **/
        static geometry_msgs::Point toPoint(double x, double y, double z)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Point32型のメッセージを返す
     * @param x: x[m]
     * @param y: y[m]
     * @param z: z[m]
     * @return geometry_msgs::Point32型のメッセージ
    **/
        static geometry_msgs::Point32 toPoint32(float x, float y, float z)
        {
            geometry_msgs::Point32 point32;
            point32.x = x;
            point32.y = y;
            point32.z = z;
            return point32;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Quaternion型のメッセージを返す
     * @param roll: ロール角度[rad]
     * @param pitch: ピッチ角度[rad]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::Quaternion型のメッセージ
    **/
        static geometry_msgs::Quaternion toQuaternion(double roll, double pitch, double yaw)
        {
            return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Quaternion型のメッセージを返す
     * @param x: x
     * @param y: y
     * @param z: z
     * @param w: w
     * @attention Quaternionのノルムは必ず1となるようにしなければならない
     * @return geometry_msgs::Quaternion型のメッセージ
    **/
        static geometry_msgs::Quaternion toQuaternion(double x, double y, double z, double w)
        {
            geometry_msgs::Quaternion quaternion;
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            quaternion.w = w;
            return quaternion;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Pose型のメッセージを返す
     * @param position: 位置成分
     * @param orientation: 方向成分
     * @return geometry_msgs::Pose型のメッセージ
    **/
        static geometry_msgs::Pose toPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
        {
            geometry_msgs::Pose pose;
            pose.position = position;
            pose.orientation = orientation;
            return pose;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Pose型のメッセージを返す
     * @param x: x[m]
     * @param y: y[m]
     * @param z: z[m]
     * @param roll: ロール角度[rad]
     * @param pitch: ピッチ角度[rad]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::Pose型のメッセージ
    **/
        static geometry_msgs::Pose toPose(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toPose(toPoint(x, y, z), toQuaternion(roll, pitch, yaw));
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Pose型のメッセージを返す
     * @param x: x[m]
     * @param y: y[m]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::Pose型のメッセージ
    **/
        static geometry_msgs::Pose toPose(double x, double y, double yaw)
        {
            return toPose(x, y, 0.0, 0.0, 0.0, yaw);
        }

        /**
     * @brief 引数に応じたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param header: ヘッダー
     * @param pose: 姿勢（位置と方向を持つもの）
     * @return geometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped toPoseStamped(std_msgs::Header header, geometry_msgs::Pose pose)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.pose = pose;
            return pose_stamped;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param header: ヘッダー
     * @param position: 位置成分
     * @param orientation: 方向成分
     * @return geometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped toPoseStamped(std_msgs::Header header, geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
        {
            return toPoseStamped(header, toPose(position, orientation));
        }

        /**
     * @brief 引数に応じたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param header: ヘッダー
     * @param x: x[m]
     * @param y: y[m]
     * @param z: z[m]
     * @param roll: ロール角度[rad]
     * @param pitch: ピッチ角度[rad]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped toPoseStamped(std_msgs::Header header, double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toPoseStamped(header, toPose(x, y, z, roll, pitch, yaw));
        }

        /**
     * @brief 引数に応じたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param frame_id: 基準フレームのID
     * @param x: x[m/s]
     * @param y: y[m/s]
     * @param z: z[m/s]
     * @param roll: ロール角度[rad]
     * @param pitch: ピッチ角度[rad]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped toPoseStamped(std::string frame_id, double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toPoseStamped(toHeader(frame_id), x, y, z, roll, pitch, yaw);
        }

        /**
     * @brief 引数に応じたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param frame_id: 基準フレームのID
     * @param x: x[m]
     * @param y: y[m]
     * @param yaw: ヨー角度[rad]
     * @return geometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped toPoseStamped(std::string frame_id, double x, double y, double yaw)
        {
            return toPoseStamped(frame_id, x, y, 0.0, 0.0, 0.0, yaw);
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Vector3型のメッセージを返す
     * @param x: x[m]
     * @param y: y[m]
     * @param z: z[m]
     * @return geometry_msgs::Vector3型のメッセージ
    **/
        static geometry_msgs::Vector3 toVector3(double x, double y, double z)
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = x;
            vector3.y = y;
            vector3.z = z;
            return vector3;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Twist型のメッセージを返す
     * @param linear: 並進速度[m/s]
     * @param angular: 回転速度[rad/s]
     * @return geometry_msgs::Twist型のメッセージ
    **/
        static geometry_msgs::Twist toTwist(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular)
        {
            geometry_msgs::Twist twist;
            twist.linear = linear;
            twist.angular = angular;
            return twist;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Twist型のメッセージを返す
     * @param x: x速度[m/s]
     * @param y: y速度[m/s]
     * @param z: z速度[m/s]
     * @param roll: ロール角速度[rad/s]
     * @param pitch: ピッチ角速度[rad/s]
     * @param yaw: ヨー角速度[rad/s]
     * @return geometry_msgs::Twist型のメッセージ
    **/
        static geometry_msgs::Twist toTwist(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toTwist(toVector3(x, y, z), toVector3(roll, pitch, yaw));
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Twist型のメッセージを返す
     * @param x: x速度[m/s]
     * @param y: y速度[m/s]
     * @param yaw: ヨー角速度[rad/s]
     * @return geometry_msgs::Twist型のメッセージ
    **/
        static geometry_msgs::Twist toTwist(double x, double y, double yaw)
        {
            return toTwist(x, y, 0.0, 0.0, 0.0, yaw);
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Accel型のメッセージを返す
     * @param linear: 並進速度[m/s]
     * @param angular: 回転速度[rad/s]
     * @return geometry_msgs::Accel型のメッセージ
    **/
        static geometry_msgs::Accel toAccel(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular)
        {
            geometry_msgs::Accel accel;
            accel.linear = linear;
            accel.angular = angular;
            return accel;
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Accel型のメッセージを返す
     * @param x: x速度[m/s]
     * @param y: y速度[m/s]
     * @param z: z速度[m/s]
     * @param roll: ロール角速度[rad/s]
     * @param pitch: ピッチ角速度[rad/s]
     * @param yaw: ヨー角速度[rad/s]
     * @return geometry_msgs::Accel型のメッセージ
    **/
        static geometry_msgs::Accel toAccel(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toAccel(toVector3(x, y, z), toVector3(roll, pitch, yaw));
        }

        /**
     * @brief 引数に応じたgeometry_msgs::Accel型のメッセージを返す
     * @param x: x速度[m/s]
     * @param y: y速度[m/s]
     * @param yaw: ヨー角速度[rad/s]
     * @return geometry_msgs::Accel型のメッセージ
    **/
        static geometry_msgs::Accel toAccel(double x, double y, double yaw)
        {
            return toAccel(x, y, 0.0, 0.0, 0.0, yaw);
        }

        /**
         * @brief 引数に応じたgeometry_msgs::Transform型のメッセージを返す
         * @param translation: 並進成分
         * @param rotation: 回転成分
         * @return geometry_msgs::Transform型のメッセージ
        **/
        static geometry_msgs::Transform toTransform(geometry_msgs::Vector3 translation, geometry_msgs::Quaternion rotation)
        {
            geometry_msgs::Transform transform;
            transform.translation = translation;
            transform.rotation = rotation;
            return transform;
        }

        /**
         * @brief 引数に応じたgeometry_msgs::Transform型のメッセージを返す
         * @param x: x速度[m/s]
         * @param y: y速度[m/s]
         * @param z: z速度[m/s]
         * @param roll: ロール角速度[rad/s]
         * @param pitch: ピッチ角速度[rad/s]
         * @param yaw: ヨー角速度[rad/s]
         * @return geometry_msgs::Transform型のメッセージ
        **/
        static geometry_msgs::Transform toTransform(double x, double y, double z, double roll, double pitch, double yaw)
        {
            return toTransform(toVector3(x, y, z), toQuaternion(roll, pitch, yaw));
        }

        /**
         * @brief 引数に応じたgeometry_msgs::Transform型のメッセージを返す
         * @param x: x速度[m/s]
         * @param y: y速度[m/s]
         * @param yaw: ヨー角速度[rad/s]
         * @return geometry_msgs::Transform型のメッセージ
        **/
        static geometry_msgs::Transform toTransform(double x, double y, double yaw)
        {
            return toTransform(x, y, 0, 0, 0, yaw);
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Point型のメッセージを返す
     * @param point: geometry_msgs::Point
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Point型のメッセージ
    **/
        static geometry_msgs::Point
        rotate(geometry_msgs::Point point, double angle)
        {
            geometry_msgs::Point reutrn_point;
            reutrn_point.x = point.x * std::cos(angle) - point.y * std::sin(angle);
            reutrn_point.y = point.x * std::sin(angle) + point.y * std::cos(angle);
            reutrn_point.z = point.z;
            return reutrn_point;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Point型のメッセージを返す
     * @param point32: geometry_msgs::Point
     * @param angle32: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Point型のメッセージ
    **/
        static geometry_msgs::Point32 rotate(geometry_msgs::Point32 point32, float angle)
        {
            geometry_msgs::Point32 reutrn_point;
            reutrn_point.x = point32.x * std::cos(angle) - point32.y * std::sin(angle);
            reutrn_point.y = point32.x * std::sin(angle) + point32.y * std::cos(angle);
            reutrn_point.z = point32.z;
            return reutrn_point;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Pose型のメッセージを返す
     * @param pose: geometry_msgs::Pose
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Pose型のメッセージ
    **/
        static geometry_msgs::Pose rotate(geometry_msgs::Pose pose, double angle)
        {
            pose.position = rotate(pose.position, angle);
            return pose;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::PoseStamped型のメッセージを返す
     * @param pose_stamped: geometry_msgs::PoseStamped
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::PoseStamped型のメッセージ
    **/
        static geometry_msgs::PoseStamped rotate(geometry_msgs::PoseStamped pose_stamped, double angle)
        {
            pose_stamped.pose = rotate(pose_stamped.pose, angle);
            return pose_stamped;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Vector3型のメッセージを返す
     * @param vector3: geometry_msgs::Vector3
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Vector3型のメッセージ
    **/
        static geometry_msgs::Vector3 rotate(geometry_msgs::Vector3 vector3, double angle)
        {
            geometry_msgs::Vector3 reutrn_vector3;
            reutrn_vector3.x = vector3.x * std::cos(angle) - vector3.y * std::sin(angle);
            reutrn_vector3.y = vector3.x * std::sin(angle) + vector3.y * std::cos(angle);
            reutrn_vector3.z = vector3.z;
            return reutrn_vector3;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Twist型のメッセージを返す
     * @param vector3: geometry_msgs::Twist
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Twist型のメッセージ
    **/
        static geometry_msgs::Twist rotate(geometry_msgs::Twist twist, double angle)
        {
            twist.linear = rotate(twist.linear, angle);
            return twist;
        }

        /**
     * @brief 原点中心に指定した角度だけ回転させたgeometry_msgs::Accel型のメッセージを返す
     * @param vector3: geometry_msgs::Accel
     * @param angle: 回転角度[rad/s]
     * @return 原点中心に指定した角度だけ回転させたgeometry_msgs::Accel型のメッセージ
    **/
        static geometry_msgs::Accel rotate(geometry_msgs::Accel accel, double angle)
        {
            accel.linear = rotate(accel.linear, angle);
            return accel;
        }
    };
}

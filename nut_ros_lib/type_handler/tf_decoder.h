/**
 * @file tf_decoder.h
 * @brief TFの中身を簡単に取り出せるようにする
 * @author Ryoga Sato
 * @date 2021/01/08
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros {

/**
 * @brief TFの中身を簡単に取り出せるようにする
**/
class TfDecoder{
private:
public:
    /**
     * @brief フレームIDからTFを読み座標関係をPose2D型のデータで取得する
     * @param parent_frame: 親フレームのID
     * @param child_frame: 子フレームのID
     * @return Pose2D型の座標関係データ
    **/
    static Pose2D<double> getPose2DFromFrameId(const std::string parent_frame, const std::string child_frame)
    {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        static Pose2D<double> pose(0, 0, 0);

        try
        {
            geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(0.1));
            pose.x = tf.transform.translation.x;
            pose.y = tf.transform.translation.y;
            pose.theta = MsgDecoder::getAngularVector(tf.transform.rotation)[2];
        }
        catch(tf2::TransformException &ex){

        }
        return pose;
    }
};
}

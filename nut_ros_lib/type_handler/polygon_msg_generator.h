/**
 * @file polygon_msg_generator.h
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
class PolygonMsgGenerator{
private:
public:

    /**
     * @brief 指定座標だけ平行移動したポリゴンを生成
     * @param x: x座標[m]
     * @param y: y座標[m]
     * @param width: 幅[m]
     * @param height: 高さ[m]
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon translation(geometry_msgs::Polygon polygon, float x, float y)
    {
        for (int i = 0; i < polygon.points.size(); ++i)
            polygon.points[i] = MsgGenerator::toPoint32(polygon.points[i].x + x, polygon.points[i].y + y, 0.0);
        return polygon;
    }

    /**
     * @brief std::vector<std::vector<float>>で定義されるポイントリストからgeometry_msgs::Polygon型のメッセージを生成
     * @param std_vector: std::vector<std::vector<float>>で定義されるポイントリスト
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon fromVector(std::vector<std::vector<float>> std_vector)
    {
        geometry_msgs::Polygon polygon;
        for (int i = 0; i < std_vector.size(); ++i)
            polygon.points.push_back(MsgGenerator::toPoint32(std_vector[i][0], std_vector[i][1], 0.0));
        return polygon;
    }

    /**
     * @brief 始点と終点の座標を指定して直線を生成
     * @param x1: 始点のx座標[m]
     * @param y1: 始点のy座標[m]
     * @param x2: 終点のx座標[m]
     * @param y2: 終点のy座標[m]
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toLine(float x1, float y1, float x2, float y2)
    {
        geometry_msgs::Polygon polygon;
        polygon.points.push_back(MsgGenerator::toPoint32(x1, y1, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x2, y2, 0.0)); 
        return polygon;
    }

    /**
     * @brief 3点の座標を指定して三角形を生成
     * @param x1: 頂点1のx座標[m]
     * @param y1: 頂点1のy座標[m]
     * @param x2: 頂点2のx座標[m]
     * @param y2: 頂点2のy座標[m]
     * @param x3: 頂点3のx座標[m]
     * @param y3: 頂点3のy座標[m]
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toTriangle(float x1, float y1, float x2, float y2, float x3, float y3)
    {
        geometry_msgs::Polygon polygon;
        polygon.points.push_back(MsgGenerator::toPoint32(x1, y1, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x2, y2, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x3, y3, 0.0)); 
        return polygon;
    }

    /**
     * @brief 中心座標，幅，高さを指定して長方形を生成
     * @param x: x座標[m]
     * @param y: y座標[m]
     * @param width: 幅[m]
     * @param height: 高さ[m]
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toRect(float x, float y, float width, float height)
    {
        geometry_msgs::Polygon polygon;
        polygon.points.push_back(MsgGenerator::toPoint32( width/2.0,  height/2.0, 0.0));
        polygon.points.push_back(MsgGenerator::toPoint32(-width/2.0,  height/2.0, 0.0));
        polygon.points.push_back(MsgGenerator::toPoint32(-width/2.0, -height/2.0, 0.0));
        polygon.points.push_back(MsgGenerator::toPoint32( width/2.0, -height/2.0, 0.0));
        polygon = translation(polygon, x, y);
        return polygon;
    }

    /**
     * @brief 3点の座標を指定して四角形を生成
     * @param x1: 頂点1のx座標[m]
     * @param y1: 頂点1のy座標[m]
     * @param x2: 頂点2のx座標[m]
     * @param y2: 頂点2のy座標[m]
     * @param x3: 頂点3のx座標[m]
     * @param y3: 頂点3のy座標[m]
     * @param x4: 頂点4のx座標[m]
     * @param y4: 頂点4のy座標[m]
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toQuad(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
    {
        geometry_msgs::Polygon polygon;
        polygon.points.push_back(MsgGenerator::toPoint32(x1, y1, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x2, y2, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x3, y3, 0.0)); 
        polygon.points.push_back(MsgGenerator::toPoint32(x4, y4, 0.0)); 
        return polygon;
    }


    /**
     * @brief 中心座標，幅，高さを指定して楕円を生成
     * @param x: x座標[m]
     * @param y: y座標[m]
     * @param width: 幅[m]
     * @param height: 高さ[m]
     * @param resolution: 頂点数（デフォルト:16）
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toEllipse(float x, float y, float width, float height, const int resolution = 16)
    {
        geometry_msgs::Polygon polygon;
        for (int i = 0; i < resolution; ++i) {
            polygon.points.push_back(MsgGenerator::toPoint32(width/2  * std::cos(i * 2*M_PI / (float)resolution),
                                                             height/2 * std::sin(i * 2*M_PI / (float)resolution),
                                                             0.0));
        }
        polygon = translation(polygon, x, y);
        return polygon;
    }

    /**
     * @brief 中心座標，半径を指定して円を生成
     * @param x: x座標[m]
     * @param y: y座標[m]
     * @param r: 半径[m]
     * @param resolution: 頂点数（デフォルト:16）
     * @return geometry_msgs::Polygon型のメッセージ
    **/
    static geometry_msgs::Polygon toCircle(float x, float y, float r, const int resolution = 16)
    {
        return toEllipse(x, y, r*2.0, r*2.0, resolution);
    }

};
}

/**
 * @file pose_2D.h
 * @brief 2次元の座標を扱う
 * @author Ryoga Sato
 * @date 2020/09/08
**/
#pragma once

#include "./../nut_generic.h"
#include "./vector2.h"

namespace nut_ros
{
    /**
 * @brief 2次元の座標を扱う
**/
    template <typename T>
    class Pose2D
    {
    public:
        T x = 0;     /**< 2次元直交座標におけるx成分 */
        T y = 0;     /**< 2次元直交座標におけるy成分 */
        T theta = 0; /**< 2次元直交座標における角度（向き）成分 [rad] */

        /**
     * @brief コンストラクタ
     */
        Pose2D() = default;

        /**
     * @brief コンストラクタ 直交座標(_x, _y, _theta)で初期化
     */
        Pose2D(T _x, T _y, T _theta) : x(_x), y(_y), theta(_theta) {}

        /**
     * @brief コンストラクタ Vector2と角度の数値で初期化
     */
        Pose2D(const Vector2<T> &v, T _theta)
        {
            x = v.x;
            y = v.y;
            theta = _theta;
        }

        /**
     * @brief コンストラクタ 直交座標(_x, _y)で初期化（角度はゼロ）
     */
        Pose2D(T _x, T _y) : x(_x), y(_y) {}

        /**
     * @brief コンストラクタ Vector2で初期化（角度はゼロ）
     */
        Pose2D(const Vector2<T> &v)
        {
            x = v.x;
            y = v.y;
        }

        /**
     * @brief 指定されたベクトルがこのベクトルと等しい場合にtrueを返す
     * @param v: 指定するベクトル
     */
        bool equals(const Pose2D &v)
        {
            return *this == v;
        }

        /**
     * @brief 直交座標形式でこのベクトルを設定
     * @param _x: 指定するベクトル
     * @param _y: 指定するベクトル
     * @param _theta: 指定するベクトル
     */
        void set(T _x, T _y, T _theta)
        {
            x = _x;
            y = _y;
            theta = _theta;
        }

        /**
     * @brief 極座標形式でこのベクトルを設定
     * @param r: 原点からの距離
     * @param angle: 原点との角度
     * @param robot_theta: ロボットの座標
     */
        void setByPolar(T r, T angle, T robot_theta)
        {
            x = r * std::cos(angle);
            y = r * std::sin(angle);
            theta = robot_theta;
        }

        /**
     * @brief このベクトルを原点中心にangle[rad]回転
     * @param angle: 回転させる角度[rad]
     */
        void rotate(T angle)
        {
            T tmp_x = x;
            T tmp_y = y;
            x = tmp_x * cos(angle) - tmp_y * sin(angle);
            y = tmp_x * sin(angle) + tmp_y * cos(angle);
        }

        /**
     * @brief 指定座標中心(rot_x, rot_y)に回転
     * @param rot_x: 回転中心のx座標
     * @param rot_y: 回転中心のy座標
     * @param angle: 回転させる角度[rad]
     */
        void rotate(T rot_x, T rot_y, T angle)
        {
            Vector2<T> p(rot_x, rot_y);
            rotate(p, angle);
        }

        /**
     * @brief 座標oを中心にangleだけ回転
     * @param o: 回転中心の座標
     * @param angle: 回転させる角度[rad]
     */
        void rotate(Vector2<T> o, T angle)
        {
            Vector2<T> p(x - o.x, y - o.y);
            p.rotate(angle);
            p.x += o.x;
            p.y += o.y;
            x = p.x;
            y = p.y;
        }

        /**
     * @brief このベクターをフォーマットした文字列を返す
     * @return フォーマットした文字列
     */
        std::string toString()
        {
            return '(' + std::to_string(x) + ", " + std::to_string(y) + ')';
        }

        /**
     * @brief このベクトルの長さを返す
     * @return このベクトルの長さ
     */
        T length() const
        {
            return magnitude();
        }

        /**
     * @brief このベクトルの長さを返す
     * @return このベクトルの長さ
     */
        T magnitude() const
        {
            return std::sqrt(sqrMagnitude());
        }

        /**
     * @brief このベクトルの長さの2乘を返す
     * @return このベクトルの長さ2乘
     */
        constexpr T sqrLength() const
        {
            return sqrMagnitude();
        }

        /**
     * @brief このベクトルの長さの2乘を返す
     * @return このベクトルの長さ2乘
     */
        constexpr T sqrMagnitude() const
        {
            return x * x + y * y;
        }

        /**
     * @brief geometry_msgs::Accel型のメッセージを返す
     * @return geometry_msgs::Accel型のメッセージ
    **/
        geometry_msgs::Accel toAccelMsg()
        {
            geometry_msgs::Accel accel;
            accel.linear.x = (double)x;
            accel.linear.y = (double)y;
            accel.angular.z = (double)theta;
            return accel;
        }

        /**
     * @brief geometry_msgs::Point型のメッセージを返す
     * @return geometry_msgs::Point型のメッセージ
    **/
        geometry_msgs::Point toPointMsg()
        {
            geometry_msgs::Point point;
            point.x = (double)x;
            point.y = (double)y;
            return point;
        }

        /**
     * @brief geometry_msgs::Point32型のメッセージを返す
     * @return geometry_msgs::Point32型のメッセージ
    **/
        geometry_msgs::Point32 toPoint32Msg()
        {
            geometry_msgs::Point32 point32;
            point32.x = (float)x;
            point32.y = (float)y;
            return point32;
        }

        /**
     * @brief geometry_msgs::Pose型のメッセージを返す
     * @return geometry_msgs::Pose型のメッセージ
    **/
        geometry_msgs::Pose toPoseMsg()
        {
            geometry_msgs::Pose pose;
            pose.position.x = (double)x;
            pose.position.y = (double)y;
            pose.orientation = tf::createQuaternionMsgFromYaw((double)theta);
            return pose;
        }

        /**
     * @brief geometry_msgs::Pose2D型のメッセージを返す
     * @return geometry_msgs::Pose2D型のメッセージ
    **/
        geometry_msgs::Pose2D toPose2DMsg()
        {
            geometry_msgs::Pose2D pose2d;
            pose2d.x = (double)x;
            pose2d.y = (double)y;
            pose2d.theta = (double)theta;
            return pose2d;
        }

        /**
     * @brief geometry_msgs::Quaternion型のメッセージを返す
     * @return geometry_msgs::Quaternion型のメッセージ
    **/
        geometry_msgs::Quaternion toQuaternionMsg()
        {
            return tf::createQuaternionMsgFromYaw((double)theta);
        }

        /**
     * @brief geometry_msgs::Transform型のメッセージを返す
     * @return geometry_msgs::Transform型のメッセージ
    **/
        geometry_msgs::Transform toTransformMsg()
        {
            geometry_msgs::Transform transform;
            transform.translation.x = (double)x;
            transform.translation.y = (double)y;
            transform.rotation = tf::createQuaternionMsgFromYaw((double)theta);
            return transform;
        }

        /**
     * @brief geometry_msgs::Twist型のメッセージを返す
     * @return geometry_msgs::Twist型のメッセージ
    **/
        geometry_msgs::Twist toTwistMsg()
        {
            geometry_msgs::Twist twist;
            twist.linear.x = (double)x;
            twist.linear.y = (double)y;
            twist.angular.z = (double)theta;
            return twist;
        }

        /**
     * @brief geometry_msgs::Vector3型のメッセージを返す
     * @details vector3のx,y,zはPose2Dのx,y,thetaに対応して格納される
     * @return geometry_msgs::Vector3型のメッセージ
    **/
        geometry_msgs::Vector3 toVector3Msg()
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = (double)x;
            vector3.y = (double)y;
            vector3.z = (double)theta;
            return vector3;
        }

        /**
     * @brief 2つのベクトルの内積を返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの内積
     */
        static T getDot(Pose2D a, Pose2D b)
        {
            return (a.x * b.x + a.y * b.y);
        }

        /**
     * @brief 2つのベクトルの外積の大きさを返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの外積の大きさ
     */
        static T getCross(Pose2D a, Pose2D b)
        {
            return (a.x * b.y - a.y * b.x);
        }

        /**
     * @brief 2つのベクトルのなす角を弧度法で返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルのなす角[rad]
     */
        static T getAngle(Pose2D a, Pose2D b)
        {
            return std::atan2(b.y - a.y, b.x - a.x);
        }

        /**
     * @brief 2つのベクトルの距離を返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの距離を返す
     */
        static T getDistance(Pose2D a, Pose2D b)
        {
            Pose2D v = (b - a);
            return v.magnitude();
        }

    /**
     * @brief ベクトルaとbの間をtで線形補間
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @param t: 媒介変数
     * @return 補間点
     */
    static Pose2D lerp(Pose2D a, Pose2D b, T t)
    {
        t = Generic::guard(t, 0.0, 1.0);
        Pose2D v = a;
        v.x += (b.x - a.x) * t;
        v.y += (b.y - a.y) * t;
        v.theta += (b.theta - a.theta) * t;
        return v;
    }

    /**
     * @brief ベクトルaとbの中点を返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 中点
     */
    static Pose2D getMidpoint(Pose2D a, Pose2D b)
    {
        return lerp(a, b, 0.5);
    }

        /**
     * @brief ベクトルの代入
     */
        Pose2D &operator=(const Pose2D &v)
        {
            x = v.x;
            y = v.y;
            theta = v.theta;
            return *this;
        }

        /**
     * @brief 全ての要素にスカラ加算
     */
        constexpr Pose2D operator+() const
        {
            return *this;
        }

        /**
     * @brief 全ての要素にスカラ減算
     */
        constexpr Pose2D operator-() const
        {
            return {-x, -y, -theta};
        }

        /**
     * @brief ベクトルの要素同士の和
     */
        constexpr Pose2D operator+(const Pose2D &v) const
        {
            return {x + v.x, y + v.y, theta + v.theta};
        }

        /**
     * @brief ベクトルの要素同士の差
     */
        constexpr Pose2D operator-(const Pose2D &v) const
        {
            return {x - v.x, y - v.y, theta - v.theta};
        }

        /**
     * @brief 全ての要素にスカラ乗算
     * @attention ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用
     */
        constexpr Pose2D operator*(T s) const
        {
            return {x * s, y * s, theta * s};
        }

        /**
     * @brief 全ての要素にスカラ除算
     * @attention ベクトル同士の除算は未定義
     */
        constexpr Pose2D operator/(T s) const
        {
            return {x / s, y / s, theta / s};
        }

        /**
     * @brief ベクトルの要素同士の和を代入（スカラとの和の場合は全ての要素に対して加算）
     */
        Pose2D &operator+=(const Pose2D &v)
        {
            x += v.x;
            y += v.y;
            theta += v.theta;
            return *this;
        }

        /**
     * @brief ベクトルの要素同士の差を代入（スカラとの和の場合は全ての要素に対して減算）
     */
        Pose2D &operator-=(const Pose2D &v)
        {
            x -= v.x;
            y -= v.y;
            theta -= v.theta;
            return *this;
        }

        /**
     * @brief 全ての要素に対してスカラ乗算して代入（ベクトル同士の乗算は未定義）
     */
        Pose2D &operator*=(T s)
        {
            x *= s;
            y *= s;
            theta *= s;
            return *this;
        }

        /**
     * @brief 全ての要素に対してスカラ除算して代入（ベクトル同士の除算は未定義）
     */
        Pose2D &operator/=(T s)
        {
            x /= s;
            y /= s;
            theta /= s;
            return *this;
        }

        /**
     * @brief 2つのベクトルが等しい場合にtrueを返す
     */
        bool operator==(const Pose2D &v) const
        {
            return ((x == v.x) && (y == v.y) && (theta == v.y));
        }

        /**
     * @brief 2つのベクトルが等しい場合にfalseを返す
     */
        bool operator!=(const Pose2D &v) const
        {
            return !((x == v.x) && (y == v.y) && (theta == v.y));
        }

    private:
    };

    template <typename Char, typename T>
    inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Pose2D<T> &v)
    {
        return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(',') << Char(' ') << v.theta << Char(')');
    }

    template <typename Char, typename T>
    inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Pose2D<T> &v)
    {
        Char unused;
        return is >> unused >> v.x >> unused >> v.y >> unused >> v.theta >> unused;
    }
}

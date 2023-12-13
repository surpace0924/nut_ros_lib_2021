/**
 * @file vector2.h
 * @brief 2要素のベクトル
 * @author Ryoga Sato
 * @date 2020/09/08
**/
#pragma once

#include "./../nut_generic.h"

namespace nut_ros {
/**
 * @brief 2要素のベクトル
**/
template <typename T>
class Vector2
{
public:
    T x = 0; /**< 2次元直交座標におけるx成分 */
    T y = 0; /**< 2次元直交座標におけるy成分 */

    /**
     * @brief コンストラクタ
     */
    constexpr Vector2() = default;

    /**
     * @brief コンストラクタ 直交座標(_x, _y)で初期化
     */
    constexpr Vector2(T _x, T _y) : x(_x), y(_y) {}

    /**
     * @brief 指定されたベクトルがこのベクトルと等しい場合にtrueを返す
     * @param v: 指定するベクトル
     */
    bool equals(const Vector2 &v)
    {
        return *this == v;
    }

    /**
     * @brief このベクトルの大きさを1にする
     */
    void normalize()
    {
        *this /= length();
    }

    /**
     * @brief 直交座標形式でこのベクトルを設定
     * @param _x: 指定するベクトル
     */
    void set(T _x, T _y)
    {
        x = _x;
        y = _y;
    }

    /**
     * @brief 極座標形式でこのベクトルを設定
     * @param r: 原点からの距離
     * @param angle: 原点との角度
     */
    void setByPolar(T r, T angle)
    {
        x = r * std::cos(angle);
        y = r * std::sin(angle);
    }

    /**
     * @brief このベクトルを原点中心にangle[rad]回転
     * @param angle: 回転させる角度[rad]
     */
    void rotate(T angle)
    {
        x = x * cos(angle) - y * sin(angle);
        y = x * sin(angle) + y * cos(angle);
    }

    /**
     * @brief 指定座標中心(rot_x, rot_y)に回転
     * @param rot_x: 回転中心のx座標
     * @param rot_y: 回転中心のy座標
     * @param angle: 回転させる角度[rad]
     */
    void rotate(T rot_x, T rot_y, T angle)
    {
        Vector2 p(rot_x, rot_y);
        rotate(p, angle);
    }

    /**
     * @brief 座標oを中心にangleだけ回転
     * @param o: 回転中心の座標
     * @param angle: 回転させる角度[rad]
     */
    void rotate(Vector2 o, T angle)
    {
        Vector2 p(x - o.x, y - o.y);
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
     * @brief 大きさが1のこのベクトルを返す
     * @return 大きさが1のこのベクトル
     */
    Vector2 normalized() const
    {
        return *this / length();
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
     * @brief 2つのベクトルの内積を返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの内積
     */
    static T getDot(Vector2 a, Vector2 b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    /**
     * @brief 2つのベクトルの外積の大きさを返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの外積の大きさ
     */
    static T getCross(Vector2 a, Vector2 b)
    {
        return (a.x * b.y - a.y * b.x);
    }

    /**
     * @brief 2つのベクトルのなす角を弧度法で返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルのなす角[rad]
     */
    static T getAngle(Vector2 a, Vector2 b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    /**
     * @brief 2つのベクトルの距離を返す
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @return 2つのベクトルの距離を返す
     */
    static T getDistance(Vector2 a, Vector2 b)
    {
        Vector2 v = (b - a);
        return v.magnitude();
    }

    /**
     * @brief ベクトルaとbの間をtで線形補間
     * @param a: 1つ目のベクトル
     * @param b: 2つ目のベクトル
     * @param t: 媒介変数
     * @return 補間点
     */
    static Vector2 lerp(Vector2 a, Vector2 b, T t)
    {
        t = Generic::guard(t, 0.0, 1.0);
        Vector2 v = a;
        v.x += (b.x - a.x) * t;
        v.y += (b.y - a.y) * t;
        return v;
    }

    /**
     * @brief 代入
     */
    Vector2 &operator=(T s)
    {
        x = s;
        y = s;
        return *this;
    }

    /**
     * @brief 全ての要素にスカラ加算
     */
    constexpr Vector2 operator+() const
    {
        return *this;
    }

    /**
     * @brief 全ての要素にスカラ減算
     */
    constexpr Vector2 operator-() const
    {
        return {-x, -y};
    }

    /**
     * @brief ベクトルの要素同士の和
     */
    constexpr Vector2 operator+(const Vector2 &v) const
    {
        return {x + v.x, y + v.y};
    }

    /**
     * @brief ベクトルの要素同士の差
     */
    constexpr Vector2 operator-(const Vector2 &v) const
    {
        return {x - v.x, y - v.y};
    }

    /**
     * @brief 全ての要素にスカラ乗算
     * @attention ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用
     */
    constexpr Vector2 operator*(T s) const
    {
        return {x * s, y * s};
    }

    /**
     * @brief 全ての要素にスカラ除算
     * @attention ベクトル同士の除算は未定義
     */
    constexpr Vector2 operator/(T s) const
    {
        return {x / s, y / s};
    }

    /**
     * @brief ベクトルの要素同士の和を代入（スカラとの和の場合は全ての要素に対して加算）
     */
    Vector2 &operator+=(const Vector2 &v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    /**
     * @brief ベクトルの要素同士の差を代入（スカラとの和の場合は全ての要素に対して減算）
     */
    Vector2 &operator-=(const Vector2 &v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    /**
     * @brief 全ての要素に対してスカラ乗算して代入（ベクトル同士の乗算は未定義）
     */
    Vector2 &operator*=(T s)
    {
        x *= s;
        y *= s;
        return *this;
    }

    /**
     * @brief 全ての要素に対してスカラ除算して代入（ベクトル同士の除算は未定義）
     */
    Vector2 &operator/=(T s)
    {
        x /= s;
        y /= s;
        return *this;
    }

    /**
     * @brief 2つのベクトルが等しい場合にtrueを返す
     */
    bool operator==(const Vector2 &v) const
    {
        return (x == v.x && (y == v.y));
    }

    /**
     * @brief 2つのベクトルが等しい場合にfalseを返す
     */
    bool operator!=(const Vector2 &v) const
    {
        return !(x == v.x && (y == v.y));
    }

private:
};

template <typename Char, typename T>
inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Vector2<T> &v)
{
    return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(')');
}

template <typename Char, typename T>
inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Vector2<T> &v)
{
    Char unused;
    return is >> unused >> v.x >> unused >> v.y >> unused;
}
}

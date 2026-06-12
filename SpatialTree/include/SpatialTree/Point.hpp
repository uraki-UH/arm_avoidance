#ifndef SPATIAL_TREE_POINT_HPP
#define SPATIAL_TREE_POINT_HPP

#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace SpatialTree {

/**
 * @brief Eigen 非依存の N次元ポイント型（std::array ベース）。
 * GNG で必要な基本的なベクトル演算を記述
 */
template <typename Scalar, int Dim>
struct Point {
    std::array<Scalar, Dim> data;

    Point() { data.fill(0); }
    
    // 値指定コンストラクタ（可変長引数）
    template<typename... Args>
    Point(Args... args) : data{{static_cast<Scalar>(args)...}} {
        static_assert(sizeof...(Args) == Dim, "Incorrect number of arguments for Point constructor");
    }

    static Point Zero() { return Point(); }

    Scalar& operator[](int i) { return data[i]; }
    const Scalar& operator[](int i) const { return data[i]; }

    // 軸別アクセス (x, y, z) - テンプレートメタプログラミングで Dim に応じて提供
    Scalar& x() { return data[0]; }
    Scalar x() const { return data[0]; }
    
    Scalar& y() { 
        static_assert(Dim >= 2, "Y axis only available for Dim >= 2");
        return data[1]; 
    }
    Scalar y() const { 
        static_assert(Dim >= 2, "Y axis only available for Dim >= 2");
        return data[1]; 
    }

    Scalar& z() { 
        static_assert(Dim >= 3, "Z axis only available for Dim >= 3");
        return data[2]; 
    }
    Scalar z() const { 
        static_assert(Dim >= 3, "Z axis only available for Dim >= 3");
        return data[2]; 
    }

    // 演算子オーバーロード
    Point& operator+=(const Point& other) {
        for (int i = 0; i < Dim; ++i) data[i] += other.data[i];
        return *this;
    }
    Point operator+(const Point& other) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] + other.data[i];
        return res;
    }

    Point& operator-=(const Point& other) {
        for (int i = 0; i < Dim; ++i) data[i] -= other.data[i];
        return *this;
    }
    Point operator-(const Point& other) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] - other.data[i];
        return res;
    }

    Point operator*(Scalar s) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] * s;
        return res;
    }
    
    Point operator/(Scalar s) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] / s;
        return res;
    }

    // ベクトル計算
    Scalar squaredNorm() const {
        Scalar res = 0;
        for (int i = 0; i < Dim; ++i) res += data[i] * data[i];
        return res;
    }
    
    Scalar norm() const {
        return std::sqrt(squaredNorm());
    }

    Scalar dot(const Point& other) const {
        Scalar res = 0;
        for (int i = 0; i < Dim; ++i) res += data[i] * other.data[i];
        return res;
    }

    bool isZero(Scalar epsilon = static_cast<Scalar>(1e-9)) const {
        for (int i = 0; i < Dim; ++i) {
            if (std::abs(data[i]) > epsilon) return false;
        }
        return true;
    }
};

// Scalar * Point
template <typename Scalar, int Dim>
Point<Scalar, Dim> operator*(Scalar s, const Point<Scalar, Dim>& p) {
    return p * s;
}

} // namespace SpatialTree

#endif // SPATIAL_TREE_POINT_HPP

#ifndef VECTOR2_HPP
#define VECTOR2_HPP

#include <iostream>
#include <cmath>
#include <type_traits>

template <typename T>
struct Point2D
{
    Point2D(T _x = T(), T _y = T()) : x(_x), y(_y) { }
    Point2D(const Point2D<T>& other) { x = other.x; y = other.y; }
    Point2D<T>& operator=(const Point2D<T>& rhs) { x = rhs.x; y = rhs.y; return *this; }
    T x, y;
};

template <typename T>
struct Vector2D
{
    Vector2D(T _x = T(), T _y = T()) : x(_x), y(_y) { }
    Vector2D(const Vector2D<T>& other) { x = other.x; y = other.y; }
    Vector2D<T>& operator= (const Vector2D<T>& rhs) { x = rhs.x; y = rhs.y; return *this; }
    Vector2D<T>& operator+=(const Vector2D<T>& rhs) { x += rhs.x; y += rhs.y; return *this; }
    Vector2D<T>& operator-=(const Vector2D<T>& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    Vector2D<T>& operator/=(T rhs) {
        if(std::is_integral<T>::value) {
            std::cerr << "/= warning: operator is not functioning properly for integral types"
                      << std::endl; }
        x /= rhs; y /= rhs; return *this; }
    Vector2D<T>& operator*=(T rhs) { x *= rhs; y *= rhs; return *this; }
    T dot(const Vector2D<T>& vec) const { return x*vec.x + y*vec.y; }
    double norm() { return std::sqrt(dot(*this)); }
    void normalize() {
        if(std::is_integral<T>::value) {
            std::cerr << "normalize() is not functioning for integral types" << std::endl;
            return;
        }
        *this /= norm(); }
    Vector2D<T> normalized() {
        if(std::is_integral<T>::value) {
            std::cerr << "normalize() is not functioning for integral types" << std::endl;
            return *this;
        }
        return Vector2D<T>(*this) /= norm();
    }
    T x, y;
};

template <typename T>
double dist(const Point2D<T>& p1, const Point2D<T>& p2) {
    Vector2D<T> v = p2-p1;
    return v.norm();
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const Vector2D<T>& vec) {
    out << "(" << vec.x << ", " << vec.y << ")";
    return out;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const Point2D<T>& pt) {
    out << "(" << pt.x << ", " << pt.y << ")";
    return out;
}

// vector + point --> point
template <typename T>
Point2D<T> operator+(const Vector2D<T>& left, const Point2D<T>& right) {
    return Point2D<T>(left.x + right.x, left.y + right.y);
}

// point + vector --> point
template <typename T>
Point2D<T> operator+(const Point2D<T>& left, const Vector2D<T>& right) {
    return Point2D<T>(left.x + right.x, left.y + right.y);
}

// point - vector --> point (point + (-vector))
template <typename T>
Point2D<T> operator-(const Point2D<T>& left, const Vector2D<T>& right) {
    return Point2D<T>(left.x - right.x, left.y - right.y);
}

// vector + vector --> vector
template <typename T>
Vector2D<T> operator+(const Vector2D<T>& left, const Vector2D<T>& right) {
    return Vector2D<T>(left.x + right.x, left.y + right.y);
}

// vector - vector --> vector
template <typename T>
Vector2D<T> operator-(const Vector2D<T>& vec) {
    return T(-1)*vec;
}

// vector - vector --> vector
template <typename T>
Vector2D<T> operator-(const Vector2D<T>& left, const Vector2D<T>& right) {
    return Vector2D<T>(left.x - right.x, left.y - right.y);
}

// point - point --> vector
template <typename T>
Vector2D<T> operator-(const Point2D<T>& target, const Point2D<T>& source) {
    return Vector2D<T>(target.x - source.x, target.y - source.y);
}

template <typename T>
Vector2D<T> operator/(const Vector2D<T>& left, T right) {
    if(std::is_integral<T>::value) {
        std::cerr << "/ warning: operator is not functioning properly for integral types"
                  << std::endl; }
    return Vector2D<T>(left.x / right, left.y / right);
}

template <typename T>
Vector2D<T> operator*(const Vector2D<T>& left, T right) {
    return Vector2D<T>(left.x * right, left.y * right);
}

template <typename T>
Vector2D<T> operator*(T left, const Vector2D<T>& right) {
    return Vector2D<T>(right.x * left, right.y * left);
}

template <typename T>
Point2D<T> operator*(const Point2D<T>& left, T right) {
    return Point2D<T>(left.x * right, left.y * right);
}

template <typename T>
Point2D<T> operator*(T left, const Point2D<T>& right) {
    return Point2D<T>(right.x * left, right.y * left);
}

#endif // VECTOR2_HPP

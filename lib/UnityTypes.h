/*
Multi-Dimensional Variables for multi-dimensional data.
Unified representation of multi-dimensional data in a single variable for
the University of Akron Nasa Mining Competition Robotics Team.

Use these types to store dimensional data in a single variable.
For example, you can store a 3D position in a float3 variable.

These types all have overloaded operators to make them easy to use, and align with common vector
operations. Note some irregularities in the operators, to align with vector operations.

Cross/Dot Product:
Default is cross product, but dot product is also available as a function: .dot()

Magnitude:
The magnitude of a vector is known as the modulus in vector math. Can be accessed with the modulus
operator: %

Absolute Value:
The absolute value of a vector is the vector with all positive components. Can be accessed with the
unary plus operator: +

Negation/Inversion:
The negation of a vector is the vector with all components negated. Can be accessed with the unary
minus operator: -

Truncation:
In the event of a vector being converted to a a lower dimension, the higher dimension components are
truncated. For example, a vector3 being converted to a vector2 will lose the z component.

If a vector3 * vector2 operation is performed, the z component is ignored.
However, if a vector3 * scalar operation is performed, the scalar is multiplied by all components.

There is no vector2 * vector3 operation, as it is redundant. If you need to multiply a vector2 by a
vector3, you can convert the vector2 to a vector3 first.


*** To Be Implemented ***
Logical Operators: &&, ||


*/

#include <math.h>
#include <stdint.h>

#include <iostream>

#ifndef UNITYTYPES_H
#define UNITYTYPES_H

namespace unity {

// 2D Vector

template <typename T>
struct Vector2 {
    T x, y;

    // Constructors
    Vector2() : x(0), y(0) {}
    Vector2(T s) : x(s), y(s) {}
    Vector2(T x, T y) : x(x), y(y) {}

    // Overloaded Operators
    Vector2 operator+(const Vector2& v) const {
        return Vector2(x + v.x, y + v.y);
    }  // Addition of two vectors
    Vector2 operator+(const T& s) const { return Vector2(x + s, y + s); }  // Addition of a scalar

    Vector2 operator-(const Vector2& v) const {
        return Vector2(x - v.x, y - v.y);
    }  // Subtraction of two vectors
    Vector2 operator-(const T& s) const {
        return Vector2(x - s, y - s);
    }  // Subtraction of a scalar

    Vector2 operator*(const Vector2& v) const {
        return Vector2(x * v.x, y * v.y);
    }  // Cross multiplication of two vectors
    Vector2 operator*(const T& s) const {
        return Vector2(x * s, y * s);
    }  // Cross multiplication of a scalar

    // Dot Product
    T dot(const Vector2& v) const { return x * v.x + y * v.y; }  // Dot product of two vectors

    Vector2 operator/(const Vector2& v) const {
        return Vector2(x / v.x, y / v.y);
    }  // Division of two vectors
    Vector2 operator/(const T& s) const { return Vector2(x / s, y / s); }  // Division of a scalar

    T operator%(const Vector2& v) const { return sqrt(x * x + y * y); }  // Magnitude of the vector
    T operator%(const T& s) const { return sqrt(x * x + y * y); }        // Magnitude of the vector

    Vector2 operator++() {
        return Vector2(++x, ++y);
    }  // Prefix Increment (why does this even exist?)
    Vector2 operator++(int) { return Vector2(x++, y++); }  // Postfix Increment

    Vector2 operator--() { return Vector2(--x, --y); }     // Prefix Decrement
    Vector2 operator--(int) { return Vector2(x--, y--); }  // Postfix Decrement

    // Unary Operators

    Vector2 operator+() const { return Vector2(abs(x), abs(y)); }  // Unary Plus
    Vector2 operator-() const { return Vector2(-x, -y); }          // Unary Minus

    // Comparison Operators

    bool operator==(const Vector2& v) const { return x == v.x && y == v.y; }  // Equality
    bool operator==(const T& s) const { return x == s && y == s; }            // Equality

    bool operator!=(const Vector2& v) const { return x != v.x || y != v.y; }  // Inequality
    bool operator!=(const T& s) const { return x != s || y != s; }            // Inequality

    bool operator>(const Vector2& v) const { return x > v.x && y > v.y; }  // Greater Than
    bool operator>(const T& s) const { return x > s && y > s; }            // Greater Than

    bool operator>=(const Vector2& v) const {
        return x >= v.x && y >= v.y;
    }  // Greater Than or Equal To
    bool operator>=(const T& s) const { return x >= s && y >= s; }  // Greater Than or Equal To

    bool operator<(const Vector2& v) const { return x < v.x && y < v.y; }  // Less Than
    bool operator<(const T& s) const { return x < s && y < s; }            // Less Than

    bool operator<=(const Vector2& v) const {
        return x <= v.x && y <= v.y;
    }  // Less Than or Equal To
    bool operator<=(const T& s) const { return x <= s && y <= s; }  // Less Than or Equal To

    // Logical Operators

    bool operator!() const { return !x && !y; }  // Logical NOT

    // Bitwise Operators

    Vector2 operator~() const { return Vector2(~x, ~y); }  // Bitwise NOT

    Vector2 operator&(const Vector2& v) const { return Vector2(x & v.x, y & v.y); }  // Bitwise AND
    Vector2 operator&(const T& s) const { return Vector2(x & s, y & s); }            // Bitwise AND

    Vector2 operator|(const Vector2& v) const { return Vector2(x | v.x, y | v.y); }  // Bitwise OR
    Vector2 operator|(const T& s) const { return Vector2(x | s, y | s); }            // Bitwise OR

    Vector2 operator^(const Vector2& v) const { return Vector2(x ^ v.x, y ^ v.y); }  // Bitwise XOR
    Vector2 operator^(const T& s) const { return Vector2(x ^ s, y ^ s); }            // Bitwise XOR

    Vector2 operator<<(const Vector2& v) const {
        return Vector2(x << v.x, y << v.y);
    }  // Bitwise Shift Left
    Vector2 operator<<(const T& s) const { return Vector2(x << s, y << s); }  // Bitwise Shift Left

    Vector2 operator>>(const Vector2& v) const {
        return Vector2(x >> v.x, y >> v.y);
    }  // Bitwise Shift Right
    Vector2 operator>>(const T& s) const { return Vector2(x >> s, y >> s); }  // Bitwise Shift Right

    // Assignment Operators

    Vector2& operator=(const Vector2& v) {
        x = v.x;
        y = v.y;
        return *this;
    }  // Assignment

    Vector2& operator+=(const Vector2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }  // Addition Assignment
    Vector2& operator+=(const T& s) {
        x += s;
        y += s;
        return *this;
    }  // Addition Assignment

    Vector2& operator-=(const Vector2& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }  // Subtraction Assignment
    Vector2& operator-=(const T& s) {
        x -= s;
        y -= s;
        return *this;
    }  // Subtraction Assignment

    Vector2& operator*=(const Vector2& v) {
        x *= v.x;
        y *= v.y;
        return *this;
    }  // Cross Multiplication Assignment
    Vector2& operator*=(const T& s) {
        x *= s;
        y *= s;
        return *this;
    }  // Cross Multiplication Assignment

    Vector2& operator/=(const Vector2& v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }  // Division Assignment
    Vector2& operator/=(const T& s) {
        x /= s;
        y /= s;
        return *this;
    }  // Division Assignment

    Vector2& operator&=(const Vector2& v) {
        x &= v.x;
        y &= v.y;
        return *this;
    }  // Bitwise AND Assignment
    Vector2& operator&=(const T& s) {
        x &= s;
        y &= s;
        return *this;
    }  // Bitwise AND Assignment

    Vector2& operator|=(const Vector2& v) {
        x |= v.x;
        y |= v.y;
        return *this;
    }  // Bitwise OR Assignment
    Vector2& operator|=(const T& s) {
        x |= s;
        y |= s;
        return *this;
    }  // Bitwise OR Assignment

    Vector2& operator^=(const Vector2& v) {
        x ^= v.x;
        y ^= v.y;
        return *this;
    }  // Bitwise XOR Assignment
    Vector2& operator^=(const T& s) {
        x ^= s;
        y ^= s;
        return *this;
    }  // Bitwise XOR Assignment

    Vector2& operator<<=(const Vector2& v) {
        x <<= v.x;
        y <<= v.y;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector2& operator<<=(const T& s) {
        x <<= s;
        y <<= s;
        return *this;
    }  // Bitwise Shift Left Assignment

    Vector2& operator>>=(const Vector2& v) {
        x >>= v.x;
        y >>= v.y;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector2& operator>>=(const T& s) {
        x >>= s;
        y >>= s;
        return *this;
    }  // Bitwise Shift Right Assignment

    // Stream Operators

    friend std::ostream& operator<<(std::ostream& os, const Vector2& v) {
        os << "(" << v.x << ", " << v.y << ")";
        return os;
    }  // Output
    friend std::istream& operator>>(std::istream& is, Vector2& v) {
        is >> v.x >> v.y;
        return is;
    }  // Input

    // Conversion Operators

    operator T() const { return x; }                                // Convert to T
    operator Vector3<T>() const { return Vector3<T>(x, y, 0); }     // Convert to Vector3
    operator Vector4<T>() const { return Vector4<T>(x, y, 0, 0); }  // Convert to Vector4

    // Access Operators

    T& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        throw std::out_of_range("Index out of range");
    }  // Access by index

    // Destructor
    ~Vector2() {}

    // Geometric Functions

    /// @brief Normalize the vector (Retain direction, set magnitude to 1)
    /// @return Vector2<T> Normalized vector
    Vector2 normalize() {
        T mag = this % 1;  // Magnitude
        return Vector2(x / mag, y / mag);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XY plane, yaw
    /// @param yaw Rotation angle in radians
    /// @return  Vector2<T> Rotated vector
    Vector2 rotate(T yaw) {
        T x = this.x * cos(yaw) - this.y * sin(yaw);
        T y = this.x * sin(yaw) + this.y * cos(yaw);
        return Vector2(x, y);
    }

    /// @brief Convert the cartesian vector to polar coordinates
    /// @return Vector2<T> Polar coordinates (r, theta)
    Vector2 toPolar() {
        T r = this % 1;
        T theta = atan2(this.y, this.x);
        return Vector2(r, theta);
    }

    /// @brief Convert the polar vector to cartesian coordinates
    /// @return Vector2<T> Cartesian coordinates (x, y)
    Vector2 toCartesian() {
        T x = this.x * cos(this.y);
        T y = this.x * sin(this.y);
        return Vector2(x, y);
    }

    /// @brief Reflect the vector about a normal vector
    /// @param normal Vector to reflect about
    /// @return Vector2<T> Reflected vector
    Vector2 reflect(Vector2 normal) { return *this - normal * 2 * this.dot(normal); }

    /// @brief Project the vector onto another vector
    /// @param v Vector to project onto
    /// @return Vector2<T> Projected vector
    Vector2 project(Vector2 v) { return v * (this.dot(v) / v % 1); }

    /// @brief Reject the vector from another vector
    /// @param v Vector to reject from
    /// @return Vector2<T> Rejected vector
    Vector2 reject(Vector2 v) { return *this - project(v); }

    /// @brief Get the angle between two vectors
    /// @param v Vector to find the angle between
    /// @return T Angle between the vectors in radians
    T angle(Vector2 v) { return acos(this.dot(v) / (this % 1 * v % 1)); }

    /// @brief Get the distance between two vectors
    /// @param v Vector to find the distance to
    /// @return T Distance between the vectors
    T distance(Vector2 v) { return (*this - v) % 1; }

    /// @brief Get the midpoint between two vectors
    /// @param v Vector to find the midpoint with
    /// @return Vector2<T> Midpoint between the vectors
    Vector2 midpoint(Vector2 v) { return (*this + v) / 2; }

    /// @brief Get the normal vector to the vector
    /// @return Vector2<T> Normal vector
    Vector2 normal() { return Vector2(-y, x); }

    /// @brief Get the area of the parallelogram formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the parallelogram
    T area(Vector2 v) { return abs(this.x * v.y - this.y * v.x); }

    /// @brief Get the area of the triangle formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the triangle
    T triangleArea(Vector2 v) { return area(v) / 2; }
};

// 3D Vector
template <typename T>
struct Vector3 {
    T x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(T s) : x(s), y(s), z(s) {}
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

    // Overloaded Operators
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }  // Addition of two vector3s
    Vector3 operator+(const Vector2<T>& v) const {
        return Vector3(x + v.x, y + v.y, z);
    }  // Addition of a vector2 and a vector3
    Vector3 operator+(const T& s) const {
        return Vector3(x + s, y + s, z + s);
    }  // Addition of a scalar

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }  // Subtraction of two vector3s
    Vector3 operator-(const Vector2<T>& v) const {
        return Vector3(x - v.x, y - v.y, z);
    }  // Subtraction of a vector2 and a vector3
    Vector3 operator-(const T& s) const {
        return Vector3(x - s, y - s, z - s);
    }  // Subtraction of a scalar

    Vector3 operator*(const Vector3& v) const {
        return Vector3(x * v.x, y * v.y, z * v.z);
    }  // Cross multiplication of two vector3s
    Vector3 operator*(const T& s) const {
        return Vector3(x * s, y * s, z * s);
    }  // Cross multiplication of a scalar

    // Dot Product
    T dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }  // Dot product of two vector3s

    Vector3 operator/(const Vector3& v) const {
        return Vector3(x / v.x, y / v.y, z / v.z);
    }  // Division of two vector3s
    Vector3 operator/(const T& s) const {
        return Vector3(x / s, y / s, z / s);
    }  // Division of a scalar

    T operator%(const Vector3& v) const {
        return sqrt(x * x + y * y + z * z);
    }  // Magnitude of the vector
    T operator%(const T& s) const {
        return sqrt(x * x + y * y + z * z);
    }  // Magnitude of the vector

    Vector3 operator++() {
        return Vector3(++x, ++y, ++z);
    }  // Prefix Increment (why does this even exist?)
    Vector3 operator++(int) { return Vector3(x++, y++, z++); }  // Postfix Increment

    Vector3 operator--() { return Vector3(--x, --y, --z); }     // Prefix Decrement
    Vector3 operator--(int) { return Vector3(x--, y--, z--); }  // Postfix Decrement

    // Unary Operators

    Vector3 operator+() const { return Vector3(abs(x), abs(y), abs(z)); }  // Unary Plus
    Vector3 operator-() const { return Vector3(-x, -y, -z); }              // Unary Minus

    // Comparison Operators

    bool operator==(const Vector3& v) const {
        return x == v.x && y == v.y && z == v.z;
    }  // Equality
    bool operator==(const Vector2<T>& v) const {
        return x == v.x && y == v.y && z == 0;
    }  // Equality
    bool operator==(const T& s) const { return x == s && y == s && z == s; }  // Equality

    bool operator!=(const Vector3& v) const {
        return x != v.x || y != v.y || z != v.z;
    }  // Inequality
    bool operator!=(const Vector2<T>& v) const {
        return x != v.x || y != v.y || z != 0;
    }  // Inequality
    bool operator!=(const T& s) const { return x != s || y != s || z != s; }  // Inequality

    bool operator>(const Vector3& v) const {
        return x > v.x && y > v.y && z > v.z;
    }  // Greater Than
    bool operator>(const Vector2<T>& v) const {
        return x > v.x && y > v.y && z > 0;
    }  // Greater Than
    bool operator>(const T& s) const { return x > s && y > s && z > s; }  // Greater Than

    bool operator>=(const Vector3& v) const {
        return x >= v.x && y >= v.y && z >= v.z;
    }  // Greater Than or Equal To
    bool operator>=(const Vector2<T>& v) const {
        return x >= v.x && y >= v.y && z >= 0;
    }  // Greater Than or Equal To
    bool operator>=(const T& s) const {
        return x >= s && y >= s && z >= s;
    }  // Greater Than or Equal To

    bool operator<(const Vector3& v) const { return x < v.x && y < v.y && z < v.z; }   // Less Than
    bool operator<(const Vector2<T>& v) const { return x < v.x && y < v.y && z < 0; }  // Less Than
    bool operator<(const T& s) const { return x < s && y < s && z < s; }               // Less Than

    bool operator<=(const Vector3& v) const {
        return x <= v.x && y <= v.y && z <= v.z;
    }  // Less Than or Equal To
    bool operator<=(const Vector2<T>& v) const {
        return x <= v.x && y <= v.y && z <= 0;
    }  // Less Than or Equal To
    bool operator<=(const T& s) const {
        return x <= s && y <= s && z <= s;
    }  // Less Than or Equal To

    // Logical Operators

    bool operator!() const { return !x && !y && !z; }  // Logical NOT

    // Bitwise Operators

    Vector3 operator~() const { return Vector3(~x, ~y, ~z); }  // Bitwise NOT

    Vector3 operator&(const Vector3& v) const {
        return Vector3(x & v.x, y & v.y, z & v.z);
    }  // Bitwise AND
    Vector3 operator&(const Vector2<T>& v) const {
        return Vector3(x & v.x, y & v.y, z);
    }  // Bitwise AND
    Vector3 operator&(const T& s) const { return Vector3(x & s, y & s, z & s); }  // Bitwise AND

    Vector3 operator|(const Vector3& v) const {
        return Vector3(x | v.x, y | v.y, z | v.z);
    }  // Bitwise OR
    Vector3 operator|(const Vector2<T>& v) const {
        return Vector3(x | v.x, y | v.y, z);
    }  // Bitwise OR
    Vector3 operator|(const T& s) const { return Vector3(x | s, y | s, z | s); }  // Bitwise OR

    Vector3 operator^(const Vector3& v) const {
        return Vector3(x ^ v.x, y ^ v.y, z ^ v.z);
    }  // Bitwise XOR
    Vector3 operator^(const Vector2<T>& v) const {
        return Vector3(x ^ v.x, y ^ v.y, z);
    }  // Bitwise XOR
    Vector3 operator^(const T& s) const { return Vector3(x ^ s, y ^ s, z ^ s); }  // Bitwise XOR

    Vector3 operator<<(const Vector3& v) const {
        return Vector3(x << v.x, y << v.y, z << v.z);
    }  // Bitwise Shift Left
    Vector3 operator<<(const Vector2<T>& v) const {
        return Vector3(x << v.x, y << v.y, z);
    }  // Bitwise Shift Left
    Vector3 operator<<(const T& s) const {
        return Vector3(x << s, y << s, z << s);
    }  // Bitwise Shift Left

    Vector3 operator>>(const Vector3& v) const {
        return Vector3(x >> v.x, y >> v.y, z >> v.z);
    }  // Bitwise Shift Right
    Vector3 operator>>(const Vector2<T>& v) const {
        return Vector3(x >> v.x, y >> v.y, z);
    }  // Bitwise Shift Right
    Vector3 operator>>(const T& s) const {
        return Vector3(x >> s, y >> s, z >> s);
    }  // Bitwise Shift Right

    // Assignment Operators

    Vector3& operator=(const Vector3& v) {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }  // Assignment

    Vector3& operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }  // Addition Assignment
    Vector3& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }  // Addition Assignment
    Vector3& operator+=(const T& s) {
        x += s;
        y += s;
        z += s;
        return *this;
    }  // Addition Assignment

    Vector3& operator-=(const Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }  // Subtraction Assignment
    Vector3& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }  // Subtraction Assignment
    Vector3& operator-=(const T& s) {
        x -= s;
        y -= s;
        z -= s;
        return *this;
    }  // Subtraction Assignment

    Vector3& operator*=(const Vector3& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }  // Cross Multiplication Assignment
    Vector3& operator*=(const Vector2<T>& v) {
        x *= v.x;
        y *= v.y;
        return *this;
    }  // Cross Multiplication Assignment
    Vector3& operator*=(const T& s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }  // Cross Multiplication Assignment

    Vector3& operator/=(const Vector3& v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }  // Division Assignment
    Vector3& operator/=(const Vector2<T>& v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }  // Division Assignment
    Vector3& operator/=(const T& s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }  // Division Assignment

    Vector3& operator&=(const Vector3& v) {
        x &= v.x;
        y &= v.y;
        z &= v.z;
        return *this;
    }  // Bitwise AND Assignment
    Vector3& operator&=(const Vector2<T>& v) {
        x &= v.x;
        y &= v.y;
        return *this;
    }  // Bitwise AND Assignment
    Vector3& operator&=(const T& s) {
        x &= s;
        y &= s;
        z &= s;
        return *this;
    }  // Bitwise AND Assignment

    Vector3& operator|=(const Vector3& v) {
        x |= v.x;
        y |= v.y;
        z |= v.z;
        return *this;
    }  // Bitwise OR Assignment
    Vector3& operator|=(const Vector2<T>& v) {
        x |= v.x;
        y |= v.y;
        return *this;
    }  // Bitwise OR Assignment
    Vector3& operator|=(const T& s) {
        x |= s;
        y |= s;
        z |= s;
        return *this;
    }  // Bitwise OR Assignment

    Vector3& operator^=(const Vector3& v) {
        x ^= v.x;
        y ^= v.y;
        z ^= v.z;
        return *this;
    }  // Bitwise XOR Assignment
    Vector3& operator^=(const Vector2<T>& v) {
        x ^= v.x;
        y ^= v.y;
        return *this;
    }  // Bitwise XOR Assignment
    Vector3& operator^=(const T& s) {
        x ^= s;
        y ^= s;
        z ^= s;
        return *this;
    }  // Bitwise XOR Assignment

    Vector3& operator<<=(const Vector3& v) {
        x <<= v.x;
        y <<= v.y;
        z <<= v.z;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector3& operator<<=(const Vector2<T>& v) {
        x <<= v.x;
        y <<= v.y;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector3& operator<<=(const T& s) {
        x <<= s;
        y <<= s;
        z <<= s;
        return *this;
    }  // Bitwise Shift Left Assignment

    Vector3& operator>>=(const Vector3& v) {
        x >>= v.x;
        y >>= v.y;
        z >>= v.z;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector3& operator>>=(const Vector2<T>& v) {
        x >>= v.x;
        y >>= v.y;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector3& operator>>=(const T& s) {
        x >>= s;
        y >>= s;
        z >>= s;
        return *this;
    }  // Bitwise Shift Right Assignment

    // Stream Operators

    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }  // Output
    friend std::istream& operator>>(std::istream& is, Vector3& v) {
        is >> v.x >> v.y >> v.z;
        return is;
    }  // Input

    // Conversion Operators

    operator T() const { return x; }                                // Convert to T
    operator Vector2<T>() const { return Vector2<T>(x, y); }        // Convert to Vector2
    operator Vector4<T>() const { return Vector4<T>(x, y, z, 0); }  // Convert to Vector4

    // Access Operators

    T& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        if (i == 2) return z;
        throw std::out_of_range("Index out of range");
    }  // Access by index
    Vector2<T> xy() { return Vector2<T>(x, y); }  // Access the xy components
    Vector2<T> xz() { return Vector2<T>(x, z); }  // Access the xz components
    Vector2<T> yz() { return Vector2<T>(y, z); }  // Access the yz components

    // Destructor
    ~Vector3() {}

    // Geometric Functions

    /// @brief Normalize the vector (Retain direction, set magnitude to 1)
    /// @return Vector3<T> Normalized vector
    Vector3 normalize() {
        T mag = this % 1;  // Magnitude
        return Vector3(x / mag, y / mag, z / mag);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XY plane, yaw
    /// @param yaw Rotation angle in radians
    /// @return  Vector3<T> Rotated vector
    Vector3 rotateZ(T yaw) {
        T x = this.x * cos(yaw) - this.y * sin(yaw);
        T y = this.x * sin(yaw) + this.y * cos(yaw);
        return Vector3(x, y, z);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XZ plane, pitch
    /// @param pitch Rotation angle in radians
    /// @return  Vector3<T> Rotated vector
    Vector3 rotateY(T pitch) {
        T x = this.x * cos(pitch) - this.z * sin(pitch);
        T z = this.x * sin(pitch) + this.z * cos(pitch);
        return Vector3(x, y, z);
    }

    /// @brief Rotate the vector by a given angle about the origin in the YZ plane, roll
    /// @param roll Rotation angle in radians
    /// @return  Vector3<T> Rotated vector
    Vector3 rotateX(T roll) {
        T y = this.y * cos(roll) - this.z * sin(roll);
        T z = this.y * sin(roll) + this.z * cos(roll);
        return Vector3(x, y, z);
    }

    /// @brief Convert the cartesian vector to spherical coordinates
    /// @return Vector3<T> Spherical coordinates (r, theta, phi)
    Vector3 toSpherical() {
        T r = this % 1;
        T theta = atan2(this.y, this.x);
        T phi = acos(this.z / r);
        return Vector3(r, theta, phi);
    }

    /// @brief Convert the spherical vector to cartesian coordinates
    /// @return Vector3<T> Cartesian coordinates (x, y, z)
    Vector3 toCartesian() {
        T x = this.x * cos(this.y) * sin(this.z);
        T y = this.x * sin(this.y) * sin(this.z);
        T z = this.x * cos(this.z);
        return Vector3(x, y, z);
    }

    /// @brief Reflect the vector about a normal vector
    /// @param normal Vector to reflect about
    /// @return Vector3<T> Reflected vector
    Vector3 reflect(Vector3 normal) { return *this - normal * 2 * this.dot(normal); }

    /// @brief Project the vector onto another vector
    /// @param v Vector to project onto
    /// @return Vector3<T> Projected vector
    Vector3 project(Vector3 v) { return v * (this.dot(v) / v % 1); }

    /// @brief Reject the vector from another vector
    /// @param v Vector to reject from
    /// @return Vector3<T> Rejected vector
    Vector3 reject(Vector3 v) { return *this - project(v); }

    /// @brief Get the angle between two vectors
    /// @param v Vector to find the angle between
    /// @return T Angle between the vectors in radians
    T angle(Vector3 v) { return acos(this.dot(v) / (this % 1 * v % 1)); }

    /// @brief Get the distance between two vectors
    /// @param v Vector to find the distance to
    /// @return T Distance between the vectors
    T distance(Vector3 v) { return (*this - v) % 1; }

    /// @brief Get the midpoint between two vectors
    /// @param v Vector to find the midpoint with
    /// @return Vector3<T> Midpoint between the vectors
    Vector3 midpoint(Vector3 v) { return (*this + v) / 2; }

    /// @brief Get the normal vector to the vector
    /// @return Vector3<T> Normal vector
    Vector3 normal() { return Vector3(-y, x, 0); }

    /// @brief Get the area of the parallelogram formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the parallelogram
    T area(Vector3 v) { return abs(this.x * v.y - this.y * v.x); }

    /// @brief Get the area of the triangle formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the triangle
    T triangleArea(Vector3 v) { return area(v) / 2; }
};

// 4D Vector

template <typename T>
struct Vector4 {
    T x, y, z, a;

    // Constructors
    Vector4() : x(0), y(0), z(0), a(0) {}
    Vector4(T s) : x(s), y(s), z(s), a(s) {}
    Vector4(T x, T y, T z, T a) : x(x), y(y), z(z), a(a) {}

    // Overloaded Operators
    Vector4 operator+(const Vector4& v) const {
        return Vector4(x + v.x, y + v.y, z + v.z, a + v.a);
    }  // Addition of two vector4s
    Vector4 operator+(const Vector3<T>& v) const {
        return Vector4(x + v.x, y + v.y, z + v.z, a);
    }  // Addition of a vector3 and a vector4
    Vector4 operator+(const Vector2<T>& v) const {
        return Vector4(x + v.x, y + v.y, z, a);
    }  // Addition of a vector2 and a vector4
    Vector4 operator+(const T& s) const {
        return Vector4(x + s, y + s, z + s, a + s);
    }  // Addition of a scalar

    Vector4 operator-(const Vector4& v) const {
        return Vector4(x - v.x, y - v.y, z - v.z, a - v.a);
    }  // Subtraction of two vector4s
    Vector4 operator-(const Vector3<T>& v) const {
        return Vector4(x - v.x, y - v.y, z - v.z, a);
    }  // Subtraction of a vector3 and a vector4
    Vector4 operator-(const Vector2<T>& v) const {
        return Vector4(x - v.x, y - v.y, z, a);
    }  // Subtraction of a vector2 and a vector4
    Vector4 operator-(const T& s) const {
        return Vector4(x - s, y - s, z - s, a - s);
    }  // Subtraction of a scalar

    Vector4 operator*(const Vector4& v) const {
        return Vector4(x * v.x, y * v.y, z * v.z, a * v.a);
    }  // Cross multiplication of two vector4s
    Vector4 operator*(const Vector3<T>& v) const {
        return Vector4(x * v.x, y * v.y, z * v.z, a);
    }  // Cross multiplication of a vector3 and a vector4
    Vector4 operator*(const Vector2<T>& v) const {
        return Vector4(x * v.x, y * v.y, z, a);
    }  // Cross multiplication of a vector2 and a vector4
    Vector4 operator*(const T& s) const {
        return Vector4(x * s, y * s, z * s, a * s);
    }  // Cross multiplication of a scalar

    // Dot Product
    T dot(const Vector4& v) const {
        return x * v.x + y * v.y + z * v.z + a * v.a;
    }  // Dot product of two vector4s

    Vector4 operator/(const Vector4& v) const {
        return Vector4(x / v.x, y / v.y, z / v.z, a / v.a);
    }  // Division of two vector4s
    Vector4 operator/(const Vector3<T>& v) const {
        return Vector4(x / v.x, y / v.y, z / v.z, a);
    }  // Division of a vector3 and a vector4
    Vector4 operator/(const Vector2<T>& v) const {
        return Vector4(x / v.x, y / v.y, z, a);
    }  // Division of a vector2 and a vector4
    Vector4 operator/(const T& s) const {
        return Vector4(x / s, y / s, z / s, a / s);
    }  // Division of a scalar

    T operator%(const Vector4& v) const {
        return sqrt(x * x + y * y + z * z + a * a);
    }  // Magnitude of the vector
    T operator%(const T& s) const {
        return sqrt(x * x + y * y + z * z + a * a);
    }  // Magnitude of the vector

    Vector4 operator++() {
        return Vector4(++x, ++y, ++z, ++a);
    }  // Prefix Increment (why does this even exist?)
    Vector4 operator++(int) { return Vector4(x++, y++, z++, a++); }  // Postfix Increment

    Vector4 operator--() { return Vector4(--x, --y, --z, --a); }     // Prefix Decrement
    Vector4 operator--(int) { return Vector4(x--, y--, z--, a--); }  // Postfix Decrement

    // Unary Operators

    Vector4 operator+() const { return Vector4(abs(x), abs(y), abs(z), abs(a)); }  // Unary Plus
    Vector4 operator-() const { return Vector4(-x, -y, -z, -a); }                  // Unary Minus

    // Comparison Operators

    bool operator==(const Vector4& v) const {
        return x == v.x && y == v.y && z == v.z && a == v.a;
    }  // Equality
    bool operator==(const Vector3<T>& v) const {
        return x == v.x && y == v.y && z == v.z && a == 0;
    }  // Equality
    bool operator==(const Vector2<T>& v) const {
        return x == v.x && y == v.y && z == 0 && a == 0;
    }  // Equality
    bool operator==(const T& s) const { return x == s && y == s && z == s && a == s; }  // Equality

    bool operator!=(const Vector4& v) const {
        return x != v.x || y != v.y || z != v.z || a != v.a;
    }  // Inequality
    bool operator!=(const Vector3<T>& v) const {
        return x != v.x || y != v.y || z != v.z || a != 0;
    }  // Inequality
    bool operator!=(const Vector2<T>& v) const {
        return x != v.x || y != v.y || z != 0 || a != 0;
    }  // Inequality
    bool operator!=(const T& s) const {
        return x != s || y != s || z != s || a != s;
    }  // Inequality

    bool operator>(const Vector4& v) const {
        return x > v.x && y > v.y && z > v.z && a > v.a;
    }  // Greater Than
    bool operator>(const Vector3<T>& v) const {
        return x > v.x && y > v.y && z > v.z && a > 0;
    }  // Greater Than
    bool operator>(const Vector2<T>& v) const {
        return x > v.x && y > v.y && z > 0 && a > 0;
    }  // Greater Than
    bool operator>(const T& s) const { return x > s && y > s && z > s && a > s; }  // Greater Than

    bool operator>=(const Vector4& v) const {
        return x >= v.x && y >= v.y && z >= v.z && a >= v.a;
    }  // Greater Than or Equal To
    bool operator>=(const Vector3<T>& v) const {
        return x >= v.x && y >= v.y && z >= v.z && a >= 0;
    }  // Greater Than or Equal To
    bool operator>=(const Vector2<T>& v) const {
        return x >= v.x && y >= v.y && z >= 0 && a >= 0;
    }  // Greater Than or Equal To
    bool operator>=(const T& s) const {
        return x >= s && y >= s && z >= s && a >= s;
    }  // Greater Than or Equal To

    bool operator<(const Vector4& v) const {
        return x < v.x && y < v.y && z < v.z && a < v.a;
    }  // Less Than
    bool operator<(const Vector3<T>& v) const {
        return x < v.x && y < v.y && z < v.z && a < 0;
    }  // Less Than
    bool operator<(const Vector2<T>& v) const {
        return x < v.x && y < v.y && z < 0 && a < 0;
    }  // Less Than
    bool operator<(const T& s) const { return x < s && y < s && z < s && a < s; }  // Less Than

    bool operator<=(const Vector4& v) const {
        return x <= v.x && y <= v.y && z <= v.z && a <= v.a;
    }  // Less Than or Equal To
    bool operator<=(const Vector3<T>& v) const {
        return x <= v.x && y <= v.y && z <= v.z && a <= 0;
    }  // Less Than or Equal To
    bool operator<=(const Vector2<T>& v) const {
        return x <= v.x && y <= v.y && z <= 0 && a <= 0;
    }  // Less Than or Equal To
    bool operator<=(const T& s) const {
        return x <= s && y <= s && z <= s && a <= s;
    }  // Less Than or Equal To

    // Logical Operators

    bool operator!() const { return !x && !y && !z && !a; }  // Logical NOT

    // Bitwise Operators

    Vector4 operator~() const { return Vector4(~x, ~y, ~z, ~a); }  // Bitwise NOT

    Vector4 operator&(const Vector4& v) const {
        return Vector4(x & v.x, y & v.y, z & v.z, a & v.a);
    }  // Bitwise AND
    Vector4 operator&(const Vector3<T>& v) const {
        return Vector4(x & v.x, y & v.y, z & v.z, a);
    }  // Bitwise AND
    Vector4 operator&(const Vector2<T>& v) const {
        return Vector4(x & v.x, y & v.y, z, a);
    }  // Bitwise AND
    Vector4 operator&(const T& s) const {
        return Vector4(x & s, y & s, z & s, a & s);
    }  // Bitwise AND

    Vector4 operator|(const Vector4& v) const {
        return Vector4(x | v.x, y | v.y, z | v.z, a | v.a);
    }  // Bitwise OR
    Vector4 operator|(const Vector3<T>& v) const {
        return Vector4(x | v.x, y | v.y, z | v.z, a);
    }  // Bitwise OR
    Vector4 operator|(const Vector2<T>& v) const {
        return Vector4(x | v.x, y | v.y, z, a);
    }  // Bitwise OR
    Vector4 operator|(const T& s) const {
        return Vector4(x | s, y | s, z | s, a | s);
    }  // Bitwise OR

    Vector4 operator^(const Vector4& v) const {
        return Vector4(x ^ v.x, y ^ v.y, z ^ v.z, a ^ v.a);
    }  // Bitwise XOR
    Vector4 operator^(const Vector3<T>& v) const {
        return Vector4(x ^ v.x, y ^ v.y, z ^ v.z, a);
    }  // Bitwise XOR
    Vector4 operator^(const Vector2<T>& v) const {
        return Vector4(x ^ v.x, y ^ v.y, z, a);
    }  // Bitwise XOR
    Vector4 operator^(const T& s) const {
        return Vector4(x ^ s, y ^ s, z ^ s, a ^ s);
    }  // Bitwise XOR

    Vector4 operator<<(const Vector4& v) const {
        return Vector4(x << v.x, y << v.y, z << v.z, a << v.a);
    }  // Bitwise Shift Left
    Vector4 operator<<(const Vector3<T>& v) const {
        return Vector4(x << v.x, y << v.y, z << v.z, a);
    }  // Bitwise Shift Left
    Vector4 operator<<(const Vector2<T>& v) const {
        return Vector4(x << v.x, y << v.y, z, a);
    }  // Bitwise Shift Left
    Vector4 operator<<(const T& s) const {
        return Vector4(x << s, y << s, z << s, a << s);
    }  // Bitwise Shift Left

    Vector4 operator>>(const Vector4& v) const {
        return Vector4(x >> v.x, y >> v.y, z >> v.z, a >> v.a);
    }  // Bitwise Shift Right
    Vector4 operator>>(const Vector3<T>& v) const {
        return Vector4(x >> v.x, y >> v.y, z >> v.z, a);
    }  // Bitwise Shift Right
    Vector4 operator>>(const Vector2<T>& v) const {
        return Vector4(x >> v.x, y >> v.y, z, a);
    }  // Bitwise Shift Right
    Vector4 operator>>(const T& s) const {
        return Vector4(x >> s, y >> s, z >> s, a >> s);
    }  // Bitwise Shift Right

    // Assignment Operators

    Vector4& operator=(const Vector4& v) {
        x = v.x;
        y = v.y;
        z = v.z;
        a = v.a;
        return *this;
    }  // Assignment

    Vector4& operator+=(const Vector4& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        a += v.a;
        return *this;
    }  // Addition Assignment
    Vector4& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }  // Addition Assignment
    Vector4& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }  // Addition Assignment
    Vector4& operator+=(const T& s) {
        x += s;
        y += s;
        z += s;
        a += s;
        return *this;
    }  // Addition Assignment

    Vector4& operator-=(const Vector4& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        a -= v.a;
        return *this;
    }  // Subtraction Assignment
    Vector4& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }  // Subtraction Assignment
    Vector4& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }  // Subtraction Assignment
    Vector4& operator-=(const T& s) {
        x -= s;
        y -= s;
        z -= s;
        a -= s;
        return *this;
    }  // Subtraction Assignment

    Vector4& operator*=(const Vector4& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        a *= v.a;
        return *this;
    }  // Cross Multiplication Assignment
    Vector4& operator*=(const Vector3<T>& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }  // Cross Multiplication Assignment
    Vector4& operator*=(const Vector2<T>& v) {
        x *= v.x;
        y *= v.y;
        return *this;
    }  // Cross Multiplication Assignment
    Vector4& operator*=(const T& s) {
        x *= s;
        y *= s;
        z *= s;
        a *= s;
        return *this;
    }  // Cross Multiplication Assignment

    Vector4& operator/=(const Vector4& v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        a /= v.a;
        return *this;
    }  // Division Assignment
    Vector4& operator/=(const Vector3<T>& v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }  // Division Assignment
    Vector4& operator/=(const Vector2<T>& v) {
        x /= v.x;
        y /= v.y;
        return *this;
    }  // Division Assignment
    Vector4& operator/=(const T& s) {
        x /= s;
        y /= s;
        z /= s;
        a /= s;
        return *this;
    }  // Division Assignment

    Vector4& operator&=(const Vector4& v) {
        x &= v.x;
        y &= v.y;
        z &= v.z;
        a &= v.a;
        return *this;
    }  // Bitwise AND Assignment
    Vector4& operator&=(const Vector3<T>& v) {
        x &= v.x;
        y &= v.y;
        z &= v.z;
        return *this;
    }  // Bitwise AND Assignment
    Vector4& operator&=(const Vector2<T>& v) {
        x &= v.x;
        y &= v.y;
        return *this;
    }  // Bitwise AND Assignment
    Vector4& operator&=(const T& s) {
        x &= s;
        y &= s;
        z &= s;
        a &= s;
        return *this;
    }  // Bitwise AND Assignment

    Vector4& operator|=(const Vector4& v) {
        x |= v.x;
        y |= v.y;
        z |= v.z;
        a |= v.a;
        return *this;
    }  // Bitwise OR Assignment
    Vector4& operator|=(const Vector3<T>& v) {
        x |= v.x;
        y |= v.y;
        z |= v.z;
        return *this;
    }  // Bitwise OR Assignment
    Vector4& operator|=(const Vector2<T>& v) {
        x |= v.x;
        y |= v.y;
        return *this;
    }  // Bitwise OR Assignment
    Vector4& operator|=(const T& s) {
        x |= s;
        y |= s;
        z |= s;
        a |= s;
        return *this;
    }  // Bitwise OR Assignment

    Vector4& operator^=(const Vector4& v) {
        x ^= v.x;
        y ^= v.y;
        z ^= v.z;
        a ^= v.a;
        return *this;
    }  // Bitwise XOR Assignment
    Vector4& operator^=(const Vector3<T>& v) {
        x ^= v.x;
        y ^= v.y;
        z ^= v.z;
        return *this;
    }  // Bitwise XOR Assignment
    Vector4& operator^=(const Vector2<T>& v) {
        x ^= v.x;
        y ^= v.y;
        return *this;
    }  // Bitwise XOR Assignment
    Vector4& operator^=(const T& s) {
        x ^= s;
        y ^= s;
        z ^= s;
        a ^= s;
        return *this;
    }  // Bitwise XOR Assignment

    Vector4& operator<<=(const Vector4& v) {
        x <<= v.x;
        y <<= v.y;
        z <<= v.z;
        a <<= v.a;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector4& operator<<=(const Vector3<T>& v) {
        x <<= v.x;
        y <<= v.y;
        z <<= v.z;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector4& operator<<=(const Vector2<T>& v) {
        x <<= v.x;
        y <<= v.y;
        return *this;
    }  // Bitwise Shift Left Assignment
    Vector4& operator<<=(const T& s) {
        x <<= s;
        y <<= s;
        z <<= s;
        a <<= s;
        return *this;
    }  // Bitwise Shift Left Assignment

    Vector4& operator>>=(const Vector4& v) {
        x >>= v.x;
        y >>= v.y;
        z >>= v.z;
        a >>= v.a;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector4& operator>>=(const Vector3<T>& v) {
        x >>= v.x;
        y >>= v.y;
        z >>= v.z;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector4& operator>>=(const Vector2<T>& v) {
        x >>= v.x;
        y >>= v.y;
        return *this;
    }  // Bitwise Shift Right Assignment
    Vector4& operator>>=(const T& s) {
        x >>= s;
        y >>= s;
        z >>= s;
        a >>= s;
        return *this;
    }  // Bitwise Shift Right Assignment

    // Stream Operators

    friend std::ostream& operator<<(std::ostream& os, const Vector4& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.a << ")";
        return os;
    }  // Output
    friend std::istream& operator>>(std::istream& is, Vector4& v) {
        is >> v.x >> v.y >> v.z >> v.a;
        return is;
    }  // Input

    // Conversion Operators

    operator T() const { return x; }                             // Convert to T
    operator Vector2<T>() const { return Vector2<T>(x, y); }     // Convert to Vector2
    operator Vector3<T>() const { return Vector3<T>(x, y, z); }  // Convert to Vector3

    // Access Operators

    T& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        if (i == 2) return z;
        if (i == 3) return a;
        throw std::out_of_range("Index out of range");
    }  // Access by index
    Vector2<T> xy() { return Vector2<T>(x, y); }  // Access the xy components
    Vector2<T> xz() { return Vector2<T>(x, z); }  // Access the xz components
    Vector2<T> xa() { return Vector2<T>(x, a); }  // Access the xa components
    Vector2<T> yz() { return Vector2<T>(y, z); }  // Access the yz components
    Vector2<T> ya() { return Vector2<T>(y, a); }  // Access the ya components
    Vector2<T> za() { return Vector2<T>(z, a); }  // Access the za components

    Vector3<T> xyz() { return Vector3<T>(x, y, z); }  // Access the xyz components
    Vector3<T> xya() { return Vector3<T>(x, y, a); }  // Access the xya components
    Vector3<T> xza() { return Vector3<T>(x, z, a); }  // Access the xza components
    Vector3<T> yza() { return Vector3<T>(y, z, a); }  // Access the yza components

    // Destructor
    ~Vector4() {}

    // Geometric Functions

    /// @brief Normalize the vector (Retain direction, set magnitude to 1)
    /// @return Vector4<T> Normalized vector
    Vector4 normalize() {
        T mag = this % 1;  // Magnitude
        return Vector4(x / mag, y / mag, z / mag, a / mag);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XY plane, yaw
    /// @param yaw Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateZ(T yaw) {
        T x = this.x * cos(yaw) - this.y * sin(yaw);
        T y = this.x * sin(yaw) + this.y * cos(yaw);
        return Vector4(x, y, z, a);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XZ plane, pitch
    /// @param pitch Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateY(T pitch) {
        T x = this.x * cos(pitch) - this.z * sin(pitch);
        T z = this.x * sin(pitch) + this.z * cos(pitch);
        return Vector4(x, y, z, a);
    }

    /// @brief Rotate the vector by a given angle about the origin in the YZ plane, roll
    /// @param roll Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateX(T roll) {
        T y = this.y * cos(roll) - this.z * sin(roll);
        T z = this.y * sin(roll) + this.z * cos(roll);
        return Vector4(x, y, z, a);
    }

    /// @brief Rotate the vector by a given angle about the origin in the XA plane
    /// @param angle Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateW(T angle) {
        T x = this.x * cos(angle) - this.a * sin(angle);
        T a = this.x * sin(angle) + this.a * cos(angle);
        return Vector4(x, y, z, a);
    }

    /// @brief Rotate the vector by a given angle about the origin in the YA plane
    /// @param angle Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateV(T angle) {
        T y = this.y * cos(angle) - this.a * sin(angle);
        T a = this.y * sin(angle) + this.a * cos(angle);
        return Vector4(x, y, z, a);
    }

    /// @brief Rotate the vector by a given angle about the origin in the ZA plane
    /// @param angle Rotation angle in radians
    /// @return  Vector4<T> Rotated vector
    Vector4 rotateU(T angle) {
        T z = this.z * cos(angle) - this.a * sin(angle);
        T a = this.z * sin(angle) + this.a * cos(angle);
        return Vector4(x, y, z, a);
    }

    /// @brief Convert the cartesian vector to spherical coordinates
    /// @return Vector4<T> Spherical coordinates (r, theta, phi, psi)
    Vector4 toSpherical() {
        T r = this % 1;
        T theta = atan2(this.y, this.x);
        T phi = acos(this.z / r);
        T psi = atan2(this.a, this.z);
        return Vector4(r, theta, phi, psi);
    }

    /// @brief Convert the spherical vector to cartesian coordinates
    /// @return Vector4<T> Cartesian coordinates (x, y, z, a)
    Vector4 toCartesian() {
        T x = this.x * cos(this.y) * sin(this.z) * cos(this.a);
        T y = this.x * sin(this.y) * sin(this.z) * cos(this.a);
        T z = this.x * cos(this.z) * cos(this.a);
        T a = this.x * sin(this.a);
        return Vector4(x, y, z, a);
    }

    /// @brief Reflect the vector about a normal vector
    /// @param normal Vector to reflect about
    /// @return Vector4<T> Reflected vector
    Vector4 reflect(Vector4 normal) { return *this - normal * 2 * this.dot(normal); }

    /// @brief Project the vector onto another vector
    /// @param v Vector to project onto
    /// @return Vector4<T> Projected vector
    Vector4 project(Vector4 v) { return v * (this.dot(v) / v % 1); }

    /// @brief Reject the vector from another vector
    /// @param v Vector to reject from
    /// @return Vector4<T> Rejected vector
    Vector4 reject(Vector4 v) { return *this - project(v); }

    /// @brief Get the angle between two vectors
    /// @param v Vector to find the angle between
    /// @return T Angle between the vectors in radians
    T angle(Vector4 v) { return acos(this.dot(v) / (this % 1 * v % 1)); }

    /// @brief Get the distance between two vectors
    /// @param v Vector to find the distance to
    /// @return T Distance between the vectors
    T distance(Vector4 v) { return (*this - v) % 1; }

    /// @brief Get the midpoint between two vectors
    /// @param v Vector to find the midpoint with
    /// @return Vector4<T> Midpoint between the vectors
    Vector4 midpoint(Vector4 v) { return (*this + v) / 2; }

    /// @brief Get the normal vector to the vector
    /// @return Vector4<T> Normal vector
    Vector4 normal() { return Vector4(-y, x, 0, 0); }

    /// @brief Get the area of the parallelogram formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the parallelogram
    T area(Vector4 v) { return abs(this.x * v.y - this.y * v.x); }

    /// @brief Get the area of the triangle formed by two vectors
    /// @param v Vector to find the area with
    /// @return T Area of the triangle
    T triangleArea(Vector4 v) { return area(v) / 2; }

    /// @brief Get the volume of the parallelepiped formed by three vectors
    /// @param v1 First vector
    /// @param v2 Second vector
    /// @return T Volume of the parallelepiped
    T volume(Vector4 v1, Vector4 v2) { return abs(this.dot(v1.cross(v2))); }
};

// Typedefs for more user-friendly names

/*
Auto-generated, see typedefgen.py to regenerate

Add more dimensions or types if needed there
*/

typedef Vector2<char> char2;
typedef Vector2<unsigned char> uchar2;
typedef Vector2<short> short2;
typedef Vector2<unsigned short> ushort2;
typedef Vector2<int> int2;
typedef Vector2<unsigned int> uint2;
typedef Vector2<long> long2;
typedef Vector2<unsigned long> ulong2;
typedef Vector2<long long> long_long2;
typedef Vector2<unsigned long long> ulong_long2;
typedef Vector2<float> float2;
typedef Vector2<double> double2;
typedef Vector2<long double> long_double2;
typedef Vector2<bool> bool2;
typedef Vector2<uint8_t> uint8_t_2;
typedef Vector2<uint16_t> uint16_t_2;
typedef Vector2<uint32_t> uint32_t_2;
typedef Vector2<uint64_t> uint64_t_2;
typedef Vector2<int8_t> int8_t_2;
typedef Vector2<int16_t> int16_t_2;
typedef Vector2<int32_t> int32_t_2;
typedef Vector3<char> char3;
typedef Vector3<unsigned char> uchar3;
typedef Vector3<short> short3;
typedef Vector3<unsigned short> ushort3;
typedef Vector3<int> int3;
typedef Vector3<unsigned int> uint3;
typedef Vector3<long> long3;
typedef Vector3<unsigned long> ulong3;
typedef Vector3<long long> long_long3;
typedef Vector3<unsigned long long> ulong_long3;
typedef Vector3<float> float3;
typedef Vector3<double> double3;
typedef Vector3<long double> long_double3;
typedef Vector3<bool> bool3;
typedef Vector3<uint8_t> uint8_t_3;
typedef Vector3<uint16_t> uint16_t_3;
typedef Vector3<uint32_t> uint32_t_3;
typedef Vector3<uint64_t> uint64_t_3;
typedef Vector3<int8_t> int8_t_3;
typedef Vector3<int16_t> int16_t_3;
typedef Vector3<int32_t> int32_t_3;
typedef Vector4<char> char4;
typedef Vector4<unsigned char> uchar4;
typedef Vector4<short> short4;
typedef Vector4<unsigned short> ushort4;
typedef Vector4<int> int4;
typedef Vector4<unsigned int> uint4;
typedef Vector4<long> long4;
typedef Vector4<unsigned long> ulong4;
typedef Vector4<long long> long_long4;
typedef Vector4<unsigned long long> ulong_long4;
typedef Vector4<float> float4;
typedef Vector4<double> double4;
typedef Vector4<long double> long_double4;
typedef Vector4<bool> bool4;
typedef Vector4<uint8_t> uint8_t_4;
typedef Vector4<uint16_t> uint16_t_4;
typedef Vector4<uint32_t> uint32_t_4;
typedef Vector4<uint64_t> uint64_t_4;
typedef Vector4<int8_t> int8_t_4;
typedef Vector4<int16_t> int16_t_4;
typedef Vector4<int32_t> int32_t_4;

// Suppose I should touch grass now

}  // namespace unity
#endif
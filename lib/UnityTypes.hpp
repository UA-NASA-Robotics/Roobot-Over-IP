#ifndef UNITY_TYPES_H
#define UNITY_TYPES_H

#include <stdint.h>

namespace UnityTypes {

enum class Axis { X, Y, Z, A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P };

template <typename T, unsigned int size>
class SpacialVector {
   private:
    T data[size];
    unsigned int length;

   public:
    SpacialVector() : length(size) {}

    SpacialVector(T *defaultValue) {
        for (int i = 0; i < size; i++) {
            data[i] = defaultValue;
        }
        length = size;
    }

    ~SpacialVector() {}

    unsigned int getLength() { return length; }

    /*----------------- Arithmetic Operators -----------------*/

    SpacialVector<T, size> operator+(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] + scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator+(SpacialVector<T, size2> &other) const {
        static_assert(size2 <= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be <= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size2; i++) {  // Add the two vectors together
            result[i] = data[i] + other[i];
        }

        for (int i = size2; i < size; i++) {  // Copy the remaining elements from the first vector
            result[i] = data[i];
        }

        return result;
    }

    SpacialVector<T, size> operator-(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] - scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator-(SpacialVector<T, size2> &other) const {
        static_assert(size2 <= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be <= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size2; i++) {  // Subtract the two vectors
            result[i] = data[i] - other[i];
        }

        for (int i = size2; i < size; i++) {  // Copy the remaining elements from the first vector
            result[i] = data[i];
        }

        return result;
    }

    SpacialVector<T, size> operator*(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] * scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator*(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {  // Multiply the two vectors
            result[i] = data[i] * other[i];
        }

        return result;
    }

    template <unsigned int size2>
    T dot(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        T result = 0;
        for (int i = 0; i < size; i++) {  // Multiply the two vectors
            result += data[i] * other[i];
        }

        return result;
    }

    SpacialVector<T, size> operator/(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] / scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator/(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {  // Divide the two vectors
            result[i] = data[i] / other[i];
        }

        return result;
    }

    /*----------------- Assignment Operators -----------------*/

    SpacialVector<T, size> &operator=(T scalar) {
        for (int i = 0; i < size; i++) {
            data[i] = scalar;
        }
        return *this;
    }

    SpacialVector<T, size> &operator+=(T scalar) {
        for (int i = 0; i < size; i++) {
            data[i] += scalar;
        }
        return *this;
    }

    template <unsigned int size2>
    SpacialVector<T, size> &operator+=(SpacialVector<T, size2> &other) {
        static_assert(size2 <= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be <= SpacialVector1");

        for (int i = 0; i < size2; i++) {  // Add the two vectors together
            data[i] += other[i];
        }

        return *this;
    }

    SpacialVector<T, size> &operator-=(T scalar) {
        for (int i = 0; i < size; i++) {
            data[i] -= scalar;
        }
        return *this;
    }

    template <unsigned int size2>
    SpacialVector<T, size> &operator-=(SpacialVector<T, size2> &other) {
        static_assert(size2 <= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be <= SpacialVector1");

        for (int i = 0; i < size2; i++) {  // Subtract the two vectors
            data[i] -= other[i];
        }

        return *this;
    }

    SpacialVector<T, size> &operator*=(T scalar) {
        for (int i = 0; i < size; i++) {
            data[i] *= scalar;
        }
        return *this;
    }

    template <unsigned int size2>
    SpacialVector<T, size> &operator*=(SpacialVector<T, size2> &other) {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {  // Multiply the two vectors
            data[i] *= other[i];
        }

        return *this;
    }

    SpacialVector<T, size> &operator/=(T scalar) {
        for (int i = 0; i < size; i++) {
            data[i] /= scalar;
        }
        return *this;
    }

    template <unsigned int size2>
    SpacialVector<T, size> &operator/=(SpacialVector<T, size2> &other) {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {  // Divide the two vectors
            data[i] /= other[i];
        }

        return *this;
    }

    T magnitude() const {
        T result = 0;
        for (int i = 0; i < size; i++) {
            result += data[i] * data[i];
        }
        return sqrt(result);
    }

    SpacialVector<T, size> operator++() {
        for (int i = 0; i < size; i++) {
            data[i]++;
        }
        return *this;
    }

    SpacialVector<T, size> operator++(int) {
        SpacialVector<T, size> result = *this;
        for (int i = 0; i < size; i++) {
            data[i]++;
        }
        return result;
    }

    SpacialVector<T, size> operator--() {
        for (int i = 0; i < size; i++) {
            data[i]--;
        }
        return *this;
    }

    SpacialVector<T, size> operator--(int) {
        SpacialVector<T, size> result = *this;
        for (int i = 0; i < size; i++) {
            data[i]--;
        }
        return result;
    }

    /*----------------- Unary Operators  --------------------*/

    SpacialVector<T, size> operator+() const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = +data[i];
        }
    }

    SpacialVector<T, size> operator-() const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = -data[i];
        }
    }

    /*----------------- Comparison Operators -----------------*/

    template <unsigned int size2>
    bool operator==(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {
            if (data[i] != other[i]) {
                return false;
            }
        }
        return true;
    }

    template <unsigned int size2>
    bool operator!=(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {
            if (data[i] != other[i]) {
                return true;
            }
        }
        return false;
    }

    /*----------------- Accessor Operators -----------------*/

#define DEFINE_ACCESSOR(name)
    T &name() {
        return data[static_cast<std::size_t>(Axis::name)];
    }  // Accessor function for each enum value
    const T &name() const {
        return data[static_cast<std::size_t>(Axis::name)];
    }  // Const accessor function for each enum value
    void name(T value) {
        data[static_cast<std::size_t>(Axis::name)] = value;
    }  // Mutator function for each enum value

    // Use the macro to define accessor functions for each enum value
    DEFINE_ACCESSOR(X)
    DEFINE_ACCESSOR(Y)
    DEFINE_ACCESSOR(Z)
    DEFINE_ACCESSOR(A)
    DEFINE_ACCESSOR(B)
    DEFINE_ACCESSOR(C)
    DEFINE_ACCESSOR(D)
    DEFINE_ACCESSOR(E)
    DEFINE_ACCESSOR(F)
    DEFINE_ACCESSOR(G)
    DEFINE_ACCESSOR(H)
    DEFINE_ACCESSOR(I)
    DEFINE_ACCESSOR(J)
    DEFINE_ACCESSOR(K)
    DEFINE_ACCESSOR(L)
    DEFINE_ACCESSOR(M)
    DEFINE_ACCESSOR(N)
    DEFINE_ACCESSOR(O)
    DEFINE_ACCESSOR(P)

    T &operator[](unsigned int index) {
        if (index >= size) {
            throw std::out_of_range("Index out of range");
        }
        return data[index];
    }

    const T &operator[](unsigned int index) const {
        if (index >= size) {
            throw std::out_of_range("Index out of range");
        }
        return data[index];
    }

    /*----------------- Logical Operators -----------------*/

    bool operator!() const {
        for (int i = 0; i < size; i++) {
            if (data[i] != 0) {
                return false;
            }
        }
        return true;
    }

    template <unsigned int size2>
    bool operator&&(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {
            if (data[i] == 0 || other[i] == 0) {
                return false;
            }
        }
        return true;
    }

    template <unsigned int size2>
    bool operator||(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        for (int i = 0; i < size; i++) {
            if (data[i] == 0 && other[i] == 0) {
                return false;
            }
        }
        return true;
    }

    /*----------------- Bitwise Operators -----------------*/

    SpacialVector<T, size> operator~() const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = ~data[i];
        }
        return result;
    }

    SpacialVector<T, size> operator&(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] & scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator&(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] & other[i];
        }
        return result;
    }

    SpacialVector<T, size> operator|(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) result[i] = data[i] | scalar;
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator|(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] | other[i];
        }
        return result;
    }

    SpacialVector<T, size> operator^(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] ^ scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator^(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] ^ other[i];
        }
        return result;
    }

    SpacialVector<T, size> operator<<(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] << scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator<<(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] << other[i];
        }
        return result;
    }

    SpacialVector<T, size> operator>>(T scalar) const {
        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] >> scalar;
        }
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> operator>>(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<T, size> result;
        for (int i = 0; i < size; i++) {
            result[i] = data[i] >> other[i];
        }
        return result;
    }

    /*----------------- Stream Operators -----------------*/

    friend std::ostream &operator<<(std::ostream &os, const SpacialVector<T, size> &vector) {
        os << "[";
        for (int i = 0; i < size; i++) {
            os << vector[i];
            if (i < size - 1) {
                os << ", ";
            }
        }
        os << "]";
        return os;
    }

    friend std::istream &operator>>(std::istream &is, SpacialVector<T, size> &vector) {
        for (int i = 0; i < size; i++) {
            is >> vector[i];
        }
        return is;
    }

    /*----------------- Conversion Operators -----------------*/

    operator T *() { return data; }

    operator const T *() const { return data; }

    template <typename U, unsigned int size2>
    operator SpacialVector<U, size2>() const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        SpacialVector<U, size2> result;
        for (int i = 0; i < size; i++) {
            result[i] = static_cast<U>(data[i]);
        }
        return result;
    }

    /*----------------- Geometric Functions -----------------*/

    SpacialVector<T, size> normalize() const {
        SpacialVector<T, Size> result;
        T mag = magnitude();
        for (int i = 0; i < size; i++) {
            result[i] = data[i] / mag;
        }
        return result;
    }

    SpacialVector<T, 2> toPolar() const {
        SpacialVector<T, 2> result;
        result[0] = magnitude();
        result[1] = atan2(data[1], data[0]);
        return result;
    }

    SpacialVector<T, 2> toCartesian() const {
        SpacialVector<T, 2> result;
        result[0] = data[0] * cos(data[1]);
        result[1] = data[0] * sin(data[1]);
        return result;
    }

    template <unsigned int size2>
    SpacialVector<T, size> reflect(SpacialVector<T, size2> &normal) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return *this - 2 * dot(normal) * normal;
    }

    template <unsigned int size2>
    SpacialVector<T, size> project(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return dot(other) / other.magnitude() * other.normalize();
    }

    template <unsigned int size2>
    SpacialVector<T, size> reject(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return *this - project(other);
    }

    template <unsigned int size2>
    T angleDelta(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return acos(dot(other) / (magnitude() * other.magnitude()));
    }

    template <unsigned int size2>
    T distanceDelta(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return (other - *this).magnitude();
    }

    template <unsigned int size2>
    SpacialVector<T, size> midpoint(SpacialVector<T, size2> &other) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return (*this + other) / 2;
    }

    template <unsigned int size2>
    SpacialVector<T, size> lerp(SpacialVector<T, size2> &other, T t) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        return *this + t * (other - *this);
    }

    template <unsigned int size2>
    SpacialVector<T, size> slerp(SpacialVector<T, size2> &other, T t) const {
        static_assert(size2 >= size,
                      "Vector Operator Size Mismatch, SpacialVector2 Must Be >= SpacialVector1");

        T dot = dot(other);
        dot = std::clamp(dot, -1, 1);
        T theta = acos(dot) * t;
        SpacialVector<T, size> relative = other - *this * dot;
        relative = relative.normalize();
        return *this * cos(theta) + relative * sin(theta);
    }
};
}  // namespace UnityTypes
#endif
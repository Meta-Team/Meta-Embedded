//
// Created by liuzikai on 2019-05-13.
//

#ifndef META_INFANTRY_AHRS_MATH_HPP
#define META_INFANTRY_AHRS_MATH_HPP

#include <cmath>

typedef float Matrix33[3][3];  // row-major: [row][column]

class Vector3D {
public:

    float x;
    float y;
    float z;

    Vector3D() : x(0), y(0), z(0) {};

    Vector3D(float a, float b, float c) : x(a), y(b), z(c) {};

    explicit Vector3D(float a[3]) : x(a[0]), y(a[1]), z(a[2]) {};

    // M times v
    friend Vector3D operator*(const Matrix33 &M, const Vector3D &v) {
        return {M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z,
                M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z,
                M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z};
    }

    friend Vector3D operator+(const Vector3D &a, const Vector3D &b) {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    friend Vector3D operator-(const Vector3D &a, const Vector3D &b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    friend Vector3D operator*(const Vector3D &a, float c) {
        return {c * a.x, c * a.y, c * a.z};
    }

    friend Vector3D operator/(const Vector3D &a, float c) {
        return {a.x / c, a.y / c, a.z / c};
    }

    friend bool operator==(const Vector3D &a, const Vector3D &b) {
        return (a.x == b.x && a.y == b.y && a.z == b.z);
    }

    friend bool operator!=(const Vector3D &a, const Vector3D &b) {
        return !(a == b);
    }

    const Vector3D cross_mult(const Vector3D &b) {
        return {y * b.z - z * b.y,
                z * b.x - x * b.z,
                x * b.y - y * b.x};
    }

    float norm() const {
        return sqrtf(x * x + y * y + z * z);
    }

};

inline void matrix33_mult(const Matrix33 &a, const Matrix33 &b, Matrix33 c) {
    // Plain matrix multiplication
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i][j] = a[i][0] * b[j][0] + a[i][1] * b[j][1] + a[i][2] * b[j][2];
        }
    }
}

#ifndef GRAV_CONSTANT
#define GRAV_CONSTANT 9.80665f
#endif

#define DEG2RAD 0.01745329251994329576923690768489f
#define RAD2DEG 57.295779513082320876798154814105f

#endif //META_INFANTRY_AHRS_MATH_HPP

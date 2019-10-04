//
// Created by liuzikai on 2019-05-13.
//

#ifndef META_INFANTRY_AHRS_MATH_HPP
#define META_INFANTRY_AHRS_MATH_HPP

typedef float Matrix33[3][3];

class Vector3D {
public:

    float x;
    float y;
    float z;

    Vector3D():x(0),y(0),z(0) {};
    Vector3D(float a, float b, float c):x(a),y(b),z(c) {};
    Vector3D(float a[3]):x(a[0]), y(a[1]), z(a[2]) {};


    friend const Vector3D operator*(Vector3D a, const Matrix33 b) {
        Vector3D converted;
        converted.x = b[0][0] * a.x + b[0][1] * a.y + b[0][2] * a.z;
        converted.y = b[1][0] * a.x + b[1][1] * a.y + b[1][2] * a.z;
        converted.z = b[2][0] * a.x + b[2][1] * a.y + b[2][2] * a.z;
        return converted;
    }

    friend const Vector3D operator+(Vector3D a, Vector3D b) {
        return Vector3D(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    friend const Vector3D operator-(Vector3D a, Vector3D b) {
        return Vector3D(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    friend const Vector3D operator*(Vector3D a, float c) {
        return Vector3D(c * a.x, c * a.y, c * a.z);
    }

    friend const Vector3D operator/(Vector3D a, float c) {
        return Vector3D(a.x / c, a.y / c, a.z / c);
    }

    friend bool operator== (const Vector3D &a, const Vector3D &b) {
        return (a.x == b.x && a.y == b.y && a.z == b.z);
    }

    friend bool operator!= (const Vector3D &a, const Vector3D &b) {
        return !(a == b);
    }

    const Vector3D crossMultiply(Vector3D b) {
        Vector3D ans;
        ans.x = y * b.z - z * b.y;
        ans.y = z * b.x - x * b.z;
        ans.z = x * b.y - y * b.x;
        return ans;
    }

};

#define GRAV_CONSTANT 9.80665f

#define DEG2RAD 0.01745329251994329576923690768489f
#define RAD2DEG 57.295779513082320876798154814105f

#endif //META_INFANTRY_AHRS_MATH_HPP

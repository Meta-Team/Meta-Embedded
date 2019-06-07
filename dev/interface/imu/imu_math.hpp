//
// Created by liuzikai on 2019-05-13.
//

#ifndef META_INFANTRY_IMU_MATH_HPP
#define META_INFANTRY_IMU_MATH_HPP

typedef float matrix3[3][3];

class Vector3D {
public:

    float x;
    float y;
    float z;
    Vector3D():x(0),y(0),z(0) {};
    Vector3D(float a, float b, float c):x(a),y(b),z(c) {};
    Vector3D(float a[3]):x(a[0]), y(a[1]), z(a[2]) {};

    /**
    * @brief rotate the vector with the bias matrix
    * @param vector3D and bias matrix
    * @return a vector
    */
    friend const Vector3D operator*(Vector3D a, matrix3 b) {
        Vector3D converted;
        converted.x = b[0][0] * a.x + b[0][1] * a.y + b[0][2] * a.z;
        converted.y = b[1][0] * a.x + b[1][1] * a.y + b[1][2] * a.z;
        converted.z = b[2][0] * a.x + b[2][1] * a.y + b[2][2] * a.z;
        return converted;
    }

    friend const Vector3D operator+(Vector3D a, Vector3D b) {
        Vector3D converted;
        converted.x = a.x + b.x;
        converted.y = a.y + b.y;
        converted.z = a.z + b.z;
        return converted;
    }


    const Vector3D crossMultiply(Vector3D b) {
        Vector3D ans;
        ans.x = y * b.z - z * b.y;
        ans.y = z * b.x - x * b.z;
        ans.z = x * b.y - y * b.x;
        return ans;
    }

};

#endif //META_INFANTRY_IMU_MATH_HPP

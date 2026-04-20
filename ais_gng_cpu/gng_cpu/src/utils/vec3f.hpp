#pragma once

#include "utils.hpp"

class Vec3f {
   public:
    Vec3f();
    Vec3f(const Vec3f& vec);

    float p[3];

    Vec3f& operator=(const Vec3f& vec);
    Vec3f& operator=(const Vec3& vec);
    Vec3f(float _x, float _y, float _z);
    ~Vec3f();

    float& operator[](int index);

    Vec3f operator+(Vec3f vec);
    Vec3f operator-(Vec3f vec);
    Vec3f operator*(Vec3f vec);
    Vec3f operator/(Vec3f vec);

    Vec3f operator+(float r);
    Vec3f operator-(float r);
    Vec3f operator*(float r);
    Vec3f operator/(float r);

    void operator+=(Vec3f vec);
    void operator-=(Vec3f vec);
    void operator*=(float r);
    void operator/=(float r);

    float norm();
    float norm(const Vec3f& vec);
    float normXY(const Vec3f& vec);
    float squaredNorm();
    float squaredNorm(const Vec3f& vec);
    float squaredNorm(const Vec3& vec);
    float squaredNormXY(const Vec3f& vec);
    float squaredNormXY(const Vec3& vec);

    float abs();
    float dot(Vec3f vec);
    Vec3f cross(Vec3f vec);
    Vec3f normalized();
    Vec3f move(Vec3f& vec, float eta, float eta2);
    void zero();
    Vec3 toVec3();
    Vec3f reverse();
    bool isZero();
};
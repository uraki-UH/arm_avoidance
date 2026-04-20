#include "vec3f.hpp"

Vec3f::Vec3f() {
    Vec3f(0, 0, 0);
}
Vec3f& Vec3f::operator=(const Vec3f& vec) {
    p[0] = vec.p[0], p[1] = vec.p[1], p[2] = vec.p[2];
    return *this;
}
Vec3f& Vec3f::operator=(const Vec3& vec) {
    p[0] = vec.x, p[1] = vec.y, p[2] = vec.z;
    return *this;
}
Vec3f::Vec3f(const Vec3f& vec) {
    p[0] = vec.p[0], p[1] = vec.p[1], p[2] = vec.p[2];
}
Vec3f::Vec3f(float _x, float _y, float _z) {
    p[0] = _x;
    p[1] = _y;
    p[2] = _z;
}
Vec3f::~Vec3f() {
}

float& Vec3f::operator[](int index) {
    return p[index];
}

Vec3f Vec3f::operator+(Vec3f vec) {
    return Vec3f(p[0] + vec.p[0], p[1] + vec.p[1], p[2] + vec.p[2]);
}
Vec3f Vec3f::operator-(Vec3f vec) {
    return Vec3f(p[0] - vec.p[0], p[1] - vec.p[1], p[2] - vec.p[2]);
}
Vec3f Vec3f::operator*(Vec3f vec) {
    return Vec3f(p[0] * vec.p[0], p[1] * vec.p[1], p[2] * vec.p[2]);
}
Vec3f Vec3f::operator/(Vec3f vec) {
    return Vec3f(p[0] / vec.p[0], p[1] / vec.p[1], p[2] / vec.p[2]);
}

Vec3f Vec3f::operator+(float r) {
    return Vec3f(p[0] + r, p[1] + r, p[2] + r);
}
Vec3f Vec3f::operator-(float r) {
    return Vec3f(p[0] - r, p[1] - r, p[2] - r);
}
Vec3f Vec3f::operator*(float r) {
    return Vec3f(p[0] * r, p[1] * r, p[2] * r);
}
Vec3f Vec3f::operator/(float r) {
    return Vec3f(p[0] / r, p[1] / r, p[2] / r);
}

void Vec3f::operator+=(Vec3f vec) {
    p[0] += vec.p[0];
    p[1] += vec.p[1];
    p[2] += vec.p[2];
}
void Vec3f::operator-=(Vec3f vec) {
    p[0] -= vec.p[0];
    p[1] -= vec.p[1];
    p[2] -= vec.p[2];
}
void Vec3f::operator*=(float r) {
    p[0] *= r;
    p[1] *= r;
    p[2] *= r;
}
void Vec3f::operator/=(float r) {
    p[0] /= r;
    p[1] /= r;
    p[2] /= r;
}

float Vec3f::norm() {
    return sqrtf(squaredNorm());
}
float Vec3f::norm(const Vec3f& vec) {
    return sqrtf(squaredNorm(vec));
}
float Vec3f::normXY(const Vec3f& vec) {
    return sqrtf(squaredNormXY(vec));
}
float Vec3f::squaredNorm() {
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
}
float Vec3f::squaredNorm(const Vec3f& vec) {
    float x = p[0] - vec.p[0];
    float y = p[1] - vec.p[1];
    float z = p[2] - vec.p[2];
    return x * x + y * y + z * z;
}
float Vec3f::squaredNorm(const Vec3& vec) {
    float x = p[0] - vec.x;
    float y = p[1] - vec.y;
    float z = p[2] - vec.z;
    return x * x + y * y + z * z;
}
float Vec3f::squaredNormXY(const Vec3f& vec) {
    float x = p[0] - vec.p[0];
    float y = p[1] - vec.p[1];
    return x * x + y * y;
}
float Vec3f::squaredNormXY(const Vec3& vec) {
    float x = p[0] - vec.x;
    float y = p[1] - vec.y;
    return x * x + y * y;
}
float Vec3f::abs() {
    return fabs(p[0]) + fabs(p[1]) + fabs(p[2]);
}
float Vec3f::dot(Vec3f vec) {
    return p[0] * vec.p[0] + p[1] * vec.p[1] + p[2] * vec.p[2];
}
Vec3f Vec3f::cross(Vec3f vec) {
    return Vec3f(p[1] * vec.p[2] - p[2] * vec.p[1], p[2] * vec.p[0] - p[0] * vec.p[2], p[0] * vec.p[1] - p[1] * vec.p[0]);
}
Vec3f Vec3f::normalized() {
    float sq = squaredNorm();
    if (sq == 0)
        return Vec3f();
    else {
        sq = 1.f / sqrtf(sq);
        return Vec3f(p[0] * sq, p[1] * sq, p[2] * sq);
    }
}
Vec3f Vec3f::move(Vec3f& vec, float eta, float eta_2) {
    Vec3f v;
    v[0] = vec[0] * eta + p[0] * eta_2;
    v[1] = vec[1] * eta + p[1] * eta_2;
    v[2] = vec[2] * eta + p[2] * eta_2;
    return v;
}
void Vec3f::zero() {
    p[0] = p[1] = p[2] = 0;
}
Vec3 Vec3f::toVec3(){
    Vec3 vec;
    vec.x = p[0];
    vec.y = p[1];
    vec.z = p[2];
    return vec;
}

Vec3f Vec3f::reverse(){
    return Vec3f(-p[0], -p[1], -p[2]);
}

bool Vec3f::isZero(){
    return p[0] == 0 && p[1] == 0 && p[2] == 0;
}
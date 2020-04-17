#include "typedef.cpp"
#include "Vector.h"

Vector::Vector(f32 x, f32 y, f32 z)
    : x(x), y(y), z(z) {
}

Vector Vector::add(Vector V) {
    return Vector(x + V.x, y + V.y, z + V.z);
}

Vector Vector::sub(Vector V) {
    return Vector(x - V.x, y - V.y, z - V.z);
}

f32 Vector::dot(Vector V) {
    return x * V.x + y * V.y + z * V.z;
}

Vector Vector::crs(Vector V) {
    return Vector(y * V.z - z * V.y, z * V.x - x * V.z, x * V.y - y * V.x);
}
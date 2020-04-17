#pragma once

struct Vector {
    f32 x, y, z;

    Vector(f32 x, f32 y, f32 z);

    Vector add(Vector V);
    Vector sub(Vector V);
    f32    dot(Vector V);
    Vector crs(Vector F);
};
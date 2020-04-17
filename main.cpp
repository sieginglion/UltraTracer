#include <vector>
#include "typedef.cpp"

struct Vector {
    f32 x, y, z;

    Vector();
    Vector(f32 x, f32 y, f32 z)
            : x(x), y(y), z(z) {
    }

    Vector operator+(Vector V) {
        return Vector(x + V.x, y + V.y, z + V.z);
    }
    Vector operator-(Vector V) {
        return Vector(x - V.x, y - V.y, z - V.z);
    }
    f32 dot(Vector V) {
        return x * V.x + y * V.y + z * V.z;
    }
    Vector crs(Vector V) {
        return Vector(y * V.z - z * V.y, z * V.x - x * V.z, x * V.y - y * V.x);
    }
};

class Ray {
    Vector O, D;
    f32 I;
};

class Tracer {

};

//f32 intersect() {
//    Vector E1 = V[1].sub(V[0]);
//    Vector E2 = V[2].sub(V[0]);
//    Vector P = D.crs(E2);
//    f32 det = E1.dot(P);
//    if (det < 0.000001) {
//        return 0.;
//    }
//    Vector T = O.sub(V[0]);
//    f32 u = T.dot(P);
//    if (u < 0. || u > det) {
//        return 0.;
//    }
//    Vector Q = T.crs(E1);
//    f32 v = D.dot(Q);
//    if (v < 0. || u + v > det) {
//        return 0.;
//    }
//    t = E2.dot(Q);
//}

#include <fstream>
#include <iostream>
#define print(x) std::cout << x << "\n"
#include <array>

using namespace std;

vector<string> split(string str, string del) {
    vector<string> subs;
    i8 s = 0, e;
    while (e != string::npos) {
        e = str.find(del, s);
        subs.push_back(str.substr(s, e - s));
        s = e + 1;
    }
    return subs;
}

int main() {
    vector<array<f32, 3>> vertices;
    ifstream file("../teapot.obj");
    for (string line; getline(file, line);) {
        if (line[0] == 'v') {
            array<f32, 3> vertex;
            vector<string> subs = split(line, " ");
            for (i8 i = 1; i < 3; i++) {
                vertex[i] = stof(subs[i]);
            }
            vertices.push_back(vertex);
        }
//        else {
//            array<array<f32, 3>, 3> plane;
//            auto subs = split(line, " ");
//            for (i8 i = 0; i < 3; i++) {
//                plane[i] = vertices[stoi(subs[i]) - 1];
//            }
//        }
    }

}
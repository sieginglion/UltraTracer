#include "typedef.cpp"

#include <cmath>
#include <array>
#include <vector>
#include <fstream>
#include <iostream>

#define print(x) std::cout << x << "\n"

using namespace std;

struct Vector {
    f32 x, y, z;
    Vector();
    Vector(f32 x, f32 y, f32 z)
            : x(x), y(y), z(z) {
    }
    Vector norm() {
        f32 l = sqrt(x * x + y * y + z * z);
        return Vector(x / l, y / l, z / l);
    }
    Vector operator*(f32 a) {
        return Vector(x * a, y * a, z * a);
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

vector<string> split(string str, string del) {
    vector<string> subs;
    i8 s = 0, e = 0;
    while (e != string::npos) {
        e = str.find(del, s);
        subs.push_back(str.substr(s, e - s));
        s = e + 1;
    }
    return subs;
}

struct Euclid {
    Vector O, D;
    f32 I = 1;
    Euclid();
    Euclid(Vector O, Vector D)
        : O(O), D(D) {
    }
};

f32 intersect(array<Vector, 3> V, Euclid R) {
    Vector E1 = V[1] - V[0];
    Vector E2 = V[2] - V[0];
    Vector P = R.D.crs(E2);
    f32 det = E1.dot(P);
    if (det < 0.000001) {
        return 0.;
    }
    Vector T = R.O - V[0];
    f32 u = T.dot(P);
    if (u < 0. || u > det) {
        return 0.;
    }
    Vector Q = T.crs(E1);
    f32 v = R.D.dot(Q);
    if (v < 0. || u + v > det) {
        return 0.;
    }
    return E2.dot(Q);
}

Vector reflect(Vector v, Vector n) {
    if (v.dot(n) > 0) {
        n = n * -1;
    }
    return v + n * v.dot(n) * 2;
}

int main() {
    vector<Vector> vertices;
    vector<array<Vector, 3>> object;
    {
        ifstream file("../teapot.obj");
        for (string line; getline(file, line);) {
            if (line[0] == 'v') {
                array<f32, 3> vertex;
                vector<string> subs = split(line, " ");
                for (int i = 0; i < 3; i++) {
                    vertex[i] = stof(subs[i + 1]);
                }
                vertices.push_back(Vector(vertex[0], vertex[1], vertex[2]));
            }
            else if (line[0] == 'f') {
                array<Vector, 3> plane;
                vector<string> subs = split(line, " ");
                for (int i = 0; i < 3; i++) {
                    plane[i] = vertices[stoi(subs[i + 1]) - 1];
                }
                object.push_back(plane);
            }
        }
        file.close();
    }

    int height = 640;
    int width = 480;
    f32 FOV = 60 * 0.0174;
    Vector cam_pos(8, 0, 1.5);
    Vector cam_dir(-1, 0, 0);
    int steps = 10;
    Vector sun(0, 0, 1);

    f32 pixel = tan(FOV / 2) * 2 / width;
    Vector dx = Vector(cam_dir.y, -cam_dir.x, 0).norm() * pixel;
    Vector dy(0, 0, -pixel);
    cam_dir = cam_dir - dx * (width / 2) - dy * (height / 2);
    Euclid rays[height][width];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            rays[i][j] = Euclid(cam_pos, cam_dir + dx * j + dy * i);
        }
    }

    for (int i = 0; i < steps; i++) {
        for (auto ray: rays) {
            if (ray->D.x || ray->D.y || ray->D.z) {
                for (auto plane: object) {
                    f32 t = intersect(plane, *ray);
                    if (t > 0) {
                        Vector n = (plane[1] - plane[0]).crs(plane[2] - plane[0]).norm();
                        ray->O = ray->O + ray->D * t;
                        ray->D = reflect(ray->D, n);
                        ray->I = ray->I * 0.9;
                    }
                    else if (ray->D.dot(sun) == 0) {
                        ray->D = {0, 0, 0};
                    }
                }
            }
        }
    }

    f32 screen[height][width];
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            screen[i][j] = rays[i][j].I;
        }
    }

    ofstream file("result.pgm");
    file << "P2\n";
    file << width << ' ' << height << '\n';
    file << "255\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < height; j++) {
            if (j == 255) {
                file << int(rays[i][j].I * 255) << '\n';
            }
            else {
                file << int(rays[i][j].I * 255) << ' ';
            }
        }
    }
    file.close();
}
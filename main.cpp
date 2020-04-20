#include <cmath>
#include <array>
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;

struct Vector {
    float x, y, z;
    float& operator[](int i) {
        switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
        }
    }
    Vector norm() {
        float l = sqrt(x * x + y * y + z * z);
        return { x / l, y / l, z / l };
    }
    Vector operator*(float a) {
        return { x * a, y * a, z * a };
    }
    Vector operator+(Vector V) {
        return { x + V.x, y + V.y, z + V.z };
    }
    Vector operator-(Vector V) {
        return { x - V.x, y - V.y, z - V.z };
    }
    float dot(Vector V) {
        return x * V.x + y * V.y + z * V.z;
    }
    Vector crs(Vector V) {
        return { y * V.z - z * V.y, z * V.x - x * V.z, x * V.y - y * V.x };
    }
};

struct Euclid {
    Vector O, D;
    float I;
};

void SplitStr(string& str, string& sep, vector<string>& substrs) {
    int start = 0, end = 0;
    while (end != string::npos) {
        end = str.find(sep, start);
        substrs.push_back(str.substr(start, end - start));
        start = end + 1;
    }
}

void LoadObject(string& filename, vector<array<Vector, 3>>& object) {
    string sep = " ";
    vector<Vector> vertices;
    ifstream file("../" + filename);
    for (string line; getline(file, line);) {
        if (line[0] == 'v') {
            Vector vertex;
            vector<string> substrs;
            SplitStr(line, sep, substrs);
            for (int i = 0; i < 3; i++) {
                vertex[i] = stof(substrs[i + 1]);
            }
            vertices.push_back(vertex);
        }
        else if (line[0] == 'f') {
            array<Vector, 3> plane;
            vector<string> substrs;
            SplitStr(line, sep, substrs);
            for (int i = 0; i < 3; i++) {
                plane[i] = vertices[stoi(substrs[i + 1]) - 1];
            }
            object.push_back(plane);
        }
    }
}

void CreateRays(float FOV, int height, int width, Vector& cam_pos, Vector& cam_dir, Euclid* rays) {
    float pixel = tan(FOV / 2) * 2 / width;
    Vector dx = Vector{ cam_dir.y, -cam_dir.x, 0 }.norm() * pixel;
    Vector dy = { 0, 0, -pixel };
    cam_dir = cam_dir - dx * (width / 2) - dy * (height / 2);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            rays[width * i + j] = { cam_pos, cam_dir + dx * j + dy * i };
        }
    }
}

float intersect(array<Vector, 3> V, Euclid R) {
    Vector E1 = V[1] - V[0];
    Vector E2 = V[2] - V[0];
    Vector P = R.D.crs(E2);
    float det = E1.dot(P);
    if (det < 0.000001) {
        return 0.;
    }
    Vector T = R.O - V[0];
    float u = T.dot(P);
    if (u < 0. || u > det) {
        return 0.;
    }
    Vector Q = T.crs(E1);
    float v = R.D.dot(Q);
    if (v < 0. || u + v > det) {
        return 0.;
    }
    return E2.dot(Q);
}

Vector ReflectRay(Vector v, Vector n) {
    if (v.dot(n) > 0) {
        n = n * -1;
    }
    return v + n * v.dot(n) * 2;
}

template<int N>
void TraceRays(array<Euclid, N>& rays, vector<array<Vector, 3>> object, int n_iters, Vector& sun) {
    for (int i = 0; i < n_iters; i++) {
        for (auto& ray: rays) {
            if (ray.x || ray.y || ray.z) {
                for (auto& plane: object) {
                    float t = CheckIntersect(plane, ray);
                    if (t > 0) {
                        Vector n = (plane[1] - plane[0]).crs(plane[2] - plane[0]).norm();
                        ray = { ray.O + ray.D * t, ReflectRay(ray.D, n), ray.I * 0.9 };
                    }
                    else if (ray.D.dot(sun) == 0) {
                        ray.D = { 0, 0, 0 };
                    }
                }
            }
        }
    }
}

int main() {
    string FILENAME = "teapot.obj";
    int HEIGHT = 640;
    int WIDTH = 480;
    float FOV = 60 * 0.0174;
    Vector CAM_POS = { 8, 0, 1.5 };
    Vector CAM_DIR = { -1, 0, 0 };
    int N_ITERS = 10;
    Vector SUN = { 0, 0, 1 };

    vector<array<Vector, 3>> object;
    Euclid rays[WIDTH * HEIGHT];

    LoadObject(FILENAME, object);
    CreateRays(FOV, HEIGHT, WIDTH, CAM_POS, CAM_DIR, rays);
//

//
//    float screen[height][width];
//    for (int i = 0; i < height; i++) {
//        for (int j = 0; j < width; j++) {
//            screen[i][j] = rays[i][j].I;
//        }
//    }
//
//    ofstream file("result.pgm");
//    file << "P2\n";
//    file << width << ' ' << height << '\n';
//    file << "255\n";
//    for (int i = 0; i < height; i++) {
//        for (int j = 0; j < height; j++) {
//            if (j == 255) {
//                file << int(rays[i][j].I * 255) << '\n';
//            }
//            else {
//                file << int(rays[i][j].I * 255) << ' ';
//            }
//        }
//    }
//    file.close();
}
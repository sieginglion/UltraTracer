#include <cmath>
#include <array>
#include <vector>
#include <fstream>

#define SUB(A, B) { A.x - B.x, A.y - B.y, A.z - B.z }
#define DOT(A, B) ( A.x * B.x + A.y * B.y + A.z * B.z )
#define CRS(A, B) { A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x }

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
    Vector operator+(Vector B) {
        return { x + B.x, y + B.y, z + B.z };
    }
    Vector operator-(Vector B) {
        return { x - B.x, y - B.y, z - B.z };
    }
    float dot(Vector B) {
        return x * B.x + y * B.y + z * B.z;
    }
    Vector crs(Vector B) {
        return { y * B.z - z * B.y, z * B.x - x * B.z, x * B.y - y * B.x };
    }
};

struct Ray {
    Vector O, D;
};

vector<string> SplitStr(string& str, string& sep) {
    vector<string> substrs;
    int start = 0, end = 0;
    while (end != string::npos) {
        end = str.find(sep, start);
        substrs.push_back(str.substr(start, end - start));
        start = end + 1;
    }
    return substrs;
}

vector<array<Vector, 3>> LoadObject(string& filename) {
    string sep = " ";
    vector<Vector> vertices;
    vector<array<Vector, 3>> object;
    ifstream file(filename);
    for (string line; getline(file, line);) {
        if (line[0] == 'v') {
            Vector vertex;
            vector<string> substrs = SplitStr(line, sep);
            for (int i = 0; i < 3; i++) {
                vertex[i] = stof(substrs[i + 1]);
            }
            vertices.push_back(vertex);
        }
        else if (line[0] == 'f') {
            array<Vector, 3> plane;
            vector<string> substrs = SplitStr(line, sep);
            for (int i = 0; i < 3; i++) {
                plane[i] = vertices[stoi(substrs[i + 1]) - 1];
            }
            object.push_back(plane);
        }
    }
    array<Vector, 3> ground = { Vector{ 100, 0, 0 }, Vector{ -100, -100, 0 }, Vector{ -100, 100, 0 } };
    array<Vector, 3> tetra0 = { Vector{ -4, 2.5, 0 }, Vector{ -1.5, -1.83, 0 }, Vector{ -4, -0.39, 7 } };
    array<Vector, 3> tetra1 = { Vector{ -4, 2.5, 0 }, Vector{ -6.5, -1.83, 0 }, Vector{ -4, -0.39, 7 } };
    object.push_back(ground);
    object.push_back(tetra0);
    object.push_back(tetra1);
    return object;
}

float GetIntersect(array<Vector, 3>& V, Ray& R) {
    Vector E1 = SUB(V[1], V[0]);
    Vector E2 = SUB(V[2], V[0]);
    Vector P = CRS(R.D, E2);
    float d = DOT(E1, P);
    if (d > -0.000001 && d < 0.000001) {
        return 0;
    }
    Vector T = SUB(R.O, V[0]);
    float u = DOT(P, T) / d;
    if (u < 0 || u > 1) {
        return 0;
    }
    Vector Q = CRS(T, E1);
    float v = DOT(R.D, Q) / d;
    if (v < 0 || u + v > 1) {
        return 0;
    }
    return DOT(E2, Q) / d;
}

//float GetIntersect(array<Vector, 3>& V, Ray& R) {
//    Vector E1 = SUB(V[1], V[0]);
//    Vector E2 = SUB(V[2], V[0]);
//    Vector P = CRS(R.D, E2);
//    float d = DOT(E1, P);
//    if (d < 0.000001) {
//        return 0;
//    }
//    Vector T = SUB(R.O, V[0]);
//    float u = DOT(P, T);
//    if (u < 0 || u > d) {
//        return 0;
//    }
//    Vector Q = CRS(T, E1);
//    float v = DOT(R.D, Q);
//    if (v < 0 || u + v > d) {
//        return 0;
//    }
//    return DOT(E2, Q) / d;
//}

//int main() {
//    array<Vector, 3> plane = { Vector{ 100, 0, 0 }, Vector{ -100, -100, 0 }, Vector{ -100, 100, 0 } };
//    Ray ray = { { 0, 0, 10 }, { 0, 0, -1 } };
//    float t = GetIntersect(plane, ray);
//}

Vector GetReflect(array<Vector, 3>& V, Vector D) {
    Vector N = (V[1] - V[0]).crs(V[2] - V[0]).norm();
    float p = D.dot(N);
    if (p > 0) {
        N = N * -1;
    }
    else {
        p = -p;
    }
    return D + N * p * 2;
}

float TraceRay(int n_iters, vector<array<Vector, 3>>& object, Ray& ray, Vector& sun) {
    float intensity = 1;
    for (int i = 0; i < n_iters; i++) {
        float T = 1000000;
        array<Vector, 3> Plane;
        for (auto& plane: object) {
            float t = GetIntersect(plane, ray);
            if (t > 0 and t < T) {
                T = t;
                Plane = plane;
            }
        }
        if (T < 1000000) {
            ray = { ray.O + ray.D * T, GetReflect(Plane, ray.D) };
            intensity *= 0.75;
        }
        else {
            return intensity * max(ray.D.dot(sun), 0.0f);
        }
    }
    return intensity * max(ray.D.dot(sun), 0.0f);
}

void SaveScreen(int width, int height, float screen[]) {
    ofstream file("screen.pgm");
    file << "P2\n";
    file << width << ' ' << height << '\n';
    file << "255\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (j == width - 1) {
                file << int(round(screen[width * i + j] * 255.0f)) << '\n';
            }
            else {
                file << int(round(screen[width * i + j] * 255.0f)) << ' ';
            }
        }
    }
}

int main() {
    string FILENAME = "teapot.obj";
    float FOV = 60 * 0.0174;
    int WIDTH = 640;
    int HEIGHT = 480;
    Vector CAM_POS = { -1, 10, 1.5 };
    Vector CAM_DIR = { 0, -1, 0 };
    int N_ITERS = 10;
    Vector SUN = { 0, 0, 1 };

    vector<array<Vector, 3>> object = LoadObject(FILENAME);

    float screen[WIDTH * HEIGHT];

    float pixel = tan(FOV / 2) * 2 / WIDTH;
    Vector dx = Vector{ CAM_DIR.y, -CAM_DIR.x, 0 }.norm() * pixel;
    Vector dy = { 0, 0, -pixel };
    CAM_DIR = CAM_DIR - dx * (WIDTH / 2) - dy * (HEIGHT / 2);

    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            Ray ray = { CAM_POS, (CAM_DIR + dy * i + dx * j).norm() };
            screen[WIDTH * i + j] = TraceRay(N_ITERS, object, ray, SUN);
        }
    }

    SaveScreen(WIDTH, HEIGHT, screen);
}
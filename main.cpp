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

vector<array<Vector, 3>> LoadScene(vector<string>& filenames) {
    string sep = " ";
    vector<array<Vector, 3>> scene;
    for (auto& filename: filenames) {
        vector<Vector> vertices;
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
                scene.push_back(plane);
            }
        }
    }
    return scene;
}

// Möller–Trumbore intersection algorithm
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
    return DOT(E2, Q) / d - 0.001; // minus 0.001 to avoid overshooting
}

Vector GetReflect(array<Vector, 3>& V, Vector& D) {
    Vector N = (V[1] - V[0]).crs(V[2] - V[0]).norm();
    float p = D.dot(N);
    if (p > 0) N = N * -1; else p = -p;
    return D + N * p * 2;
}

float TraceRay(int max_iters, vector<array<Vector, 3>>& scene, Ray& ray, float reflectance, Vector& sun) {
    float intensity = 1;
    for (int i = 0; i < max_iters; i++) {
        float min_t = 1000000;
        array<Vector, 3>* plane_ptr;
        for (auto& plane: scene) { // find plane which intersect with the ray first (closest)
            float t = GetIntersect(plane, ray);
            if (t > 0 and t < min_t) {
                min_t = t;
                plane_ptr = &plane;
            }
        }
        if (min_t < 1000000) {
            ray = { ray.O + ray.D * min_t, GetReflect(*plane_ptr, ray.D) };
            intensity *= reflectance;
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
                file << int(screen[width * i + j] * 255) << '\n';
            }
            else {
                file << int(screen[width * i + j] * 255) << ' ';
            }
        }
    }
}

int main() {
    vector<string> FILENAMES = { "ground.obj", "teapot.obj", "tetrahedron.obj" };
    float FOV = 60 * 0.0174; // field of view
    int WIDTH = 1280;
    int HEIGHT = 720;
    Vector CAM_POS = { -1.5, 10, 1.5 }; // camera position
    Vector CAM_DIR = { 0, -1, 0 }; // camera direction
    int MAX_ITERS = 4; // max iterations of tracer
    float REFLECTANCE = 0.75;
    Vector SUN = { 0, 0.18, 1 }; // vector pointing the sun

    vector<array<Vector, 3>> scene = LoadScene(FILENAMES);
    float pixel = tan(FOV / 2) * 2 / WIDTH;
    Vector dx = Vector{ CAM_DIR.y, -CAM_DIR.x, 0 }.norm() * pixel;
    Vector dy = { 0, 0, -pixel };
    CAM_DIR = CAM_DIR - dx * (WIDTH / 2) - dy * (HEIGHT / 2);
    float screen[WIDTH * HEIGHT];
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            Ray ray = { CAM_POS, (CAM_DIR + dy * i + dx * j).norm() };
            screen[WIDTH * i + j] = TraceRay(MAX_ITERS, scene, ray, REFLECTANCE, SUN);
        }
    }
    SaveScreen(WIDTH, HEIGHT, screen);
}

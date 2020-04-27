#include <cmath>
#include <array>
#include <vector>
#include <fstream>
#include <iostream>
#include <xmmintrin.h>
#include <pmmintrin.h>

using namespace std;

__m128 Norm(__m128 a) {
    __m128 _ = a * a;
    _ = _mm_hadd_ps(_, _);
    return a / sqrt(_mm_hadd_ps(_, _)[0]);
}

float Dot(__m128& a, __m128& b) {
    __m128 _ = a * b;
    _ = _mm_hadd_ps(_, _);
    return _mm_hadd_ps(_, _)[0];
}

__m128 Cross(__m128& a, __m128& b) {
    __m128 _ = _mm_sub_ps(
        _mm_mul_ps(a, _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1))),
        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), b)
    );
    return _mm_shuffle_ps(_, _, _MM_SHUFFLE(3, 0, 2, 1));
}

struct Ray {
    __m128 O, D;
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

vector<array<__m128, 3>> LoadScene(vector<string>& filenames) {
    string sep = " ";
    vector<array<__m128, 3>> scene;
    for (auto& filename: filenames) {
        vector<__m128> vertices;
        ifstream file(filename);
        for (string line; getline(file, line);) {
            if (line[0] == 'v') {
                vector<string> substrs = SplitStr(line, sep);
                vertices.push_back(__m128{ stof(substrs[1]), stof(substrs[2]), stof(substrs[3]), 0 });
            }
            else if (line[0] == 'f') {
                vector<string> substrs = SplitStr(line, sep);
                scene.push_back({
                    vertices[stoi(substrs[1]) - 1],
                    vertices[stoi(substrs[2]) - 1],
                    vertices[stoi(substrs[3]) - 1]
                });
            }
        }
    }
    return scene;
}

// Möller–Trumbore intersection algorithm
float GetIntersect(array<__m128, 3>& V, Ray& R) {
    __m128 E1 = V[1] - V[0];
    __m128 E2 = V[2] - V[0];
    __m128 P = Cross(R.D, E2);
    float d = Dot(E1, P);
    if (d > -0.000001f && d < 0.000001f) {
        return 0.0f;
    }
    __m128 T = R.O - V[0];
    float u = Dot(P, T) / d;
    if (u < 0.0f || u > 1.0f) {
        return 0.0f;
    }
    __m128 Q = Cross(T, E1);
    float v = Dot(R.D, Q) / d;
    if (v < 0.0f || u + v > 1.0f) {
        return 0.0f;
    }
    return Dot(E2, Q) / d;
}

__m128 GetReflect(array<__m128, 3>& V, __m128& D) {
    __m128 E1 = V[1] - V[0];
    __m128 E2 = V[2] - V[0];
    __m128 N = Norm(Cross(E1, E2));
    return D - N * Dot(D, N) * 2.0f;
}

float TraceRay(int max_iters, vector<array<__m128, 3>>& scene, Ray& ray, float reflectance, __m128& sun) {
    float intensity = 1.0f;
    for (int i = 0; i < max_iters; i++) {
        float min_t = 1000000.0f;
        array<__m128, 3>* plane_ptr;
        for (auto& plane: scene) { // find plane which intersect with the ray first (closest)
            float t = GetIntersect(plane, ray) - 0.001f; // minus 0.001 to avoid overshooting
            if (t > 0.0f and t < min_t) {
                min_t = t;
                plane_ptr = &plane;
            }
        }
        if (min_t < 1000000.0f) {
            ray = { ray.O + ray.D * min_t, GetReflect(*plane_ptr, ray.D) };
            intensity *= reflectance;
        }
        else {
            break;
        }
    }
    return intensity * max(Dot(ray.D, sun), 0.0f);
}

void SaveScreen(int width, int height, float screen[]) {
    ofstream file("screen.pgm");
    file << "P2\n";
    file << width << ' ' << height << '\n';
    file << "255\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (j == width - 1) {
                file << static_cast<int>(screen[width * i + j] * 255.0f) << '\n';
            }
            else {
                file << static_cast<int>(screen[width * i + j] * 255.0f) << ' ';
            }
        }
    }
}

struct Timer {
    std::chrono::time_point<std::chrono::steady_clock> T0, T1;
    Timer() {
        T0 = std::chrono::steady_clock::now();
    }
    void stop() {
        T1 = std::chrono::steady_clock::now();
        std::cout << (T1 - T0).count() / 1000000 << "ms\n";
    }
};

int main() {
//    vector<string> FILENAMES = { "../ground.obj", "../teapot.obj", "../tetrahedron.obj" };
    vector<string> FILENAMES = { "ground.obj", "teapot.obj", "tetrahedron.obj" };
    float FOV = 60 * 0.0174; // field of view
    int WIDTH = 640;
    int HEIGHT = 360;
    __m128 CAM_POS = { -1.5, 10, 1.5 }; // camera position
    __m128 CAM_DIR = { 0, -1, 0 }; // camera direction
    int MAX_ITERS = 4; // max iterations of tracer
    float REFLECTANCE = 0.75;
    __m128 SUN = { 0, 0.18, 1 }; // vector pointing the sun

    vector<array<__m128, 3>> scene = LoadScene(FILENAMES);
    float pixel = tan(FOV / 2) * 2 / WIDTH;
    __m128 dx = Norm(__m128{ CAM_DIR[1], -CAM_DIR[0], 0 }) * pixel;
    __m128 dy = { 0, 0, -pixel };
    CAM_DIR = CAM_DIR - dx * (WIDTH / 2.0f) - dy * (HEIGHT / 2.0f);
    float screen[WIDTH * HEIGHT];
    Timer timer;
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            Ray ray = { CAM_POS, Norm(CAM_DIR + dy * static_cast<float>(i) + dx * static_cast<float>(j)) };
            screen[WIDTH * i + j] = TraceRay(MAX_ITERS, scene, ray, REFLECTANCE, SUN);
        }
    }
    timer.stop();
    SaveScreen(WIDTH, HEIGHT, screen);
}
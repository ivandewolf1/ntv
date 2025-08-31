#include <iostream>
#include <nvec3.h>
#include "nanoExr.h"
using namespace std;
#define vec3 ntv::nvec3<float>

void nvecToArray(float* image, int index, vec3 pixel_color) {
    image[index + 0] = pixel_color[0];
    image[index + 1] = pixel_color[1];
    image[index + 2] = pixel_color[2];
}

int main(int argc, const char** argv) {
    int w = 640, h = 480;
    auto* image = new float[3 * w * h];
    for (int y = 0, idx = 0; y < h; y++)
        for (int x = 0; x < w; x++, idx += 3) {
            vec3 pixel_color(float(x)/w, float(h-y)/h, 0.25);
            nvecToArray(image, idx, pixel_color);
        }
    Exr(argc < 2 ? "../test.exr" : argv[1], w, h, image);
    delete [] image;
}

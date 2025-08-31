#include <iostream>
#include "nanoExr.h"

int main(int argc, const char** argv) {
    int w = 640, h = 480;
    auto* image = new float[3 * w * h];
    for (int y = 0, idx = 0; y < h; y++)
        for (int x = 0; x < w; x++, idx += 3)
            image[idx + 0] = static_cast<float>(x) / w,
            image[idx + 1] = static_cast<float>(y) / h,
            image[idx + 2] = 0;
    Exr(argc < 2 ? "../test.exr" : argv[1], w, h, image);
    delete [] image;
    return 0;
}

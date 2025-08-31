#include <cstdio>

// https://gist.github.com/fpsunflower/e5c99116ff94114d1c
// writes a tiled OpenEXR file from a float RGB buffer
struct Exr {
    FILE* f; enum{TS=64}; ~Exr() { fclose(f); }
    void B(unsigned char i)      { fputc(i,f); }
    void I(unsigned int i)       { fwrite(&i,1,4,f); }
    void L(unsigned long long i) { fwrite(&i,1,8,f); }
    void F(float x)              { fwrite(&x,1,4,f); }
    void S(const char* str)      { fputs(str,f);B(0); }
    void C(const char* name)     { S(name);L(2);I(1);I(1); }
    Exr(const char* fn, int w, int h, const float* c) : f(fopen(fn, "w")) {
        I(20000630);I(514); // magic/version/tiled
        S("channels");S("chlist");I(55);C("B");C("G");C("R");B(0);
        S("compression");S("compression");I(1);B(0);
        S(   "dataWindow");S("box2i");I(16);I(0);I(0);I(w-1);I(h-1);
        S("displayWindow");S("box2i");I(16);I(0);I(0);I(w-1);I(h-1);
        S("lineOrder");S("lineOrder");I(1);B(2);
        S("pixelAspectRatio");S("float");I(4);F(1.0f);
        S("screenWindowCenter");S("v2f");I(8);F(0.0f);F(0.0f);
        S("screenWindowWidth");S("float");I(4);F(1.0f);
        S("tiles");S("tiledesc");I(9);I(TS);I(TS);B(0);B(0);
        int tx = (w + TS - 1) / TS, ty = (h + TS - 1) / TS;
        long off = ftell(f) + tx * ty * 8; float out[TS * 3];
        for (int k = 2; k--;) for (int y = 0; y < h; y += TS)
            for (int x = 0; x < w; x += TS) {
                int tw = w-x > TS ? TS : w-x;
                int th = h-y > TS ? TS : h-y, sz=tw*th*12;
                if (k) { L(off); off += 20 + sz; continue; }
                I(x / TS);I(y / TS);I(0);I(0);I(sz);
                for (int ty = 0; ty < th; ty++) {
                    const float* in = c + 3 * (x+w*(y+ty));
                    for (int tx = 0; tx < tw; tx++)
                        out[      tx] = in[3*tx+2],
                        out[   tw+tx] = in[3*tx+1],
                        out[tw+tw+tx] = in[3*tx+0];
                    fwrite(out, 12, tw, f);
                }
            }
    }
};

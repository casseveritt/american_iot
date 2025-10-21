#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "smi_leds.h"

#include "linear.h"

using namespace r3;

#define STRIPS 16
#define PIXELS_PER_STRIP 320

#define LED_BRIGHTNESS 16

#define BRANCHES_PER_RING 8

#define RINGS 8

#define BRANCH_PITCH (-M_PI / 4.0)
#define PIXEL_SPACING 0.016667

// [0 .. branchLen/2 - 1] are the outside pixels starting at the trunk
// [branch_Len/2 ... branchLen - 1] are the inside pixels starting at the branch tip
constexpr uint32_t branchLen[RINGS] = {74, 74, 74, 74, 56, 38, 38, 20};
constexpr uint32_t branchesPerStrip[RINGS] = {4, 4, 4, 4, 4, 8, 8, 8};

struct PixInfo
{
    uint32_t index;
    Vec3f position;
    bool outside;
};

std::vector<PixInfo> get_pix_info()
{
    int pixels = 0;
    for (int i = 0; i < RINGS; i++)
    {
        pixels += BRANCHES_PER_RING * branchLen[i];
    }
    std::vector<PixInfo> pi;
    pi.reserve(pixels);
    int ring_start_index = 0;
    int indexAtRing[RINGS] = {};

    for (int i = 1; i < RINGS; i++)
    {
        indexAtRing[i] = indexAtRing[i - 1] + PIXELS_PER_STRIP * (BRANCHES_PER_RING / branchesPerStrip[i - 1]);
        printf("indexAtRing[%d] = %d\n", i, indexAtRing[i]);
    }

    for (int i = 0; i < RINGS; i++)
    {
        if (i != 1)
        {
            continue;
        }
        int half_branch_length = branchLen[i] / 2;
        for (int j = 0; j < BRANCHES_PER_RING; j++)
        {
            int branch_in_strip = j % branchesPerStrip[i];
            int index_in_strip = branch_in_strip * branchLen[i];
            int strip_in_ring = j / branchesPerStrip[i];
            int index = indexAtRing[i] + strip_in_ring * PIXELS_PER_STRIP + index_in_strip;
            float angle = (j * (2.0 * M_PI)) / BRANCHES_PER_RING;
            float branch_height = // in meters
                +0.6096           // height of first branch at trunk
                + i * 0.1778;     // branch spacing

            size_t branch_start = pi.size();
            // down the branch on the outside
            for (int k = 0; k < half_branch_length; k++)
            {
                float x = k * PIXEL_SPACING * cos(BRANCH_PITCH);
                float y = k * PIXEL_SPACING * sin(BRANCH_PITCH) + branch_height;
                PixInfo pix;
                pix.index = index + k;
                pix.position = Vec3f(cos(angle) * x, y, sin(angle) * x);
                pix.outside = true;
                pi.push_back(pix);
            }
            // back up the branch on the inside
            for (int k = half_branch_length - 1; k >= 0; k--)
            {
                float x = k * PIXEL_SPACING * cos(BRANCH_PITCH);
                float y = k * PIXEL_SPACING * sin(BRANCH_PITCH) + branch_height;
                PixInfo pix;
                pix.index = index + half_branch_length + (half_branch_length - 1 - k);
                pix.position = Vec3f(cos(angle) * x, y, sin(angle) * x);
                pix.outside = false;
                pi.push_back(pix);
            }
        }
    }

    return pi;
}

int64_t get_time()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000 + ts.tv_nsec;
}

Vec3f rgb_from_hsv(const Vec3f &hsv)
{
    float h = hsv.x;
    float s = hsv.y;
    float v = hsv.z;

    int i = floor(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6)
    {
    case 0:
        return Vec3f(v, t, p);
    case 1:
        return Vec3f(q, v, p);
    case 2:
        return Vec3f(p, v, t);
    case 3:
        return Vec3f(p, q, v);
    case 4:
        return Vec3f(t, p, v);
    case 5:
        return Vec3f(v, p, q);
    }
    return Vec3f(0, 0, 0); // should never reach here
}

void set_color(uint8_t *&ptr, const Vec3f &color)
{
    *(ptr++) = (int)(color.x * 255);
    *(ptr++) = (int)(color.y * 255);
    *(ptr++) = (int)(color.z * 255);
}

void set_color(uint8_t *ptr, int idx, const Vec3f &color)
{
    uint8_t *p = ptr + 3 * idx;
    set_color(p, color);
}

int main(int argc, char *argv[])
{
    constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
    uint8_t buffer[sz] = {};
    uint8_t *ptr;

    std::vector<int> strip_indexes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    auto pixInfo = get_pix_info();

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(PIXELS_PER_STRIP, LED_BRIGHTNESS);
    printf("compiled for %d strips\n", leds_num_strips());

    leds_clear();

    int64_t start = get_time();
    int count = 0;
    int64_t freecount = 0;
    for (float t = 0.0;; t += .0001)
    {
        uint64_t t_ns = get_time();
        // Manually and slowly build the color buffer. 3 bytes per pixel (RGB)
        // for PIXELS_PER_STRIP pixels and STRIPS strips
        ptr = buffer;
        for (int strip = 0; strip < STRIPS; strip++)
        {
            float hue = fmod(t + (strip * .04), 1.0);

            Vec3f orange(1.0, 0.15, 0.0);

            Vec3f rgb = rgb_from_hsv(Vec3f(hue, 1.0, 1.0));

            for (int led = 0; led < PIXELS_PER_STRIP; led++)
            {
                set_color(ptr, orange);
            }
        }

        for (int j = 0; j < RINGS * BRANCHES_PER_RING / 2; j++)
        {
            for (int i = 0; i < branchLen[0]; i++)
            {
                uint64_t fc = (t_ns / 20000000) + i + j * branchLen[0] * 2;
                int sidx = strip_indexes[(fc / PIXELS_PER_STRIP) % strip_indexes.size()];
                int pidx = fc % PIXELS_PER_STRIP;
                Vec3f purple(0.05, 0, 0.05);
                ptr = buffer + (sidx * PIXELS_PER_STRIP + pidx) * 3;
                set_color(ptr, purple);
            }
        }

        Vec3f blk(0, 0, 0);
        for (int strip = 0; strip < STRIPS; strip++)
        {
            uint8_t *pixels = buffer + (strip * PIXELS_PER_STRIP * 3);
            Vec3f red(50 / 255.0f, 0, 0);
            for (int led = 0; led < strip; led++)
            {
                set_color(pixels, red);
            }

            for (int b = 0; false && b < 4; b++)
            {
                pixels = buffer + ((strip * PIXELS_PER_STRIP + b * 74) * 3);
                int ring = strip / 2;
                for (int i = 0; i < ring * 4; i++)
                {
                    set_color(pixels, 37 + i, blk);
                    set_color(pixels, 36 - i, blk);
                }
            }
        }

        if (freecount > 0)
        {
            for (auto p : pixInfo)
            {
                set_color(buffer, p.index, Vec3f(0, 0.25, 0));
            }
        }

        // Send the buffer to the SMI buffer
        leds_set(buffer);

        // Actually send them to the LEDs:
        leds_send();

        count++;
        freecount++;
        if (count > 1000)
        {
            int64_t end = get_time();
            double delta = (end - start) * 1e-9;
            printf("fps = %d\n", int(1000 / delta));
            start = end;
            count = 0;
        }
    }
}

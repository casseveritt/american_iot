#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "smi_leds.h"

#include "linear.h"

using namespace r3;

#define NUM_STRIPS 16
#define NUM_PIXELS_PER_STRIP 296

#define LED_BRIGHTNESS 16

#define BRANCHES_PER_RING 8
#define BRANCHES_PER_STRIP 4

#define RINGS 7

// [0 .. branchLen/2 - 1] are the outside pixels starting at the trunk
// [branch_Len/2 ... branchLen - 1] are the inside pixels starting at the branch tip 
constexpr uint32_t branchLen[] = { 74, 74, 74, 74, 56, 38, 38, 20 };

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
    constexpr size_t sz = NUM_PIXELS_PER_STRIP * NUM_STRIPS * 3;
    uint8_t buffer[sz] = {};
    uint8_t *ptr;

    std::vector<int> strip_indexes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(NUM_PIXELS_PER_STRIP, LED_BRIGHTNESS);
    printf("compiled for %d strips\n", leds_num_strips());

    int64_t start = get_time();
    int count = 0;
    int64_t freecount = 0;
    for (float t = 0.0;; t += .0001)
    {
        uint64_t t_ns = get_time();
        // Manually and slowly build the color buffer. 3 bytes per pixel (RGB)
        // for NUM_PIXELS_PER_STRIP pixels and NUM_STRIPS strips
        ptr = buffer;
        for (int strip = 0; strip < NUM_STRIPS; strip++)
        {
            float hue = fmod(t + (strip * .04), 1.0);

            Vec3f orange(1.0, 0.15, 0.0);

            Vec3f rgb = rgb_from_hsv(Vec3f(hue, 1.0, 1.0));

            for (int led = 0; led < NUM_PIXELS_PER_STRIP; led++)
            {
                set_color(ptr, orange);
            }
        }

        for (int j = 0; j < RINGS * BRANCHES_PER_RING / 2; j++)
        {
            for (int i = 0; i < branchLen[0]; i++)
            {
                uint64_t fc = (t_ns / 20000000) + i + j * branchLen[0] * 2;
                int sidx = strip_indexes[(fc / NUM_PIXELS_PER_STRIP) % strip_indexes.size()];
                int pidx = fc % NUM_PIXELS_PER_STRIP;
                Vec3f purple(0.05, 0, 0.05);
                ptr = buffer + (sidx * NUM_PIXELS_PER_STRIP + pidx) * 3;
                set_color(ptr, purple);
            }
        }

        Vec3f blk(0, 0, 0);
        for (int strip = 0; strip < NUM_STRIPS; strip++)
        {
            uint8_t *pixels = buffer + (strip * NUM_PIXELS_PER_STRIP * 3);
            Vec3f red(50 / 255.0f, 0, 0);
            for (int led = 0; led < strip; led++)
            {
                set_color(pixels, red);
            }

            for (int b = 0; false && b < 4; b++)
            {
                pixels = buffer + ((strip * NUM_PIXELS_PER_STRIP + b * 74) * 3);
                int ring = strip / 2;
                for (int i = 0; i < ring * 4; i++)
                {
                    set_color(pixels, 37 + i, blk);
                    set_color(pixels, 36 - i, blk);
                }
            }
        }

        // Send the buffer to the SMI buffer
        leds_set(buffer);



        // Actually send them to the LEDs:
        leds_send();

        // Sleep for a while
        // usleep(50000);
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
	//usleep(800);
    }
}

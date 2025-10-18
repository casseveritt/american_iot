#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include<time.h>
#include<unistd.h>
#include <vector>

#include "smi_leds.h"

#define NUM_LEDS 296
#define NUM_STRIPS 16

int64_t get_time() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000 + ts.tv_nsec;
}

void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b) 
{
    int i = floor(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6)
    {
        case 0: *r = v, *g = t, *b = p; break;
        case 1: *r = q, *g = v, *b = p; break;
        case 2: *r = p, *g = v, *b = t; break;
        case 3: *r = p, *g = q, *b = v; break;
        case 4: *r = t, *g = p, *b = v; break;
        case 5: *r = v, *g = p, *b = q; break;
    }
}

int main(int argc, char *argv[])
{
    uint8_t buffer[NUM_LEDS * NUM_STRIPS * 3] = {};
    uint8_t *ptr;

    std::vector<int> strip_indexes = {0, 1, 2, 3, 4, 5};

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(NUM_LEDS, 35);
    printf("compiled for %d strips\n", leds_num_strips());

    int64_t start = get_time();
    int count = 0;
    int64_t freecount = 0;
    for(float t = 0.0;; t += .0002)
    {
        // Manually and slowly build the color buffer. 3 bytes per pixel (RGB)
        // for NUM_LEDS pixels and NUM_STRIPS strips
        ptr = buffer;
        for(int strip = 0; strip < NUM_STRIPS; strip++)
        {
            float hue = fmod(t + ((strip & (~1)) * .04), 1.0);
            float r, g, b;

            hsv_to_rgb(hue, 1.0, 1.0, &r, &g, &b);
            for(int led = 0; led < NUM_LEDS; led++)
            {
                *(ptr++) = (int)(r * 255);
                *(ptr++) = (int)(g * 255);
                *(ptr++) = (int)(b * 255);
            }

            uint8_t* pixels = buffer + (strip * NUM_LEDS * 3);
            for (int led = 0; led < strip; led++) {
                pixels[3 * led + 0] = uint8_t(50);
                pixels[3 * led + 1] = uint8_t(0);
                pixels[3 * led + 2] = uint8_t(0);
            }

        }

    int sidx = strip_indexes[(freecount / NUM_LEDS) % strip_indexes.size()];
    int pidx = freecount % NUM_LEDS;
	ptr = buffer + (sidx * NUM_LEDS + pidx) * 3;
	*(ptr++) = (int)(0);
	*(ptr++) = (int)(0);
	*(ptr++) = (int)(0);

        // Send the buffer to the SMI buffer
        leds_set(buffer);

        // Actually send them to the LEDs:
        leds_send();

        // Sleep for a while
        // usleep(50000);
	count++;
	freecount++;
	if (count > 1000) {
	    int64_t end = get_time();
            double delta = (end - start) * 1e-9;
	    printf("fps = %d\n", int(1000 / delta));
	    start = end;
	    count = 0;
	}
    }
}

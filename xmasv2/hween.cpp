#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "smi_leds.h"

#include "color.h"
#include "hsv.h"
#include "linear.h"
#include "time.h"
#include "tree.h"

using namespace r3;

int main(int argc, char *argv[])
{
    constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
    uint8_t buffer[sz] = {};
    uint8_t *ptr;

    auto pixInfo = get_pix_info();

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(PIXELS_PER_STRIP, LED_BRIGHTNESS);
    printf("compiled for %d strips\n", leds_num_strips());

    leds_clear();

    int64_t start = get_time_nsec();
    int count = 0;
    int64_t freecount = 0;
    for (float t = 0.0;; t += .004)
    {
        uint64_t t_ns = get_time_nsec();

        if (freecount > 0)
        {
            for (auto p : pixInfo)
            {
                float theta = atan2(p.position.z, p.position.x) + M_PI;
                Vec3f rgb = rgb_from_hsv(Vec3f(fmod(t + theta / (M_PI * 2.0f), 1.0), 1.0, 1.0));
                if (fmod(p.position.y + 0.1 * t_ns * 1e-9, 0.2f) < 0.1f)
                {
                    rgb *= 0.2f;
                }
                set_color(buffer, p.index, rgb);
            }
        }

#define SHOW_STRIP_INDEX 0
#if SHOW_STRIP_INDEX
        Vec3f red(0.2f, 0, 0);
        for (int strip = 0; strip < STRIPS; strip++)
        {
            uint8_t *pixels = buffer + (strip * PIXELS_PER_STRIP * 3);
            for (int led = 0; led < strip; led++)
            {
                set_color(buffer, strip * PIXELS_PER_STRIP + led, red);
            }
        }
#endif

        // Send the buffer to the SMI buffer
        leds_set(buffer);

        // Actually send them to the LEDs:
        leds_send();

        count++;
        freecount++;
        if (count > 1000)
        {
            int64_t end = get_time_nsec();
            double delta = (end - start) * 1e-9;
            printf("fps = %d\n", int(1000 / delta));
            start = end;
            count = 0;
        }
        usleep(700);
    }
}

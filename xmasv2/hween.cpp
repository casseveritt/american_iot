#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <unordered_map>
#include <vector>

#include "smi_leds.h"

#include "color.h"
#include "hsv.h"
#include "linear.h"
#include "time.h"
#include "tree.h"

#define NUM_BUFFERS 3

using namespace r3;

typedef void (*PixelFunc)(std::span<PixInfo> pixInfo, uint8_t *buffer, float t_s);

bool terminate = false;

enum Status
{
    Waiting,
    Processing,
    Done
};

struct Work
{
    PixelFunc pixelFunc;
    std::span<PixInfo> pixInfo;
    uint8_t *buffer;
    float t_s;
    Status status = Waiting;
};

std::deque<Work> workQueue;

std::mutex workMutex;

void push_work(const Work &work)
{
    std::scoped_lock(workMutex);
    workQueue.push_back(work);
}

std::optional<Work> pop_work()
{
    std::scoped_lock(workMutex);
    if (workQueue.size() == 0)
    {
        return {};
    }
    auto work = workQueue.front();
    //printf("pop_work() status = %d\n", int(work.status));
    if (work.status != Done)
    {
        return {};
    }
    workQueue.pop_front();
    return work;
}

std::optional<Work *> fetch_work()
{
    std::scoped_lock(workMutex);
    if (workQueue.size() == 0)
    {
        return {};
    }
    for (auto& w : workQueue) {
        if (w.status == Waiting)
        {
            w.status = Processing;
            return &w;
        }
    }
    return {};
}

void mark_work_done(Work *w)
{
    std::scoped_lock(workMutex);
    w->status = Done;
}

void color_pixels(std::span<PixInfo> pixInfo, uint8_t *buffer, float t)
{
    for (auto p : pixInfo)
    {
        float theta = atan2(p.position.z, p.position.x) + M_PI;
        Vec3f rgb = rgb_from_hsv(Vec3f(fmod(t + theta / (M_PI * 2.0f), 1.0), 1.0, 1.0));

        constexpr float width = 0.2;
        constexpr float half_width = width / 2.0f;
        float sc = pow(fabs(fmod(p.position.y + 0.2 * t, width) - half_width) / half_width, 3.0);
        rgb *= sc;
        set_color(buffer, p.index, rgb);
    }
}

void show_strip_index(uint8_t *buffer)
{
    for (int strip = 0; strip < STRIPS; strip++)
    {
        uint8_t *pixels = buffer + (strip * PIXELS_PER_STRIP * 3);
        Vec3f color = rgb_from_hsv(Vec3f(float(strip) / STRIPS, 1.0f, 1.0f));
        for (int led = 0; led < PIXELS_PER_STRIP; led++)
        {
            set_color(buffer, strip * PIXELS_PER_STRIP + led, color);
        }
    }
}

void worker(int id)
{
    printf("Starting worker %d\n", id);
    while (terminate == false)
    {
        std::optional<Work *> work = fetch_work();
        if (work)
        {
            auto &w = *work.value();
            w.pixelFunc(w.pixInfo, w.buffer, w.t_s);
            w.status = Done;
            mark_work_done(&w);
        }
        else
        {
            usleep(100);
        }
    }
}

int main(int argc, char *argv[])
{
    constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
    uint8_t buffers[NUM_BUFFERS][sz] = {};

    auto pixInfo = get_pix_info();

    // initialize the smi_leds module, starting with a 35% brightness
    leds_init(PIXELS_PER_STRIP, LED_BRIGHTNESS);
    printf("compiled for %d strips\n", leds_num_strips());

    leds_clear();

    std::deque<float> job_times;

    int64_t start = get_time_nsec();
    int count = 0;
    int64_t freecount = 0;

    std::thread t1(worker, 1);
    std::thread t2(worker, 2);
    std::thread t3(worker, 3);
    while (true)
    {
        uint64_t t_ns = get_time_nsec();
        float t_s = t_ns * 1e-9;

        uint8_t *buffer = buffers[freecount % NUM_BUFFERS];

        Work w = {color_pixels, pixInfo, buffer, t_s};

        job_times.push_back(t_s);
        push_work(w);

        if (job_times.size() < NUM_BUFFERS)
        {
            usleep(1000);
            continue;
        }

        do
        {
            auto w = pop_work();
            if (w)
            {
                buffer = w.value().buffer;
		job_times.pop_front();
                break;
            } 
	    usleep(1000);

        } while (true);

        // show_strip_index(buffer);

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
        usleep(800);
    }
}

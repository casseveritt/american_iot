#include <getopt.h>
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

#include "cmap.h"
#include "color.h"
#include "get_time.h"
#include "hsv.h"
#include "linear.h"
#include "perlin.h"
#include "smi_leds.h"
#include "tree.h"

#define NUM_BUFFERS 3

#define LED_BRIGHTNESS 16

using namespace r3;

namespace {

struct TreeShader {
  virtual void shade(std::span<PixInfo> pixInfo, uint8_t *buffer, float t) = 0;
};

bool terminate = false;

enum Status { Waiting, Processing, Done };

struct Work {
  TreeShader *shader;
  std::span<PixInfo> pixInfo;
  uint8_t *buffer;
  float t_s;
  Status status = Waiting;
};

std::deque<Work> workQueue;

std::mutex workMutex;

int work_size() { return int(workQueue.size()); }

void push_work(const Work &work) {
  std::scoped_lock lock(workMutex);
  workQueue.push_back(work);
}

std::optional<Work> pop_work() {
  std::scoped_lock lock(workMutex);
  if (workQueue.size() == 0) {
    return {};
  }
  auto work = workQueue.front();
  // printf("pop_work() status = %d\n", int(work.status));
  if (work.status != Done) {
    return {};
  }
  workQueue.pop_front();
  return work;
}

std::optional<Work *> fetch_work() {
  std::scoped_lock lock(workMutex);
  if (workQueue.size() == 0) {
    return {};
  }
  for (auto &w : workQueue) {
    if (w.status == Waiting) {
      w.status = Processing;
      return &w;
    }
  }
  return {};
}

void mark_work_done(Work *w) {
  std::scoped_lock lock(workMutex);
  w->status = Done;
}

struct HueShader : public TreeShader {
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer, float t) override {
    for (auto p : pixInfo) {
      float theta = atan2(p.position.z, p.position.x) + M_PI;
      Vec3f rgb = rgb_from_hsv(
          Vec3f(fmod((t * 0.07f) + (theta / (M_PI * 2.0f)), 1.0), 1.0, 1.0));

      constexpr float width = 0.2;
      constexpr float half_width = width / 2.0f;
      float sc =
          pow(fabs(fmod(p.position.y + (0.03f * t), width) - half_width) /
                  half_width,
              3.0);
      rgb *= sc;
      set_color(buffer, p.index, rgb);
    }
  }
};

struct NoiseShader : public TreeShader {
  const ColorMap &colorMap;
  float scale;
  float speed;

  NoiseShader(const ColorMap &cmap, float scaleIn, float speedIn)
      : colorMap(cmap), scale(scaleIn), speed(speedIn) {}

  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer, float t) override {
    for (auto p : pixInfo) {
      Vec3f pp = p.position * scale;
      float noise =
          0.5f + 0.5f * ImprovedNoise::noise(pp.x, pp.y + t * speed, pp.z);
      auto c = colorMap.lookupClamped(noise);
      set_color(buffer, p.index, c);
    }
  }
};

struct SparkleState {
  float sparkleTime = 0.0f;
  float sparkleDelay = 0.0f;
};
#if 0
void eiffel(std::span<PixInfo> pixInfo, uint8_t *buffer, float t) {
  static float old_t = 0.0f;
  float dt = t - old_t;
  int dt = frameTime.dt();
  int modeMs = frameTime.t0() - newProgStartMs;
  int cmap = (newProgStartMs & 128) > 0 ? 1 : 0;

  for (auto &li : led) {
    const int i = li.index;
    float t = 0.25f + sparkleTime[i] / 1000.0f;  // nominal sparkle duration
    strip[i] = sparkle[cmap].lookup(t);
    sparkleTime[i] += dt;
    if (sparkleTime[i] >= sparkleDelay[i]) {
      sparkleTime[i] = 0;
      sparkleDelay[i] = (rand() % (3000 - 1000 + 1)) + 1000;
    }
  }
  if (modeMs < 2000) {
    clear(CRGB::Black);
  }
  old_t = t;
}
#endif

void show_strip_index(uint8_t *buffer) {
  for (int strip = 0; strip < STRIPS; strip++) {
    uint8_t *pixels = buffer + (strip * PIXELS_PER_STRIP * 3);
    Vec3f color = rgb_from_hsv(Vec3f(float(strip) / STRIPS, 1.0f, 1.0f));
    for (int led = 0; led < PIXELS_PER_STRIP; led++) {
      set_color(buffer, strip * PIXELS_PER_STRIP + led, color);
    }
  }
}

void worker(int id) {
  printf("Starting worker %d\n", id);
  while (!terminate) {
    std::optional<Work *> work = fetch_work();
    if (work) {
      auto &w = *work.value();
      w.shader->shade(w.pixInfo, w.buffer, w.t_s);
      w.status = Done;
      mark_work_done(&w);
    } else {
      usleep(100);
    }
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
  uint8_t buffers[NUM_BUFFERS][sz] = {};

  init_cmaps();

  auto pixInfo = get_pix_info();

  // initialize the smi_leds module, starting with a 35% brightness
  leds_init(PIXELS_PER_STRIP, 50);
  printf("compiled for %d strips\n", leds_num_strips());

  leds_clear();

  int64_t epoch = get_time_nsec();
  int64_t start = epoch;
  int64_t count = 0;

  HueShader hueShader;
  NoiseShader iceShader(blueBlack, 20.0f, 0.5f);
  NoiseShader halloweenShader(halloween, 5.0f, 0.7f);

  float progCycleTime = 180.0f;  // 3 minutes per program...
  std::vector<TreeShader *> progs = {&iceShader, &halloweenShader, &hueShader};
  auto randomProg = [&progs]() -> TreeShader * {
    return progs[rand() % progs.size()];
  };

  std::unordered_map<std::string, TreeShader *> progMap = {
      {"ice_noise", &iceShader},
      {"halloween_noise", &halloweenShader},
      {"hue", &hueShader}};

  TreeShader *startProg = randomProg();

  // Parse command line arguments
  int opt;
  static struct option long_options[] = {{"listprogs", no_argument, 0, 0},
                                         {"prog", required_argument, 0, 0},
                                         {0, 0, 0, 0}};

  int option_index = 0;
  while ((opt = getopt_long(argc, argv, "", long_options, &option_index)) !=
         -1) {
    switch (opt) {
      case 0:                     // long option
        if (option_index == 0) {  // --listprogs
          printf("Available programs:\n");
          for (const auto &pair : progMap) {
            printf("  %s\n", pair.first.c_str());
          }
          exit(0);
        }
        if (option_index == 1) {  // --prog
          std::string progName = optarg;
          auto it = progMap.find(progName);
          if (it != progMap.end()) {
            startProg = it->second;
            printf("Starting with program: %s\n", progName.c_str());
          } else {
            printf("Unknown program: %s\n", progName.c_str());
          }
        }
        break;
      default:
        break;
    }
  }

  std::thread t1(worker, 1);
  std::thread t2(worker, 2);
  // std::thread t3(worker, 3);
  float prev_t_s = 0.0f;
  TreeShader *prog = startProg;

  while (true) {
    uint64_t t_ns = get_time_nsec() - epoch;
    float t_s = t_ns * 1e-9;

    uint8_t *buffer = buffers[count % NUM_BUFFERS];

    if (int(t_s / progCycleTime) != int(prev_t_s / progCycleTime)) {
      prog = randomProg();
    }

    Work w = {prog, pixInfo, buffer, t_s};

    push_work(w);

    int jobs = work_size();

    if (jobs < NUM_BUFFERS) {
      usleep(1000);
      continue;
    }

    do {
      auto w = pop_work();
      if (w) {
        buffer = w.value().buffer;
        break;
      }
      usleep(100);
    } while (true);

    // show_strip_index(buffer);

    // Send the buffer to the SMI buffer
    leds_set(buffer);

    // Actually send them to the LEDs:
    leds_send();

    count++;
    if ((count % 1000) == 999) {
      int64_t end = get_time_nsec();
      double delta = (end - start) * 1e-9;
      printf("fps = %d\n", int(1000 / delta));
      start = end;
    }
    usleep(1500);
    prev_t_s = t_s;
  }
}

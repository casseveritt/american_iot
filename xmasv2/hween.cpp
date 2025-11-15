#include <getopt.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <optional>
#include <span>
#include <string>
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
#include "worker.h"

#define NUM_BUFFERS 3

#define LED_BRIGHTNESS 16

constexpr float k2pi = 2.0f * M_PI;

using namespace r3;

namespace {

struct HueShader : public TreeShader {
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;  // Convert to seconds
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

struct RandShader : public TreeShader {
  std::vector<Color> colors;
  float fracOn = 1.00f;
  float power = 4.2f;

  RandShader(float fracOnIn = 1.0f, float powerIn = 4.2f)
      : fracOn(fracOnIn), power(powerIn) {}

  void init(std::span<PixInfo> pixInfo, uint64_t t_ns) override {
    colors.resize(pixInfo.size());
    for (auto &c : colors) {
      c = BLACK;
    }
  }

  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;  // Convert to seconds
    int idx = 0;
    float onRand = rand() / float(RAND_MAX);
    Color randomColor = BLACK;
    if (onRand <= fracOn) {
      randomColor = Color(rand() / float(RAND_MAX), rand() / float(RAND_MAX),
                          rand() / float(RAND_MAX));
      randomColor.x = pow(randomColor.x, power);
      randomColor.y = pow(randomColor.y, power);
      randomColor.z = pow(randomColor.z, power);
    }
    int randomIndex = rand() % colors.size();
    colors[randomIndex] = randomColor;

    for (auto p : pixInfo) {
      Color &c = colors[idx];
      set_color(buffer, p.index, c);
      idx++;
    }
  }
};

struct NoiseShader : public TreeShader {
  const ColorMap &colorMap;
  float scale;
  float speed;

  NoiseShader(const ColorMap &cmap, float scaleIn, float speedIn)
      : colorMap(cmap), scale(scaleIn), speed(speedIn) {}

  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    for (auto p : pixInfo) {
      Vec3f pp = p.position * scale;
      float noise =
          0.5f + 0.5f * ImprovedNoise::noise(pp.x, pp.y + t * speed, pp.z);
      auto c = colorMap.lookupClamped(noise);
      set_color(buffer, p.index, c);
    }
  }
};

struct EiffelShader : public TreeShader {
  struct SparkleState {
    float sparkleStart = -5.0f;
  };

  EiffelShader(int numPixels, const ColorMap &cmapIn) : cmap(cmapIn) {
    sparkleStates.resize(numPixels);
  }
  const ColorMap &cmap;
  void init(std::span<PixInfo> pixInfo, uint64_t t_ns) override {
    double t = t_ns * 1e-9;
  }
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    int count = 0;

    for (auto p : pixInfo) {
      auto &state = sparkleStates[count++];
      const float nominalSparkleDuration = 0.45f;  // in seconds
      float dt = t - state.sparkleStart;
      if (dt > nominalSparkleDuration) {
        state.sparkleStart = t + (rand() % 8000) * 0.001f;  // 0-5 seconds
      }
      dt = t - state.sparkleStart;
      Color color = BLACK;
      if (dt >= 0.0f) {
        float sparkleProgress = dt / nominalSparkleDuration;
        color = cmap.lookupClamped(sparkleProgress);
      }
      set_color(buffer, p.index, color);
    }
  }
  std::vector<SparkleState> sparkleStates;
};

struct RotYShader : public TreeShader {
  const Vec3f center = {0.0f, 0.75f, 0.0f};

  const int n = 8;  // number of sections
  const float secPerRev = 32.0f;
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    // make a function that is sloped, then flat for a bit, then sloped, etc.

    for (const auto &p : pixInfo) {
      Vec3f pos = p.position - center;
      // the difference in time between the first and last pixel transitions
      // is phase seconds...
      float phase = 0.5f * (atan2(pos.x, pos.z) / k2pi + 0.5f);
      const float revProgress = fmod((t + phase) / secPerRev, 1.0f);
      const float sectionWidth = 1.0f / n;
      const float slope = 4.0f;
      const float sectionStep = floor(revProgress * n);
      const float sectionProgress =
          std::min(fmod(revProgress * n, 1.0f) * slope, 1.0f);
      const float rev = (sectionStep + sectionProgress) * sectionWidth;

      Color color = rgbcmy.lookupWrapped(rev + t * 0.001321f);
      set_color(buffer, p.index, color);
    }
  }
};

struct Twist : public TreeShader {
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    constexpr int twistPeriod = 12.0f;  // seconds
    float twistRevsPerMeter =
        2.5f * sin(k2pi * (fmod(t / twistPeriod, 1.0f)) + 1.0);

    auto &cmap = blueBlack;

    constexpr float secondsPerRev = 1.0f;
    const float baseRevs = fmod(t / secondsPerRev, 1.0f);

    for (const auto &pix : pixInfo) {
      Vec3f p = pix.position;
      p = Rotationf(Vec3f(0, 1, 0),
                    k2pi * (-twistRevsPerMeter * p.y + baseRevs))
              .Rotate(p);
      float theta = atan2(p.x, p.z) / k2pi + 0.5f;
      auto color = cmap.lookupWrapped(theta);
      set_color(buffer, pix.index, color);
    }
  }
};

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
  NoiseShader redWhiteShader(cm[0], 25.0f, 0.15f);
  NoiseShader halloweenShader(halloween, 5.0f, 0.7f);
  EiffelShader eiffelShaderLucas(pixInfo.size(), sparkle[0]);
  EiffelShader eiffelShaderCass(pixInfo.size(), sparkle[1]);
  RandShader randomShader;
  RandShader random2Shader(0.25f, 8.0f);
  RotYShader rotYShader;
  Twist twistShader;

  float progCycleTime = 180.0f;  // 3 minutes per program...
  std::vector<TreeShader *> progs = {
      &iceShader,         &redWhiteShader,   &halloweenShader, &hueShader,
      &eiffelShaderLucas, &eiffelShaderCass, &rotYShader,      &randomShader,
      &random2Shader,     &twistShader};
  auto randomProg = [&progs]() -> TreeShader * {
    return progs[rand() % progs.size()];
  };

  std::unordered_map<std::string, TreeShader *> progMap = {
      {"ice_noise", &iceShader},
      {"red_white_noise", &redWhiteShader},
      {"halloween_noise", &halloweenShader},
      {"hue", &hueShader},
      {"eiffel_lucas", &eiffelShaderLucas},
      {"eiffel_cass", &eiffelShaderCass},
      {"random", &randomShader},
      {"random2", &random2Shader},
      {"rot_y", &rotYShader},
      {"twist", &twistShader}};

  TreeShader *startProg = randomProg();
  // Parse command line arguments
  int opt;
  static struct option long_options[] = {{"listprogs", no_argument, 0, 0},
                                         {"prog", required_argument, 0, 0},
                                         {"cycletime", required_argument, 0, 0},
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
        if (option_index == 2) {  // --cycletime
          float cycleTime = std::stof(optarg);
          progCycleTime = cycleTime;
          printf("Setting program cycle time: %f\n", cycleTime);
        }
        break;
      default:
        break;
    }
  }

  add_worker(1);
  // add_worker(2);
  float prev_t_s = 0.0f;
  TreeShader *prog = startProg;
  prog->init(pixInfo, epoch);

  while (true) {
    uint64_t t_ns = get_time_nsec() - epoch;
    float t_s = t_ns * 1e-9;

    uint8_t *buffer = buffers[count % NUM_BUFFERS];

    if (int(t_s / progCycleTime) != int(prev_t_s / progCycleTime)) {
      prog = randomProg();
      prog->init(pixInfo, t_ns);
    }

    Work w = {prog, pixInfo, buffer, t_ns};

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
    if ((count % 1) == 0) {
      leds_set(buffer);
    } else {
      leds_clear();
    }

    // Actually send them to the LEDs:
    leds_send();

    count++;
    if ((count % 1000) == 999) {
      int64_t end = get_time_nsec();
      double delta = (end - start) * 1e-9;
      printf("fps = %d\n", int(1000 / delta));
      start = end;
    }
    usleep(3000);
    prev_t_s = t_s;
  }
}

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

inline float rnd() { return rand() / float(RAND_MAX); }

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
    float onRand = rnd();
    Color randomColor = BLACK;
    if (onRand <= fracOn) {
      randomColor = Color(rnd(), rnd(), rnd());
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
      float noise_up = ImprovedNoise::noise(pp.x, pp.y + t * speed, pp.z);
      float noise_dn = ImprovedNoise::noise(pp.x, pp.y - t * speed, pp.z);
      float noise_c = (noise_up + noise_dn) * 0.5f;
      float noise = noise_c * 0.5f + 0.5f;

      auto c = colorMap.lookupClamped(noise);
      set_color(buffer, p.index, c);
    }
  }
};

struct EiffelShader : public TreeShader {
  struct SparkleState {
    float start = -5.0f;
    float duration = 0.45f;
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
      float dt = t - state.start;
      if (dt > state.duration) {
        state.start = t + (rand() % 8000) * 0.001f;
        state.duration = 0.45f + (rand() % 2000) * 0.0001f;
      }
      dt = t - state.start;
      Color color = BLACK;
      if (dt >= 0.0f) {
        float progress = dt / state.duration;
        color = cmap.lookupClamped(progress);
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

struct SphereShader : public TreeShader {
  struct Sphere {
    Vec3f position;
    Vec3f velocity;
    float radius;
    Color color;
  };

  std::vector<Sphere> spheres;
  const int numSpheres = 25;
  double lastTime = 0.0;

  void init(std::span<PixInfo> pixInfo, uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    lastTime = t;
    spheres.clear();

    for (int i = 0; i < numSpheres; i++) {
      Sphere sphere;
      // Random starting position within cylinder bounds (radius 1.5m, height
      // 0-2.5m)
      float r = rnd() * 1.5f;
      float theta = rnd() * k2pi;
      sphere.position = Vec3f(r * cos(theta),  // x within cylinder
                              rnd() * 2.5f,    // y: 0 to 2.5
                              r * sin(theta)   // z within cylinder
      );

      // Random velocity direction with varying speeds (0.1 to 0.5 m/s)
      float speed = 0.1f + rnd() * 0.4f;
      float vtheta = rnd() * k2pi;
      float vphi = rnd() * M_PI - M_PI / 2.0f;
      sphere.velocity =
          Vec3f(cos(vphi) * cos(vtheta) * speed, sin(vphi) * speed,
                cos(vphi) * sin(vtheta) * speed);

      // Random radius (0.1 to 0.3 meters)
      sphere.radius = 0.05f + rnd() * 0.1f;

      // Random bright color
      auto rc = []() -> float { return pow(rnd(), 2.2f); };
      sphere.color = Color(rc(), rc(), rc());

      spheres.push_back(sphere);
    }
  }

  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    double dt = t - lastTime;
    lastTime = t;

    // Update sphere positions
    for (auto &sphere : spheres) {
      sphere.position = sphere.position + sphere.velocity * dt;

      // Cylindrical boundary reflection (radius 1.5m, y: 0 to 2.5m)
      float radialDist = sqrt(sphere.position.x * sphere.position.x +
                              sphere.position.z * sphere.position.z);

      if (radialDist > 1.5f) {
        // Hit cylinder wall - reflect velocity
        Vec3f radialDir(sphere.position.x, 0.0f, sphere.position.z);
        float len = radialDir.Length();
        if (len > 0.0f) {
          radialDir = radialDir / len;  // Normalize
          // Reflect velocity around the normal (radial direction)
          Vec3f radialVel = radialDir * (sphere.velocity.Dot(radialDir));
          Vec3f tangentialVel = sphere.velocity - radialVel;
          sphere.velocity = tangentialVel - radialVel;

          // Push position back inside cylinder
          float scale = 1.5f / radialDist;
          sphere.position.x *= scale;
          sphere.position.z *= scale;
        }
      }

      // Y-axis boundaries (top and bottom of cylinder)
      if (sphere.position.y < 0.0f) {
        sphere.velocity.y = -sphere.velocity.y;
        sphere.position.y = 0.0f;
      } else if (sphere.position.y > 2.5f) {
        sphere.velocity.y = -sphere.velocity.y;
        sphere.position.y = 2.5f;
      }
    }

    // Render pixels
    for (const auto &pix : pixInfo) {
      Color finalColor = BLACK;
      float nearestDist = std::numeric_limits<float>::max();
      const Sphere *nearestSphere = nullptr;

      // Find the nearest sphere that intersects this pixel
      for (const auto &sphere : spheres) {
        float dist = (pix.position - sphere.position).Length();

        if (dist < sphere.radius * 1.2f) {
          if (dist < nearestDist) {
            nearestDist = dist;
            nearestSphere = &sphere;
          }
        }
      }

      // Color based on nearest sphere
      if (nearestSphere != nullptr) {
        if (nearestDist < nearestSphere->radius) {
          // Inside sphere - use full color
          finalColor = nearestSphere->color;
        } else {
          // Edge softening
          float blend = 1.0f - (nearestDist - nearestSphere->radius) /
                                   (nearestSphere->radius * 0.2f);
          finalColor = nearestSphere->color * blend;
        }
      }

      set_color(buffer, pix.index, finalColor);
    }
  }
};

struct ParticleShader : public TreeShader {
  struct Particle {
    Vec3f position;
    Vec3f velocity;
    Color color;
  };

  std::vector<Particle> particles;
  const int numParticles = 80;
  const float repulsionDist = 0.04f;       // 4 cm
  const float attractionCutoff = 0.5f;     // 50 cm - max attraction distance
  const float illuminationRadius = 0.15f;  // 15 cm
  const float cylinderRadius = 1.5f;
  const float cylinderHeight = 2.5f;
  double lastTime = 0.0;

  void init(std::span<PixInfo> pixInfo, uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    lastTime = t;
    particles.clear();

    for (int i = 0; i < numParticles; i++) {
      Particle particle;

      // Random starting position uniformly distributed within cylinder
      // Use sqrt(r) for uniform spatial distribution (not biased to center)
      float r = rnd() * cylinderRadius;
      float theta = rnd() * k2pi;
      particle.position =
          Vec3f(r * cos(theta), rnd() * cylinderHeight, r * sin(theta));

      // Random initial velocity in all directions
      particle.velocity = Vec3f((rnd() - 0.5f) * 0.2f, (rnd() - 0.5f) * 0.2f,
                                (rnd() - 0.5f) * 0.2f);

      // Random bright color
      particle.color = Color(rnd(), rnd(), rnd());

      particles.push_back(particle);
    }
  }

  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = t_ns * 1e-9;
    double dt = t - lastTime;
    lastTime = t;

    // Limit dt to avoid instability
    dt = std::min(dt, 0.1);

    // Update particle physics
    for (size_t i = 0; i < particles.size(); i++) {
      auto &p1 = particles[i];
      Vec3f force(0.0f, 0.0f, 0.0f);

      // Calculate forces from other particles
      for (size_t j = 0; j < particles.size(); j++) {
        if (i == j) continue;

        auto &p2 = particles[j];
        Vec3f delta = p1.position - p2.position;
        float dist = delta.Length();

        if (dist < 0.001f) continue;  // Avoid division by zero

        Vec3f direction = delta / dist;

        float forceMag = 0.0f;

        if (dist < repulsionDist) {
          // Repulsive force: zero at repulsionDist, increases as distance
          // approaches zero Using inverse relationship: stronger as distance
          // decreases
          float ratio = dist / repulsionDist;  // 0 to 1
          forceMag =
              0.5f * (1.0f / ratio -
                      1.0f);  // Goes to infinity as ratio->0, zero at ratio=1
          force = force + direction * forceMag;
        } else {
          // Attractive force: maximum at 2*repulsionDist, exponential falloff
          // At 3*repulsionDist, should be 0.1 of maximum
          // exp(-k*(3-2)) = 0.1, so k = -ln(0.1) = 2.3026
          float maxAttractionDist = 2.0f * repulsionDist;
          float attrStrength = 0.1f;  // Base attraction strength

          if (dist > maxAttractionDist) {
            // Exponential falloff after maximum
            float exponent =
                2.3026f * (dist - maxAttractionDist) / repulsionDist;
            forceMag = attrStrength * exp(-exponent);
          } else {
            // Linear ramp from 0 at repulsionDist to max at 2*repulsionDist
            float ratio = (dist - repulsionDist) / repulsionDist;  // 0 to 1
            forceMag = attrStrength * ratio;
          }
          force = force - direction * forceMag;
        }
      }

      // Update velocity and position (no damping for energy conservation)
      p1.velocity = p1.velocity + force * dt;

      // Limit max velocity to 0.10 m/s
      float speed = p1.velocity.Length();
      if (speed > 0.10f) {
        p1.velocity = p1.velocity * (0.10f / speed);
      }

      p1.position = p1.position + p1.velocity * dt;

      // Cylindrical boundary reflection (elastic)
      float radialDist =
          sqrt(p1.position.x * p1.position.x + p1.position.z * p1.position.z);

      if (radialDist > cylinderRadius) {
        Vec3f radialDir(p1.position.x, 0.0f, p1.position.z);
        float len = radialDir.Length();
        if (len > 0.0f) {
          radialDir = radialDir / len;
          Vec3f radialVel = radialDir * (p1.velocity.Dot(radialDir));
          Vec3f tangentialVel = p1.velocity - radialVel;
          p1.velocity = tangentialVel - radialVel;  // Elastic reflection

          float scale = cylinderRadius / radialDist;
          p1.position.x *= scale;
          p1.position.z *= scale;
        }
      }

      // Y-axis boundaries (elastic)
      if (p1.position.y < 0.0f) {
        p1.velocity.y = -p1.velocity.y;
        p1.position.y = 0.0f;
      } else if (p1.position.y > cylinderHeight) {
        p1.velocity.y = -p1.velocity.y;
        p1.position.y = cylinderHeight;
      }
    }

    // Render pixels
    for (const auto &pix : pixInfo) {
      Color finalColor = BLACK;

      // Accumulate light additively from all particles within range
      float maxLightDist =
          2.0f * repulsionDist;  // Light reaches to 2x repulsion radius

      for (const auto &particle : particles) {
        float dist = (pix.position - particle.position).Length();

        if (dist < maxLightDist) {
          // Intensity falls off linearly with distance
          float intensity = std::max(0.0f, 1.0f - (dist / maxLightDist));
          intensity = pow(intensity, 2.2f);  // Sharper falloff
          finalColor = finalColor + particle.color * intensity;
        }
      }

      // Clamp to prevent oversaturation
      finalColor.x = std::min(finalColor.x, 1.0f);
      finalColor.y = std::min(finalColor.y, 1.0f);
      finalColor.z = std::min(finalColor.z, 1.0f);

      set_color(buffer, pix.index, finalColor);
    }
  }
};

struct Calib : public TreeShader {
  void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
             uint64_t t_ns) override {
    double t = (t_ns * 1e-9) * 60.0f;

    for (const auto &pix : pixInfo) {
      Vec3f p = pix.position;
      float theta = (atan2(p.x, p.z) / k2pi + 0.5f) * 36.0f;
      int theta_i = int(theta) % 36;
      int t_i = int(t) % 36;
      float r = sqrt(p.x * p.x + p.z * p.z);
      constexpr float epsilon = 2.0f / 255.0f;
      Color color(epsilon, epsilon, epsilon);  // almost black
      if (theta_i == t_i) {
        color = WHITE;
      }
      set_color(buffer, pix.index, color);
    }
  }
};

}  // namespace

int main(int argc, char *argv[]) {
  constexpr size_t sz = PIXELS_PER_STRIP * STRIPS * 3;
  uint8_t buffers[NUM_BUFFERS][sz] = {};

  // Seed the random number generator with current time
  srand(time(NULL));

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
  NoiseShader rgbishShader(rgbish, 20.0f, 0.05f);
  NoiseShader halloweenShader(sparkle[0], 20.0f, 0.5f);
  NoiseShader redWhiteShader(cm[0], 25.0f, 0.15f);
  // NoiseShader halloweenShader(halloween, 5.0f, 0.7f);
  EiffelShader eiffelShaderLucas(pixInfo.size(), sparkle[0]);
  EiffelShader eiffelShaderCass(pixInfo.size(), sparkle[1]);
  RandShader randomShader;
  RandShader random2Shader(0.25f, 8.0f);
  RotYShader rotYShader;
  Twist twistShader;
  SphereShader sphereShader;
  ParticleShader particleShader;
  Calib calibShader;

  float progCycleTime = 180.0f;  // 3 minutes per program...
  std::vector<TreeShader *> progs = {
      &iceShader,         &redWhiteShader,   &halloweenShader, &hueShader,
      &eiffelShaderLucas, &eiffelShaderCass, &rotYShader,      &randomShader,
      &random2Shader,     &twistShader,      &rgbishShader,    &sphereShader};
  auto randomProg = [&progs](TreeShader *currProg = nullptr) -> TreeShader * {
    TreeShader *prog = currProg;
    while (prog == currProg) {
      prog = progs[rand() % progs.size()];
    }
    return prog;
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
      {"twist", &twistShader},
      {"rgbish_noise", &rgbishShader},
      {"sphere", &sphereShader},
      {"particle", &particleShader},
      {"calib", &calibShader}};

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

  uint8_t max_brightness = prog->get_max_brightness();
  uint8_t curr_brightness = 0;
  float ramp_time = 2.0f;
  float dead_time = 0.5f;
  float trans_time = ramp_time + dead_time;

  leds_brightness(0);

  while (true) {
    uint64_t t_ns = get_time_nsec() - epoch;
    float t_s = t_ns * 1e-9;

    uint8_t *buffer = buffers[count % NUM_BUFFERS];

    if (int(t_s / progCycleTime) != int(prev_t_s / progCycleTime)) {
      prog = randomProg(prog);
      prog->init(pixInfo, t_ns);
      max_brightness = prog->get_max_brightness();
    }

    float progTime = fmod(t_s, progCycleTime);
    uint8_t ramp_brightness = curr_brightness;
    if (progTime < trans_time) {
      float progRamp =
          (std::clamp(progTime, dead_time, trans_time) - dead_time) / ramp_time;
      ramp_brightness = uint8_t(progRamp * max_brightness);
    } else if (progTime > (progCycleTime - 1.0f)) {
      float progRamp =
          (std::clamp(progCycleTime - progTime, dead_time, trans_time) -
           dead_time) /
          ramp_time;
      ramp_brightness = uint8_t(progRamp * max_brightness);
    } else {
      ramp_brightness = max_brightness;
    }

    if (ramp_brightness != curr_brightness) {
      leds_brightness(ramp_brightness);
      curr_brightness = ramp_brightness;
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

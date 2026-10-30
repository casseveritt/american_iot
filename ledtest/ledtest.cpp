// Basic test patterns for 8 or 16 SMI-driven LED strips.
//
// The strip count comes from the smi_leds driver build (LED_NCHANS), so the
// same source produces ledtest8 and ledtest16.

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <vector>

#include "smi_leds.h"

namespace {

volatile sig_atomic_t quit = 0;

void on_signal(int) { quit = 1; }

struct Rgb {
  uint8_t r, g, b;
};

Rgb hsv(float h, float s, float v) {
  h = fmodf(h, 1.0f);
  if (h < 0.0f) h += 1.0f;
  int i = int(h * 6.0f);
  float f = h * 6.0f - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  float r, g, b;
  switch (i % 6) {
    case 0: r = v, g = t, b = p; break;
    case 1: r = q, g = v, b = p; break;
    case 2: r = p, g = v, b = t; break;
    case 3: r = p, g = q, b = v; break;
    case 4: r = t, g = p, b = v; break;
    default: r = v, g = p, b = q; break;
  }
  return {uint8_t(r * 255), uint8_t(g * 255), uint8_t(b * 255)};
}

double now_s() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

class Frame {
 public:
  Frame(int strips, int leds)
      : strips_(strips), leds_(leds), data_(strips * leds * 3) {}

  int strips() const { return strips_; }
  int leds() const { return leds_; }

  void clear() { memset(data_.data(), 0, data_.size()); }

  void set(int strip, int led, Rgb c) {
    uint8_t *p = &data_[(strip * leds_ + led) * 3];
    p[0] = c.r;
    p[1] = c.g;
    p[2] = c.b;
  }

  void fill_strip(int strip, Rgb c) {
    for (int led = 0; led < leds_; led++) set(strip, led, c);
  }

  void fill(Rgb c) {
    for (int s = 0; s < strips_; s++) fill_strip(s, c);
  }

  void show() {
    leds_set(data_.data());
    leds_send();
  }

 private:
  int strips_;
  int leds_;
  std::vector<uint8_t> data_;
};

// Each pattern renders one frame for time t (seconds since pattern start).
typedef void (*Pattern)(Frame &f, double t);

// Whole display cycles through the hue wheel, each strip slightly offset.
void hue_cycle(Frame &f, double t) {
  for (int s = 0; s < f.strips(); s++) {
    f.fill_strip(s, hsv(t * 0.2f + float(s) / f.strips() * 0.25f, 1, 1));
  }
}

// Rainbow running along every strip.
void hue_rainbow(Frame &f, double t) {
  for (int s = 0; s < f.strips(); s++) {
    for (int led = 0; led < f.leds(); led++) {
      f.set(s, led, hsv(float(led) / 60.0f - t * 0.5f, 1, 1));
    }
  }
}

// Each strip a solid, distinct hue so the lines can be identified.
void strip_id(Frame &f, double t) {
  for (int s = 0; s < f.strips(); s++) {
    f.fill_strip(s, hsv(float(s) / f.strips(), 1, 1));
  }
}

// One strip on at a time, stepping through strips 0..N-1.
void strip_sequence(Frame &f, double t) {
  int on = int(t * 2.0) % f.strips();
  f.clear();
  f.fill_strip(on, hsv(float(on) / f.strips(), 1, 1));
}

// Strips turn on one by one, then all turn off, and repeat.
void strip_stack(Frame &f, double t) {
  int step = int(t * 3.0) % (f.strips() + 2);
  f.clear();
  for (int s = 0; s < f.strips() && s < step; s++) {
    f.fill_strip(s, hsv(float(s) / f.strips(), 1, 1));
  }
}

// A short lit segment chasing down every strip.
void pixel_chase(Frame &f, double t) {
  constexpr int kLen = 5;
  int head = int(t * 60.0) % f.leds();
  f.clear();
  for (int s = 0; s < f.strips(); s++) {
    Rgb c = hsv(float(s) / f.strips(), 1, 1);
    for (int i = 0; i < kLen; i++) {
      f.set(s, (head - i + f.leds()) % f.leds(), c);
    }
  }
}

// Every LED white on, then off, once per second.
void blink(Frame &f, double t) {
  f.fill(fmod(t, 1.0) < 0.5 ? Rgb{255, 255, 255} : Rgb{0, 0, 0});
}

// Solid red, green, blue, white, two seconds each.
void rgbw(Frame &f, double t) {
  static const Rgb kColors[] = {
      {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};
  f.fill(kColors[int(t / 2.0) % 4]);
}

struct NamedPattern {
  const char *name;
  Pattern fn;
  const char *desc;
};

const NamedPattern kPatterns[] = {
    {"hue", hue_cycle, "all strips cycle through hues"},
    {"rainbow", hue_rainbow, "rainbow running along each strip"},
    {"strip_id", strip_id, "each strip a distinct solid hue"},
    {"sequence", strip_sequence, "one strip on at a time, in order"},
    {"stack", strip_stack, "strips turn on one by one, then all off"},
    {"chase", pixel_chase, "lit segment chasing down each strip"},
    {"blink", blink, "all white on/off"},
    {"rgbw", rgbw, "solid red, green, blue, white"},
};
constexpr int kNumPatterns = sizeof(kPatterns) / sizeof(kPatterns[0]);

void usage(const char *prog) {
  printf(
      "usage: sudo %s [options]\n"
      "  -n, --leds N        LEDs per strip (default 320)\n"
      "  -b, --brightness B  brightness 0-100 (default 25)\n"
      "  -p, --pattern NAME  run only this pattern (default: cycle all)\n"
      "  -c, --cycle SECS    seconds per pattern when cycling (default 10)\n"
      "  -l, --list          list patterns\n"
      "  -h, --help\n",
      prog);
}

}  // namespace

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IOLBF, 0);

  int leds = 320;
  int brightness = 25;
  int pattern = -1;
  double cycle = 10.0;

  static struct option long_options[] = {
      {"leds", required_argument, 0, 'n'},
      {"brightness", required_argument, 0, 'b'},
      {"pattern", required_argument, 0, 'p'},
      {"cycle", required_argument, 0, 'c'},
      {"list", no_argument, 0, 'l'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0}};

  int opt;
  while ((opt = getopt_long(argc, argv, "n:b:p:c:lh", long_options, 0)) !=
         -1) {
    switch (opt) {
      case 'n':
        leds = atoi(optarg);
        break;
      case 'b':
        brightness = atoi(optarg);
        break;
      case 'c':
        cycle = atof(optarg);
        break;
      case 'p':
        for (int i = 0; i < kNumPatterns; i++) {
          if (strcmp(optarg, kPatterns[i].name) == 0) pattern = i;
        }
        if (pattern < 0) {
          fprintf(stderr, "unknown pattern '%s' (try --list)\n", optarg);
          return 1;
        }
        break;
      case 'l':
        for (const auto &p : kPatterns) printf("  %-10s %s\n", p.name, p.desc);
        return 0;
      case 'h':
        usage(argv[0]);
        return 0;
      default:
        usage(argv[0]);
        return 1;
    }
  }

  if (leds < 1 || leds > 450) {
    fprintf(stderr, "LEDs per strip must be 1-450\n");
    return 1;
  }
  if (brightness < 0 || brightness > 100) {
    fprintf(stderr, "brightness must be 0-100\n");
    return 1;
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  if (!leds_init(leds, brightness)) {
    fprintf(stderr, "leds_init failed (are you root?)\n");
    return 1;
  }
  Frame frame(leds_num_strips(), leds);
  printf("%d strips x %d LEDs, brightness %d\n", frame.strips(), leds,
         brightness);

  int current = pattern < 0 ? 0 : pattern;
  printf("pattern: %s\n", kPatterns[current].name);
  double start = now_s();
  while (!quit) {
    double t = now_s() - start;
    if (pattern < 0 && t >= cycle) {
      current = (current + 1) % kNumPatterns;
      printf("pattern: %s\n", kPatterns[current].name);
      start += cycle;
      t -= cycle;
    }
    kPatterns[current].fn(frame, t);
    frame.show();
    usleep(10000);
  }

  frame.clear();
  frame.show();
  printf("\ncleared, exiting\n");
  return 0;
}

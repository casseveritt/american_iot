// Renders the self-identifying LED code sequence described in ../PLAN.md.
//
// Every LED gets a seeded random code word. The whole display repeatedly
// plays one cycle on a fixed symbol clock:
//
//   preamble (W O W O W O W O) | marker (W W O) | payload | check | gap (O O)
//
// The preamble, marker and gap are identical on every LED, so a decoder can
// find cycle boundaries from overall brightness. Payload symbols are
// per-LED. In mode A a symbol is a 3-bit R/G/B on/off combination; in mode B
// it is one of red, green, blue, white.
//
// The strip count comes from the smi_leds driver build (LED_NCHANS), so the
// same source produces idseq8 and idseq16. `make host` builds a hardware-free
// idseq_host that only supports --print and --dump.

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <random>
#include <vector>

#ifdef LEDMAP_NO_HW
bool leds_init(int, int) { return false; }
int leds_num_strips() { return 0; }
void leds_set(const uint8_t *) {}
void leds_send() {}
#else
#include "smi_leds.h"
#endif

namespace {

volatile sig_atomic_t quit = 0;

void on_signal(int) { quit = 1; }

struct Rgb {
  uint8_t r, g, b;
};

double now_s() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Symbol alphabets. Index is the symbol value stored in the code word.
const Rgb kModeA[8] = {{0, 0, 0},     {255, 0, 0},   {0, 255, 0},
                       {255, 255, 0}, {0, 0, 255},   {255, 0, 255},
                       {0, 255, 255}, {255, 255, 255}};
const Rgb kModeB[4] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};

const Rgb kWhite = {255, 255, 255};
const Rgb kOff = {0, 0, 0};

struct Config {
  int strips = 0;     // active strips (0 = all the driver has)
  int leds = 300;     // LEDs per strip
  int brightness = 25;
  char mode = 'A';
  int payload = 0;    // payload symbols (0 = default for mode)
  int check = 2;      // check symbols
  int min_dist = 4;   // minimum Hamming distance between payloads
  unsigned seed = 1;
  double symbol_s = 0.100;  // nominal, before --slow
  double fps = 60.0;        // output rate; symbols are a whole number of frames
  double slow = 1.0;        // symbol time multiplier
  double scale = 1.0;       // color value multiplier
  int sym_frames = 6;       // derived: frames per symbol
};

Rgb scaled(Rgb c, double s) {
  return {uint8_t(c.r * s + 0.5), uint8_t(c.g * s + 0.5),
          uint8_t(c.b * s + 0.5)};
}

int alphabet_size(char mode) { return mode == 'A' ? 8 : 4; }

Rgb symbol_color(char mode, int sym) {
  return mode == 'A' ? kModeA[sym] : kModeB[sym];
}

int hamming(const uint8_t *a, const uint8_t *b, int n) {
  int d = 0;
  for (int i = 0; i < n; i++) d += a[i] != b[i];
  return d;
}

// Random code words, rejecting any that is closer than min_dist to one
// already accepted. Returns n * len symbols, or empty if it gives up.
std::vector<uint8_t> make_codes(int n, int len, int alphabet, int min_dist,
                                unsigned seed) {
  std::mt19937_64 rng(seed);
  std::vector<uint8_t> codes;
  codes.reserve(size_t(n) * len);
  std::vector<uint8_t> cand(len);
  long tries = 0;
  const long max_tries = 5000L * n + 100000;
  for (int i = 0; i < n;) {
    if (++tries > max_tries) return {};
    bool lit = false;
    for (int k = 0; k < len; k++) {
      cand[k] = rng() % alphabet;
      lit |= cand[k] != 0;
    }
    if (!lit) continue;  // an all-off code could never be seen
    bool ok = true;
    for (int j = 0; j < i && ok; j++) {
      ok = hamming(cand.data(), &codes[size_t(j) * len], len) >= min_dist;
    }
    if (!ok) continue;
    codes.insert(codes.end(), cand.begin(), cand.end());
    i++;
  }
  return codes;
}

// Check symbol k is the sum of payload symbols k, k+C, k+2C, ... mod alphabet.
void append_checks(std::vector<uint8_t> &word, int payload, int check,
                   int alphabet) {
  for (int k = 0; k < check; k++) {
    int sum = 0;
    for (int i = k; i < payload; i += check) sum += word[i];
    word.push_back(sum % alphabet);
  }
}

// One transmission cycle for every LED, as colors per slot.
struct Cycle {
  int slots = 0;
  int n = 0;
  std::vector<Rgb> color;  // [slot * n + id]
  Rgb &at(int slot, int id) { return color[size_t(slot) * n + id]; }
};

Cycle build_cycle(const Config &c, int n, const std::vector<uint8_t> &codes) {
  const int alphabet = alphabet_size(c.mode);
  static const Rgb kCommon[] = {
      kWhite, kOff, kWhite, kOff, kWhite, kOff, kWhite, kOff,  // preamble
      kWhite, kWhite, kOff,                                    // marker
  };
  constexpr int kCommonSlots = sizeof(kCommon) / sizeof(kCommon[0]);
  constexpr int kGapSlots = 2;

  Cycle cy;
  cy.n = n;
  cy.slots = kCommonSlots + c.payload + c.check + kGapSlots;
  cy.color.assign(size_t(cy.slots) * n, kOff);
  for (int id = 0; id < n; id++) {
    std::vector<uint8_t> word(&codes[size_t(id) * c.payload],
                              &codes[size_t(id) * c.payload] + c.payload);
    append_checks(word, c.payload, c.check, alphabet);
    for (int s = 0; s < kCommonSlots; s++) cy.at(s, id) = kCommon[s];
    for (size_t k = 0; k < word.size(); k++) {
      cy.at(kCommonSlots + int(k), id) = symbol_color(c.mode, word[k]);
    }
  }
  return cy;
}

const char *kSymbolChars = "KRGYBMCW";  // mode A names, indexed by symbol

char slot_char(const Config &c, Rgb col) {
  if (c.mode == 'A') {
    for (int i = 0; i < 8; i++) {
      if (!memcmp(&col, &kModeA[i], 3)) return kSymbolChars[i];
    }
  } else {
    if (col.r && col.g && col.b) return 'W';
    if (col.r) return 'R';
    if (col.g) return 'G';
    if (col.b) return 'B';
    return 'K';
  }
  return '?';
}

void print_cycle(const Config &c, const Cycle &cy, int strips, int show) {
  printf("cycle: %d slots x %.0f ms = %.2f s\n", cy.slots, c.symbol_s * 1e3,
         cy.slots * c.symbol_s);
  printf("       preamble 8 | marker 3 | payload %d | check %d | gap 2\n",
         c.payload, c.check);
  for (int id = 0; id < cy.n && id < show; id++) {
    printf("  id %5d (strip %2d led %3d): ", id, id / c.leds, id % c.leds);
    for (int s = 0; s < cy.slots; s++) {
      if (s == 8 || s == 11 || s == 11 + c.payload ||
          s == 11 + c.payload + c.check) {
        putchar(' ');
      }
      putchar(slot_char(c, cy.color[size_t(s) * cy.n + id]));
    }
    putchar('\n');
  }
  (void)strips;
}

bool dump_codes(const char *path, const Config &c, const Cycle &cy) {
  FILE *f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "# mode=%c seed=%u payload=%d check=%d min_dist=%d symbol_ms=%.0f"
             " slots=%d\n",
          c.mode, c.seed, c.payload, c.check, c.min_dist, c.symbol_s * 1e3,
          cy.slots);
  fprintf(f, "id,strip,led,symbols\n");
  for (int id = 0; id < cy.n; id++) {
    fprintf(f, "%d,%d,%d,", id, id / c.leds, id % c.leds);
    for (int s = 11; s < 11 + c.payload + c.check; s++) {
      fputc(slot_char(c, cy.color[size_t(s) * cy.n + id]), f);
    }
    fputc('\n', f);
  }
  fclose(f);
  return true;
}

// Writes one slot's colors into the driver's strip-major buffer.
// LEDs with enabled[id] == 0 stay dark (an empty `enabled` means all).
void render_slot(Cycle &cy, int slot, int active_strips, int driver_strips,
                 int leds, double scale, const std::vector<char> &enabled,
                 std::vector<uint8_t> &buf) {
  std::fill(buf.begin(), buf.end(), 0);
  for (int s = 0; s < active_strips && s < driver_strips; s++) {
    for (int led = 0; led < leds; led++) {
      if (!enabled.empty() && !enabled[size_t(s) * leds + led]) continue;
      Rgb col = scaled(cy.at(slot, s * leds + led), scale);
      uint8_t *p = &buf[(size_t(s) * leds + led) * 3];
      p[0] = col.r;
      p[1] = col.g;
      p[2] = col.b;
    }
  }
}

// One LED per line: "strip,led" or a global id (strip * leds + led); # comments.
bool read_only_list(const char *path, int strips, int leds,
                    std::vector<char> &enabled, int &count) {
  FILE *f = fopen(path, "r");
  if (!f) return false;
  enabled.assign(size_t(strips) * leds, 0);
  count = 0;
  char line[256];
  while (fgets(line, sizeof line, f)) {
    int a, b;
    long id;
    if (line[0] == '#' || line[0] == '\n') continue;
    if (sscanf(line, "%d,%d", &a, &b) == 2) {
      id = long(a) * leds + b;
      if (a < 0 || a >= strips || b < 0 || b >= leds) id = -1;
    } else if (sscanf(line, "%ld", &id) != 1) {
      continue;
    }
    if (id < 0 || id >= long(enabled.size())) {
      fprintf(stderr, "%s: ignoring out-of-range entry: %s", path, line);
      continue;
    }
    if (!enabled[id]) count++;
    enabled[id] = 1;
  }
  fclose(f);
  return true;
}

void show(std::vector<uint8_t> &buf) {
  leds_set(buf.data());
  leds_send();
}

// Camera setup aid: every LED in each symbol color in turn.
void test_colors(const Config &c, int driver_strips, double secs_each) {
  std::vector<uint8_t> buf(size_t(driver_strips) * c.leds * 3);
  const int alphabet = alphabet_size(c.mode);
  for (int sym = 0; sym < alphabet && !quit; sym++) {
    Rgb col = symbol_color(c.mode, sym);
    printf("color %c (%d,%d,%d)\n", slot_char(c, col), col.r, col.g, col.b);
    col = scaled(col, c.scale);
    for (size_t i = 0; i < buf.size(); i += 3) {
      buf[i] = col.r;
      buf[i + 1] = col.g;
      buf[i + 2] = col.b;
    }
    double end = now_s() + secs_each;
    while (!quit && now_s() < end) {
      show(buf);
      usleep(10000);
    }
  }
}

void usage(const char *prog) {
  printf(
      "usage: sudo %s [options]\n"
      "  -s, --strips N      active strips (default: all the driver has)\n"
      "  -n, --leds N        LEDs per strip (default 300)\n"
      "  -b, --brightness B  brightness 0-100 (default 25)\n"
      "  -m, --mode A|B      A: 8 colors/3 bits, B: 4 colors/2 bits "
      "(default A)\n"
      "  -p, --payload N     payload symbols (default 10 in A, 16 in B)\n"
      "  -k, --check N       check symbols (default 2)\n"
      "  -d, --min-dist N    min Hamming distance between codes (default 4)\n"
      "  -S, --seed N        code seed (default 1)\n"
      "  -t, --symbol-ms MS  symbol duration (default 100)\n"
      "  -f, --fps HZ        output update rate (default 60); symbols last a\n"
      "                      whole number of frames\n"
      "  -x, --slow F        multiply symbol time by F, e.g. 10 (default 1)\n"
      "  -c, --scale F       multiply color values by F, 0-1 (default 1)\n"
      "  -T, --test-colors   show each symbol color solid for 3 s, then exit\n"
      "  -P, --print [N]     print the cycle for the first N LEDs (default "
      "8) and exit\n"
      "  -D, --dump FILE     write the code table to FILE\n"
      "  -o, --only FILE     light only the LEDs listed in FILE (lines of\n"
      "                      \"strip,led\" or a global id); the others stay dark.\n"
      "                      Ids and colors are the same as without it.\n"
      "  -h, --help\n",
      prog);
}

}  // namespace

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IOLBF, 0);

  Config c;
  bool do_test_colors = false;
  int print_n = 0;
  const char *dump_path = nullptr;
  const char *only_path = nullptr;

  static struct option long_options[] = {
      {"strips", required_argument, 0, 's'},
      {"leds", required_argument, 0, 'n'},
      {"brightness", required_argument, 0, 'b'},
      {"mode", required_argument, 0, 'm'},
      {"payload", required_argument, 0, 'p'},
      {"check", required_argument, 0, 'k'},
      {"min-dist", required_argument, 0, 'd'},
      {"seed", required_argument, 0, 'S'},
      {"symbol-ms", required_argument, 0, 't'},
      {"fps", required_argument, 0, 'f'},
      {"slow", required_argument, 0, 'x'},
      {"scale", required_argument, 0, 'c'},
      {"test-colors", no_argument, 0, 'T'},
      {"print", optional_argument, 0, 'P'},
      {"dump", required_argument, 0, 'D'},
      {"only", required_argument, 0, 'o'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0}};

  int opt;
  while ((opt = getopt_long(argc, argv, "s:n:b:m:p:k:d:S:t:f:x:c:TP::D:o:h",
                            long_options, 0)) != -1) {
    switch (opt) {
      case 's': c.strips = atoi(optarg); break;
      case 'n': c.leds = atoi(optarg); break;
      case 'b': c.brightness = atoi(optarg); break;
      case 'm': c.mode = optarg[0] & ~0x20; break;
      case 'p': c.payload = atoi(optarg); break;
      case 'k': c.check = atoi(optarg); break;
      case 'd': c.min_dist = atoi(optarg); break;
      case 'S': c.seed = atoi(optarg); break;
      case 't': c.symbol_s = atof(optarg) / 1e3; break;
      case 'f': c.fps = atof(optarg); break;
      case 'x': c.slow = atof(optarg); break;
      case 'c': c.scale = atof(optarg); break;
      case 'T': do_test_colors = true; break;
      case 'P': print_n = optarg ? atoi(optarg) : 8; break;
      case 'D': dump_path = optarg; break;
      case 'o': only_path = optarg; break;
      case 'h': usage(argv[0]); return 0;
      default: usage(argv[0]); return 1;
    }
  }

  if (c.mode != 'A' && c.mode != 'B') {
    fprintf(stderr, "mode must be A or B\n");
    return 1;
  }
  if (c.payload == 0) c.payload = c.mode == 'A' ? 10 : 16;
  if (c.leds < 1 || c.leds > 450 || c.brightness < 0 || c.brightness > 100 ||
      c.check < 1 || c.payload < c.check || c.symbol_s <= 0 || c.fps <= 0 ||
      c.slow <= 0 || c.scale < 0 || c.scale > 1) {
    fprintf(stderr, "bad option value (see --help)\n");
    return 1;
  }
  // Symbols last a whole number of output frames; use the effective time from
  // here on so --print, --dump and the player agree.
  c.sym_frames = std::max(1, int(c.symbol_s * c.slow * c.fps + 0.5));
  c.symbol_s = c.sym_frames / c.fps;

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  const bool want_hw = !print_n;
  int driver_strips = 0;
  if (want_hw || do_test_colors) {
    if (!leds_init(c.leds, c.brightness)) {
      fprintf(stderr, "leds_init failed (are you root? hardware build?)\n");
      return 1;
    }
    driver_strips = leds_num_strips();
  }
  int strips = c.strips ? c.strips : driver_strips;
  if (!strips) strips = 1;  // --print/--dump without hardware
  if (driver_strips && strips > driver_strips) {
    fprintf(stderr, "driver has only %d strips\n", driver_strips);
    return 1;
  }
  const int n = strips * c.leds;

  if (do_test_colors) {
    test_colors(c, driver_strips, 3.0);
    std::vector<uint8_t> off(size_t(driver_strips) * c.leds * 3, 0);
    show(off);
    return 0;
  }

  std::vector<uint8_t> codes = make_codes(
      n, c.payload, alphabet_size(c.mode), c.min_dist, c.seed);
  if (codes.empty()) {
    fprintf(stderr,
            "could not find %d codes with min distance %d at length %d; "
            "lower --min-dist or raise --payload\n",
            n, c.min_dist, c.payload);
    return 1;
  }
  Cycle cy = build_cycle(c, n, codes);
  printf("%d strips x %d LEDs = %d ids, mode %c, seed %u\n", strips, c.leds,
         n, c.mode, c.seed);
  print_cycle(c, cy, strips, print_n ? print_n : 4);
  if (dump_path) {
    if (!dump_codes(dump_path, c, cy)) {
      perror(dump_path);
      return 1;
    }
    printf("wrote %s\n", dump_path);
  }
  std::vector<char> enabled;
  if (only_path) {
    int count = 0;
    if (!read_only_list(only_path, strips, c.leds, enabled, count)) {
      perror(only_path);
      return 1;
    }
    printf("only lighting %d of %d LEDs from %s\n", count, n, only_path);
  }
  if (!want_hw) return 0;

  std::vector<uint8_t> buf(size_t(driver_strips) * c.leds * 3);
  printf("%.0f Hz, %d frames/symbol (%.0f ms), scale %.2f\n", c.fps,
         c.sym_frames, c.symbol_s * 1e3, c.scale);
  std::vector<uint8_t> sent;  // what the strips currently hold
  long last_frame = -1;
  long last_slot = -1;
  double start = now_s();
  while (!quit) {
    long frame = long((now_s() - start) * c.fps);
    if (frame != last_frame) {
      long slot_abs = frame / c.sym_frames;
      if (slot_abs != last_slot) {
        int slot = slot_abs % cy.slots;
        if (slot == 0) printf("cycle %ld\n", slot_abs / cy.slots);
        render_slot(cy, slot, strips, driver_strips, c.leds, c.scale, enabled,
                    buf);
        last_slot = slot_abs;
        // WS2812s hold their value, so only send when something changed.
        if (buf != sent) {
          show(buf);
          sent = buf;
        }
      }
      last_frame = frame;
    }
    usleep(1000);
  }

  std::fill(buf.begin(), buf.end(), 0);
  show(buf);
  printf("\ncleared, exiting\n");
  return 0;
}

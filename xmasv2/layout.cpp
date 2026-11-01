#include "layout.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

std::vector<PixInfo> load_layout(const char *path) {
  std::ifstream f(path);
  if (!f) {
    fprintf(stderr, "layout: cannot open %s\n", path);
    return {};
  }
  std::vector<PixInfo> pi;
  std::set<uint32_t> seen;
  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    line_no++;
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> col;
    std::stringstream ss(line);
    for (std::string c; std::getline(ss, c, ',');) col.push_back(c);
    if (col.size() < 5) {
      fprintf(stderr, "layout: %s:%d: expected strip,led,x,y,z\n", path,
              line_no);
      return {};
    }
    char *end;
    long strip = strtol(col[0].c_str(), &end, 10);
    if (end == col[0].c_str()) continue;  // header line
    long led = strtol(col[1].c_str(), nullptr, 10);
    if (strip < 0 || strip >= STRIPS || led < 0 || led >= PIXELS_PER_STRIP) {
      fprintf(stderr, "layout: %s:%d: strip %ld led %ld out of range\n", path,
              line_no, strip, led);
      return {};
    }
    PixInfo pix;
    pix.index = strip * PIXELS_PER_STRIP + led;
    pix.position = r3::Vec3f(strtof(col[2].c_str(), nullptr),
                             strtof(col[3].c_str(), nullptr),
                             strtof(col[4].c_str(), nullptr));
    pix.outside = true;
    if (!seen.insert(pix.index).second) {
      fprintf(stderr, "layout: %s:%d: strip %ld led %ld listed twice\n", path,
              line_no, strip, led);
      return {};
    }
    pi.push_back(pix);
  }
  if (pi.empty()) fprintf(stderr, "layout: %s has no pixels\n", path);
  return pi;
}

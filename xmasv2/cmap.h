#pragma once

#include <algorithm>
#include <vector>

#include "color.h"

struct ColorMap {
  ColorMap() {}

  void clear() { cm.clear(); }

  void addColor(const Color &c) { cm.push_back(c); }

  Color lookupClamped(float t) const {
    const float ts = std::clamp(t * cm.size(), 0.5f, cm.size() - 0.5f) - 0.5f;
    const int ti = std::clamp(int(ts), 0, int(cm.size()) - 1);
    const float tfr = ts - ti;
    return cm[ti].lerp(cm[std::min(int(cm.size()) - 1, ti + 1)], tfr);
  }

  r3::Vec3f lookupNearest(float t) const {
    const float ts = t * cm.size();
    const int ti = std::clamp(int(ts + 0.5f), 0, int(cm.size() - 1));
    return cm[ti];
  }

  r3::Vec3f lookupWrapped(float t) const {
    t = t - floor(t);
    const float ts = t * cm.size() + cm.size() - 0.5f;
    const int ti0 = int(ts) % cm.size();
    const int ti1 = (ti0 + 1) % cm.size();
    const float tfr = ts - floor(ts);
    return cm[ti0].lerp(cm[ti1], tfr);
  }

  std::vector<Color> cm;
};
extern ColorMap blueBlack;
extern ColorMap halloween;
extern ColorMap rgbcmy;
extern ColorMap maroonWhite;
extern ColorMap sparkle[2];
extern ColorMap cm[2];
extern ColorMap rgbish;

void init_cmaps();
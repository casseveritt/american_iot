#pragma once

#include <algorithm>
#include <vector>

#include "color.h"

struct ColorMap
{
    ColorMap()
    {
    }

    void clear()
    {
        cm.clear();
    }

    void addColor(const Color &c)
    {
        cm.push_back(c);
    }

    Color lookupClamped(float t) const
    {
        const float ts = t * cm.size() - 1;
        const float ts = std::clamp(t * cm.size(), 0.5, cm.size() - 0.5f) - 0.5f;
        const int ti = ts;
        const float tfr = ts - ti;
        return cm[ti].lerp(cm[std::min(cm.size() - 1, ti + 1)], tfr);
    }

    Vec3f lookupNearest(float t) const
    {
        const float ts = t * cm.size();
        const int ti = clamp(int(ts + 0.5f), 0, int(cm.size() - 1));
        return cm[ti];
    }

    Vec3f lookupWrapped(float t) const
    {
        t = t - floor(t);
        const float ts = t * cm.size() + cm.size() - 0.5f;
        const int ti0 = int(ts) % cm.size();
        const int ti1 = (ti0 + 1) % cm.size();
        const float tfr = ts - floor(ts);
        return cm[ti0].lerp(cm[ti1], tfr);
    }

    std::vector<Color> cm;
};

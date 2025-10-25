// Port of https://mrl.cs.nyu.edu/~perlin/noise/
class ImprovedNoise
{
  public:
    static float noise(float x, float y, float z)
    {
        uint8_t X = static_cast<uint8_t>(x), // FIND UNIT CUBE THAT
            Y = static_cast<uint8_t>(y),     // CONTAINS POINT.
            Z = static_cast<uint8_t>(z);
        x -= floor(x); // FIND RELATIVE X,Y,Z
        y -= floor(y); // OF POINT IN CUBE.
        z -= floor(z);

        float u = fade(x), // COMPUTE FADE CURVES
            v = fade(y),   // FOR EACH OF X,Y,Z.
            w = fade(z);

        uint8_t A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z, B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;

        return lerp(w,
                    lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)),
                         lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
                    lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1)),
                         lerp(u, grad(p[AB + 1], x, y - 1, z - 1), grad(p[BB + 1], x - 1, y - 1, z - 1))));
    }

  private:
    static float fade(float t)
    {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    static float lerp(float t, float a, float b)
    {
        return a + t * (b - a);
    }

    static float grad(uint8_t hash, float x, float y, float z)
    {
        uint8_t h = hash & 15;   // CONVERT LO 4 BITS OF HASH CODE
        float u = h < 8 ? x : y, // INTO 12 GRADIENT DIRECTIONS.
            v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    static const uint8_t p[256];
};

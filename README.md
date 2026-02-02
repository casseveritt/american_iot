# american_iot
I'm an American, and I have recently started playing with iot devices in America.

## Collision Probability for 60-bit Random Numbers

For information about collision probabilities when using 60-bit random numbers (e.g., for device IDs or session tokens), see:

- **[Collision Probability Documentation](collision_probability.md)** - Detailed explanation and tables
- **[Collision Calculator Script](collision_calc.py)** - Interactive calculator tool

### Quick Answer

For 60-bit random numbers:
- **1 million IDs**: ~0.000043% collision chance (essentially none)
- **100 million IDs**: ~0.43% collision chance (very low)
- **1 billion IDs**: ~35% collision chance (getting significant)

You need approximately **152 million IDs** to reach a **1% collision probability**.

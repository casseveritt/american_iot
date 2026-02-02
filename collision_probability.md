# Collision Probability for 60-bit Random Numbers

## Overview

This document explains the collision probability when using 60-bit random numbers, based on the birthday paradox problem.

## The Birthday Paradox Formula

The probability of at least one collision when generating `n` random IDs from a space of `N` possible values is approximately:

```
P(collision) ≈ 1 - e^(-n² / 2N)
```

For 60-bit random numbers:
- Total space: N = 2^60 = 1,152,921,504,606,846,976 (approximately 1.15 quintillion)

## Collision Probabilities

### Number of IDs Required for Different Collision Probabilities

| Collision Probability | Number of IDs Required | Approximate Value |
|----------------------|------------------------|-------------------|
| 0.1% (1 in 1,000) | ~50,659,773 | ~50.7 million |
| 1% (1 in 100) | ~161,135,149 | ~161 million |
| 10% (1 in 10) | ~516,397,778 | ~516 million |
| 50% (1 in 2) | ~1,280,000,000 | ~1.28 billion |

### Formula Derivation

For a desired collision probability `p`, the number of IDs `n` can be calculated as:

```
n ≈ √(2N × ln(1 / (1 - p)))
```

Where:
- `N = 2^60` for 60-bit random numbers
- `p` is the desired collision probability
- `ln` is the natural logarithm

## Practical Examples

### Example 1: IoT Device IDs
If you're generating 60-bit random IDs for IoT devices:
- With **1 million devices**, collision probability is approximately **0.00000043%** (extremely unlikely)
- With **100 million devices**, collision probability is approximately **0.43%** (still very low)
- With **1 billion devices**, collision probability is approximately **35%** (getting significant)

### Example 2: Session Identifiers
For web session IDs:
- Generating **1,000 sessions per second** continuously would take approximately **16 years** to reach a 1% collision probability
- Generating **10,000 sessions per second** would take approximately **6 months** to reach a 1% collision probability

## Recommendation

60-bit random numbers provide excellent collision resistance for most applications:
- ✅ **Suitable** for device IDs in systems with millions of devices
- ✅ **Suitable** for short-lived session tokens
- ⚠️ **Consider longer IDs** (128-bit) for:
  - Systems expecting billions of IDs
  - Long-lived globally unique identifiers
  - Cryptographic applications requiring higher entropy

## Mathematical Details

The exact formula for collision probability without approximation is:

```
P(collision) = 1 - (N! / ((N-n)! × N^n))
```

However, for large N and relatively small n (n << N), the approximation is:

```
P(collision) ≈ 1 - e^(-n(n-1) / 2N) ≈ 1 - e^(-n² / 2N)
```

This is known as the birthday paradox, as it's analogous to calculating the probability that two people in a room share the same birthday.

## Quick Reference

For 60-bit random numbers (N = 2^60):
- Square root of N: √N ≈ 1,073,741,824 (approximately 1.07 billion)
- 50% collision probability at: n ≈ 1.18 × √N ≈ 1.28 billion IDs
- 1% collision probability at: n ≈ 0.15 × √N ≈ 161 million IDs

## Calculation Script

To calculate collision probability for specific scenarios, use the following formula:

```python
import math

N = 2**60  # Total space for 60-bit numbers
n = 1000000  # Number of IDs to generate

# Approximate collision probability
p = 1 - math.exp(-n**2 / (2 * N))
print(f"Collision probability: {p * 100:.6f}%")
```

Or to find n for a desired probability p:

```python
import math

N = 2**60  # Total space for 60-bit numbers
p = 0.01  # Desired collision probability (1%)

# Number of IDs for desired collision probability
n = math.sqrt(2 * N * math.log(1 / (1 - p)))
print(f"Number of IDs for {p*100}% collision: {n:,.0f}")
```

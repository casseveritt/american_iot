#!/usr/bin/env python3
"""
Unit tests for collision_calc.py
"""

import math
import unittest
import sys
import os

# Add parent directory to path to import collision_calc
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import collision_calc


class TestCollisionCalculator(unittest.TestCase):
    """Test cases for collision probability calculations."""
    
    def test_collision_probability_zero_ids(self):
        """Test that 0 IDs gives 0% collision probability."""
        p = collision_calc.collision_probability(0)
        self.assertAlmostEqual(p, 0.0, places=10)
    
    def test_collision_probability_one_id(self):
        """Test that 1 ID gives 0% collision probability."""
        p = collision_calc.collision_probability(1)
        self.assertAlmostEqual(p, 0.0, places=10)
    
    def test_collision_probability_small_numbers(self):
        """Test collision probability for small numbers."""
        # 1 million IDs should have very low collision probability
        p = collision_calc.collision_probability(1_000_000)
        self.assertLess(p, 0.0001)  # Less than 0.01%
        self.assertGreater(p, 0.0)   # Greater than 0
    
    def test_collision_probability_medium_numbers(self):
        """Test collision probability approaches expected values."""
        # 100 million IDs should have about 0.43% collision probability
        p = collision_calc.collision_probability(100_000_000)
        self.assertGreater(p, 0.004)  # Greater than 0.4%
        self.assertLess(p, 0.005)     # Less than 0.5%
    
    def test_collision_probability_large_numbers(self):
        """Test collision probability for large numbers."""
        # 1 billion IDs should have significant collision probability
        p = collision_calc.collision_probability(1_000_000_000)
        self.assertGreater(p, 0.3)  # Greater than 30%
        self.assertLess(p, 0.4)     # Less than 40%
    
    def test_collision_probability_different_bit_sizes(self):
        """Test collision probability with different bit sizes."""
        # 32-bit should have higher collision probability for same n
        p_32 = collision_calc.collision_probability(1_000_000, bits=32)
        p_60 = collision_calc.collision_probability(1_000_000, bits=60)
        p_128 = collision_calc.collision_probability(1_000_000, bits=128)
        
        # Smaller bit space should have higher collision probability
        self.assertGreater(p_32, p_60)
        self.assertGreater(p_60, p_128)
    
    def test_ids_for_probability_1_percent(self):
        """Test that IDs calculation for 1% probability is reasonable."""
        n = collision_calc.ids_for_probability(0.01)
        # Should be around 150 million for 60-bit
        self.assertGreater(n, 140_000_000)
        self.assertLess(n, 170_000_000)
    
    def test_ids_for_probability_50_percent(self):
        """Test that IDs calculation for 50% probability is reasonable."""
        n = collision_calc.ids_for_probability(0.5)
        # Should be around 1.18 * sqrt(2^60) ≈ 1.27 billion
        self.assertGreater(n, 1_200_000_000)
        self.assertLess(n, 1_300_000_000)
    
    def test_ids_for_probability_invalid_input(self):
        """Test that invalid probabilities raise ValueError."""
        with self.assertRaises(ValueError):
            collision_calc.ids_for_probability(0.0)
        
        with self.assertRaises(ValueError):
            collision_calc.ids_for_probability(1.0)
        
        with self.assertRaises(ValueError):
            collision_calc.ids_for_probability(-0.1)
        
        with self.assertRaises(ValueError):
            collision_calc.ids_for_probability(1.1)
    
    def test_roundtrip_consistency(self):
        """Test that probability -> IDs -> probability is consistent."""
        target_p = 0.05  # 5% collision probability
        
        # Calculate IDs needed for target probability
        n = collision_calc.ids_for_probability(target_p)
        
        # Calculate probability for those IDs
        actual_p = collision_calc.collision_probability(int(n))
        
        # Should be very close to target
        self.assertAlmostEqual(target_p, actual_p, delta=0.001)
    
    def test_birthday_paradox_formula(self):
        """Verify the birthday paradox formula is correctly implemented."""
        # For 365 days (birthday problem), 23 people gives ~50% collision
        bits_for_365 = math.log2(365)
        n = collision_calc.ids_for_probability(0.5, bits=bits_for_365)
        
        # Should be close to 23 (actual is ~22.49)
        self.assertGreater(n, 20)
        self.assertLess(n, 25)
    
    def test_format_number(self):
        """Test number formatting function."""
        self.assertEqual(collision_calc.format_number(500), "500")
        self.assertEqual(collision_calc.format_number(5000), "5.00 thousand")
        self.assertEqual(collision_calc.format_number(5_000_000), "5.00 million")
        self.assertEqual(collision_calc.format_number(5_000_000_000), "5.00 billion")
        self.assertEqual(collision_calc.format_number(5_000_000_000_000), "5.00 trillion")
    
    def test_specific_values_from_documentation(self):
        """Test specific values mentioned in the documentation."""
        # 1 million IDs: ~0.000043%
        p = collision_calc.collision_probability(1_000_000)
        self.assertAlmostEqual(p * 100, 0.000043, delta=0.000001)
        
        # 100 million IDs: ~0.43%
        p = collision_calc.collision_probability(100_000_000)
        self.assertAlmostEqual(p * 100, 0.43, delta=0.01)
        
        # 1 billion IDs: ~35%
        p = collision_calc.collision_probability(1_000_000_000)
        self.assertAlmostEqual(p * 100, 35, delta=1)


class TestMathematicalProperties(unittest.TestCase):
    """Test mathematical properties of the collision probability."""
    
    def test_monotonic_increase_with_n(self):
        """Test that probability increases as n increases."""
        p_prev = 0
        for n in [1000, 10000, 100000, 1000000, 10000000]:
            p = collision_calc.collision_probability(n)
            self.assertGreater(p, p_prev)
            p_prev = p
    
    def test_probability_bounds(self):
        """Test that probability is always between 0 and 1."""
        for n in [1, 100, 10000, 1000000, 100000000]:
            p = collision_calc.collision_probability(n)
            self.assertGreaterEqual(p, 0.0)
            self.assertLessEqual(p, 1.0)
    
    def test_approximation_accuracy_for_small_n(self):
        """Test that approximation is accurate when n << sqrt(N)."""
        N = 2 ** 60
        sqrt_N = math.sqrt(N)
        
        # Test for n = sqrt(N) / 100 (well within approximation range)
        n = int(sqrt_N / 100)
        p = collision_calc.collision_probability(n)
        
        # Should be very small
        self.assertLess(p, 0.01)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)

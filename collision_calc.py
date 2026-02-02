#!/usr/bin/env python3
"""
Collision Probability Calculator for 60-bit Random Numbers

This script calculates the collision probability for 60-bit random numbers
using the birthday paradox formula.
"""

import math
import sys


def collision_probability(n, bits=60):
    """
    Calculate the collision probability for n random numbers in a space of 2^bits.
    
    Args:
        n: Number of random IDs to generate
        bits: Number of bits in the random number (default: 60)
    
    Returns:
        Collision probability as a float between 0 and 1
    """
    N = 2 ** bits
    
    # Using the approximation: P(collision) ≈ 1 - e^(-n² / 2N)
    # This is accurate when n << N
    try:
        exponent = -(n * n) / (2 * N)
        p = 1 - math.exp(exponent)
        return p
    except OverflowError:
        # If n is very large, collision is essentially certain
        return 1.0


def ids_for_probability(p, bits=60):
    """
    Calculate the number of IDs needed to achieve a given collision probability.
    
    Args:
        p: Desired collision probability (0 to 1)
        bits: Number of bits in the random number (default: 60)
    
    Returns:
        Number of IDs needed
    """
    if p <= 0 or p >= 1:
        raise ValueError("Probability must be between 0 and 1 (exclusive)")
    
    N = 2 ** bits
    
    # Formula: n ≈ √(2N × ln(1 / (1 - p)))
    n = math.sqrt(2 * N * math.log(1 / (1 - p)))
    return n


def format_number(n):
    """Format large numbers with appropriate suffixes."""
    if n >= 1e12:
        return f"{n/1e12:.2f} trillion"
    elif n >= 1e9:
        return f"{n/1e9:.2f} billion"
    elif n >= 1e6:
        return f"{n/1e6:.2f} million"
    elif n >= 1e3:
        return f"{n/1e3:.2f} thousand"
    else:
        return f"{n:.0f}"


def print_table():
    """Print a table of collision probabilities for various numbers of IDs."""
    print("\n" + "="*70)
    print("Collision Probability for 60-bit Random Numbers")
    print("="*70)
    print()
    
    # Table 1: Probability for different numbers of IDs
    print("Number of IDs vs Collision Probability:")
    print("-" * 70)
    print(f"{'Number of IDs':<25} {'Collision Probability':<25}")
    print("-" * 70)
    
    test_values = [
        1_000_000,        # 1 million
        10_000_000,       # 10 million
        50_000_000,       # 50 million
        100_000_000,      # 100 million
        500_000_000,      # 500 million
        1_000_000_000,    # 1 billion
        1_500_000_000,    # 1.5 billion
    ]
    
    for n in test_values:
        p = collision_probability(n)
        print(f"{format_number(n):<25} {p*100:.6f}%")
    
    print()
    
    # Table 2: IDs needed for specific probabilities
    print("Collision Probability vs Number of IDs Required:")
    print("-" * 70)
    print(f"{'Collision Probability':<25} {'Number of IDs Required':<25}")
    print("-" * 70)
    
    probabilities = [0.001, 0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9]
    
    for p in probabilities:
        n = ids_for_probability(p)
        print(f"{p*100:.1f}% (1 in {1/p:.0f})".ljust(25) + f"{format_number(n)}")
    
    print("="*70)
    print()


def interactive_mode():
    """Run in interactive mode for custom calculations."""
    print("\nCollision Probability Calculator")
    print("Choose an option:")
    print("1. Calculate collision probability for a given number of IDs")
    print("2. Calculate number of IDs for a given collision probability")
    print("3. Exit")
    
    choice = input("\nEnter your choice (1-3): ").strip()
    
    if choice == "1":
        try:
            n = int(input("Enter the number of IDs: "))
            if n <= 0:
                print("Error: Number of IDs must be positive")
                return
            
            p = collision_probability(n)
            print(f"\nFor {format_number(n)} IDs:")
            print(f"  Collision probability: {p*100:.6f}%")
            if p < 0.01:
                print(f"  That's approximately 1 in {1/p:,.0f}")
        except ValueError:
            print("Error: Invalid input. Please enter a valid integer.")
    
    elif choice == "2":
        try:
            p = float(input("Enter the desired collision probability (e.g., 0.01 for 1%): "))
            if p <= 0 or p >= 1:
                print("Error: Probability must be between 0 and 1 (exclusive)")
                return
            
            n = ids_for_probability(p)
            print(f"\nFor a {p*100:.2f}% collision probability:")
            print(f"  You need approximately {format_number(n)} IDs")
            print(f"  (Exact: {n:,.0f})")
        except ValueError:
            print("Error: Invalid input. Please enter a valid number between 0 and 1.")
    
    elif choice == "3":
        print("Exiting...")
        return
    
    else:
        print("Invalid choice. Please enter 1, 2, or 3.")


def main():
    """Main entry point."""
    if len(sys.argv) > 1:
        # Command-line mode
        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
            print("Usage:")
            print("  collision_calc.py              # Show table and enter interactive mode")
            print("  collision_calc.py --table      # Show table only")
            print("  collision_calc.py --prob N     # Calculate probability for N IDs")
            print("  collision_calc.py --ids P      # Calculate IDs for probability P")
            print("  collision_calc.py --help       # Show this help")
            return
        
        elif sys.argv[1] == "--table":
            print_table()
            return
        
        elif sys.argv[1] == "--prob" and len(sys.argv) > 2:
            try:
                n = int(sys.argv[2])
                p = collision_probability(n)
                print(f"Collision probability for {format_number(n)} IDs: {p*100:.6f}%")
            except ValueError:
                print("Error: Invalid number of IDs")
            return
        
        elif sys.argv[1] == "--ids" and len(sys.argv) > 2:
            try:
                p = float(sys.argv[2])
                n = ids_for_probability(p)
                print(f"IDs needed for {p*100:.2f}% collision probability: {format_number(n)} ({n:,.0f})")
            except ValueError as e:
                print(f"Error: {e}")
            return
    
    # Default: show table and interactive mode
    print_table()
    
    while True:
        try:
            interactive_mode()
            again = input("\nCalculate again? (y/n): ").strip().lower()
            if again != 'y':
                break
        except (KeyboardInterrupt, EOFError):
            print("\n\nExiting...")
            break


if __name__ == "__main__":
    main()

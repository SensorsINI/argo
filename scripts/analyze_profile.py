#!/usr/bin/env python3
"""
Profile Analysis Helper for rudder_sail_radio.py

Analyzes CPU profiling data from cProfile to identify performance bottlenecks.

Usage:
    python3 scripts/analyze_profile.py <profile_file.prof> [--top N] [--sort SORT_TYPE]

Examples:
    # Show top 30 functions by cumulative time
    python3 scripts/analyze_profile.py rudder_sail_radio_profile_20250101_120000.prof

    # Show top 50 functions by total time
    python3 scripts/analyze_profile.py profile.prof --top 50 --sort tottime

    # Show top functions by number of calls
    python3 scripts/analyze_profile.py profile.prof --sort ncalls
"""

import pstats
import sys
import argparse

def analyze_profile(profile_file, top_n=30, sort_by='cumulative'):
    """Analyze a profile file and print statistics."""
    try:
        stats = pstats.Stats(profile_file)
        
        print(f"\n{'='*80}")
        print(f"CPU Profile Analysis: {profile_file}")
        print(f"{'='*80}\n")
        
        # Print summary
        stats.print_stats()
        
        print(f"\n{'='*80}")
        print(f"Top {top_n} functions sorted by {sort_by}:")
        print(f"{'='*80}\n")
        
        # Sort by requested metric
        if sort_by == 'cumulative':
            stats.sort_stats('cumulative')
        elif sort_by == 'tottime':
            stats.sort_stats('tottime')
        elif sort_by == 'ncalls':
            stats.sort_stats('ncalls')
        elif sort_by == 'percall':
            stats.sort_stats('tottime', 'cumulative')
        else:
            print(f"Unknown sort type: {sort_by}")
            print("Valid options: cumulative, tottime, ncalls, percall")
            return
        
        stats.print_stats(top_n)
        
        # Additional insights
        print(f"\n{'='*80}")
        print("Profile Analysis Tips:")
        print(f"{'='*80}")
        print("- Functions with high 'cumulative' time are the overall bottlenecks")
        print("- Functions with high 'tottime' are expensive per call")
        print("- Functions with high 'ncalls' are called frequently (may need optimization)")
        print("- Look for ROS2 internal functions that might be called too often")
        print("- Check for file I/O operations that could be throttled")
        print(f"{'='*80}\n")
        
    except FileNotFoundError:
        print(f"Error: Profile file '{profile_file}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error analyzing profile: {e}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description='Analyze CPU profiling data from rudder_sail_radio.py',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('profile_file', help='Path to .prof file')
    parser.add_argument('--top', type=int, default=30,
                       help='Number of top functions to show (default: 30)')
    parser.add_argument('--sort', choices=['cumulative', 'tottime', 'ncalls', 'percall'],
                       default='cumulative',
                       help='Sort by: cumulative (total time including subcalls), '
                            'tottime (time in function excluding subcalls), '
                            'ncalls (number of calls), percall (time per call)')
    
    args = parser.parse_args()
    analyze_profile(args.profile_file, args.top, args.sort)

if __name__ == '__main__':
    main()


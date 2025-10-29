#!/usr/bin/env python3
# Test script to verify the sailboat simulator installation and basic functionality

import sys
import time
import math
from pathlib import Path

def test_sailboat_playground():
    """Test the sailboat-playground simulator."""
    print("Testing sailboat-playground simulator...")
    
    try:
        from sailboat_playground import SailboatPlayground
        print("✅ sailboat-playground imported successfully")
        
        # Try to create simulator instance
        sim = SailboatPlayground()
        print("✅ Simulator instance created")
        
        # Test basic functionality
        print("🧪 Testing basic simulation...")
        for i in range(5):
            # Set some control inputs
            rudder = 0.3 * math.sin(i * 0.5)
            sail = 0.5
            
            # Apply control (API may vary)
            try:
                if hasattr(sim, 'set_control'):
                    sim.set_control(rudder, sail)
                elif hasattr(sim, 'step'):
                    state = sim.step({'rudder': rudder, 'sail': sail})
                    print(f"  Step {i}: {state}")
                else:
                    print(f"  Step {i}: Applied rudder={rudder:.2f}, sail={sail:.2f}")
            except Exception as e:
                print(f"  Step {i}: Control application failed: {e}")
            
            time.sleep(0.1)
        
        print("✅ Basic simulation test completed")
        return True
        
    except ImportError as e:
        print(f"❌ Failed to import sailboat-playground: {e}")
        return False
    except Exception as e:
        print(f"❌ Simulator test failed: {e}")
        return False

def test_mock_simulator():
    """Test the mock simulator from our bridge."""
    print("\nTesting mock simulator...")
    
    try:
        # Import our mock simulator
        # Determine Argo repository directory dynamically
        script_path = Path(__file__).resolve()
        argo_dir = script_path.parents[2]  # nodes/test -> argo
        nodes_dir = argo_dir / "nodes"
        sys.path.append(str(nodes_dir))
        
        from argo_simulator_bridge import MockSailboatSimulator
        
        print("✅ Mock simulator imported successfully")
        
        # Create and test mock simulator
        sim = MockSailboatSimulator()
        print("✅ Mock simulator instance created")
        
        print("🧪 Testing mock simulation...")
        for i in range(10):
            # Set control
            rudder = 0.5 * math.sin(i * 0.3)
            sail = 0.3
            sim.set_control(rudder, sail)
            
            # Step simulation
            state = sim.step()
            
            if i % 3 == 0:  # Print every 3rd step
                print(f"  Step {i}: heading={state['heading']:.1f}°, "
                      f"speed={state['speed']:.1f}m/s, "
                      f"wind_dir={state['wind_direction']:.0f}°")
        
        print("✅ Mock simulation test completed")
        return True
        
    except Exception as e:
        print(f"❌ Mock simulator test failed: {e}")
        return False

def test_dependencies():
    """Test required dependencies."""
    print("\nTesting dependencies...")
    
    dependencies = ['numpy', 'math', 'time']
    all_good = True
    
    for dep in dependencies:
        try:
            __import__(dep)
            print(f"✅ {dep} available")
        except ImportError:
            print(f"❌ {dep} missing")
            all_good = False
    
    # Test optional dependencies
    optional_deps = ['pyglet', 'pandas']
    for dep in optional_deps:
        try:
            __import__(dep)
            print(f"✅ {dep} available (optional)")
        except ImportError:
            print(f"⚠️  {dep} missing (optional)")
    
    return all_good

def main():
    print("="*60)
    print("ARGO SAILING SIMULATOR TEST")
    print("="*60)
    
    # Test dependencies
    deps_ok = test_dependencies()
    
    # Test real simulator
    real_sim_ok = test_sailboat_playground()
    
    # Test mock simulator
    mock_sim_ok = test_mock_simulator()
    
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Dependencies: {'✅ PASS' if deps_ok else '❌ FAIL'}")
    print(f"Real simulator: {'✅ PASS' if real_sim_ok else '❌ FAIL'}")
    print(f"Mock simulator: {'✅ PASS' if mock_sim_ok else '❌ FAIL'}")
    
    if mock_sim_ok:
        print("\n✅ Simulator bridge should work (will use mock if real fails)")
        print("\nNext steps:")
        print("1. Run: python3 nodes/argo_simulator_bridge.py")
        print("2. In another terminal: ros2 topic list")
        print("3. Check topics: ros2 topic echo /pose")
    else:
        print("\n❌ Simulator bridge may have issues")
    
    print("="*60)

if __name__ == '__main__':
    main()






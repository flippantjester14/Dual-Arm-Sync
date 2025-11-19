#!/usr/bin/env python3
"""
Quick verification script to check if everything is installed correctly.
"""

import sys
import os

def check_imports():
    """Check if all required packages are installed."""
    print("Checking required packages...")
    
    packages = [
        ("pybullet", "PyBullet"),
        ("numpy", "NumPy"),
        ("yaml", "PyYAML"),
        ("cv2", "OpenCV"),
        ("scipy", "SciPy"),
        ("matplotlib", "Matplotlib"),
    ]
    
    all_ok = True
    for module, name in packages:
        try:
            __import__(module)
            print(f"  ✓ {name}")
        except ImportError:
            print(f"  ✗ {name} - NOT INSTALLED")
            all_ok = False
    
    return all_ok

def check_directories():
    """Check if all required directories exist."""
    print("\nChecking directory structure...")
    
    dirs = [
        "src/simulation",
        "src/perception",
        "src/control",
        "src/motion",
        "src/utils",
        "config",
        "assets",
        "scripts",
        "results",
    ]
    
    all_ok = True
    for dir_path in dirs:
        if os.path.exists(dir_path):
            print(f"  ✓ {dir_path}")
        else:
            print(f"  ✗ {dir_path} - MISSING")
            all_ok = False
    
    return all_ok

def check_config_files():
    """Check if configuration files exist."""
    print("\nChecking configuration files...")
    
    files = [
        "config/simulation_config.yaml",
        "config/robot_config.yaml",
    ]
    
    all_ok = True
    for file_path in files:
        if os.path.exists(file_path):
            print(f"  ✓ {file_path}")
        else:
            print(f"  ✗ {file_path} - MISSING")
            all_ok = False
    
    return all_ok

def check_pybullet_data():
    """Check if PyBullet data is accessible."""
    print("\nChecking PyBullet data...")
    
    try:
        import pybullet_data
        data_path = pybullet_data.getDataPath()
        print(f"  ✓ PyBullet data path: {data_path}")
        
        # Check for robot URDFs
        robot_files = [
            "ur5/ur5.urdf",
            "franka_panda/panda.urdf",
        ]
        
        for robot_file in robot_files:
            full_path = os.path.join(data_path, robot_file)
            if os.path.exists(full_path):
                print(f"  ✓ {robot_file}")
            else:
                print(f"  ✗ {robot_file} - NOT FOUND")
        
        return True
    except Exception as e:
        print(f"  ✗ Error accessing PyBullet data: {e}")
        return False

def main():
    print("="*60)
    print("Dual-Arm Synchronization - Setup Verification")
    print("="*60)
    
    checks = [
        check_imports(),
        check_directories(),
        check_config_files(),
        check_pybullet_data(),
    ]
    
    print("\n" + "="*60)
    if all(checks):
        print("✓ All checks passed! Setup is complete.")
        print("\nNext step: Run component tests with:")
        print("  python3 scripts/test_components.py")
    else:
        print("✗ Some checks failed. Please review the errors above.")
    print("="*60)

if __name__ == "__main__":
    main()
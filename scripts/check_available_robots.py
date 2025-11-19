#!/usr/bin/env python3
"""Check available robot URDFs in PyBullet data."""

import os
import pybullet_data

data_path = pybullet_data.getDataPath()
print(f"PyBullet data path: {data_path}\n")

print("Available robot URDFs:")
print("=" * 60)

for root, dirs, files in os.walk(data_path):
    for file in files:
        if file.endswith('.urdf'):
            rel_path = os.path.relpath(os.path.join(root, file), data_path)
            print(f"  {rel_path}")
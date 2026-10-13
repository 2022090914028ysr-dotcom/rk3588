import os
import sys

# Try different library paths
library_paths = [
    '/usr/lib',
    '/usr/lib64',
    '/usr/local/lib',
    '/home/ubuntu/miniforge/envs/control/lib',
]

for path in library_paths:
    if os.path.exists(os.path.join(path, 'librknnrt.so')):
        print(f"Found librknnrt.so in: {path}")
        os.environ['LD_LIBRARY_PATH'] = path + ':' + os.environ.get('LD_LIBRARY_PATH', '')

print(f"\nLD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', '')}")

from rknnlite.api import RKNNLite

print("\n=== Testing RKNN Runtime ===")

# Create instance
rknn = RKNNLite()
print("RKNNLite instance created")

# Load model
model_path = '/userdata/app/Robit_SlamNav/NavOA_ws/src/NavOA_ros/example/gazebo_limo/pretrain_limo/model_5000_quant.rknn'
ret = rknn.load_rknn(model_path)
print(f"load_rknn returned: {ret}")

if ret != 0:
    print("Failed to load model")
    sys.exit(1)

# Try to list available platforms/targets
print("\n=== Testing platform detection ===")
try:
    platforms = rknn.list_support_target_platform()
    print(f"Supported platforms: {platforms}")
except Exception as e:
    print(f"Cannot list platforms: {e}")

# Try to list devices
try:
    devices = rknn.list_devices()
    print(f"Available devices: {devices}")
except Exception as e:
    print(f"Cannot list devices: {e}")

# Try different init_runtime approaches
print("\n=== Testing init_runtime ===")
test_cases = [
    # (target, core_mask, description)
    ('rk3588', 0, 'RK3588 with auto core'),
    ('rk3588', None, 'RK3588 no core mask'),
    ('rk3588', 'auto', 'RK3588 string auto'),
    ('rknn2', 0, 'RKNN2 platform'),
    ('rockchip', 0, 'Rockchip platform'),
    ('', 0, 'Empty target'),
    (None, 0, 'None target'),
    ('android', 0, 'Android target'),
    ('linux', 0, 'Linux target'),
]

for target, core_mask, desc in test_cases:
    print(f"\nTrying: {desc}")
    try:
        if core_mask is not None:
            ret = rknn.init_runtime(target=target, core_mask=core_mask)
        else:
            ret = rknn.init_runtime(target=target)
        print(f"  Result: {ret} (0 means success)")
        if ret == 0:
            print(f"  SUCCESS with {desc}!")
            
            # Try inference to verify
            try:
                import numpy as np
                dummy_input = np.random.randn(1, 100, 2).astype(np.float32)
                outputs = rknn.inference(inputs=[dummy_input])
                print(f"  Inference successful! Output shape: {outputs[0].shape}")
            except Exception as e:
                print(f"  Inference failed: {e}")
            
            break
    except Exception as e:
        print(f"  Error: {e}")

rknn.release()

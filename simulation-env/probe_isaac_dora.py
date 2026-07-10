import sys
import traceback

sys.path.append("/home/zhz/fsd-car/simulation-env")

print("🛰️ NEXUS - Starting Rigorous Diagnostic Probe...", flush=True)

try:
    print("1. Attempting to import isaac_dora_node...", flush=True)
    import isaac_dora_node
    print("✓ Import succeeded!", flush=True)
    
    print("2. Testing CLIDDEngine instantiation with ONNX...", flush=True)
    engine = isaac_dora_node.CLIDDEngine("/home/zhz/fsd-car/model/xfeat_640x640.onnx")
    print("✓ CLIDDEngine instantiation succeeded!", flush=True)
    
    print("3. Testing BionicFrogEye instantiation...", flush=True)
    frog = isaac_dora_node.BionicFrogEye(640, 480)
    print("✓ BionicFrogEye instantiation succeeded!", flush=True)

    print("4. Executing main() inside guarded try-except block...", flush=True)
    isaac_dora_node.main()
    
except Exception as e:
    print("\n❌ CRITICAL CRASH DETECTED IN PROBE!", flush=True)
    print("========================================================", flush=True)
    print(f"Exception Type: {type(e).__name__}", flush=True)
    print(f"Exception Message: {e}", flush=True)
    print("Traceback:", flush=True)
    traceback.print_exc(file=sys.stdout)
    print("========================================================", flush=True)
    sys.exit(1)

import subprocess
import time

# Number of vehicles
VEHICLE_COUNT = 2

# Location (from Tools/autotest/locations.txt)
LOCATION = "YEGVHARD"

# Base ports
BASE_UDP_MP = 14550
BASE_UDP_MGCS = 14780
BASE_TCP_COMPANION = 5760

# Enable/disable console/map windows
ENABLE_CONSOLE = False
ENABLE_MAP = False

def run_vehicle(instance_id):
    mp_port = BASE_UDP_MP + instance_id
    mgcs_port = BASE_UDP_MGCS + instance_id
    comp_port = BASE_TCP_COMPANION + instance_id

    cmd = [
        "sim_vehicle.py",
        "-v", "ArduPlane",
        f"-I{instance_id}",
        "-L", LOCATION,
        "--no-mavproxy",
        f"--out=udp:127.0.0.1:{mp_port}",
        f"--out=udp:127.0.0.1:{mgcs_port}",
        f"--out=tcp:127.0.0.1:{comp_port}"
    ]

    if ENABLE_CONSOLE:
        cmd.append("--console")
    if ENABLE_MAP:
        cmd.append("--map")

    print(f"[INFO] Launching vehicle {instance_id} at {LOCATION} → MP:{mp_port}, MGCS:{mgcs_port}, COMP:{comp_port}")
    return subprocess.Popen(cmd)

if __name__ == "__main__":
    processes = []
    for i in range(VEHICLE_COUNT):
        p = run_vehicle(i)
        processes.append(p)
        time.sleep(1)  # brief delay between starts

    try:
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Terminating all vehicles...")
        for p in processes:
            p.terminate()

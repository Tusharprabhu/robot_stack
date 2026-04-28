import time
import serial
from rplidar import RPLidar

def connect_lidar(port='/dev/serial0', baudrate=115200):
    """Establish reliable connection to RPLidar with proper initialization"""
    lidar = RPLidar(port, baudrate=baudrate)
    
    # Critical initialization sequence
    for attempt in range(3):
        try:
            print(f"Attempt {attempt+1} to connect to lidar...")
            lidar.reset()
            time.sleep(1)
            
            # Try to read health first - some LIDARs need this
            try:
                health = lidar.get_health()
                print(f"LIDAR health: {health}")
            except:
                pass
            
            # Try to read info
            try:
                info = lidar.get_info()
                print(f"Connected to RPLidar model {info['model']}")
                return lidar
            except:
                pass
            
            # If health/info fails, try express scan mode
            try:
                for _ in lidar.iter_scans(max_buf_meas=1, min_len=1):
                    print("✅ Connected to RPLidar via express scan")
                    return lidar
            except:
                pass
            
            time.sleep(0.5)
            
        except Exception as e:
            print(f"Connection attempt {attempt+1} failed: {e}")
            time.sleep(0.5)
    
    raise ConnectionError("Failed to connect to RPLidar after 3 attempts")

def main():
    LIDAR_PORT = "/dev/ttyUSB0"
    
    # Connect with proper initialization
    lidar = connect_lidar(LIDAR_PORT)
    
    try:
        print("LIDAR info:", lidar.get_info())
        print("LIDAR health:", lidar.get_health())

        # Use express scan mode for better reliability
        while True:
            try:
                # This bypasses the problematic iter_scans() 
                scan = lidar.iter_scans(timeout=1.0)
                scan_data = next(scan)
                
                # Process scan data
                angles = []
                distances = []
                for quality, angle, distance_mm in scan_data:
                    if distance_mm > 0:  # Skip invalid measurements
                        angles.append(angle)
                        distances.append(distance_mm / 1000.0)  # Convert to meters
                
                if angles:  # Only process if we have valid data
                    print(f"Scan: {len(angles)} measurements")
                    # Here you would implement your obstacle avoidance logic
                    # Example: check front distance
                    front_distance = min(distances) if distances else float('inf')
                    print(f"Front distance: {front_distance:.2f}m")
                    
                    # Simple obstacle avoidance
                    if front_distance < 0.5:
                        print("OBSTACLE DETECTED! Stopping or turning...")
                        # Add your send() command here for controlling robot
                    else:
                        print("Path clear - moving forward")
                
                time.sleep(0.05)

            except StopIteration:
                # No more scans in buffer, wait for new data
                time.sleep(0.1)
            except Exception as e:
                print(f"Error during scanning: {e}")
                time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("\nStopping lidar...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == "__main__":
    main()
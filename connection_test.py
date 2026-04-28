import serial
import time
from rplidar import RPLidar

def test_lidar_connection():
    print("🔍 Testing LIDAR connection...")
    port = "/dev/ttyUSB0"
    
    try:
        # Try basic connection
        lidar = RPLidar(port, baudrate=115200)
        print("✅ RPLidar object created")
        
        # Try to get health info
        try:
            health = lidar.get_health()
            print(f"✅ LIDAR health: {health}")
        except:
            print("⚠️  Could not read health, trying alternative...")
        
        # Try express scan
        try:
            print("🔄 Starting express scan...")
            lidar.start()
            time.sleep(1)
            
            # Read a few measurements
            measurements = 0
            for scan in lidar.iter_scans(max_buf_meas=1, min_len=1):
                for quality, angle, distance in scan:
                    if distance > 0:
                        measurements += 1
                        if measurements > 5:
                            break
                if measurements > 5:
                    break
            
            if measurements > 5:
                print("✅ LIDAR is working! Received measurements")
                return True
            else:
                print("❌ No measurements received")
                return False
                
        except Exception as e:
            print(f"❌ Error during scan: {e}")
            return False
            
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return False
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

def test_serial_connection():
    print("🔍 Testing serial connection...")
    port = "/dev/serial0"
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print("✅ Serial port opened")
        
        # Send a test command
        test_cmd = "{{\"T\":13,\"X\":0,\"Z\":0}}\n"
        ser.write(test_cmd.encode())
        print("✅ Test command sent")
        
        # Read response
        response = ser.readline()
        if response:
            print(f"✅ Response received: {response.decode().strip()}")
        else:
            print("⚠️  No response received")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return False

def main():
    print("🦹 Robot Stack Connection Test")
    print("=" * 40)
    
    # Test serial connection first
    print("\n1. Testing Serial Connection to Rover:")
    serial_ok = test_serial_connection()
    print(f"Result: {'✅ Working' if serial_ok else '❌ Failed'}")
    
    # Test LIDAR connection
    print(f"\n2. Testing LIDAR Connection:")
    lidar_ok = test_lidar_connection()
    print(f"Result: {'✅ Working' if lidar_ok else '❌ Failed'}")
    
    # Summary
    print("\n" + "=" * 40)
    if serial_ok and lidar_ok:
        print("🎉 All systems working! Ready to run robot control")
        print("📋 Next steps:")
        print("   1. Run: python boka.py")
        print("   2. Robot should start moving and avoiding obstacles")
    else:
        print("❌ Some connections failed. Check troubleshooting below:")
        if not serial_ok:
            print("   • Check rover is connected to /dev/serial0")
            print("   • Verify rover power and serial cable")
        if not lidar_ok:
            print("   • Check LIDAR is connected to /dev/ttyUSB0")
            print("   • Verify LIDAR power and USB connection")
            print("   • Try different USB port")
    
    print("\n📋 Run this test again if connections change")

if __name__ == "__main__":
    main()
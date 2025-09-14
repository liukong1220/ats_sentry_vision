#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time

class TFTreeValidator(Node):
    def __init__(self):
        super().__init__('tf_tree_validator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Give some time for tf data to accumulate
        time.sleep(2.0)
        
        # Test tf tree connectivity
        self.test_tf_tree()
        
    def test_tf_tree(self):
        """Test tf tree connectivity for rv2 namespace"""
        frames_to_test = [
            ('rv2/rv2_odom', 'rv2/rv2_gimbal_link'),
            ('rv2/rv2_gimbal_link', 'rv2/rv2_camera_link'),
            ('rv2/rv2_camera_link', 'rv2/rv2_camera_optical_frame'),
            # Test connection to navigation odom if exists
            ('odom', 'rv2/rv2_odom')
        ]
        
        print("=== TF Tree Validation Results ===")
        all_good = True
        
        for parent, child in frames_to_test:
            try:
                # Try to get transform
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
                )
                print(f"✓ Transform {parent} -> {child}: OK")
                
                # Print transform details
                trans = transform.transform.translation
                rot = transform.transform.rotation
                print(f"  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}")
                print(f"  Rotation: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}")
                
            except Exception as e:
                print(f"✗ Transform {parent} -> {child}: FAILED - {str(e)}")
                all_good = False
        
        # Test complete chain
        try:
            transform = self.tf_buffer.lookup_transform(
                'rv2/rv2_odom', 'rv2/rv2_camera_optical_frame', 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            print(f"✓ Complete chain rv2/rv2_odom -> rv2/rv2_camera_optical_frame: OK")
        except Exception as e:
            print(f"✗ Complete chain rv2/rv2_odom -> rv2/rv2_camera_optical_frame: FAILED - {str(e)}")
            all_good = False
            
        if all_good:
            print("\n🎉 All tf transforms are working correctly!")
        else:
            print("\n⚠️  Some tf transforms are not working. Check the errors above.")
            
        # List all available frames
        print(f"\n=== Available Frames ===")
        try:
            frame_yaml = self.tf_buffer.get_frames_as_yaml()
            print(frame_yaml)
        except Exception as e:
            print(f"Failed to get frames: {e}")

def main():
    rclpy.init()
    validator = TFTreeValidator()
    
    try:
        # Keep node alive to receive tf data
        rclpy.spin_once(validator, timeout_sec=5.0)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
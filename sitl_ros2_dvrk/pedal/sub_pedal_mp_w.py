# ROS2 Libraries
import rclpy

# Custom node
from nodes.pedal import sub_pedal_mp_w

def main(args=None):
    rclpy.init(args=args)
    params = {
        "node_name"  : "sub_pedal_mp_write",
        "queue_size" : 5,
        "port"       : "/dev/ttyACM0",
        "baud"       : 57600,
    }
    
    app = sub_pedal_mp_w.SUB_PEDAL_MP_W(params)

    try:
        rclpy.spin(app)
    except:
        pass
    finally:
        app.arduino.close()
        app.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
    
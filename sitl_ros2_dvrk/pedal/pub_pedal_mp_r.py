# ROS2 Libraries
import rclpy

# Custom node
from nodes.pedal import pub_pedal_mp_r

def main(args=None):
    rclpy.init(args=args)
    params = {
        "node_name"  : "pub_pedal_mp_read",
        "queue_size" : 5,
        "port"       : "/dev/ttyACM0",
        "baud"       : 57600,
        "hz"         : 100,
    }
    
    app = pub_pedal_mp_r.PUB_PEDAL_MP_R(params)

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

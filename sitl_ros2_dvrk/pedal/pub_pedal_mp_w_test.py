# ROS2 Libraries
import rclpy

# Custom node
from nodes.pedal import pub_pedal_mp_w_test

def main(args=None):
    rclpy.init(args=args)
    params = {
        "node_name"  : "pub_pedal_mp_write_test",
        "queue_size" : 5,
        "command"    : "L",
        "hz"         : 0.2,
    }
    
    app = pub_pedal_mp_w_test.PUB_PEDAL_MP_W(params)

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

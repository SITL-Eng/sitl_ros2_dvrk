import rclpy

from nodes.pedal import pub_pedal_dvrk

def main(args=None):
    rclpy.init(args=args)
    params = {
        "node_name": "pub_pedal_dvrk",
        "queue_size": 10,
        "pub_rate": 100,
    }
    node = pub_pedal_dvrk.PUB_DVRK_PEDALS(params)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
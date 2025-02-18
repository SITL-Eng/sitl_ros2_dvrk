import rclpy
from nodes.auto import dissect_key

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name': 'grasp_key',
        'hz': 100,
        'queue_size': 5,
    }

    app = dissect_key.DISSECT_KEY(params)
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy_node()
        rclpy.shutdown()
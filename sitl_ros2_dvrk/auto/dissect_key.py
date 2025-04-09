import rclpy
from nodes.auto import dissect_key

# "Press one of the following keys..."
# "a: align instrument before following boundary."
# "f: follow the tissue boundary"
# "r: return to the position before following boundary."
# "i: go to the initial position."
# "b: stop the following process"

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name': 'dissect_key',
        'hz': 100,
        'queue_size': 1,
        'key': 'i',
    }

    app = dissect_key.DISSECT_KEY(params)
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy_node()
        rclpy.shutdown()
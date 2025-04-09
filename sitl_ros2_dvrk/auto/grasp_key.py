import rclpy
from nodes.auto import grasp_key

# "Press one of the following keys..."
# "a: align forceps before grasping."
# "g: grasp the target tissue."
# "p: pull the target tissue."
# "r: release the target tissue."
# "o: open the jaws of the forceps."
# "c: close the jaws of the forceps."
# "i: go to the initial position."
# "s: stop the pulling process"

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name': 'grasp_key',
        'hz': 100,
        'queue_size': 1,
        'key': 'o',
    }

    app = grasp_key.GRASP_KEY(params)
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy_node()
        rclpy.shutdown()
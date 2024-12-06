import rospy
from .camera import CameraSubscriber
from .steering import DifferentialSteering

class Labyrinth:
    def __init__(self, robot_name):
        self.camera = CameraSubscriber(robot_name)
        self.steering = DifferentialSteering(robot_name)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    robot_name = "phi"
    solve = Labyrinth(robot_name)
    solve.run()

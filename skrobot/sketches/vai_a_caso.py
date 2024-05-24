from coppeliasim_zmqremoteapi_client import RemoteAPIClient


sim = RemoteAPIClient().getObject('sim')
sim.startSimulation()

# Get the handle of the robot
left_motor_handle = sim.getObject("./leftMotor")
right_motor_handle = sim.getObject("./rightMotor")



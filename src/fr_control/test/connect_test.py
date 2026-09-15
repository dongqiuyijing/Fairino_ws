from fairino import Robot

robot = Robot.RPC("192.168.58.2")

ret = robot.GetActualJointPosDegree()
print(ret)

ret = robot.GetActualTCPPose()
print(ret)

robot.CloseRPC()
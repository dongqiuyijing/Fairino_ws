from fairino import Robot

robot = Robot.RPC("192.168.58.2")

ret, joints = robot.GetActualJointPosDegree()
print("ret =", ret)
print("joints =", joints)

ret, tcp = robot.GetActualTCPPose()
print("TCP =", tcp)

robot.CloseRPC()
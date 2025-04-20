from rtde_receive import RTDEReceiveInterface

rtde_receive = RTDEReceiveInterface("172.17.201.180")
print(rtde_receive.getActualQ())
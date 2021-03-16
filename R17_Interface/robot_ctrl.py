import st, serial

P1 = [-13671,14750,1041]
P2 = [3137,7931,7043]
P3 = [0,0,0]

robot = st.StArm('/dev/ttyS13')
robot.roboforth()
robot.start()
robot.joint()
robot.home()
robot.where()
# robot.abort()



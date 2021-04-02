import st, serial, os, subprocess,signal, threading, logging, multiprocessing, pdb, sys


# def receive_signal(signum, stack, robot):
#     print('Received:', signum)
#     robot.abort()

def kill_signal(stdin, robot):
    print("thread working")
    sys.stdin = stdin
    while True:
        # p = subprocess.Popen([sys.executable, "alarm_trigger.py"],
        #         #stdin=subprocess.PIPE,
        #         stdout=subprocess.PIPE)
        # out, _ = p.communicate(s.encode())
        # trigger = out.decode()
        # logging.info(out.decode())
        userIn = input("Press enter to abort.")
        if userIn == "":
            logging.info("abort detected")
            robot.abort()
            #os.kill(os.getpid(), signal.SIGUSR1)
            break

    # signal.signal(signal.SIGUSR1, receive_signal)
    # os.getpid()
# def play():
#     i = 1
#     while True:
#         logging.info("Thread successfully running! x%d", i)
#         i+=1

def robot_cmd(robot):
    P1 = ['0','3500','0']
    P2 = ['0','1000','']

    robot.roboforth()
    robot.start()
    robot.cartesian()
    robot.move_to(P1)
    #robot.move(P2)
    robot.home()
    robot.where()
    # robot.abort()

def main():
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%g")

    robot = st.StArm('/dev/ttyS13')

    x = threading.Thread(target=kill_signal, args=(sys.stdin,robot,))
    x.start()
    print("x process should have completed")
    robot_cmd(robot)

if __name__ == "__main__":
    main()

import st, serial, os, subprocess,signal, threading, logging, multiprocessing, pdb, sys, socket
import time
import datetime

# def receive_signal(signum, stack, robot):
#     print('Received:', signum)
#     robot.abort()
def current_milli_time():
    return round(time.time() * 1000)

def kill_signal(stdin, robot, lock,sock):
    while True:
        data,address = sock.recvfrom(4096)
        str_data = data.decode()
        print(str_data)
        if "abort" in str_data:
            logging.info("abort detected")
            start = current_milli_time()
            logging.info("start time = %d", % start)
            try:
                lock.acquire()
                robot.abort()
                lock.release()
            finally:
                robot.close_com()
                endT = current_milli_time()
                logging.info("end time = %d, totoal time = %d", % endT, % (endT-start))
                break
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
    P2 = ['3000','3500','0']
    try:
        robot.roboforth()
        robot.start()
        robot.cartesian()
        for x in range(5):
            robot.move_to(P1)
            time.sleep(1)
            robot.move_to(P2)
            time.sleep(1)
        robot.home()
        robot.where()
    except Exception as e:
        if "not open" in str(e):
            print("Object too close.")
    # robot.abort()

def main():
    HOST = '127.0.0.1'
    PORT = 65432

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%g")
    lock = threading.Lock()

    robot = st.StArm('/dev/ttyUSB0')

    x = threading.Thread(target=kill_signal, args=(sys.stdin,robot,lock,s,), daemon=True)
    x.start()

    robot_cmd(robot)
    s.close()

if __name__ == "__main__":
    main()

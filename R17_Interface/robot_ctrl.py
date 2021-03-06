import st, serial, os, subprocess,signal, threading, logging, multiprocessing, pdb, sys, socket
import time
import datetime
from queue import Queue

# def receive_signal(signum, stack, robot):
#     print('Received:', signum)
#     robot.abort()
def current_milli_time():
    return round(time.time() * 1000)

def kill_signal(stdin, robot, lock,sock,q):
    while True:
        data,address = sock.recvfrom(4096)
        str_data = data.decode()
        print(str_data)
        if "abort" in str_data:
            logging.info("abort detected")
            start = current_milli_time()
            logging.info("start time = %s" % start)
            try:
                lock.acquire()
                robot.abort()
                lock.release()
            finally:
                robot.close_com()
                endT = current_milli_time()
                logging.info("end time = %d, total time = %s" % (endT, (endT-start)))
                break
            break
        if "speed" in str_data:
            logging.info("Reducing speed...")
            start_sp = current_milli_time()
            logging.info("Start time for speed reduction: %s" % start_sp)
            q.put(1)
            #break
            # lock.acquire()
            # try:
            #     robot.set_speed(5000) #set speed to half of default speed
            # finally:
            #     robot.flush()
            #     lock.release()

def robot_cmd(robot, q):
    P1 = ['0','3500','0']
    P2 = ['3000','3500','0']
    try:
        robot.roboforth()
        robot.start()
        robot.cartesian()
        for x in range(10):
            if not q.empty():
                robot.set_speed(5000) #set speed to half of default speed
                enT = current_milli_time()
                logging.info("End time = %s" % enT)
            else:
                robot.set_speed(10000)
            robot.move_to(P1)
            #time.sleep(1)
            robot.move_to(P2)
            #time.sleep(1)
        robot.home()
        robot.where()
    except Exception as e:
        if "not open" in str(e):
            print("Object too close.")
    # robot.abort()

def main():
    HOST = '127.0.0.1'
    PORT = 65432

    q = Queue()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%g")
    lock = threading.Lock()

    robot = st.StArm('/dev/ttyUSB0')

    x = threading.Thread(target=kill_signal, args=(sys.stdin,robot,lock,s,q,), daemon=True)
    x.start()

    robot_cmd(robot,q)
    s.close()

if __name__ == "__main__":
    main()

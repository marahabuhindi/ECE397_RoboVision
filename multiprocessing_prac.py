from multiprocessing import Process
from  contextlib import redirect_stdout
import logging, os, sys, io, socket
import tmov

def detect_stop():
    tmov.execution()
    
def main():
    HOST = '127.0.0.1'
    PORT = 65432

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))

    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%g")
    
    x = Process(target=detect_stop, args= (), daemon = True)
    x.start()
    
    while True:
        data,address = s.recvfrom(4096)
        print(data)
        if("abort" in data.decode()):
            print("abort...")
            s.close
            break


if __name__=="__main__":
    main()
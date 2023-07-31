import time
from threading import Thread

def sleep(t):
    while True:
        t1 = time.time()
        time.sleep(t)
        print(time.time() - t1)

if __name__ == "__main__":
    sleep_thread = Thread(target=sleep, args=[0.01])
    sleep_thread.start()
    while True:
        pass
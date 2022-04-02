# Copyright 2020 Sanggyu Lee. All rights reserved.
# sanggyu.developer@gmail.com
import zmq
import time
import socket
import numpy as np
import traceback
import imagezmq
import cv2
import threading
import random

if 'jajucha' in __name__:
    from jajucha import config
else:
    import config


def getIp(server_ip=config.server_ip):
    """https://stackoverflow.com/questions/166506/finding-local-ip-addresses-using-pythons-stdlib"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((server_ip, 20000))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        s.close()
        return random.randrange(10000)


ip = getIp()


# https://github.com/jeffbass/imagezmq/blob/master/examples/pub_sub_receive.py
# Helper class implementing an IO deamon thread
class VideoStreamSubscriber:

    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self._stop = False
        self._data_ready = threading.Event()
        self._front_ready = False
        self._back_ready = False
        self._msg = None
        self._thread = threading.Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def receive(self, timeout=10.0):
        flag = self._data_ready.wait(timeout=timeout)
        if not flag:
            raise TimeoutError(
                "Timeout while reading from subscriber tcp://{}:{}".format(self.hostname, self.port))
        self._data_ready.clear()
        self._front_ready = False
        self._back_ready = False
        return self._msg, self._front, self._back

    def _run(self):
        receiver = imagezmq.ImageHub("tcp://{}:{}".format(self.hostname, self.port), REQ_REP=False)
        while not self._stop:
            msg, frame = receiver.recv_jpg()
            msg = msg.split(',')
            self._msg = msg[0]
            if msg[1] == 'front':
                self._front = frame
                self._front_ready = True
            elif msg[1] == 'rear':
                self._back = frame
                self._back_ready = True
            if self._front_ready and self._back_ready:
                self._data_ready.set()
        receiver.close()

    def close(self):
        self._stop = True


class Client:
    ctx = zmq.Context()

    def __init__(self, address):
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(address)
        self.pollin = zmq.Poller()
        self.pollin.register(self.sock, zmq.POLLIN)
        self.id = None
        self.connect()

    def connect(self):
        if self.id:
            return

        self.sock.send_string('HI')
        if not self.pollin.poll(5000):
            raise TimeoutError()
        msg = self.sock.recv_string()
        msg = msg.split()
        if msg[0] == 'OK':
            self.id = msg[1]
        else:
            self.id = None

    def override(self):
        if self.id:
            return
        success = False
        for id in range(1, 100):
            msg = self.send('BYE %s' % id)
            if msg == 'OK':
                success = True
                break
        if success:
            self.connect()
            if self.id is not None:
                return True

    def send(self, msg):
        try:
            # if self.pollout.poll(100):
            self.sock.send_string(msg)
            # else:
            #     raise TimeoutError()
            # if self.pollin.poll(100):
            if not self.pollin.poll(1000):
                raise TimeoutError()
            ans = self.sock.recv_string()
            # else:
            #     raise TimeoutError()
        except zmq.error.ZMQError:
            time.sleep(0.1)
        return ans

    def sendCommand(self, steer, velocity):
        if self.id:
            if steer > 100:
                steer = 100
            elif steer < -100:
                steer = -100
            if velocity > 300:
                velocity = 300
            elif velocity < -300:
                velocity = -300
            if self.send("DO %s %d %.1f" % (self.id, steer, velocity)) == 'OK':
                return True

    def disconnect(self):
        if self.id:
            self.send('BYE %s' % self.id)
            self.id = None

    def exit(self):
        if self.id:
            if self.send('EXIT %s' % self.id) == 'BYE':
                self.id = None
                self.quit()
                return True

    def quit(self):
        self.sock.close()
        # print("Command Connection Closed")


if __name__ == '__main__':
    # Initialize image SUB first!
    ##ctx = zmq.Context()
    ##sock = ctx.socket(zmq.REQ)
    ##sock.connect("tcp://%s:%d"%config.address)
    ##sock.send_string("INIT "+ip)
    ##res = sock.recv_string()
    ##
    receiver = VideoStreamSubscriber(*config.image_address)
    # receiver = imagezmq.ImageHub("tcp://{}:{}".format(*config.image_address), REQ_REP=False)
    try:
        while True:
            msg, front, back = receiver.receive()
            front = cv2.imdecode(np.frombuffer(front, dtype='uint8'), -1)
            back = cv2.imdecode(np.frombuffer(back, dtype='uint8'), -1)
            msg = msg.split()[0]
            print(time.time() - float(msg))
            cv2.imshow('front', front)
            cv2.imshow('back', back)

            cv2.waitKey(1)
    except (KeyboardInterrupt, SystemExit):
        print('Exit due to keyboard interrupt')
    except Exception as ex:
        print('Python error with no Exception handler:')
        print('Traceback error:', ex)
        traceback.print_exc()
    finally:
        receiver.close()

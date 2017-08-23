import bluesock
import struct


class Brick(object):

    RX_DATA_FORMAT = 'IbbHiihhhHhhi'

    def __init__(self, name, host, port):
        self.name = name
        self.host = host
        self.port = port
        self.sock = bluesock.BlueSock(self.host, self.port)

    def connect(self):
        self.sock.connect()

    def send(self, data):
        self.sock.send(data)

    def recv(self, numbytes):
        return self.sock.recv(numbytes)

    def send_motor_power(self, mot_l, mot_r):
        self.sock.send(self._pack_tx_data(1, mot_l, mot_r))

    def send_motor_pid_ref(self, mot_l, mot_r):
        self.sock.send(self._pack_tx_data(1, mot_l, mot_r))

    def recv_data(self):
        data = self.sock.recv(32)
        return self._unpack_rx_data(data)

    def _pack_tx_data(self, opcode, mot_l, mot_r):
        return struct.pack("iiii", opcode, mot_l, mot_r, 0)

    def _unpack_rx_data(self, data):
        return struct.unpack(self.RX_DATA_FORMAT, data)
        
    def send_motor_vel(self, vl, vr):
        self.sock.send(struct.pack("bb30x",vl,vr))
    

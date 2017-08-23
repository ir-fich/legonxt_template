import sys


class BlueSock(object):
    type = 'bluetooth'

    def __init__(self, host, port=1):
        self.host = host
        self.sock = None
        self.port = port
        self.connected = False

    def __str__(self):
        return 'Bluetooth (%s)' % self.host

    def connect(self):
        if sys.platform.startswith('darwin'):
            import lightblue as bluesock

            self.sock = bluesock.socket(bluesock.RFCOMM)
        else:
            import bluetooth as bluesock

            self.sock = bluesock.BluetoothSocket(bluesock.RFCOMM)

        try:
            self.sock.connect((self.host, self.port))
            self.connected = True
        except (bluesock.BluetoothError, IOError):
            raise

    def close(self):
        if self.sock:
            self.sock.close()
            self.connected = False
            self.sock = None

    def send(self, data):
        try:
            self.sock.send(data)
        except:
            raise

    def recv(self, numbytes):
        try:
            return self.sock.recv(numbytes)
        except:
            raise

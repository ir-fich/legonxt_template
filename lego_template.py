import numpy as np
import time

from nxt_lib.nxt_list import nxt_list, data_idx
import nxt_lib.brick as brick


Delta = 0.10

# Connect to Lego NXT
nxt_conf = nxt_list["05"]
nxt = brick.Brick(nxt_conf["name"], nxt_conf["mac"], nxt_conf["port"])
nxt.connect()

do_loop = True
first_run = True
t = 0
if nxt.sock.connected:
    raw_input("Press enter to continue...")
    while do_loop:
        _data = nxt.recv_data()
        if _data:
            starttime = time.time()
            print _data
            _uk = np.around([0,0] , decimals=0)
            print "%3d (%10.5g s)" % (t, time.time()-starttime)
            nxt.send_motor_power(_uk[0], _uk[1])
            t+=1

nxt.send_motor_power(0, 0)

#!/usr/bin/env python
'''
devsim.py -- a simple device simulator.

'''
import os
import threading
import logging

from pymodbus.framer.rtu_framer import ModbusRtuFramer
from pymodbus.server.sync import StartTcpServer, StartSerialServer
from pymodbus.datastore import (ModbusSequentialDataBlock,
                                ModbusSlaveContext,
                                ModbusServerContext)

import random
import time

version = "0.1"

logger = logging.getLogger('modbridge')
logger.setLevel(logging.DEBUG)

pymodbus_logger = logging.getLogger('pymodbus')
pymodbus_logger.setLevel(logging.DEBUG)
"""
               Register Type    Function Code
Coils               0               1
Discrete Input      1               2
Input Register      3               4
Holding Register    4               3

Function Code 	Action 	        Table Name
1            	Read 	        Discrete Output Coils
5           	Write single 	Discrete Output Coil
15          	Write multiple 	Discrete Output Coils
2           	Read 	        Discrete Input Contacts
4           	Read 	        Analog Input Registers
3           	Read 	        Analog Output Holding Registers
6              	Write single 	Analog Output Holding Register
16           	Write multiple 	Analog Output Holding Registers
"""


def sync(context):
    """Update Modbus context."""
    random.seed()
    while True:
        values = [random.randint(0, 1000) for i in range(10)]
        for slave_id in range(1, 4):
            context[slave_id].setValues(4, 0, values)
        time.sleep(0.1)


if __name__ == "__main__":
    fmt = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    file_handler = logging.FileHandler(os.getcwd() + '/log')
    # logging.CRITICAL
    # logging.ERROR
    # logging.WARNING
    # logging.INFO
    # logging.DEBUG
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(fmt)
    logger.addHandler(file_handler)

    pymodbus_logger.addHandler(file_handler)

    block = ModbusSequentialDataBlock(0, [0]*10)
    slaves = {
        1: ModbusSlaveContext(di=block, co=block, hr=block, ir=block),
        2: ModbusSlaveContext(di=block, co=block, hr=block, ir=block),
        3: ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
    }
    context = ModbusServerContext(slaves=slaves, single=False)

    t = threading.Thread(target=sync, args=(context,))
    t.daemon = True
    t.start()

    StartTcpServer(context, address=('127.0.0.1', 5020))
    # StartSerialServer(context,
    #                   framer=ModbusRtuFramer,
    #                   port='/dev/ttyUSB0',
    #                   baudrate=115200)

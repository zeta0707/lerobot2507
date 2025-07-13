#!/usr/bin/env python

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
LX16A_END = 0
LX16A_MODEL_NUM = 1000

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1  # Port is busy (in use)
COMM_TX_FAIL = -2  # Failed transmit instruction packet
COMM_RX_FAIL = -3  # Failed get status packet
COMM_TX_ERROR = -4  # Incorrect instruction packet
COMM_RX_WAITING = -5  # Now recieving status packet
COMM_RX_TIMEOUT = -6  # There is no status packet
COMM_RX_CORRUPT = -7  # Incorrect status packet
COMM_NOT_AVAILABLE = -9  #

# Macro for Control Table Value
def LX16AGETEND():
    global LX16A_END
    return LX16A_END

def LX16ASETEND(e):
    global LX16A_END
    LX16A_END = e

def LX16ATOHOST(a, b):
    if (a & (1<<b)):
        return -(a & ~(1<<b))
    else:
        return a

def LX16ATOSCS(a, b):
    if (a<0):
        return (-a | (1<<b))
    else:
        return a

def LX16A_MAKEWORD(a, b):
    global LX16A_END
    if LX16A_END==0:
        return (a & 0xFF) | ((b & 0xFF) << 8)
    else:
        return (b & 0xFF) | ((a & 0xFF) << 8)

def LX16A_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16

def LX16A_LOWORD(l):
    return l & 0xFFFF

def LX16A_HIWORD(l):
    return (l >> 16) & 0xFFFF

def LX16A_LOBYTE(w):
    global LX16A_END
    if LX16A_END==0:
        return w & 0xFF
    else:
        return (w >> 8) & 0xFF

def LX16A_HIBYTE(w):
    global LX16A_END
    if LX16A_END==0:
        return (w >> 8) & 0xFF
    else:
        return w & 0xFF
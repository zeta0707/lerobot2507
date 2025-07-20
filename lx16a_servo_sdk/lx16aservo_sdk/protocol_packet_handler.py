#!/usr/bin/env python

from .lx16aservo_def import *
from time import sleep

TXPACKET_MAX_LEN = 250
RXPACKET_MAX_LEN = 250

# for Protocol Packet
PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_PARAMETER0 = 5

# Protocol Error bit
ERRBIT_VOLTAGE = 1
ERRBIT_ANGLE = 2
ERRBIT_OVERHEAT = 4
ERRBIT_OVERELE = 8
ERRBIT_OVERLOAD = 32

def _checksum(packet: list[int]) -> int:
    s = ~sum(packet[2:])
    return s % 256

def _to_servo_range(angle: float) -> int:
    return round(angle * 25 / 6)

def _from_servo_range(angle: int) -> float:
    return angle * 6 / 25

class protocol_packet_handler(object):
    def getProtocolVersion(self):
        return 1.0

    def getTxRxResult(self, result):
        if result == COMM_SUCCESS:
            return "[TxRxResult] Communication success!"
        elif result == COMM_PORT_BUSY:
            return "[TxRxResult] Port is in use!"
        elif result == COMM_TX_FAIL:
            return "[TxRxResult] Failed transmit instruction packet!"
        elif result == COMM_RX_FAIL:
            return "[TxRxResult] Failed get status packet from device!"
        elif result == COMM_TX_ERROR:
            return "[TxRxResult] Incorrect instruction packet!"
        elif result == COMM_RX_WAITING:
            return "[TxRxResult] Now receiving status packet!"
        elif result == COMM_RX_TIMEOUT:
            return "[TxRxResult] There is no status packet!"
        elif result == COMM_RX_CORRUPT:
            return "[TxRxResult] Incorrect status packet!"
        elif result == COMM_NOT_AVAILABLE:
            return "[TxRxResult] Protocol does not support this function!"
        else:
            return ""

    def getRxPacketError(self, error):
        if error & ERRBIT_VOLTAGE:
            return "[RxPacketError] Input voltage error!"

        if error & ERRBIT_ANGLE:
            return "[RxPacketError] Angle sen error!"

        if error & ERRBIT_OVERHEAT:
            return "[RxPacketError] Overheat error!"

        if error & ERRBIT_OVERELE:
            return "[RxPacketError] OverEle error!"
        
        if error & ERRBIT_OVERLOAD:
            return "[RxPacketError] Overload error!"

        return ""
        
    def txPacket(self, port, txpacket):
        if port.is_using:
            return COMM_PORT_BUSY
        port.is_using = True

        packet = [0x55, 0x55, *txpacket]
        packet.append(_checksum(packet))
        total_packet_length = len(packet)

        # tx packet
        port.clearPort()
        written_packet_length = port.writePort(packet)
        #print ("[TxPacket]", packet)

        if total_packet_length == written_packet_length:
            port.is_using = False
            return COMM_SUCCESS
        
        port.is_using = False
        return COMM_TX_FAIL

    def rxPacket(self, port):
        rxpacket = []
        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 6  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while True:
            rxpacket.extend(port.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0x55) and (rxpacket[idx + 1] == 0x55):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[PKT_ID] > 0xFD) or (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet, header0/11, ID
                    if wait_length != (rxpacket[PKT_LENGTH] + 3):
                        wait_length = rxpacket[PKT_LENGTH] + 3
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if port.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            else:
                # check timeout
                if port.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        port.is_using = False

        #print ("[RxPacket]:",  rxpacket)

        return rxpacket, result

    def txRxPacket(self, port, txpacket):
        rxpacket = None
        error = 0

        # tx packet
        result = self.txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            return rxpacket, result, error

        # set packet timeout
        port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket(port)
            #txpacket[0] is servoID
            if result != COMM_SUCCESS or txpacket[0] == rxpacket[PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[0] == rxpacket[PKT_ID]:
            error = COMM_SUCCESS

        return rxpacket, result, error
    
    def ping(self, port, lx16a_id):
        error = 0

        txpacket = [lx16a_id, 3, 14]
        rxpacket, result, error = self.txRxPacket(port, txpacket)
        if rxpacket[PKT_ID] == lx16a_id:
            return LX16A_MODEL_NUM, result, error
        
        return None, result, error
    
    def get_action(self, port, lx16a_id):
        error = 0

        txpacket = [lx16a_id, 3, 28]
        rxpacket, result, error = self.txRxPacket(port, txpacket)

        #return angle when id is correct
        if rxpacket[PKT_ID] == lx16a_id:
            #value, 0 ~ 1000
            value = rxpacket[PKT_PARAMETER0] + rxpacket[PKT_PARAMETER0+1]* 256
            #angle shoud be 0 ~ 240.0
            angle = _from_servo_range(value - 65536 if value > 32767 else value)
            #120.0 is center
            angle = angle - 120.0
            return angle, result, error
        
        return 0.0, result, error
    
    def set_action(self, port, lx16a_id, angle):
        #motor move time
        move_time = 300

        angle = angle + 120.0
        if(angle < 0): 
            angle = 0
        if(angle > 240.0):
            angle = 240.0

        #value, 0 ~ 1000
        value = _to_servo_range(angle)
        txpacket = [lx16a_id, 7, 1, LX16A_LOBYTE(value), LX16A_HIBYTE(value), LX16A_LOBYTE(move_time), LX16A_HIBYTE(move_time)]
        result = self.txPacket(port, txpacket)
        sleep(0.05)
        return result

    def writeTxRx(self, port, lx16a_id, address, length, data):
        #no need to run, dummy command
        if address == 0:
            return 0, 0
        #no parameter command, actually LX16 don't have this kind of command. 
        if data == -1:
            txpacket= [lx16a_id, length + 3, address]
        else:
            #need to run, but change to txRxpacket instead of REG write
            txpacket= [lx16a_id, length + 5, address, data]

        #print("writeTxRx", txpacket)
        rxpacket, result, error = self.txRxPacket(port, txpacket)

        return result, error

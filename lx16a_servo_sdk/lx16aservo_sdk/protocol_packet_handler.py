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
            angle = rxpacket[PKT_PARAMETER0] + rxpacket[PKT_PARAMETER0+1]* 256
            angle = angle - 65536 if angle > 32767 else angle
            return angle, result, error
        
        return 0, result, error
    
    def set_action(self, port, lx16a_id, angle):
        time = 300

        if(angle > 1000):
            angle = 1000
        elif(angle < 0):
            angle = 0

        txpacket = [lx16a_id, 7, 1, LX16A_LOBYTE(angle), LX16A_HIBYTE(angle), LX16A_LOBYTE(time), LX16A_HIBYTE(time)]
        result = self.txPacket(port, txpacket)
        sleep(0.05)
        return result
    
    def readTx(self, port, lx16a_id, address, length):

        txpacket = [0] * 8

        if lx16a_id > BROADCAST_ID:
            return COMM_NOT_AVAILABLE

        txpacket[PKT_ID] = lx16a_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        result = self.txPacket(port, txpacket)

        # set packet timeout
        if result == COMM_SUCCESS:
            port.setPacketTimeout(length + 6)

        return result

    def readRx(self, port, lx16a_id, length):
        result = COMM_TX_FAIL
        error = 0

        rxpacket = None
        data = []

        while True:
            rxpacket, result = self.rxPacket(port)

            if result != COMM_SUCCESS or rxpacket[PKT_ID] == lx16a_id:
                break

        if result == COMM_SUCCESS and rxpacket[PKT_ID] == lx16a_id:
            error = COMM_SUCCESS
            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def readTxRx(self, port, lx16a_id, address, length):
        txpacket = [0] * 8
        data = []

        if lx16a_id > BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[PKT_ID] = lx16a_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        rxpacket, result, error = self.txRxPacket(port, txpacket)
        if result == COMM_SUCCESS:
            error = COMM_SUCCESS
            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def read1ByteTx(self, port, lx16a_id, address):
        return self.readTx(port, lx16a_id, address, 1)

    def read1ByteRx(self, port, lx16a_id):
        data, result, error = self.readRx(port, lx16a_id, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read1ByteTxRx(self, port, lx16a_id, address):
        data, result, error = self.readTxRx(port, lx16a_id, address, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTx(self, port, lx16a_id, address):
        return self.readTx(port, lx16a_id, address, 2)

    def read2ByteRx(self, port, lx16a_id):
        data, result, error = self.readRx(port, lx16a_id, 2)
        data_read = LX16A_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTxRx(self, port, lx16a_id, address):
        data, result, error = self.readTxRx(port, lx16a_id, address, 2)
        data_read = LX16A_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTx(self, port, lx16a_id, address):
        return self.readTx(port, lx16a_id, address, 4)

    def read4ByteRx(self, port, lx16a_id):
        data, result, error = self.readRx(port, lx16a_id, 4)
        data_read = LX16A_MAKEWORD(LX16A_MAKEWORD(data[0], data[1]),
                                  LX16A_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTxRx(self, port, lx16a_id, address):
        data, result, error = self.readTxRx(port, lx16a_id, address, 4)
        data_read = LX16A_MAKEWORD(LX16A_MAKEWORD(data[0], data[1]),
                                  LX16A_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def writeTxOnly(self, port, lx16a_id, address, length, data):
        txpacket = [0] * (length + 6)

        txpacket[PKT_ID] = lx16a_id
        txpacket[PKT_LENGTH] = length + 3
        #no address write, instead command
        txpacket[PKT_INSTRUCTION] = address
        txpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length] = data[0: length]

        print("writeonly", txpacket)
        result = self.txPacket(port, txpacket)
        port.is_using = False

        return result

    def writeTxRx(self, port, lx16a_id, address, length, data):
        #no need to run
        if address == 0:
            return 0, 0
        
        if data == -1:
            txpacket= [lx16a_id, length + 3, address]
        else:
            #need to run, but change to txRxpacket instead of REG write
            txpacket= [lx16a_id, length + 5, address, data]

        print("writeTxRx", txpacket)
        rxpacket, result, error = self.txRxPacket(port, txpacket)

        return result, error

    def write1ByteTxOnly(self, port, lx16a_id, address, data):
        data_write = [data]
        return self.writeTxOnly(port, lx16a_id, address, 1, data_write)

    def write1ByteTxRx(self, port, lx16a_id, address, data):
        data_write = [data]
        return self.writeTxRx(port, lx16a_id, address, 1, data_write)

    def write2ByteTxOnly(self, port, lx16a_id, address, data):
        data_write = [LX16A_LOBYTE(data), LX16A_HIBYTE(data)]
        return self.writeTxOnly(port, lx16a_id, address, 2, data_write)

    def write2ByteTxRx(self, port, lx16a_id, address, data):
        data_write = [LX16A_LOBYTE(data), LX16A_HIBYTE(data)]
        return self.writeTxRx(port, lx16a_id, address, 2, data_write)

    def write4ByteTxOnly(self, port, lx16a_id, address, data):
        data_write = [LX16A_LOBYTE(LX16A_LOWORD(data)),
                      LX16A_HIBYTE(LX16A_LOWORD(data)),
                      LX16A_LOBYTE(LX16A_HIWORD(data)),
                      LX16A_HIBYTE(LX16A_HIWORD(data))]
        return self.writeTxOnly(port, lx16a_id, address, 4, data_write)

    def write4ByteTxRx(self, port, lx16a_id, address, data):
        data_write = [LX16A_LOBYTE(LX16A_LOWORD(data)),
                      LX16A_HIBYTE(LX16A_LOWORD(data)),
                      LX16A_LOBYTE(LX16A_HIWORD(data)),
                      LX16A_HIBYTE(LX16A_HIWORD(data))]
        return self.writeTxRx(port, lx16a_id, address, 4, data_write)

    def regWriteTxOnly(self, port, lx16a_id, address, length, data):
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = lx16a_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

        result = self.txPacket(port, txpacket)
        port.is_using = False

        return result

    def regWriteTxRx(self, port, lx16a_id, address, length, data):
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = lx16a_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

        _, result, error = self.txRxPacket(port, txpacket)

        return result, error

    def syncReadTx(self, port, start_address, data_length, param, param_length):
        txpacket = [0] * (param_length + 8)
        # 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN CHKSUM

        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4  # 7: INST START_ADDR DATA_LEN CHKSUM
        txpacket[PKT_INSTRUCTION] = INST_SYNC_READ
        txpacket[PKT_PARAMETER0 + 0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length

        txpacket[PKT_PARAMETER0 + 2: PKT_PARAMETER0 + 2 + param_length] = param[0: param_length]

        result = self.txPacket(port, txpacket)
        if result == COMM_SUCCESS:
            port.setPacketTimeout((6 + data_length) * param_length)

        return result


    def syncWriteTxOnly(self, port, start_address, data_length, param, param_length):
        txpacket = [0] * (param_length + 8)
        # 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4  # 4: INST START_ADDR DATA_LEN ... CHKSUM
        txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE
        txpacket[PKT_PARAMETER0 + 0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length

        txpacket[PKT_PARAMETER0 + 2: PKT_PARAMETER0 + 2 + param_length] = param[0: param_length]

        _, result, _ = self.txRxPacket(port, txpacket)

        return result

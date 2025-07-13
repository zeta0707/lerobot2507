#!/usr/bin/env python

from .lx16aservo_def import *

class GroupSyncRead:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:  # len(self.data_dict.keys()) == 0:
            return

        self.param = []

        for lx16a_id in self.data_dict:
            self.param.append(lx16a_id)

    def addParam(self, lx16a_id):
        if lx16a_id in self.data_dict:  # lx16a_id already exist
            return False

        self.data_dict[lx16a_id] = []  # [0] * self.data_length

        self.is_param_changed = True
        return True

    def removeParam(self, lx16a_id):
        if lx16a_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[lx16a_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(self.port, self.start_address, self.data_length, self.param,
                                  len(self.data_dict.keys()) * 1)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for lx16a_id in self.data_dict:
            self.data_dict[lx16a_id], result, _ = self.ph.readRx(self.port, lx16a_id, self.data_length)
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, lx16a_id, address, data_length):
        #if self.last_result is False or lx16a_id not in self.data_dict:
        if lx16a_id not in self.data_dict:
            return False

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False

        if len(self.data_dict[lx16a_id])<data_length:
            return False
        return True

    def getData(self, lx16a_id, address, data_length):
        if not self.isAvailable(lx16a_id, address, data_length):
            return 0

        if data_length == 1:
            return self.data_dict[lx16a_id][address - self.start_address]
        elif data_length == 2:
            return LX16AMAKEWORD(self.data_dict[lx16a_id][address - self.start_address],
                                self.data_dict[lx16a_id][address - self.start_address + 1])
        elif data_length == 4:
            return LX16AMAKEDWORD(LX16AMAKEWORD(self.data_dict[lx16a_id][address - self.start_address + 0],
                                              self.data_dict[lx16a_id][address - self.start_address + 1]),
                                 LX16AMAKEWORD(self.data_dict[lx16a_id][address - self.start_address + 2],
                                              self.data_dict[lx16a_id][address - self.start_address + 3]))
        else:
            return 0
#!/usr/bin/env python

from .lx16aservo_def import *

class GroupSyncWrite:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for lx16a_id in self.data_dict:
            if not self.data_dict[lx16a_id]:
                return

            self.param.append(lx16a_id)
            self.param.extend(self.data_dict[lx16a_id])

    def addParam(self, lx16a_id, data):
        if lx16a_id in self.data_dict:  # lx16a_id already exist
            return False

        if len(data) > self.data_length:  # input data is longer than set
            return False

        self.data_dict[lx16a_id] = data

        self.is_param_changed = True
        return True

    def removeParam(self, lx16a_id):
        if lx16a_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[lx16a_id]

        self.is_param_changed = True

    def changeParam(self, lx16a_id, data):
        if lx16a_id not in self.data_dict:  # NOT exist
            return False

        if len(data) > self.data_length:  # input data is longer than set
            return False

        self.data_dict[lx16a_id] = data

        self.is_param_changed = True
        return True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncWriteTxOnly(self.port, self.start_address, self.data_length, self.param,
                                       len(self.data_dict.keys()) * (1 + self.data_length))
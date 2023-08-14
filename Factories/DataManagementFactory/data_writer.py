import threading
from datetime import datetime
from csv import DictWriter
from threading import Thread
import time
from Factories.DataManagementFactory.DataWriterConfigurations.online_writer_configuration import FIELDNAMES_TELEMETRY_NAMES_MAPPING
class DataWriter:
    def __init__(self, datafields, path):
        self.data_fields = datafields
        self.path = path
        self.data = None
        self.file = None

    def __call__(self, filename):
        raise NotImplementedError

class DataWriterThread(DataWriter, Thread):
    def __init__(self, datafields, path):
        DataWriter.__init__(self, datafields, path)
        Thread.__init__(self)

        self.writing_event = threading.Event()
        self.data_set = threading.Event()
        self.filename = None
        self.writing_ok = False
        self.start()
    def run(self):
        while True:
            if not self._control_execution():
                continue
            data_to_write = self._get_data()
            self.data_write(data_to_write)

    def _get_data(self):
        self.data_set.wait()
        data = self.data
        data_to_write = {}
        for data_field in self.data_fields:
            if data_field == 'TIME':
                data_to_write[data_field] = datetime.now()
                continue
            indices = FIELDNAMES_TELEMETRY_NAMES_MAPPING[data_field]
            if isinstance(indices, tuple):
                key, id = indices
                data_to_write[data_field] = data[key][id]
            else:
                data_to_write[data_field] = data[indices]
        self.data_set.clear()
        return data_to_write


    def _control_execution(self):
        if self.writing_event.is_set():
            if self.file is None:
                self.filename = self.path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_' + self.filename + '.csv'
                self.open_file(self.filename)
            return True
        else:
            if self.file is not None:
                self.file.close()
                self.file = None
                self.writer = None
                self.data = None
                self.filename = None
            time.sleep(0.5)
            return False

    def open_file(self, filename):
        self.file = open(filename, 'a')
        self.writer = DictWriter(self.file, fieldnames=self.data_fields)
        self.writer.writeheader()
        self.file.flush()
    def close_file(self):
        self.writing_ok = False
        self.file.close()

    def data_write(self, data):
        try:
            self.writer.writerow(data)
            self.file.flush()
            self.writing_ok = True
            return True
        except:
            self.writing_ok = False
            print("DATA_WRITER: DATA DIDN'T SAVE!!!")
            return False

class TestsDataWriter(DataWriter):
    def __init__(self, datafields, path, filename):
        DataWriter.__init__(self, datafields, path)
        self.filename = self.path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_' + filename + '.csv'
        self.open_file()
    def open_file(self, filename):
        self.file = open(filename, 'a')
        self.writer = DictWriter(self.file, fieldnames=self.data_fields)
        self.writer.writeheader()
        self.file.flush()

    def data_write(self, data):
        self.writer.writerow(data)
        self.file.flush()




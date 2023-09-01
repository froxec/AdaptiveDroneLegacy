from QuadcopterIntegration.Utilities import dronekit_commands
from dronekit import VehicleMode
import time
from datetime import datetime
import csv
class ThrottleToThrustIdentification:
    def __init__(self,
                 db_interface,
                 vehicle,
                 logs_path):
        self.db_interface = db_interface
        self.vehicle = vehicle
        self.procedure_started = False
        self.t_start = None
        self.history = {'time': [],
                        'throttle': [],
                        'x': []
                        }
        self.logs_path = logs_path
        self.throttle = None
    def run(self, x):
        if not self.is_running():
            if self.procedure_started == True:
                self.procedure_started = False
                self.vehicle.mode = VehicleMode("RTL")
                self.save_history_to_file()
                self.reset_history()
                self.throttle = None
            return False
        if self.procedure_started == False:
            self.t_start = time.time()
            self.procedure_started = True
        throttle = self.db_interface.get_throttle()
        if throttle is not None and x[2] > 2.5:
            self.throttle = throttle
            self.set_throttle(throttle)
        if time.time() - self.t_start > 0.5:
            self.db_interface.stop_identification_procedure()
        # append data to history
        self.history['time'].append(time.time() - self.t_start)
        self.history['throttle'].append(throttle)
        self.history['x'].append(x)
        return True


    def is_running(self):
        return self.db_interface.is_identification_running()

    def set_throttle(self, throttle):
        dronekit_commands.set_attitude(self.vehicle, 0.0, 0.0, 0.0, throttle)

    def save_history_to_file(self):
        fieldnames = self.history.keys()
        filename = self.logs_path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_identification_' + str(self.throttle) + '.csv'
        print("Saving data to", filename)
        file = open(filename, 'w')
        writer = csv.writer(file)
        data = []
        for key in self.history.keys():
            data.append(self.history[key])
        rows = zip(*data)
        writer.writerow(fieldnames)
        writer.writerows(rows)
        file.close()


    def reset_history(self):
        self.history = {'time': [],
                        'throttle': [],
                        'x': []}
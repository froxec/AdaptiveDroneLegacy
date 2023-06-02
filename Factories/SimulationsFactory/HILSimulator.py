from typing import Type
from Simulation.model import quadcopterModel, loadPendulum
from Simulation.attitude_control import attitudeControler
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
from Factories.CommunicationFactory.interfaces import PCInterface
from Factories.CommunicationFactory.redis_db_commands import redis_stream_add
from Simulation.model import system
import numpy as np
import redis
import plotly.graph_objects as go
from plotly.subplots import make_subplots

class HILSimulator():
    ## TODO add trajectories with terminals support
    def __init__(self,
                 quad: Type[quadcopterModel],
                 load: Type[loadPendulum],
                 attitude_controller: Type[attitudeControler],
                 esc: Type[ElectronicSpeedControler],
                 INNER_LOOP_FREQ: int,
                 OUTER_LOOP_FREQ: int,
                 interface: Type[PCInterface] = None,
                 db_parameters: dict = None):
        self.quad = quad
        self.load = load
        self.attitude_controller = attitude_controller
        self.esc = esc
        self.interface = interface
        self.inner_loop_freq = INNER_LOOP_FREQ
        self.outer_loop_freq = OUTER_LOOP_FREQ
        self.modulo_freq = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
        self.dT = 1/self.inner_loop_freq
        self.history = {'x': [],
                        'u': []}
        self.i = 1
        self.u = np.array([0, 0, 0])
        self.db_parameters = db_parameters
        if db_parameters is not None:
            self.database = redis.Redis(host=db_parameters['ip'], port=db_parameters['port'], decode_responses=db_parameters['decode_responses'])

    def __call__(self, u=None):
        if self.interface is not None and self.i == 1:
            u = self.interface('recv')
        if u is not None:
            self.u = np.array(u)
        attitude_setpoint = np.concatenate([self.u[1:], np.array([0.0])])
        throttle = self.u[0]
        ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12], throttle)
        motors = self.esc(ESC_PWMs)
        x = system(np.array(motors), self.dT, self.quad, self.load)[:12]
        if self.interface is not None and self.i == 1:
            self.interface('send', x[:6])
        self.save_history(x, self.u)
        self.calculate_iterator()
        return x[:6]
    def save_history(self, x, u):
        self.history['x'].append(list(x))
        self.history['u'].append(list(u))
        if self.db_parameters is not None:
            redis_stream_add(self.database, 'x', list(x))
            redis_stream_add(self.database, 'u', list(u))

    def calculate_iterator(self):
        self.i = (self.i % self.modulo_freq) + 1
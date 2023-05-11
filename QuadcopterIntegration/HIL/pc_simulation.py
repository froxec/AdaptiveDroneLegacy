from Factories.SimulationsFactory.HILSimulator import HILSimulator
from QuadcopterIntegration.HIL.simulation_parameters import *
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, AttitudeControllerConfiguration
from QuadcopterIntegration.HIL.simulation_parameters import *
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.CommunicationFactory.interfaces import PCInterface
from QuadcopterIntegration.HIL.communication_parameters import *
import numpy as np

def main():
    print("Initialisation started...")
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)
    attitude_controller_conf = AttitudeControllerConfiguration(INNER_LOOP_FREQ, PWM_RANGE, PWM0=PWM_RANGE[0])
    interface = PCInterface(PC_IP, PC_PORT, RASP_IP, RASP_PORT)
    hil_simulation = HILSimulator(quad_conf.quadcopter, quad_conf.load, attitude_controller_conf.attitude_controller,
                                  quad_conf.esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, interface=interface, db_parameters=DB_PARAMETERS)
    print("Initialisation finished.")
    while True:
        hil_simulation()

if __name__ == "__main__":
    main()

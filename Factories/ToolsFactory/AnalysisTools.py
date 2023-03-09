from typing import Dict
class ParametersPerturber():
    def __init__(self, nominal_parameters: Dict):
        self.nominal_parameters = nominal_parameters
        self.perturbed_parameters = nominal_parameters
        self.keys = nominal_parameters.keys()
    def __call__(self, perturbation):
        for key in perturbation.keys():
            self.perturbed_parameters[key] = self.nominal_parameters[key] + perturbation[key]
        return self.perturbed_parameters

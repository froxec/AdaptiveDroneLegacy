class ParametersManager:
    def __init__(self,
                 parameters_holder,
                 predictive_model,
                 input_converter,
                 output_converter,
                 uncertain_predictive_model=None):
        self.parameters_holder = parameters_holder
        self.predictive_model = predictive_model
        self.input_converter = input_converter
        self.output_converter = output_converter
        self.uncertain_predictive_model = uncertain_predictive_model

    def update_parameters(self, parameters_dict):
        for key in parameters_dict.keys():
            self.parameters_holder.__setattr__(key, parameters_dict[key])
        self.predictive_model.update_parameters()
        self.input_converter.update(update_u_ss=True)
        self.output_converter.update()
        if self.uncertain_predictive_model is not None:
            self.uncertain_predictive_model.update_parameters()
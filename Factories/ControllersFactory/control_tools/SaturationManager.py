class SaturationManager:
    def __init__(self,
                 ref_ramp_saturation,
                 output_saturation):

        self.ref_ramp_saturation = ref_ramp_saturation
        self.output_saturation = output_saturation
    def __call__(self):
        print(self.ref_ramp_saturation.triggered)
        for i, trigger in enumerate(self.ref_ramp_saturation.triggered):
            if trigger['lower']:
                self.output_saturation.lower_bounds[i] = self.ref_ramp_saturation.output[i]
            else:
                self.output_saturation.lower_bounds[i] = self.output_saturation.lower_bounds_nominal[i]
            if trigger['upper']:
                self.output_saturation.upper_bounds[i] = self.ref_ramp_saturation.output[i]
            else:
                self.output_saturation.upper_bounds[i] = self.output_saturation.upper_bounds_nominal[i]

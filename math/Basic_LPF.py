class Basic_LPF:
    """
    Super simple first order lowpass IIR filter
    """
    def __init__(self, start_value, new_fraction):
        self._previous = start_value
        self.new_fraction = new_fraction
        
    def update(self, new_value):
        output = self._previous * (1-self.new_fraction) + self.new_fraction * new_value
        self._previous = output
        return output
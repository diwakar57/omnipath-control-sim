import numpy as np

class CameraSensor:
    def __init__(self, noise_std=0.05):
        self.noise_std = noise_std
        self.fault_active = False

    def get_measurement(self, true_y, time_step):
        # 1. Add normal Gaussian noise (vibration/pixel noise)
        noise = np.random.normal(0, self.noise_std)
        measured_y = true_y + noise

        # 2. Simulate a "Sudden Jump" Fault (e.g., lens flare or misidentification)
        # We'll trigger a massive 2.0m error between seconds 10 and 12
        if 10.0 <= time_step <= 12.0:
            measured_y += 3.0  # The "Fault"
            self.fault_active = True
        else:
            self.fault_active = False

        return measured_y
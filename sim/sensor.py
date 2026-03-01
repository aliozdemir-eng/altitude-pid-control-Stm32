import numpy as np


class AltitudeSensor:
    """
    Simulated altitude sensor.

    Features:
    - Gaussian noise
    - First-order low-pass filtering
    """

    def __init__(self, noise_std=0.02, alpha=0.2):
        self.noise_std = noise_std
        self.alpha = alpha
        self.filtered_value = 0.0

    def measure(self, true_height):
        noisy = true_height + np.random.normal(0, self.noise_std)

        # Low-pass filter
        self.filtered_value = (
            self.alpha * noisy +
            (1 - self.alpha) * self.filtered_value
        )

        return self.filtered_value

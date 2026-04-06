class ReliabilityMonitor:
    def __init__(self, threshold=0.5):
        self.threshold = threshold
        self.is_sensor_healthy = True

    def check_health(self, sensor_val, model_prediction):
        # Calculate the "Residual" (Difference between math and reality)
        residual = abs(sensor_val - model_prediction)
        
        # If the gap is too big, the sensor is likely "Faulty"
        if residual > self.threshold:
            self.is_sensor_healthy = False
        else:
            self.is_sensor_healthy = True
            
        return self.is_sensor_healthy, residual
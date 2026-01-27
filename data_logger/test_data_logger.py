import time
import math
import os
from data_logger import DataLogger

# Ensure logs directory exists
if not os.path.exists("logs"):
    os.makedirs("logs")

logger = DataLogger("test_flight")

print("Simulating Flight...")
try:
    start_time = time.time()
    
    # Simulate a 10-second flight at 50Hz
    for i in range(500):
        t = time.time() - start_time
        
        # Generate fake sine wave data
        pitch = 10 * math.sin(t)
        yaw = 5 * math.cos(t)
        
        # Log the data (Time, State, Pitch, Yaw...)
        # Zeros are placeholders for sensors we don't have yet
        data = [t, "FLIGHT", pitch, yaw, 0, 0, 0, 0, 1, 0, 0, 0]
        
        logger.log(data)
        
        time.sleep(0.02) # Run at approx 50Hz

except KeyboardInterrupt:
    pass

logger.close()
print("Done! Check the 'logs' folder.")
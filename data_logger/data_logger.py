import time
import csv
import os
from threading import Thread
from queue import Queue

class DataLogger:
    def __init__(self, filename="flight_log.csv"):
        self.filename = self._generate_unique_filename(filename)
        self.queue = Queue()
        self.running = True
        self.thread = Thread(target=self._write_loop)
        
        # Define columns for your CSV
        self.headers = [
            "Time", "State", 
            "Pitch", "Yaw", 
            "ServoY", "ServoZ", 
            "AccelX", "AccelY", "AccelZ",
            "GyroX", "GyroY", "GyroZ"
        ]
        
        # Create file and write headers immediately
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)
            
        # Start the background worker
        self.thread.start()
        print(f"Logger: Recording to {self.filename}")

    def _generate_unique_filename(self, base_name):
        # Prevents overwriting previous flight logs
        # Returns flight_log_1.csv, flight_log_2.csv, etc.
        i = 1
        while os.path.exists(f"logs/{base_name}_{i}.csv"):
            i += 1
        return f"logs/{base_name}_{i}.csv"

    def log(self, data_list):
        """
        Call this from your main loop. It's extremely fast.
        data_list: A list of values matching self.headers
        """
        self.queue.put(data_list)

    def _write_loop(self):
        """
        This runs on a separate core/thread.
        It waits for data and writes it in batches.
        """
        buffer = []
        while self.running or not self.queue.empty():
            if not self.queue.empty():
                # Grab item from queue
                data = self.queue.get()
                buffer.append(data)
                
                # Write to disk every 50 lines (optimizes SD card speed)
                if len(buffer) >= 50:
                    self._flush_to_disk(buffer)
                    buffer = []
            else:
                # Save CPU if queue is empty
                time.sleep(0.01)
        
        # Final flush on exit
        if buffer:
            self._flush_to_disk(buffer)

    def _flush_to_disk(self, data_buffer):
        try:
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data_buffer)
        except Exception as e:
            print(f"Logging Error: {e}")

    def close(self):
        print("Logger: Saving remaining data...")
        self.running = False
        self.thread.join() # Wait for the thread to finish writing
        print("Logger: Closed.")
#!/usr/bin/env python3
import serial
import time
import struct
import threading
from collections import deque

class RS485Test:
    def __init__(self):
        self.port = None
        self.buffer = bytearray()
        self.running = True
        self.left_speed = 200
        self.right_speed = 200
        self.error_count = 0
        self.last_valid_message = None
        
    def initialize_port(self):
        """Initialize the serial port with UART-specific settings"""
        try:
            self.port = serial.Serial(
                '/dev/ttyCH341USB0',
                baudrate=115200,
                timeout=0.1,  # Increased timeout
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,  # Disabled software flow control
                rtscts=False,   # Disabled hardware flow control
                dsrdtr=False,   # Disabled hardware flow control
                write_timeout=0.2  # Increased write timeout
            )
            
            # Clear any existing data
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
            
            # Set DTR and RTS for proper UART initialization
            self.port.setDTR(True)
            self.port.setRTS(True)
            time.sleep(0.2)  # Increased stabilization time
            
            print(f"Port initialized on {self.port.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to initialize port: {str(e)}")
            return False

    def process_message(self, msg):
        """Process received message"""
        try:
            # Try to unpack the message (same format as hoverboard)
            feedback = struct.unpack('<HhhhhhhH2xH', msg)
            calculated_checksum = (feedback[0] ^ feedback[1] ^ feedback[2] ^ 
                                 feedback[3] ^ feedback[4] ^ feedback[5] ^ 
                                 feedback[6] ^ feedback[7])
            
            if calculated_checksum == feedback[8]:
                self.last_valid_message = feedback
                print("\n=== Message Details ===")
                print(f"Start Frame: 0x{feedback[0]:04X}")
                print(f"Battery: {feedback[1]}")
                print(f"Temperature: {feedback[2]}")
                print(f"Left Speed: {feedback[3]}")
                print(f"Right Speed: {feedback[4]}")
                print(f"Left Current: {feedback[5]}")
                print(f"Right Current: {feedback[6]}")
                print(f"Status: {feedback[7]}")
                print(f"Checksum: 0x{feedback[8]:04X}")
                print("=====================")
            else:
                self.error_count += 1
                print(f"Checksum mismatch: calculated=0x{calculated_checksum:04X}, received=0x{feedback[8]:04X}")
                print(f"Error count: {self.error_count}")
                print(f"Raw message: {msg.hex()}")
        except struct.error as e:
            self.error_count += 1
            print(f"Failed to unpack message: {str(e)}")
            print(f"Raw message: {msg.hex()}")

    def read_loop(self):
        """Main reading loop with UART-specific error handling"""
        while self.running:
            try:
                if self.port.in_waiting:
                    # Read all available bytes
                    data = self.port.read(self.port.in_waiting)
                    self.buffer.extend(data)
                    
                    # Process complete messages
                    while len(self.buffer) >= 20:
                        # Look for start frame (0xABCD)
                        start_idx = -1
                        for i in range(len(self.buffer) - 1):
                            if self.buffer[i] == 0xCD and self.buffer[i+1] == 0xAB:
                                start_idx = i
                                break
                        
                        if start_idx >= 0 and len(self.buffer) >= start_idx + 20:
                            msg = self.buffer[start_idx:start_idx+20]
                            self.process_message(msg)
                            self.buffer = self.buffer[start_idx+20:]
                        else:
                            # No complete message found, keep remaining bytes
                            if len(self.buffer) > 20:
                                self.buffer = self.buffer[-20:]
                            break
                            
            except serial.SerialException as e:
                print(f"Error reading from port: {str(e)}")
                time.sleep(1)
            except Exception as e:
                print(f"Unexpected error: {str(e)}")
                time.sleep(1)

    def send_speed_command(self):
        """Send speed command to the device with UART-specific handling"""
        try:
            # Clear any existing data before sending
            self.port.reset_input_buffer()
            
            start_frame = 0xABCD
            checksum = (start_frame & 0xFFFF) ^ (self.left_speed & 0xFFFF) ^ (self.right_speed & 0xFFFF)
            
            command = struct.pack('<HhhH', start_frame, self.left_speed, self.right_speed, checksum)
            
            # Debug print
            print(f"\nSending command:")
            print(f"Start frame: 0x{start_frame:04X}")
            print(f"Left speed: {self.left_speed} (0x{self.left_speed & 0xFFFF:04X})")
            print(f"Right speed: {self.right_speed} (0x{self.right_speed & 0xFFFF:04X})")
            print(f"Checksum: 0x{checksum:04X}")
            
            # Send command with retry
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.port.write(command)
                    self.port.flush()
                    time.sleep(0.05)  # Wait for data to be sent
                    break
                except serial.SerialTimeoutException:
                    if attempt == max_retries - 1:
                        raise
                    print(f"Write timeout, retrying... (attempt {attempt + 1}/{max_retries})")
                    time.sleep(0.1)
            
        except Exception as e:
            print(f"Error sending speed command: {str(e)}")

    def run(self):
        """Main run method"""
        if not self.initialize_port():
            return

        # Start reading thread
        read_thread = threading.Thread(target=self.read_loop)
        read_thread.daemon = True
        read_thread.start()

        try:
            while True:
                self.send_speed_command()
                time.sleep(0.3)  # Increased delay between commands
                
                # Print error statistics every 100 commands
                if self.error_count > 0 and self.error_count % 100 == 0:
                    print(f"\nError Statistics:")
                    print(f"Total errors: {self.error_count}")
                    print(f"Last valid message: {self.last_valid_message}")
        except KeyboardInterrupt:
            print("\nExiting...")
            print(f"Final error count: {self.error_count}")
        finally:
            self.running = False
            if self.port:
                self.port.close()

def main():
    test = RS485Test()
    test.run()

if __name__ == '__main__':
    main()
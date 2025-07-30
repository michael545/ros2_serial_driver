#!/usr/bin/env python3
"""
 -sim of serial portoutput
"""
import serial
import time
import sys

def main():
    port = "/tmp/ttyS10"  
    baud_rate = 115200
    
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"sim connected to {port} at {baud_rate} baud")
        
        message_count = 0
        while True:
            message = f"msg #{message_count}\n"
            ser.write(message.encode('utf-8'))
            print(f"Sent: {message.strip()}")
            
            message_count += 1
            time.sleep(0.2)  #5hz 
            
    except KeyboardInterrupt:
        print("\stopped via user")
        ser.close()

if __name__ == "__main__":
    main()

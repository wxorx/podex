#!/usr/bin/env python3
"""
CAN Bus Traffic Capture for Volvo S60 P2
Captures 29-bit CAN messages at 125 kbit/s and logs them to both terminal and file.
"""

import can
import time
import signal
import sys
import os
from datetime import datetime

class CANCapture:
    def __init__(self, interface='can0', bitrate=125000):
        self.interface = interface
        self.bitrate = bitrate
        self.running = True
        self.bus = None
        self.log_file = None
        self.start_time = None
        
        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C and termination signals gracefully"""
        print("\n\nReceived interrupt signal. Shutting down...", file=sys.stderr)
        self.running = False
        
        if self.log_file:
            self.log_file.close()
            print(f"Log file closed: {self.log_filename}", file=sys.stderr)
        
        if self.bus:
            self.bus.shutdown()
            print("CAN interface shut down", file=sys.stderr)
        
        sys.exit(0)
    
    def generate_log_filename(self):
        """Generate log filename in format: candump_yyyymmdd_HMMSS_ms.txt"""
        now = datetime.now()
        
        # Format date part: yyyymmdd
        date_part = now.strftime("%Y%m%d")
        time_part = f"{now.hour:02d}{now.minute:02d}{now.second:02d}"
        
        # Add milliseconds
        ms_part = f"{now.microsecond // 1000:03d}"
        
        filename = f"candump_{date_part}_{time_part}_{ms_part}.txt"
        return filename
    
    def format_timestamp(self, timestamp):
        """Format timestamp with millisecond resolution"""
        # Split into seconds and fractional part
        secs = int(timestamp)
        frac = timestamp - secs
        ms = int(frac * 1000)
        
        # Get datetime object from timestamp
        dt = datetime.fromtimestamp(secs)
        
        # Format: HH:MM:SS.mmm (24-hour format with leading zeros)
        return f"{dt.hour:02d}:{dt.minute:02d}:{dt.second:02d}.{ms:03d}"
    
    def format_message(self, msg, timestamp):
        """Format CAN message according to specifications"""
        # Timestamp with milliseconds
        timestamp_str = self.format_timestamp(timestamp)
        
        # ID as 8-digit hex (29-bit extended ID)
        if msg.is_extended_id:
            id_str = f"{msg.arbitration_id:08X}"
        else:
            # For 11-bit IDs, pad to 8 digits as well
            id_str = f"{msg.arbitration_id:08X}"
        
        # DLC (Data Length Code)
        dlc = msg.dlc
        
        # Flags (simplified: just show if extended, remote, or error)
        flags = ""
        if msg.is_extended_id:
            flags += "E"
        if msg.is_remote_frame:
            flags += "R"
        if msg.is_error_frame:
            flags += "E"  # Error frame flag
        
        # Payload as hex bytes with single space between octets
        data_str = ""
        if not msg.is_remote_frame:
            data_bytes = msg.data[:8]  # Ensure we only take first 8 bytes
            data_str = " ".join(f"{b:02X}" for b in data_bytes)
        else:
            data_str = "R"  # Remote frame has no data
        
        # Format: timestamp ID DLC flags data
        if flags:
            return f"{timestamp_str} {id_str} {dlc:02d} {flags} {data_str}"
        else:
            return f"{timestamp_str} {id_str} {dlc:02d}    {data_str}"
    
    def setup_interface(self):
        """Set up the CAN interface"""
        # Bring up the interface (safe to run repeatedly)
        os.system(f'sudo ip link set {self.interface} up type can bitrate {self.bitrate} 2>/dev/null || true')

        try:
            # Try to bring up the interface if not already up
            # Note: This requires sudo, so we'll just attempt to connect
            self.bus = can.interface.Bus(
                channel=self.interface,
                interface='socketcan',
                bitrate=self.bitrate
            )
            return True
        except can.CanError as e:
            print(f"Error setting up CAN interface: {e}", file=sys.stderr)
            print(f"Make sure the interface is up: sudo ip link set {self.interface} up type can bitrate {self.bitrate}", file=sys.stderr)
            return False
    
    def open_log_file(self):
        """Open the log file for writing"""
        try:
            self.log_filename = self.generate_log_filename()
            self.log_file = open(self.log_filename, 'w')
            print(f"Logging to: {self.log_filename}", file=sys.stderr)
            return True
        except IOError as e:
            print(f"Error opening log file: {e}", file=sys.stderr)
            return False
    
    def log_message(self, msg, timestamp):
        """Log a message to both terminal and file"""
        formatted = self.format_message(msg, timestamp)
        
        # Print to terminal
        print(formatted)
        
        # Write to file with flush to ensure it's saved even if interrupted
        if self.log_file:
            self.log_file.write(formatted + '\n')
            self.log_file.flush()  # Flush to ensure data is written
    
    def capture(self):
        """Main capture loop"""
        print(f"Starting CAN capture on {self.interface} at {self.bitrate} bps", file=sys.stderr)
        print("Press Ctrl+C to stop and exit cleanly", file=sys.stderr)
        print("\n" + "="*80, file=sys.stderr)
        print("Timestamp          ID        DLC Flags Data", file=sys.stderr)
        print("="*80, file=sys.stderr)
        
        # Print header to log file as well
        if self.log_file:
            self.log_file.write("# CAN capture from Volvo S60 P2\n")
            self.log_file.write(f"# Interface: {self.interface}\n")
            self.log_file.write(f"# Bitrate: {self.bitrate} bps\n")
            self.log_file.write(f"# Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}\n")
            self.log_file.write("# Timestamp ID DLC Flags Data\n")
            self.log_file.flush()
        
        self.start_time = time.time()
        
        try:
            while self.running:
                # Receive message with timeout
                msg = self.bus.recv(timeout=1.0)
                
                if msg is not None:
                    # Use current time or message timestamp if available
                    if hasattr(msg, 'timestamp') and msg.timestamp:
                        timestamp = msg.timestamp
                    else:
                        timestamp = time.time()
                    
                    self.log_message(msg, timestamp)
                    
                # Check if we should keep running
                if not self.running:
                    break
                    
        except can.CanError as e:
            print(f"CAN error during capture: {e}", file=sys.stderr)
        except Exception as e:
            print(f"Unexpected error: {e}", file=sys.stderr)
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources and remove empty log file if no traffic was captured"""
        self.running = False
        
        # Close the log file first
        if self.log_file:
            self.log_file.close()
            print(f"Log file closed: {self.log_filename}", file=sys.stderr)
            
            # Check if the file exists and is empty (or only contains header comments)
            try:
                with open(self.log_filename, 'r') as f:
                    lines = f.readlines()
                    
                    # Filter out comment lines (starting with #) and empty lines
                    data_lines = [line for line in lines if line.strip() and not line.startswith('#')]
                    
                    if not data_lines:
                        # No actual data captured, remove the file
                        os.remove(self.log_filename)
                        print(f"WARNING: No traffic was captured. Empty log file deleted: {self.log_filename}", file=sys.stderr)
                    else:
                        print(f"Log file saved: {self.log_filename} ({len(data_lines)} messages)", file=sys.stderr)
            except FileNotFoundError:
                # File might have been deleted elsewhere, ignore
                pass
            except Exception as e:
                print(f"Error checking log file: {e}", file=sys.stderr)
        
        # Shutdown the CAN bus
        if self.bus:
            self.bus.shutdown()
            print("CAN interface shut down", file=sys.stderr)

def main():
    """Main entry point"""
    # Parse command line arguments if needed
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Capture CAN bus traffic from Volvo S60 P2'
    )
    parser.add_argument(
        '-i', '--interface',
        default='can0',
        help='CAN interface name (default: can0)'
    )
    parser.add_argument(
        '-b', '--bitrate',
        type=int,
        default=125000,
        help='Bitrate in bps (default: 125000)'
    )
    
    args = parser.parse_args()
    
    # Create capture instance
    try:
        capture = CANCapture(interface=args.interface, bitrate=args.bitrate)
        
        # Open log file
        if not capture.open_log_file():
            sys.exit(1)
        
        # Setup CAN interface
        if not capture.setup_interface():
            sys.exit(1)
        
        # Start capturing
        capture.capture()
    except Exception as e:
        print('Ouch', e)
        capture.cleanup()

if __name__ == "__main__":
    main()

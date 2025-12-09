#!/usr/bin/env python3

import sys
import json
import time
from pymavlink import mavutil

def fetch_all_parameters(connection_string='udp:127.0.0.1:14550', timeout=30):
    """Fetch all parameters from PX4 via MAVLink and output as JSON"""

    print(f"Connecting to MAVLink at {connection_string}...", file=sys.stderr)

    try:
        mav = mavutil.mavlink_connection(connection_string)
        print("Waiting for heartbeat...", file=sys.stderr)
        mav.wait_heartbeat(timeout=10)
        print(f"Connected! System {mav.target_system} Component {mav.target_component}", file=sys.stderr)
    except Exception as e:
        print(f"Failed to connect to MAVLink: {e}", file=sys.stderr)
        sys.exit(1)

    # Request the full parameter list
    print("Requesting parameter list...", file=sys.stderr)
    mav.mav.param_request_list_send(
        mav.target_system,
        mav.target_component
    )

    # Collect all parameters
    parameters = {}
    start_time = time.time()
    last_param_time = start_time
    param_count = None
    received_count = 0

    print("Receiving parameters...", file=sys.stderr)

    while True:
        # Check timeout
        if time.time() - last_param_time > timeout:
            print(f"Timeout after {timeout}s waiting for parameters", file=sys.stderr)
            break

        # Receive parameter
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)

        if msg:
            last_param_time = time.time()

            # Handle both bytes and str for param_id (different pymavlink versions)
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode('utf-8').rstrip('\x00')
            elif isinstance(param_id, str):
                param_id = param_id.rstrip('\x00')

            param_value = msg.param_value
            param_type = msg.param_type
            param_index = msg.param_index
            param_total = msg.param_count

            # Update expected count
            if param_count is None:
                param_count = param_total
                print(f"Expecting {param_count} parameters...", file=sys.stderr)

            # Store parameter info
            if param_id not in parameters:  # Avoid duplicates
                received_count += 1

                # Determine type name
                type_names = {
                    1: "UINT8",
                    2: "INT8",
                    3: "UINT16",
                    4: "INT16",
                    5: "UINT32",
                    6: "INT32",
                    7: "UINT64",
                    8: "INT64",
                    9: "REAL32",
                    10: "REAL64"
                }
                type_name = type_names.get(param_type, f"UNKNOWN_{param_type}")

                parameters[param_id] = {
                    "value": param_value,
                    "type": type_name,
                    "type_id": param_type,
                    "index": param_index
                }

                # Progress indicator
                if received_count % 50 == 0:
                    print(f"Received {received_count}/{param_count} parameters...", file=sys.stderr)

            # Check if we got all parameters
            if param_count and received_count >= param_count:
                print(f"Received all {received_count} parameters!", file=sys.stderr)
                break

        # If we haven't received anything for a while and we have most parameters, assume we're done
        elif param_count and received_count >= param_count * 0.95:
            print(f"Received {received_count}/{param_count} parameters (95%+), stopping...", file=sys.stderr)
            break

    # Output as JSON to stdout
    output = {
        "timestamp": time.time(),
        "system_id": mav.target_system,
        "component_id": mav.target_component,
        "total_parameters": received_count,
        "parameters": parameters
    }

    print(json.dumps(output, indent=2))
    print(f"\nSuccessfully retrieved {received_count} parameters", file=sys.stderr)

    return output


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Fetch all PX4 parameters via MAVLink and output as JSON')
    parser.add_argument('--connection', '-c',
                        default='udp:127.0.0.1:14550',
                        help='MAVLink connection string (default: udp:127.0.0.1:14550)')
    parser.add_argument('--timeout', '-t',
                        type=int,
                        default=30,
                        help='Timeout in seconds (default: 30)')
    parser.add_argument('--output', '-o',
                        help='Output file (default: stdout)')

    args = parser.parse_args()

    # Fetch parameters
    result = fetch_all_parameters(args.connection, args.timeout)

    # Write to file if specified
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"Parameters written to {args.output}", file=sys.stderr)


if __name__ == '__main__':
    main()

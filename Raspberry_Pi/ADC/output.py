#!/usr/bin/python3

# this program formats nice outputs
import random
import time

print('Reading MCP3008 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} | {8:>4} | {9:>4} | {10:>4} | {11:>4} | {12:>4} | {13:>4} | {14:>4} | {15:>4} |'.format(*range(16)))
print('-' *113 )
# Main program loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0]*16
    for i in range(16):
        # The read_adc function will get the value of the specified channel (0-7).
        values[i] = random.randint(0, 1000)
    # Print the ADC values.
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} | {8:>4} | {9:>4} | {10:>4} | {11:>4} | {12:>4} | {13:>4} | {14:>4} | {15:>4} |'.format(*values))
    # Pause for half a second.
    time.sleep(0.5)

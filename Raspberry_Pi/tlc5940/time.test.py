#!/usr/bin/python3

# from time import thread_time, time_ns
import time

x = time.perf_counter()
for name in ('clock', 'monotonic', 'perf_counter', 'process_time', 'time'):
    print(name, time.get_clock_info(name), sep=': ')

print ("****************")
y = time.perf_counter()

print ("time difference = ", y-x)
# print (time.perf_counter())
# print (time.time())
# print (time.thread_time())
# print (time.time_ns())
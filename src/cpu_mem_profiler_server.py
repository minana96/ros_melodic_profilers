#!/usr/bin/env python  

import rospy
import threading
import psutil
import time

from ros_profilers_msgs.srv import StartResourceMeasurments
from ros_profilers_msgs.srv import StopResourceMeasurements


class ResourceProfilerServer:
    """
    Provides services for starting and stopping resource profiling.
    
    Currently supports:
    - CPU
    - MEMORY
    
    The psutil library is used to collect the measurements.
    """

    def __init__(self):
        self.__start_resource_measurement_service = rospy.Service('start_resource_measurements', StartResourceMeasurments, self.__start_resource_measurements_callback, buff_size=65536)
        self.__stop_resource_measurement_service = rospy.Service('stop_resource_measurements', StopResourceMeasurements, self.__stop_resource_measurements_callback, buff_size=65536)

        self.__ros_rate = rospy.Rate(rospy.get_param('frequency', 10))
        self.__thread_running = False
        self.__data = {}

        rospy.loginfo('Resource profiler ready')


    def __start_resource_measurements_callback(self, request):
        """
        Starts the profiling thread upon service invocation.
        """
        self.__data = {
            'timestamps': [], 
            'cpu': [], 
            'mem': []
        } 

        self.__thread_running = True
        self.__profiling_thread = threading.Thread(target=self.__resource_profiler)
        self.__profiling_thread.start()
        
        return {
            'started': True
        }

    def __stop_resource_measurements_callback(self, request):
        """
        Stops the profiling thread upon service invocation and saves data to the .csv file 
        whose name is passed as an argument
        """
        self.__thread_running = False
        self.__profiling_thread.join()

        return {
            'timestamps': self.__data["timestamps"],
            'cpu': self.__data["cpu"],
            'mem': self.__data["mem"]
        }
    
    def exit(self):
        self.__thread_running = False
        self.__profiling_thread.join()
        self.__start_resource_measurement_service.shutdown('exit')
        self.__stop_resource_measurement_service.shutdown('exit')

    def __resource_profiler(self):
        """Profiles resource utilisation and adds data in the dictionary"""        

        while self.__thread_running:
            # Add current time, CPU utilisation and memory utilisation in the dictionary
            self.__data['timestamps'].append(int(time.time() * 1000))
            self.__data['cpu'].append(psutil.cpu_percent(interval=0.0))
            virtual_memory = psutil.virtual_memory()
            self.__data['mem'].append(virtual_memory.total - virtual_memory.available)

            self.__ros_rate.sleep()
                

def main():
    """Runs the profiler server on a node."""
    rospy.init_node("resource_profiler_server")
    resource_profiler = ResourceProfilerServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        resource_profiler.exit()
        rospy.loginfo('Server stopped cleanly')
    except BaseException:
        rospy.logerr('Exception in server!')
        raise


if __name__ == '__main__':
    main()
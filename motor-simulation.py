#!/usr/bin/env python
import numpy as np
from datetime import datetime
import time
import matplotlib
import signal
import scipy
matplotlib.use('Qt5Cairo')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def handler(signum, frame):
    print("Ctrl-c was pressed. Closing...")
    plt.close('all')
    exit(1)

class SimpleSystem:

 def __init__(self, stime):

  #Set the parameters of the system
  self.sample_time = stime
  self.max_speed = 13.0
  self.min_input = 0.25
  self.param_K = 1.3
  self.param_T = 0.9
  self.init_conditions = 0

  # Setup Variables to be used
  self.first = True
  self.start_time = 0.0
  self.current_time = 0.0
  self.last_time = 0.0
  self.proc_output = 0.0
  self.noise = 0.0
  self.sim_time = [0]
  self.input_history = [0]
  self.output_history = [0]
  self.fig = None
  self.ax1 = None
  self.ax2 = None

  # Declare the input
  self.input_val = 0.0
  self.input_time = 0.0

  #Control variables
  self.dt = 0.0
  self.dx = 4.0
  self.dy = 5.165
  self.t1 = 0.02 #Tengo que poner valores de t1, t2, dx y dy deoendiendo de lo que salió en la gráfica
  self.t2 = 1.6
  self.error = 0.0
  self.Error_prev = 0.0
  self.Error_Int = 0.0
  self.u_val = 0.0
  self.u_time = 0.0
  self.angularVelocity = 0.0
  self.setPoint = 4.0
  self.ko = (self.dx * self.t2) / (self.dy * self.t1)

  #P
  self.kp = self.ko
  self.ki = 0
  self.kd = 0

  #PI
  #self.kp =  0.9 * self.ko
  #self.ki = (0.27 * self.ko) / self.t1
  #self.kd = 0

  #PID
  #self.kp = 1.2 * self.ko
  #self.ki = (0.60 * self.ko) / self.t1
  #self.kd = 0.60 * self.ko * self.t1

  # Declare the process output message
  self.output_val = self.init_conditions
  self.output_time = self.input_time
  self.output_status = "Motor Not Turning"
  self.MotorStatus(self.init_conditions)

 #Define the main RUN function
 def run (self, frame):

  #Variable setup
  if self.first == True:
   self.start_time = 0.0 
   self.last_time = self.start_time
   self.current_time = self.last_time + self.sample_time
   self.first = False

   #Plot
   self.line1, = self.ax1.plot(self.sim_time, self.input_history, '-', color="b")
   self.line2, = self.ax2.plot(self.sim_time, self.output_history, '-', color="r")
   self.ax1.grid()
   self.ax2.grid()
   self.ax2.set_xlabel("Time [sec]")
   self.ax1.set_ylabel("Setpoint [rpm]")
   self.ax2.set_ylabel("Output [rpm]")
   self.ax1.set_yticks(np.arange(-16, 16, 2.0))
   self.ax2.set_yticks(np.arange(-16, 16, 2.0))
   self.line = [self.line1, self.line2]

  #System
  else:
   # Define sampling time
   self.current_time = self.last_time + self.sample_time
   dt = self.sample_time
   self.last_time = self.current_time

   self.input_time = self.current_time - self.start_time

   self.input_val = 4.0

   #Dynamical System Simulation

   if dt >= self.sample_time:

    #Dead-Zone (DOI: 10.1109/LARS.2010.36)
    if(abs(self.u_val)<=self.min_input):
      self.proc_output+= (-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * 0.0) * dt
    #Saturation (DOI: 10.1109/TE.2004.825533)
    elif (((-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * self.u_val)>0.0 and self.proc_output> self.max_speed)or ((-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * self.u_val)<0.0 and self.proc_output< -self.max_speed)):
      self.proc_output+= (-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * ((1/self.param_K)*self.proc_output)) * dt
    #Dynamic System (https://www.halvorsen.blog/documents/programming/python/resources/powerpoints/Discrete%20Systems%20with%20Python.pdf)
    else:
      self.proc_output+= (-1.0/self.param_T * self.proc_output + self.param_K/self.param_T * self.u_val) * dt    
    
    self.output_val = self.proc_output
    self.output_time = self.input_time
    self.MotorStatus(self.proc_output)
    self.proc_output = self.proc_output + self.noise

    print(str(self.output_time) + " : " + str(self.output_val) + " , " + str(self.u_val) + " , " + str(self.input_val))
    print(self.output_status)

    #PID controller
    self.error = self.input_val - self.output_val
    self.Error_Int += self.error * dt
    self.u_val = (self.kp * self.error) + (self.ki * self.Error_Int) + (self.kd * (self.error - self.Error_prev) / dt)
    self.noise = 0.17 * np.cos(135.09 * self.input_time) / 2.0
    self.Error_prev = self.error

    self.angularVelocity = 10.0 #?
    self.setPoint = 4.0 #?
    #self.u_val  = self.input_val #?
    self.u_time = 0.0 #?

   else:
    print(str(self.output_time) + " : " + str(self.output_val) + " , " + str(self.u_val) + " , " + str(self.input_val))
    print(self.output_status)

   # update the data
   self.sim_time.append(self.output_time)
   self.input_history.append(self.input_val)
   self.output_history.append(self.output_val)
 
   #Plot
   self.line[0].set_data(self.sim_time, self.input_history) 
   self.line[1].set_data(self.sim_time, self.output_history)
   self.ax1.relim()
   self.ax1.autoscale_view()
   self.ax2.relim()
   self.ax2.autoscale_view()
   self.ax1.set_xticks(np.arange(min(self.sim_time), max(self.sim_time)+1, 0.5))
   self.ax2.set_xticks(np.arange(min(self.sim_time), max(self.sim_time)+1, 0.5))
   return self.line

 # Motor Status Function
 def MotorStatus(self,speed):
  if (abs(speed)<=abs(self.param_K*self.input_val*0.8) and abs(self.input_val)<=self.min_input):
    self.output_status = "Motor Not Turning"
  elif (abs(speed)>=self.max_speed):
    self.output_status = "Motor Max Speed"
  else:
    self.output_status = "Motor Turning"


if __name__=='__main__':
 signal.signal(signal.SIGINT, handler)
 #Initialise and Setup
 
 loop_rate = 100.0 #Hz?
 System = SimpleSystem(1.0/loop_rate)
 print("The Motor is Running")

 System.fig, (System.ax1, System.ax2) = plt.subplots(2,1)
 
 System.fig.suptitle('System simulation')

 animation = FuncAnimation(System.fig, System.run, interval=1000.0/loop_rate)
 plt.show()


 
import tkinter as tk
from tkinter import *
import math as m
import os
import sys
import subprocess
import ctypes

# MQ RACER SIMULATION
#
# This code simulates a two wheeled racer on the MQ Racer race track. It computes kineamatics and dynamics and
# includes simulations of motors and sensors. It compiles a user file "controller.c" that controls the simulated
# racer. The "controller.c" code should be directly useable on the real robot by compiling it in the MPLAB 
# environment. The Python code calls the gcc compiler directly and sets up the links between Python and C 
# using intermediate code in "sim_link.c".
#
# Simulation is paused at the start - hit spacebar to toggle the pause state.
# Quit at any time by pressing "q"

# This code is supplied as is and without warranty. While every care has been taking to ensure its accuracy
# and reliability, it may contain errors. Feel free to modify this code as you need for your project.
# (c) Gordon Wyeth 2026 - the author asserts moral rights to this code

# Critical Simulation Parameters - make sure these match your real racer and your C Code

dt = 0.005          # (s) time between calls to racer_control_step()
car_radius = 0.04   # (m) half the distance between the two wheels
wheel_radius = 0.015 # (m) the radius of the wheel
car_length = 0.05   # (m) used to determine distance from sensors to centre line of motors (car_length/2) and to shape graphic
car_mass = 0.2     # (kg) mass used in car dynamics, inertia is estimated using 1/2 car_mass * car_radius^2
spacing = 0.008     # (m) distance between sensors

# Display parameters

frame_rate = 50     # (Hz) how often the screen is updated
speed_up = 120      # (unitless) 120 is about real time
pixpm = 550         # screen pixels per simulated metre

steps_per_frame = round(speed_up/100 * (1/frame_rate) / dt) # sim steps between graphic frames

# start and end points of the racetrack straights in m, with (0,0) in top left corner
straights = [[1.850, 1.050, 1.050, 1.050],
             [1.050, 1.025, 1.050, 1.075],
             [0.850, 0.850, 0.350, 0.850],
             [0.350, 0.825, 0.350, 0.875],
             [0.150, 0.650, 0.150, 0.400],
             [0.125, 0.400, 0.175, 0.400],
             [0.350, 0.200, 1.500, 0.200],
             [1.500, 0.175, 1.500, 0.225],
             [1.800, 0.500, 2.100, 0.500],
             [2.100, 0.475, 2.100, 0.525]]

# 90 degree curve descriptors - centre_x, centre_y, radius, starting angle in degrees
curves = [[1.050, 0.950, 0.100, 180],
          [0.850, 0.950, 0.100,   0],
          [0.350, 0.650, 0.200, 180],
          [0.350, 0.400, 0.200,  90],
          [1.500, 0.350, 0.150,   0],
          [1.800, 0.350, 0.150, 180],
          [2.100, 0.650, 0.150,   0],
          [2.100, 0.650, 0.150, 270],
          [2.100, 0.900, 0.100,  90],
          [1.850, 0.900, 0.150, 270]]

# starting position and orientation of the racer in window coordinate frame 
start_x = 1.45 + car_length/2.0
start_y = 1.05
start_ang = 180

# Turn entry/exit gates: (x, y, forward_x, forward_y), in metres.
# Gates follow the track geometry above and measure the car's centre, not
# the front sensors. Each section includes ALL its consecutive radii.
TURN_GATES = (
    ((1.050, 1.050, -1, 0), (0.850, 0.850, -1, 0)),  # T1: R100 + R100
    ((0.350, 0.850, -1, 0), (0.150, 0.650, 0, -1)),  # T2: R200
    ((0.150, 0.400, 0, -1), (0.350, 0.200, 1, 0)),   # T3: R200
    ((1.500, 0.200, 1, 0), (1.800, 0.500, 1, 0)),    # T4: R150 + R150
    ((2.100, 0.500, 1, 0), (1.850, 1.050, -1, 0)),   # T5: R150 semicircle + R100 + R150
)
TURN_GATE_HALF_WIDTH = 0.080  # Ignore crossings far away from the track.


class TurnSpeedTracker:
    """Path length / simulation time between consecutive turn gates."""

    def __init__(self):
        self.next_turn = 0
        self.active = False
        self.lap = 1
        self.distance = 0.0
        self.elapsed = 0.0
        self.results = [None] * len(TURN_GATES)

    @staticmethod
    def crossing_fraction(previous, current, gate):
        gx, gy, nx, ny = gate
        before = (previous[0] - gx) * nx + (previous[1] - gy) * ny
        after = (current[0] - gx) * nx + (current[1] - gy) * ny
        # Only count crossings in the direction of travel around the track.
        if not (before <= 0.0 and after > 0.0):
            return None
        fraction = -before / (after - before)
        x = previous[0] + fraction * (current[0] - previous[0])
        y = previous[1] + fraction * (current[1] - previous[1])
        sideways = -(x - gx) * ny + (y - gy) * nx
        if abs(sideways) > TURN_GATE_HALF_WIDTH:
            return None
        return fraction

    def update(self, previous, current, step_time):
        if step_time <= 0.0:
            return
        step_distance = m.hypot(current[0] - previous[0],
                                current[1] - previous[1])
        begin = 0.0
        if not self.active:
            begin = self.crossing_fraction(previous, current,
                                            TURN_GATES[self.next_turn][0])
            if begin is None:
                return
            self.active = True
            self.distance = 0.0
            self.elapsed = 0.0

        end = self.crossing_fraction(previous, current,
                                     TURN_GATES[self.next_turn][1])
        if end is not None and end < begin:
            end = None
        fraction = (1.0 if end is None else end) - begin
        self.distance += step_distance * fraction
        self.elapsed += step_time * fraction
        if end is not None and self.elapsed > 0.0:
            average = self.distance / self.elapsed
            self.results[self.next_turn] = (average, self.lap)
            print(f"Lap {self.lap} T{self.next_turn + 1}: {average:.3f} m/s "
                  f"({self.distance:.3f} m / {self.elapsed:.3f} s)")
            self.active = False
            self.next_turn = (self.next_turn + 1) % len(TURN_GATES)
            if self.next_turn == 0:
                self.lap += 1

    def display_text(self):
        labels = []
        for turn, result in enumerate(self.results):
            if self.active and turn == self.next_turn and self.elapsed > 0.0:
                value = f"{self.distance / self.elapsed:.3f} m/s (L{self.lap}*)"
            elif result is not None:
                value = f"{result[0]:.3f} m/s (L{result[1]})"
            else:
                value = "-- m/s"
            labels.append(f"T{turn + 1}: {value}")
        return "   |   ".join(labels)

# The hardworking graphics window class
class Drawing:

    # Set up the Drawing object and its variables
    def __init__(self, winwidth, winheight):
        self.winwidth = winwidth
        self.winheight = winheight
        self.window = tk.Tk() #iniate the tk window
        self.window.title("MQ Racer Sim 1.0")
        self.car_sim = Racer(start_x, start_y, start_ang) # initiate the racer at the start location and orientation
        self.debug_id = None # graphical id of debug text
        self.time = 0.0 # simulation time
        self.turn_speeds = TurnSpeedTracker()
        # Measure the initial 400 mm dash using simulation time.
        self.dash_start_x = self.car_sim.x
        self.dash_previous_distance = 0.0
        self.dash_time = None
        self.dash_velocity = None
        self.track_idlist = [] # graphical id list of track elements used for sensors
        self.start_lineid = 0 # graphical id of start line used for lap detection
        self.lapcount = 0 # number of laps
        self.start_flag = False # for lap timing logic
        self.old_flag = False
        self.lap_start_time = 0.0 # lap time
        self.is_paused = True # for pause
        self.window.bind("<space>", self.toggle_pause)
        self.window.bind("q", self.quit_sim)
        # setup the tk drawing canvas
        self.cnvs = tk.Canvas(self.window, bg="white", height=winheight, width=winwidth)
        # draw the track
        self.draw_track()
        self.car_id = self.draw_racer(self.car_sim.x * pixpm, self.car_sim.y * pixpm, self.car_sim.ang)
        self.time_id = self.draw_text (50, 50, "Press Spacebar to start ...")
        self.turn_speed_id = self.cnvs.create_text(
            winwidth / 2, 18, text=self.turn_speeds.display_text(),
            font=("Arial", 11), fill="navy")
        #initialise the graphing tool
        self.graph=Graph(self.cnvs, 1.75, 0.4, 2.0, 5.0)
    
    # pause method tied to spacebar
    def toggle_pause(self, event):
        #pylint: disable=unused-argument
        self.is_paused = not self.is_paused
    
    # Draw the racing track keeping track of graphical ids for simulated sensors
    def draw_track (self):
        # Read the list of endpoints from the global straights list and draw
        for lines in straights:
            id = self.draw_line(lines[0], lines[1], lines[2], lines[3], line_width = 0.02 * pixpm)
            self.track_idlist.append (id)
        # Read the list of arc parameters from the global curves list and draw
        for arcs in curves:
            id = self.draw_quadrant(arcs[0], arcs[1], arcs[2], arcs[3], line_width = 0.02 * pixpm)
            self.track_idlist.append (id)
        # Draw the start-finish 
        self.start_lineid = self.draw_line(1.440, 1.000, 1.440, 1.100, line_color="red", line_width= 0.02 * pixpm)

    # Draw a line
    def draw_line (self, x1, y1, x2, y2, line_color = "black", line_width = 1):
        return self.cnvs.create_line(x1 * pixpm, y1 * pixpm, x2 * pixpm, y2 * pixpm, fill=line_color, width=line_width)
  
    # Draw an arc with: centre cx, cy; radius r ; starting angle q
    def draw_quadrant(self, cx, cy, r, q, line_color = "black", line_width = 1):
        return self.cnvs.create_arc((cx-r) * pixpm,(cy-r) * pixpm,(cx+r) * pixpm,(cy+r) * pixpm,start = q, extent = 90, style=tk.ARC, outline=line_color, width=line_width)

    # Write a string at x, y
    def draw_text (self, x, y, text_string):
        return self.cnvs.create_text(x, y, text=text_string)

    # Draw a rectangle centred at x, y oriented at angle ang. Dimensions taken from globals car_radius (half width) and car_length (total length)
    def draw_racer (self, x, y, ang):
        # Convert angle to radians for math functions
        angle_rad = m.radians(ang)
        
        # Calculate half-dimensions to find corners relative to center
        hh, hw = car_radius * pixpm, car_length * pixpm / 2
        
        # Define the 4 corners relative to (0,0)
        corners = [
            (-hw, -hh), # Top-left
            (hw, -hh),  # Top-right
            (hw, hh),   # Bottom-right
            (-hw, hh)   # Bottom-left
        ]
        
        # calculate the rotated position of the corners in the global reference frame
        rotated_points = []
        for cx, cy in corners:
            rx = x + (cx * m.cos(angle_rad) - cy * m.sin(angle_rad))
            ry = y + (cx * m.sin(angle_rad) + cy * m.cos(angle_rad))
            rotated_points.extend([rx, ry])
            
        # Draw as a polygon to allow rotation and return a handle to the polygon
        return self.cnvs.create_polygon(rotated_points, fill="blue")

    # Simulate reading eight digital sensors on the front of the car with spacing from global spacing
    def read_sensors(self):
        
        # number of sensors
        n = 8
        # orientation in radians
        angle_rad = m.radians(self.car_sim.ang)
        # extract position of racer from sim and convert to pixels
        rx = round(self.car_sim.x * pixpm)
        ry = round(self.car_sim.y * pixpm)

        # calculate the relative position of each sensor to centre
        # x axis is forward direction of racer
        x = (car_length / 2) * pixpm  # !!!! Check this is the real distance from the axle line to the sensors !!!!
        sensors = []
        for i in range(-n+1, n, 2):
            y = (i * spacing/2) * pixpm
            sensors.append((x,y))

        # calculate the rotated position of each sensor in the global reference frame
        points = []
        for sx, sy in sensors:
            ix = round(rx + (sx * m.cos(angle_rad) - sy * m.sin(angle_rad)))
            iy = round(ry + (sx * m.sin(angle_rad) + sy * m.cos(angle_rad)))
            points.append((ix, iy))

        # find each sensor reading by querying track image
        sensval = []
        for sx, sy in points:
            # create a tiny bounding box around each sensor and find the graphical items in bounding box
            bx1 = sx - 1
            by1 = sy - 1
            bx2 = sx + 1
            by2 = sy + 1
            # find the graphical items in bounding box
            items = self.cnvs.find_overlapping(bx1, by1, bx2, by2)
            # if any of the items are part of the track the sensor is on
            found = 0
            for id in items:
                if id in self.track_idlist:
                    found = 1
            sensval.append(found)
        return sensval

    #check whether the racer has crossed the start line
    def check_startline(self):
        
        # create a row of test points from the middle to the front of the car and check whether any contain the startline
        # this prevents missing the start line if the car is flying across it
        y = 0
        angle_rad = m.radians(self.car_sim.ang)

        #for each of the test points
        for x in range(0, round(car_length * pixpm/2), 1):
            nx = round(self.car_sim.x * pixpm) + (x * m.cos(angle_rad) - y * m.sin(angle_rad))
            ny = round(self.car_sim.y * pixpm) + (x * m.sin(angle_rad) + y * m.cos(angle_rad))
            #check if the point is overs the start line
            items = self.cnvs.find_overlapping(nx - 1, ny -1 , nx + 1, ny + 1)
            for id in items:
                if id == self.start_lineid:
                    return True
        return False
    
    # run the controller and perform the graphical update
    def animate(self):

        # if paused do nothing except set up next cycle
        if self.is_paused:
            self.window.after(round(1000/frame_rate), self.animate)
            return

        # Initialise start line flag as undetected
        self.start_flag = False
        
        # run for as many steps as needed to match controller step size with frame rate
        for _ in range(steps_per_frame):
            
            # Load up the simulated sensor reading so that the C code can read it
            self.car_sim.sensor_reading = self.read_sensors()
            
            # Run the C function from controller.c
            sim_lib.racer_control_step()
            
            # Update the sim based on new wheel velocities
            previous_position = (self.car_sim.x, self.car_sim.y)
            self.car_sim.take_step()
            self.turn_speeds.update(previous_position,
                                    (self.car_sim.x, self.car_sim.y), dt)
        
            # Update the graph with latest data point
            self.time += dt
            # The initial straight runs left: decreasing x is forward progress.
            if self.dash_time is None:
                distance = self.dash_start_x - self.car_sim.x
                if distance >= 0.400:
                    # Interpolate the finish crossing within this time step.
                    step_distance = distance - self.dash_previous_distance
                    fraction = (0.400 - self.dash_previous_distance) / step_distance
                    self.dash_time = self.time - dt + fraction * dt
                    self.dash_velocity = 0.400 / self.dash_time
                    print(
                        f"400 mm dash: {self.dash_time:.3f} s | "
                        f"Average velocity: {self.dash_velocity:.3f} m/s"
                    )
                self.dash_previous_distance = distance
            self.graph.add_new_data(self.time, self.car_sim.total_a)

            # check whether the startline is crossed
            if (self.check_startline()):
                self.start_flag = True

        # update the graphical representation of the racer
        if self.car_id:
            self.cnvs.delete(self.car_id)
        self.car_id = self.draw_racer(self.car_sim.x * pixpm, self.car_sim.y * pixpm, self.car_sim.ang)
        
        # update the simulation time display
        if self.time_id:
            self.cnvs.delete(self.time_id)
        self.time_id = self.draw_text (50, 50, str(round(self.time,2)))

        # if the car has crossed the start line update the lap counter
        if self.start_flag:
            if not self.old_flag:
                # crossed line
                self.old_flag = True
                if not self.lapcount == 0:
                    self.draw_text (50, 50 + 20*self.lapcount, f"L{self.lapcount} : {round(self.time - self.lap_start_time,2)}")
                self.lap_start_time = self.time
                self.lapcount += 1
        else:
            self.old_flag = False
        
        # Use this string to print debugging text at the top of the screen
        # Note that you can also use print() to send data to the console
        if self.dash_time is None:
            debug_text = f"400 mm dash: timing... {self.time:.2f} s"
        else:
            debug_text = (
                f"400 mm dash: {self.dash_time:.3f} s\n"
                f"Average velocity: {self.dash_velocity:.3f} m/s"
            )
        
        # update the debugging text
        if self.debug_id:
            self.cnvs.delete(self.debug_id)
        self.debug_id = self.draw_text(300,50,debug_text)
        self.cnvs.itemconfigure(self.turn_speed_id,
                                text=self.turn_speeds.display_text())

        # draw the graph
        self.graph.draw_graph()

        # tell TK to update the screen
        self.update()

        # setup next animate call in time for next frame
        self.window.after(round(1000/frame_rate), self.animate)
        

    # wipe the screen
    def clear (self):
        self.cnvs.delete('all')

    # kill the sim
    def quit_sim(self, event):
        self.window.destroy()

    # update the screen
    def update(self):
        self.cnvs.pack()
        self.window.update()
    
    # run the event loop    
    def loop(self):
        self.cnvs.pack()
        self.window.mainloop()

# The Racer class simulates the kinematics and dynamics of the racer.
class Racer:

    def __init__(self, init_x, init_y, init_ang):
        self.x = init_x
        self.y = init_y
        self.ang = init_ang
        self.lw = 0.0
        self.rw = 0.0
        self.speed = 0.0
        self.angdot = 0.0
        self.total_a = 0.0
        self.sensor_reading = [0, 0, 0, 0, 0, 0, 0, 0]
        self.lpwm = 0
        self.rpwm = 0
        self.lm = Motor()
        self.rm = Motor()

    # Load the pwm values into the data structure. 
    def set_wheel_vels(self, lw, rw):
        self.lpwm = lw
        self.rpwm = rw

    # Returns the current sensor reading
    def get_sensors(self):
        return self.sensor_reading

    # Update the kinematics and dynamics of the racer
    def take_step(self):
        
        # use the motor class to calculate torques based on the applied PWMs and current motor speeds
        l_tau = self.lm.motor(self.lpwm * 7.2 / 255.0, self.lw / wheel_radius, dt)
        r_tau = self.rm.motor(self.rpwm * 7.2 / 255.0, self.rw / wheel_radius, dt)

        # Find total force and torque applied to the racer
        v_f = l_tau / wheel_radius + r_tau / wheel_radius
        v_tau = (l_tau / wheel_radius - r_tau / wheel_radius) * car_radius

        # Compute linear acceleration, displacement and speed
        a = v_f / car_mass
        d = (self.speed + a/2 * dt) * dt
        self.speed += a * dt

        # Compute rotational acceleration, angle and speed
        alpha = m.degrees(v_tau / (0.5 * car_mass * car_radius ** 2))
        self.ang += self.angdot * dt + 0.5 * alpha * dt * dt
        self.angdot += alpha * dt

        # Update wheel speeds based on vehicle dynamics (assumes perfect grip)
        self.lw = (self.speed + m.radians(self.angdot) * car_radius)
        self.rw = (self.speed - m.radians(self.angdot) * car_radius)
 
        # Compute total acceleration for graphing
        # centripetal acceleration
        ctptl = m.radians(self.angdot) * self.speed
        # rotational acceleration assuming racer is a cylinder
        rot = m.radians(alpha) * car_radius/2
        self.total_a = m.sqrt((abs(a) + abs(rot))**2 + ctptl**2)

        #update position of racer
        ang = m.radians(self.ang)
        self.x +=  d * m.cos(ang) 
        self.y +=  d * m.sin(ang)

# This class simulates the first order dynamics of the Pololu 5185 Micro Metal Gearmotor
# It does not include the effects of the winding inductance as the time constants are too
# small to be meaningful in this simulation.

class Motor:

    def __init__(self):
        self.counts_per_rev = 12.0 * 30.0 # Number of encoder clicks per output shaft revolution
        self.Kb = (6/1100) * (60 / (2.0 * m.pi))  # Vs / rad BackEMF constant
        self.Km = 0.030 # Nm / A Torque constant
        self.R = 4 # Ohms resistance
        self.Tc = 0.0039 # Nm Stiction torque
        self.Tv = 0.0020 # Nm Rolling torque loss
        self.epsilon = 0.1 / (60 / (2.0 * m.pi)) # rad/s transition speed from stiction

        self.theta = self.omega = self.i = 0.0 # initial state at rest
        self.clicks = self.oldclicks = 0 # intialise encoder

    # Returns the torque supplied by the motor based on input voltage vin (V) and motor speed (rad/s)
    # for time step dt (s)
    def motor (self, vin, omega, dt):
        alpha = omega - self.omega # compute acceleration over last time step
        self.theta += self.omega * dt + 0.5 * alpha * dt * dt # update motor position
        self.omega = omega #update the motor speed
        self.clicks = int((self.theta / (2.0 * m.pi)) * self.counts_per_rev) #compute encoder counts
        self.i = (vin - self.Kb * self.omega) / self.R #compute winding current
        self.tau = self.Km * self.i #compute generated torque
        #compute torque loss
        if m.fabs(self.omega) > self.epsilon:
            # rolling torque case
            delta_tau = m.copysign(self.Tv, self.omega)
            self.tau -= delta_tau
        else:
            #stiction torque case
            self.omega = 0
            if m.fabs(self.tau) < self.Tc:
                self.tau = 0
            else:
                self.tau -= m.copysign(min(self.Tc, m.fabs(self.tau)), self.tau)
        return self.tau
    
    # Returns encoder counts since last call
    def read_enc(self):
        clicks = self.clicks - self.oldclicks
        self.oldclicks = self.clicks
        return clicks

# Class for a scrolling graph to display useful measures
class Graph:
    def __init__(self, canvas, gx, gy, window_duration_secs=2.0, max_val=5.0):
        
        # Geometry configurations
        self.width = 0.6 * pixpm
        self.height = 0.35 * pixpm
        self.px = gx * pixpm
        self.py = gy * pixpm
        self.max_val = max_val
        
        # Simulation parameters
        self.window_duration = window_duration_secs  # Time window width (3 seconds)
        
        # Queues to hold incoming telemetry data points: (timestamp, value)
        self.data_points = []
        
        # Create Main Drawing Canvas
        self.canvas = canvas
        
        # Unpack local bounds variables for layout code safety
        w, h, px, py = self.width, self.height, self.px, self.py
        
        # Draw static background graph UI lines
        self.canvas.create_line(px, py, px + w, py, width=2) # X Axis Base
        self.canvas.create_line(px, py, px, py - h, width=2)         # Y Axis Base
        
        # Draw gridlines
        for x in range(round(px + w/4), round(px + w + 1), round(w/4)):
            self.canvas.create_line(x, py, x, py - h, dash=(4, 4), fill="gray")
        for y in range(round(py - h), round(py), round(h/5)):
            self.canvas.create_line(px, y, px + w, y, dash=(4, 4), fill="gray")


        # Labels
        self.canvas.create_text(px - 15, py - h, text=f"{self.max_val}", anchor="e")
        self.canvas.create_text(px - 15, py, text="0.0", anchor="e")
        self.canvas.create_text(px + 15, py + 15, text="0.0", anchor="e")
        self.canvas.create_text(px + w, py + 15, text=f"{window_duration_secs}", anchor="e")
        self.canvas.create_text(px + w / 2, py + 15, text="Total acceleration")
        
        # Instantiate ONE primary variable graph trace vector line.
        # It initializes empty and out of sight.
        self.trace_id = self.canvas.create_line(0, 0, 0, 0, fill="blue", width=2)

    def add_new_data(self, t, val):
        self.data_points.append((t, val))

    def draw_graph(self):
        
        # Slide the sliding window filter domain criteria metrics
        latest_time = self.data_points[len(self.data_points)-1][0]
        earliest_allowed_time = latest_time - self.window_duration
        
        # Cull expired out-of-bounds history array entries exceeding extent edge
        while self.data_points and self.data_points[0][0] < earliest_allowed_time:
            self.data_points.pop(0)
            
        px, py = self.px, self.py
        
        # Check if timeline has filled to the maximum horizontal width scale limits
        if latest_time < self.window_duration:
            # Data is filling up from left-to-right (0 to 3 seconds fixed axis scale)
            min_x_time = 0.0
            max_x_time = self.window_duration
        else:
            # The time window slides dynamically along the x axis
            min_x_time = earliest_allowed_time
            max_x_time = latest_time
            
        time_range = max_x_time - min_x_time
        
        # Generate flattened linear vector vertex array chain map
        line_vertices = []
        for pt_time, pt_val in self.data_points:
            # Map time data cleanly to X-pixel matrix space
            x = px + ((pt_time - min_x_time) / time_range) * self.width
            
            # Map mathematical float value to Y-pixel space (Invert Y because 0,0 is Top-Left)
            normalized_y = pt_val  / self.max_val 
            y = py - (normalized_y * self.height)
            
            line_vertices.extend([x, y]) # Append coordinates sequentially
            
        # Overwrite the trace line coordinates natively for rapid high-speed performance
        if len(line_vertices) >= 4:  # Requires at least 2 coordinate pairs (x1, y1, x2, y2)
            self.canvas.coords(self.trace_id, *line_vertices)

# Helper interfaces to make simulator look like real racer to C code
def py_read_sensors():
    sensor_list = dwg.car_sim.get_sensors()
    byte_val = 0
    for i, sensor_state in enumerate(sensor_list):
        if sensor_state:
            byte_val |= (1 << i)
    return byte_val
        
def py_set_speeds(l_speed, r_speed):
    dwg.car_sim.set_wheel_vels(l_speed, r_speed)

def py_read_left_enc():
    clicks = dwg.car_sim.lm.read_enc()
    return clicks

def py_read_right_enc():
    clicks = dwg.car_sim.rm.read_enc()
    return clicks

# Compile the C code in "controller.c" (user code) and "sim_link.c" (supplied code) and link to 
# Python simulation
def build_C_code():

    # Build command: compile controller.c into a shared library (.dll)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print (dir_path)
    c_source = os.path.join(dir_path, "controller.c")
    c_wrapper = os.path.join(dir_path, "sim_link.c")
    dll_output = os.path.join(dir_path, "controller.dll")
    print(c_source)
    compile_cmd = [
       "gcc", "-shared", "-o", dll_output, c_source, c_wrapper, f"-I{dir_path}"
    ]

    print("Compiling 'controller.c' ...")
    try:
        subprocess.check_call(compile_cmd)
        print("Compilation successful!")
    except subprocess.CalledProcessError as e:
        print("Compilation failed! Check your C code syntax.")
        sys.exit(1)
    return dll_output

# END of Class Definitions
# Start of code execution

# Compile the C code 
dll_output = build_C_code()

# Load the compiled binary
sim_lib = ctypes.CDLL(dll_output)

# Link the C code to Python
# Define the ctypes callback signatures matching the C prototypes
READ_SENSORS_CALLBACK = ctypes.CFUNCTYPE(ctypes.c_ubyte)
SET_SPEEDS_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_int)
LEFT_ENC_CALLBACK = ctypes.CFUNCTYPE(ctypes.c_byte)
RIGHT_ENC_CALLBACK = ctypes.CFUNCTYPE(ctypes.c_byte)

# Map the entry point function execution rule
sim_lib.racer_control_step.argtypes = []
sim_lib.racer_control_step.restype = None
sim_lib.racer_init.argtypes = []
sim_lib.racer_init.restype = None

c_read_sensors_hook = READ_SENSORS_CALLBACK(py_read_sensors)
c_set_speeds_hook = SET_SPEEDS_CALLBACK(py_set_speeds)
c_left_enc_hook = LEFT_ENC_CALLBACK(py_read_left_enc)
c_right_enc_hook = RIGHT_ENC_CALLBACK(py_read_right_enc)

# Register the Python functions inside the DLL memory layout
sim_lib.init_simulator_links(c_read_sensors_hook, c_set_speeds_hook, 
                                c_left_enc_hook, c_right_enc_hook)

#call the racer's initialisation code
sim_lib.racer_init()

# Set up the Drawing object with dimensions scaled based on global pixpm
dwg = Drawing(round(2.4 * pixpm), round(1.2 * pixpm))

# Start the animation loop
dwg.animate()
dwg.loop()

#                       FRONT
#         ------ ------       ------ ------
#         |FLS2| |FLS1|       |FRS1| |FRS2|
#         ------ ------       ------ ------                         ^ X       
#                |FLS3|       |FRS3|                                |
#                   |            |                                  |
#    LEFT           |            |            RIGHT          Y <----# 
#                   |            |
#                ------       ------
#                |BLS3|       |BRS3|   
#         ------ ------       ------ ------
#         |BLS2| |BLS1|       |BRS1| |BRS2|   
#         ------ ------       ------ ------
#                       BACK

import numpy as np
import time
from math import comb
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50

class Robot:
    
    def __init__(self):
        # [FRS1, FRS2, FRS3,     BRS1, BRS2, BRS3,      FLS1, FLS2, FLS3,       BLS1, BLS2, BLS3]  
        self.servo_angles = {
            'FRS1': 0, 
            'FRS2': 0,
            'FRS3': 90, 
            
            'BRS1': 0,
            'BRS2': 0,
            'BRS3': 90,
            
            'FLS1': 0,
            'FLS2': 0,
            'FLS3': 90,

            'BLS1': 0,      
            'BLS2': 0,     
            'BLS3': 90,      
        }
                                   # x, y, z 
        self.starting_foot_cords_front = [0, 0, 15]
        self.starting_foot_cords_back = [0, -2, 15]
        
        self.a1 = 10
        self.a2 = 10
        
        V_desired = 18
        stride_length = 20
        
        self.walk_control_points = [
            [-3, 16],
            [-np.round(3 + V_desired**2 / (11 * stride_length), 0), 16],
            [-5, 14],
            [-5, 14],
            [-5, 14],
            [0, 14],
            [0, 14],
            [0, 13],
            [5, 13],
            [5, 13],
            [np.round(3 + V_desired**2 / (8 * stride_length), 0), 16],
            [3, 16]
        ]
        
        self.cords_respect_CoM = {
            'FRx': 10.75  + self.starting_foot_cords_front[0],
            'FRy': -9.85  + self.starting_foot_cords_front[1],
            'FRz': -self.starting_foot_cords_front[2],

            'BRx': -10.75  + self.starting_foot_cords_back[0],
            'BRy': -9.85  + self.starting_foot_cords_back[1],
            'BRz': -self.starting_foot_cords_back[2],
            
            'FLx': 10.75  + self.starting_foot_cords_front[0],
            'FLy': 9.85  + self.starting_foot_cords_front[1],
            'FLz': -self.starting_foot_cords_front[2],
                        
            'BLx': -10.75  + self.starting_foot_cords_back[0],
            'BLy': 9.85  + self.starting_foot_cords_back[1],
            'BLz': -self.starting_foot_cords_back[2]
        }
        
    def starting_position(self):
        q1_F, q2_F, q3_F = self.IK(x=self.starting_foot_cords_front[0], y=self.starting_foot_cords_front[1], z=self.starting_foot_cords_front[2])
        q1_B, q2_B, q3_B = self.IK(x=self.starting_foot_cords_back[0], y=self.starting_foot_cords_back[1], z=self.starting_foot_cords_back[2])
                
        # Right side:
        self.rad_to_degrees('FRS1', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('FRS2', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('FRS3', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('BRS1', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)
        self.rad_to_degrees('BRS2', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)
        self.rad_to_degrees('BRS3', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)
        
        # Left side:
        self.rad_to_degrees('FLS1', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('FLS2', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('FLS3', radQ1=q1_F, radQ2=q2_F, radQ3=q3_F)
        self.rad_to_degrees('BLS1', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)
        self.rad_to_degrees('BLS2', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)
        self.rad_to_degrees('BLS3', radQ1=q1_B, radQ2=q2_B, radQ3=q3_B)

        self.write_servo()
        
    def pitch(self, angle=(10*np.pi/180), robot_length=21.5):
        foot_cords = [0, 0, 15]
        y = foot_cords[1] - robot_length / 2 * (1 - np.cos(angle))
        h = foot_cords[2]
        
        # Back 
        zB = h - np.sin(angle) * robot_length / 2
        q1B, q2B, q3 = self.IK(x=foot_cords[0], y=y, z=zB)
        self.rad_to_degrees('BLS1', radQ1=q1B, radQ2=q2B, radQ3=q3)
        self.rad_to_degrees('BLS2', radQ1=q1B, radQ2=q2B, radQ3=q3)
        self.rad_to_degrees('BRS1', radQ1=q1B, radQ2=q2B, radQ3=q3)
        self.rad_to_degrees('BRS2', radQ1=q1B, radQ2=q2B, radQ3=q3)
        
        # Front
        zF = h + np.sin(angle) * robot_length / 2
        q1F, q2F, q3 = self.IK(x=foot_cords[0], y=y, z=zF)
        self.rad_to_degrees('FLS1', radQ1=q1F, radQ2=q2F, radQ3=q3)
        self.rad_to_degrees('FLS2', radQ1=q1F, radQ2=q2F, radQ3=q3)
        self.rad_to_degrees('FRS1', radQ1=q1F, radQ2=q2F, radQ3=q3)
        self.rad_to_degrees('FRS2', radQ1=q1F, radQ2=q2F, radQ3=q3)
    
        self.write_servo()
            
    def bezier_curve(self, t, control_points):
            n = len(control_points) - 1
            point = np.zeros_like(control_points[0], dtype=float)  # Ensure float type
            for i in range(n + 1):
                bernstein_poly = comb(n, i) * (1 - t)**(n - i) * t**i
                point += bernstein_poly * np.array(control_points[i], dtype=float)  # Ensure float type
            return point
       
    def shift_list_left(self, list, n):
        return list[-n:] + list[:-n]
      
    def prepare_to_walk(self, num_steps=5):
        self.starting_position()
        
        # Prepare Front-Right leg(FR)
        FR_start = [self.starting_foot_cords_front[1], self.starting_foot_cords_front[2]] 
        FR_between = [(self.walk_control_points[-1][0] - self.starting_foot_cords_front[1])/2, self.walk_control_points[0][1] - 2]
        FR_target = [self.walk_control_points[-1][0], self.walk_control_points[-1][1]]
        for i in range(num_steps+1):
            t_FR = i/num_steps
            
            FR_point = self.bezier_curve(t_FR, [FR_start, FR_between, FR_target])
            
            FR_x = 0
            FR_y, FR_z = FR_point
            
            FR_q1, FR_q2, FR_q3 = self.IK(FR_x, FR_y, FR_z)
            
            self.rad_to_degrees('FRS1', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            self.rad_to_degrees('FRS2', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            self.rad_to_degrees('FRS3', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            
            self.write_servo()
            
        time.sleep(0.01)
                   
               
        # Prepare Front-Left leg(FL)
        FL_start = [self.starting_foot_cords_front[1], self.starting_foot_cords_front[2]] 
        FL_between = [(self.walk_control_points[0][0] - np.abs(self.starting_foot_cords_front[1]))/2, self.walk_control_points[0][1] - 2]
        FL_target = [self.walk_control_points[0][0], self.walk_control_points[0][1]]
        for i in range(num_steps+1):
            t_FL = i/num_steps
            
            FL_point = self.bezier_curve(t_FL, [FL_start, FL_between, FL_target])
            
            FL_x = 0
            FL_y, FL_z = FL_point
            
            FL_q1, FL_q2, FL_q3 = self.IK(FL_x, FL_y, FL_z)
            
            self.rad_to_degrees('FLS1', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            self.rad_to_degrees('FLS2', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            self.rad_to_degrees('FLS3', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            
            self.write_servo()
            
        time.sleep(0.01)
            
        # Prepare Back-Left leg(BL)
        # Prepare Back-Right leg(BR)
        BL_target_x, BL_target_y, BL_target_z = self.starting_foot_cords_back
        BL_q1, BL_q2, BL_q3 = self.IK(BL_target_x, BL_target_y, self.walk_control_points[0][1])
        
        self.rad_to_degrees('BLS1', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BLS2', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BLS3', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        
        self.rad_to_degrees('BGS1', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BGS2', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BGS3', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
            
        self.write_servo()
        time.sleep(0.01)  

    def walk(self, how_many_cycles, num_steps=5):
        phase_leg_list = ['FR', 'BR', 'FL', 'BL']

        distance_Phase1_2y = np.abs(self.walk_control_points[-1][0])
        distance_Phase2_3y = np.abs(self.walk_control_points[0][0])
        
        step_size_Phase1_2y = distance_Phase1_2y/num_steps
        step_size_Phase2_3y = distance_Phase2_3y/num_steps
        
        self.prepare_to_walk()
        for k in range(4*how_many_cycles): # It can by only multiplication of 4 so: 4, 8, 12
            for i in range(num_steps + 1):
            # Phase 1 -> 2
                Phase1_x = 0
                Phase1_y = self.walk_control_points[-1][0] - step_size_Phase1_2y*i
                Phase1_z = self.walk_control_points[-1][1]
                
                Phase1_q1, Phase1_q2, Phase1_q3 =  self.IK(Phase1_x, Phase1_y, Phase1_z)

                self.rad_to_degrees(f'{phase_leg_list[0]}S1', radQ1=Phase1_q1, radQ2=Phase1_q2, radQ3=Phase1_q3)
                self.rad_to_degrees(f'{phase_leg_list[0]}S2', radQ1=Phase1_q1, radQ2=Phase1_q2, radQ3=Phase1_q3)
                self.rad_to_degrees(f'{phase_leg_list[0]}S3', radQ1=Phase1_q1, radQ2=Phase1_q2, radQ3=Phase1_q3)
                
                
            # Phase 2 -> 3
                Phase2_x = 0
                Phase2_y = -step_size_Phase2_3y*i
                Phase2_z = self.walk_control_points[0][1]
                
                Phase2_q1, Phase2_q2, Phase2_q3 =  self.IK(Phase2_x, Phase2_y, Phase2_z)

                self.rad_to_degrees(f'{phase_leg_list[1]}S1', radQ1=Phase2_q1, radQ2=Phase2_q2, radQ3=Phase2_q3)
                self.rad_to_degrees(f'{phase_leg_list[1]}S2', radQ1=Phase2_q1, radQ2=Phase2_q2, radQ3=Phase2_q3)
                self.rad_to_degrees(f'{phase_leg_list[1]}S3', radQ1=Phase2_q1, radQ2=Phase2_q2, radQ3=Phase2_q3)               
                
                
            # Phase 3 -> 4
                Phase3_t = (i*3)/(4*num_steps)  # Phase3_t = (i)/(2*num_steps)
                
                Phase3_point = self.bezier_curve(Phase3_t, self.walk_control_points)
                Phase3_x = 0
                Phase3_y, Phase3_z = Phase3_point
                
                Phase3_q1, Phase3_q2, Phase3_q3 =  self.IK(Phase3_x, Phase3_y, Phase3_z)
                
                self.rad_to_degrees(f'{phase_leg_list[2]}S1', radQ1=Phase3_q1, radQ2=Phase3_q2, radQ3=Phase3_q3)
                self.rad_to_degrees(f'{phase_leg_list[2]}S2', radQ1=Phase3_q1, radQ2=Phase3_q2, radQ3=Phase3_q3)
                self.rad_to_degrees(f'{phase_leg_list[2]}S3', radQ1=Phase3_q1, radQ2=Phase3_q2, radQ3=Phase3_q3)        
                
            # Phase 4 -> 1
                Phase4_t = i/(4*num_steps) + 0.75   # Phase4_t = i/(2*num_steps) + 0.5
                
                Phase4_point = self.bezier_curve(Phase4_t, self.walk_control_points)
                Phase4_x = 0
                Phase4_y, Phase4_z = Phase4_point
                
                Phase4_q1, Phase4_q2, Phase4_q3 =  self.IK(Phase4_x, Phase4_y, Phase4_z)
        
                self.rad_to_degrees(f'{phase_leg_list[3]}S1', radQ1=Phase4_q1, radQ2=Phase4_q2, radQ3=Phase4_q3)
                self.rad_to_degrees(f'{phase_leg_list[3]}S2', radQ1=Phase4_q1, radQ2=Phase4_q2, radQ3=Phase4_q3)
                self.rad_to_degrees(f'{phase_leg_list[3]}S3', radQ1=Phase4_q1, radQ2=Phase4_q2, radQ3=Phase4_q3)  
                
                print(f'k: {k}; Phase: {i}; Phase3_t: {Phase3_t}; Phase4_t: {Phase4_t}')
                time.sleep(0.01)
                self.write_servo()              

            print(phase_leg_list)
            phase_leg_list = self.shift_list_left(phase_leg_list, 1)
            
        self.prepare_to_stop()
                 
    def prepare_to_stop(self, num_steps=5):
        # Back-Left go back to middle position from PHASE 4
        BL_target_x, BL_target_y, BL_target_z = self.starting_foot_cords_back
        BL_q1, BL_q2, BL_q3 = self.IK(BL_target_x, BL_target_y, BL_target_z)
        
        self.rad_to_degrees('BLS1', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BLS2', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BLS3', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        
        self.rad_to_degrees('BRS1', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BRS2', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
        self.rad_to_degrees('BRS3', radQ1=BL_q1, radQ2=BL_q2, radQ3=BL_q3)
            
        self.write_servo()
        time.sleep(0.01)        
        
        # Front-Right go back to middle position from PHASE 3 
        FR_target = [self.starting_foot_cords_front[1], self.starting_foot_cords_front[2]] 
        FR_between = [(self.walk_control_points[-1][0] - np.abs(self.starting_foot_cords_front[1]))/2, self.walk_control_points[0][1] - 2]
        FR_start = [self.walk_control_points[-1][0], self.walk_control_points[-1][1]]
        for i in range(num_steps+1):
            t_FR = i/num_steps
            
            FR_point = self.bezier_curve(t_FR, [FR_start, FR_between, FR_target])
            
            FR_x = self.starting_foot_cords_front[0]
            FR_y, FR_z = FR_point
            
            FR_q1, FR_q2, FR_q3 = self.IK(FR_x, FR_y, FR_z)
            
            self.rad_to_degrees('FRS1', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            self.rad_to_degrees('FRS2', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            self.rad_to_degrees('FRS3', radQ1=FR_q1, radQ2=FR_q2, radQ3=FR_q3)
            
            self.write_servo()
        time.sleep(0.01)
           
        # Front-Left go back to middle position from PHASE 3
        FL_target = [self.starting_foot_cords_front[1], self.starting_foot_cords_front[2]] 
        FL_between = [(self.walk_control_points[0][0] - np.abs(self.starting_foot_cords_front[1]))/2, self.walk_control_points[0][1] - 2]
        FL_start = [self.walk_control_points[0][0], self.walk_control_points[0][1]]
        for i in range(num_steps+1):
            t_FR = i/num_steps
            
            FL_point = self.bezier_curve(t_FR, [FL_start, FL_between, FL_target])
            
            FL_x = self.starting_foot_cords_front[0]
            FL_y, FL_z = FL_point
            
            FL_q1, FL_q2, FL_q3 = self.IK(FL_x, FL_y, FL_z)
            
            self.rad_to_degrees('FLS1', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            self.rad_to_degrees('FLS2', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            self.rad_to_degrees('FLS3', radQ1=FL_q1, radQ2=FL_q2, radQ3=FL_q3)
            
            self.write_servo()
            
        time.sleep(0.01)    
        self.starting_position()
 
    def IK(self, x, y, z):
        A = 3.075 # Offset of the leg in Servo 3
        C = (z**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if -1 <= C <= 1:
            q2 = np.arccos(C)
            q1 = np.arctan2(y, z) - np.arctan2(self.a2 * np.sin(q2), self.a1 + self.a2 * np.cos(q2))
        else:
            raise SystemExit('Inverse kinematics out of the range!!')    
        
        if x == 0:
            q3 = 0
        else: 
            x += A
            q3 = np.arctan2(x, z) + np.arctan2(np.sqrt(x**2 + z**2 - A**2), A) - np.pi/2
        
        if q3 < -np.pi/2 or q3 > np.pi/2:
            print('q3 outside the range!')
        
        return q1, q2, q3
    
    def write_servo(self):
        i = 0
        for name, angle in self.servo_angles.items():
            my_servo = servo.Servo(pca.channels[i], actuation_range=180, min_pulse=500, max_pulse=2500)
            my_servo.angle = angle
            print(f"servo: {name}; angle: {angle}; servo channel {i}")
            i += 1
           
    def rad_to_degrees(self, name, radQ1=0, radQ2=0, radQ3=0):
        if name == 'FRS1':
            self.servo_angles['FRS1'] = round((np.pi + radQ1) * 180 / np.pi, 1) - 5 #adjustment
        if name == 'FRS2':
            self.servo_angles['FRS2'] = round((radQ1 + radQ2) * 180 / np.pi, 1)
        if name == 'FRS3':
            self.servo_angles['FRS3'] = round((.5 * np.pi - radQ3) * 180 / np.pi, 1) - 4 #adjustment
    
        if name == 'BRS1':
            self.servo_angles['BRS1'] = round((np.pi + radQ1) * 180 / np.pi, 1)
        if name == 'BRS2':
            self.servo_angles['BRS2'] = round((radQ1 + radQ2) * 180 / np.pi, 1) - 7 #adjustment
        if name == 'BRS3':
            self.servo_angles['BRS3'] = round((.5 * np.pi + radQ3) * 180 / np.pi, 1) - 2 #adjustment
            
        if name == 'FLS1':
            self.servo_angles['FLS1'] = round((radQ1) * -180 / np.pi, 1) - 1 #adjustment
        if name == 'FLS2':
            self.servo_angles['FLS2'] = round((np.pi - (radQ1 + radQ2)) * 180 / np.pi, 1) + 6 #adjustment
        if name == 'FLS3':
            self.servo_angles['FLS3'] = round((.5 * np.pi + radQ3) * 180 / np.pi, 1) + 2 #adjustment    
    
        if name == 'BLS1':
            self.servo_angles['BLS1'] = round((radQ1) * -180 / np.pi, 1)
        if name == 'BLS2':
            self.servo_angles['BLS2'] = round((np.pi - (radQ1 + radQ2)) * 180 / np.pi, 1) + 7 #adjustment    
        if name == 'BLS3':
            self.servo_angles['BLS3'] = round((.5 * np.pi - radQ3) * 180 / np.pi, 1) + 2 #adjustment 
            
if __name__ == "__main__":
    robot = Robot()
    robot.starting_position()
    time.sleep(5)
    robot.walk(6)

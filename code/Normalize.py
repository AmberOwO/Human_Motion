import math
import numpy as np
import sys
import os
from sympy.solvers import solve
from sympy import symbols, Matrix, Eq
from sympy.solvers.solveset import linsolve

class Normalize:
    def __init__(self):
        self.model = sys.argv[1]
        self.outdir = sys.argv[3]
        self.tpose = []
        self.radius_arm = 0
        self.radius_leg = 0
        self.radius_body = 0
        self.radius_gap = 0
        self.move_dis = 0

        if self.model == "1":
            self.tpose = [0, 1.442, -0.05, 0, 1.32, -0.05, 0.13, 1.26, -0.05,0.35,1.256, -0.02,0.6,1.26, -0.03,-0.13,1.26, -0.05,-0.35,1.27, -0.02,-0.6,1.26, -0.03,0,1.04, -0.027,0.063,0.9, -0.034,0.087,0.595, 0.1,0.06,0.104, -0.018,-0.063,0.9, -0.034,-0.087,0.595, 0.1,-0.06,0.109, -0.018]
            self.radius_arm = 0.05
            self.radius_leg = 0.08
            self.radius_body = 0.13
            self.radius_gap = 0.7
            self.move_dis = 0.0043

        elif self.model == "2":
            self.tpose = [0, 1.248, -0.025, 0, 1.1, -0.025, 0.13, 1.068, -0.027,0.299,1.066, -0.03,0.528,1.061, -0.038,-0.13,1.07, -0.025,-0.35,1.071, -0.02,-0.528,1.077, -0.029,0,0.827, -0.027,0.07,0.67, -0.032,0.087,0.513, 0.06,0.07,0.11, 0.02,-0.07,0.67,  -0.03,-0.087,0.516, 0.06,-0.09,0.13, 0.02]
            self.radius_arm = 0.045
            self.radius_leg = 0.07
            self.radius_body = 0.1
            self.radius_gap = 0.06
            self.move_dis = 0.0037

        elif self.model == "3":
            self.tpose = [0, 1.601, -0.01, 0, 1.426, -0.01, 0.129, 1.428, -0.01,0.359,1.395, -0.02,0.601,1.415, -0.018,-0.129,1.428, -0.01,-0.359,1.395, -0.02,-0.601,1.415, -0.018,0,1.103, -0.027,0.1,0.835, -0.032,0.087,0.513, 0.04,0.1,0.11, 0.02,-0.091,0.832,  -0.03,-0.087,0.516, 0.04,-0.09,0.13, 0.02]
            self.radius_arm = 0.07
            self.radius_leg = 0.1
            self.radius_body = 0.2
            self.radius_gap = 0.15
            self.move_dis = 0.005

        elif self.model == "4":
            self.tpose = [0, 1.596, -0.0905, 0, 1.407, -0.0905, 0.115, 1.407, -0.0905,0.376,1.396, -0.2,0.56,1.38, -0.08,-0.115,1.407, -0.0905,-0.376,1.396, -0.2,-0.56,1.38, -0.08,0,1.157, -0.027,0.063,0.993, -0.032,0.087,0.595, 0.1,0.109,0.064, 0,-0.063,0.993, -0.0905,-0.087,0.595, 0.1,-0.109,0.064, 0]
            self.radius_arm = 0.055
            self.radius_leg = 0.08
            self.radius_body = 0.15
            self.radius_gap = 0.7
            self.move_dis = 0.0043

        self.data = []
        self.tpose_len = []
        self.data_len = []
        self.pairs = [(1,0),(1,2),(1,5),(1,8),(2,3),(3,4),(5,6),(6,7),(8,9),(9,10),(10,11),(8,12),(12,13),(13,14)]
                    #   0     1     2     3     4     5     6     7     8     9       10     11      12      13    
        self.out_path = '' 
  
        self.body_r_hand_collide_frame = []
        self.body_l_hand_collide_frame = []
        self.body_r_arm_collide_frame = []
        self.body_l_arm_collide_frame = []       
        self.r_arm_l_uarm_collide_frame = []
        self.r_uarm_l_arm_collide_frame = []
        self.r_arm_l_arm_collide_frame = []
        self.r_uleg_l_leg_collide_frame = []
        self.r_leg_l_leg_collide_frame = []
        self.r_uleg_l_uleg_collide_frame = []
        self.r_leg_l_uleg_collide_frame = []
        self.r_arm_r_uleg_collide_frame = []
        self.l_arm_l_uleg_collide_frame = []
        self.r_arm_gap_collide_frame = []
        self.l_arm_gap_collide_frame = []

    def init(self):
        self.body_r_hand_collide_frame = []
        self.body_l_hand_collide_frame = []
        self.body_r_arm_collide_frame = []
        self.body_l_arm_collide_frame = [] 
        self.r_arm_l_uarm_collide_frame = []
        self.r_uarm_l_arm_collide_frame = []
        self.r_arm_l_arm_collide_frame = []
        self.r_uleg_l_leg_collide_frame = []
        self.r_leg_l_leg_collide_frame = []
        self.r_uleg_l_uleg_collide_frame = []
        self.r_leg_l_uleg_collide_frame = []
        self.r_arm_r_uleg_collide_frame = []
        self.l_arm_l_uleg_collide_frame = []
        self.r_arm_gap_collide_frame = []
        self.l_arm_gap_collide_frame = []      

    def line_to_line_shortest_point(self, v1, v2, p1, p2):
        cross_value = np.cross(v1,v2)        
        M = Matrix(((cross_value[1]*v1[0] - cross_value[0]*v1[1], -(cross_value[1]*v2[0] - cross_value[0]*v2[1]), cross_value[0]*(p1[1]-p2[1]) - cross_value[1]*(p1[0]-p2[0])),
                (cross_value[2]*v1[0] - cross_value[0]*v1[2], -(cross_value[2]*v2[0] - cross_value[0]*v2[2]), cross_value[2]*(p2[0]-p1[0]) - cross_value[0]*(p2[2]-p1[2]))))
        t, k = symbols('t k')
        system = A, b = M[:, :-1], M[:, -1]
        ans = list(linsolve(system, t, k))

        P1 = [p1[0]+v1[0]*ans[0][0], p1[1]+v1[1]*ans[0][0], p1[2]+v1[2]*ans[0][0]]
        P2 = [p2[0]+v2[0]*ans[0][1], p2[1]+v2[1]*ans[0][1], p2[2]+v2[2]*ans[0][1]]

        return P1, P2
        
    def read_coordinate(self,_file):
        with open(_file, 'r') as f:
            
            lines = f.readlines()
            for line in lines:
                line = line[1:-2]
                data = []
                data = line.split(',')
                for d in  range(len(data)):
                    data[d] = float(data[d])
                
                self.data.append(data)

    def get_length(self,coor):
        return math.pow((math.pow(coor[0],2) + math.pow(coor[1],2) + math.pow(coor[2],2)),0.5)

    def get_data(self, frame,index):
        return self.data[frame][3*index],self.data[frame][3*index+1],self.data[frame][3*index+2]

    def get_tpose(self,index):
        return self.tpose[3*index],self.tpose[3*index+1],self.tpose[3*index+2]        

    def get_point_length(self, frame, index1, index2):
        v1 = self.get_data(frame, index1)
        v2 = self.get_data(frame, index2)
        return self.get_length([v1[0]-v2[0],v1[1] - v2[1],v1[2]-v2[2]])

    def get_tpose_length(self, index1, index2):
        v1 = self.get_tpose(index1)
        v2 = self.get_tpose(index2)
        return self.get_length([v1[0]-v2[0],v1[1] - v2[1],v1[2]-v2[2]])

    def nrotate(self,angle,valuex,valuey,pointx,pointy):
        valuex = np.array(valuex)
        valuey = np.array(valuey)
        nRotatex = (valuex-pointx)*math.cos(angle) - (valuey-pointy)*math.sin(angle) + pointx
        nRotatey = (valuex-pointx)*math.sin(angle) + (valuey-pointy)*math.cos(angle) + pointy
        return nRotatex, nRotatey

    def srotate(self,angle,valuex,valuey,pointx,pointy):
        valuex = np.array(valuex)
        valuey = np.array(valuey)
        sRotatex = (valuex-pointx)*math.cos(angle) + (valuey-pointy)*math.sin(angle) + pointx
        sRotatey = (valuey-pointy)*math.cos(angle) - (valuex-pointx)*math.sin(angle) + pointy
        return sRotatex,sRotatey

    def get_body_vector(self,data_pair_idx,frame):
        p1 = self.get_data(frame,self.pairs[data_pair_idx][0])
        p2 = self.get_data(frame,self.pairs[data_pair_idx][1])
        v = []
        for i in range(3):
            v.append(p2[i] - p1[i])
        return v

    def get_movement_vector(self,data_point_idx,frame):

        length = self.tpose_len[data_point_idx]/self.data_len[data_point_idx]
        v = self.get_body_vector(data_point_idx,frame)

        if data_point_idx == 3:
            for i in range(len(v)):
                v[i] = -v[i]

        new_v = []
        for i in range(3):
            new_v.append(v[i]*length)

        return_data = []
        for i in range(3):
            return_data.append(new_v[i] - v[i])

        return return_data

    def get_all_tpose_len(self):
        for i  in self.pairs:
            self.tpose_len.append(self.get_tpose_length(i[0],i[1]))
        return 

    def get_all_data_len(self,frame):
        data_len = []
        for i in self.pairs:
            data_len.append(self.get_point_length(frame,i[0],i[1]))

        return data_len

    def fix_data(self,frame,mov_vec,l_range,u_range):
        for i in range(l_range,u_range):
            for j in  range(3):
                self.data[frame][3*i + j] = self.data[frame][3*i + j] + mov_vec[j]   

    def dump(self):
        with open(self.out_path, 'w') as f:
            for d in self.data:
                f.write(str(d) + '\n')

    def is_collide(self,r1,r2,d,l_p1,l_p2,r_p1,r_p2, d_p):
        if ((r1+r2) > d):
            if(max(l_p1[0],l_p2[0]) > d_p[0][0] and d_p[0][0] > min(l_p1[0],l_p2[0]) and max(l_p1[1],l_p2[1]) > d_p[0][1] and d_p[0][1] > min(l_p1[1],l_p2[1]) and max(l_p1[2],l_p2[2]) > d_p[0][2] and d_p[0][2] > min(l_p1[2],l_p2[2]) \
            and max(r_p1[0],r_p2[0]) > d_p[1][0] and d_p[1][0] > min(r_p1[0],r_p2[0]) and max(r_p1[1],r_p1[1]) > d_p[1][1] and d_p[1][1] > min(r_p1[1],r_p2[1]) and max(r_p1[2],r_p2[2]) > d_p[1][2] and d_p[1][2] > min(r_p1[2],r_p2[2])):
                return 1
            else:
                if(self.get_length([l_p1[0]-r_p1[0],l_p1[1]-r_p1[1],l_p1[2]-r_p1[2]]) < d or self.get_length([l_p1[0]-r_p2[0],l_p1[1]-r_p2[1],l_p1[2]-r_p2[2]]) < d \
                or self.get_length([l_p2[0]-r_p1[0],l_p2[1]-r_p1[1],l_p2[2]-r_p1[2]]) < d or self.get_length([l_p2[0]-r_p2[0],l_p2[1]-r_p2[1],l_p2[2]-r_p2[2]]) < d):
                    return 1
        return 0
                   

    def get_distance(self,frame,data_pair_idx1,data_pair_idx2):

        vec1 = np.array(self.get_body_vector(data_pair_idx1,frame))
        vec2 = np.array(self.get_body_vector(data_pair_idx2,frame))
        p1 = self.get_data(frame,self.pairs[data_pair_idx1][0])
        p2 = self.get_data(frame,self.pairs[data_pair_idx2][0])
        v = []
        for i in range(3):
            v.append(p2[i] - p1[i])
        v = np.array(v)

        cross_value = np.cross(vec1,vec2)
        abs_value = ((cross_value[0]**2)+(cross_value[1]**2)+(cross_value[2]**2))**0.5

        dis = abs(np.dot(np.cross(vec1,vec2),v))/abs_value 

        return dis

    def scale(self,frame):

        self.data_len = self.get_all_data_len(frame)
        
        mov_vec = self.get_movement_vector(3,frame)
        self.fix_data(frame,mov_vec,0,8)

        mov_vec = self.get_movement_vector(0,frame)
        self.fix_data(frame,mov_vec,0,1)

        mov_vec = self.get_movement_vector(1,frame)
        self.fix_data(frame,mov_vec,2,5)

        mov_vec = self.get_movement_vector(2,frame)
        self.fix_data(frame,mov_vec,5,8)

        mov_vec = self.get_movement_vector(4,frame)
        self.fix_data(frame,mov_vec,3,5)

        mov_vec = self.get_movement_vector(6,frame)
        self.fix_data(frame,mov_vec,6,8)

        mov_vec = self.get_movement_vector(5,frame)
        self.fix_data(frame,mov_vec,4,5)

        mov_vec = self.get_movement_vector(7,frame)
        self.fix_data(frame,mov_vec,7,8)

        mov_vec = self.get_movement_vector(8,frame)
        self.fix_data(frame,mov_vec,9,12)

        mov_vec = self.get_movement_vector(11,frame)
        self.fix_data(frame,mov_vec,12,15)

        mov_vec = self.get_movement_vector(9,frame)
        self.fix_data(frame,mov_vec,10,12)

        mov_vec = self.get_movement_vector(12,frame)
        self.fix_data(frame,mov_vec,13,15)

        mov_vec = self.get_movement_vector(10,frame)
        self.fix_data(frame,mov_vec,11,12)

        mov_vec = self.get_movement_vector(13,frame)
        self.fix_data(frame,mov_vec,14,15)    

    def move(self, frame,point, dir):
        if dir == 'r':
            if frame-5 >= 0:
                self.data[frame-5][3*point] += self.move_dis
            if frame-4 >= 0:
                self.data[frame-4][3*point] += self.move_dis*1.1
            if frame-3 >= 0:
                self.data[frame-3][3*point] += self.move_dis*1.25
            if frame-2 >= 0:
                self.data[frame-2][3*point] += self.move_dis*1.45
            if frame-1 >= 0:
                self.data[frame-1][3*point] += self.move_dis*1.7
            self.data[frame][3*point] += self.move_dis*2
            if frame+1 <= len(self.data)-1:
                self.data[frame+1][3*point] += self.move_dis*1.7
            if frame+2 <= len(self.data)-1:
                self.data[frame+2][3*point] += self.move_dis*1.45
            if frame+3 <= len(self.data)-1:
                self.data[frame+3][3*point] += self.move_dis*1.25
            if frame+4 <= len(self.data)-1:
                self.data[frame+4][3*point] += self.move_dis*1.1
            if frame+5 <= len(self.data)-1:
                self.data[frame+5][3*point] += self.move_dis   
        if dir == 'l':
            if frame-5 >= 0:
                self.data[frame-5][3*point] -= self.move_dis
            if frame-4 >= 0:
                self.data[frame-4][3*point] -= self.move_dis*1.1
            if frame-3 >= 0:
                self.data[frame-3][3*point] -= self.move_dis*1.25
            if frame-2 >= 0:
                self.data[frame-2][3*point] -= self.move_dis*1.45
            if frame-1 >= 0:   
                self.data[frame-1][3*point] -= self.move_dis*1.7
            self.data[frame][3*point] -= self.move_dis*2
            if frame+1 <= len(self.data)-1:
                self.data[frame+1][3*point] -= self.move_dis*1.7
            if frame+2 <= len(self.data)-1:
                self.data[frame+2][3*point] -= self.move_dis*1.45
            if frame+3 <= len(self.data)-1:
                self.data[frame+3][3*point] -= self.move_dis*1.25
            if frame+4 <= len(self.data)-1:
                self.data[frame+4][3*point] -= self.move_dis*1.1
            if frame+5 <= len(self.data)-1:
                self.data[frame+5][3*point] -= self.move_dis

    def convert(self,_file):
        self.init()
        self.read_coordinate(_file)
        self.get_all_tpose_len()

        print('process start')        
        print(len(self.data)-1)

        for frame in range(len(self.data)):
            self.data_len = self.get_all_data_len(frame)
            self.scale(frame)

            l_hand = self.get_data(frame,7)
            l_shoulder = self.get_data(frame,5)
            l_elbow = self.get_data(frame,6) 
            r_hand = self.get_data(frame,4)
            r_shoulder = self.get_data(frame,2)
            r_elbow = self.get_data(frame,3)
            u_body = self.get_data(frame,1)
            l_body = self.get_data(frame,8)

            d = self.get_distance(frame,3,5)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(3,frame),self.get_body_vector(5,frame),u_body,r_hand)
            if (self.is_collide(self.radius_arm,self.radius_body,d,u_body,l_body,r_elbow,r_hand,d_p)):
                self.body_r_hand_collide_frame.append(frame)

            d = self.get_distance(frame,3,7)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(3,frame),self.get_body_vector(7,frame),u_body,l_hand)
            if (self.is_collide(self.radius_arm,self.radius_body,d,u_body,l_body,l_elbow,l_hand,d_p)):
                self.body_l_hand_collide_frame.append(frame)

            d = self.get_distance(frame,3,4)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(3,frame),self.get_body_vector(4,frame),u_body,r_elbow)
            if (self.is_collide(self.radius_arm,self.radius_body,d,u_body,l_body,r_elbow,r_shoulder,d_p)):
                self.body_r_arm_collide_frame.append(frame)

            d = self.get_distance(frame,3,6)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(3,frame),self.get_body_vector(6,frame),u_body,l_elbow)
            if (self.is_collide(self.radius_arm,self.radius_body,d,u_body,l_body,l_elbow,l_shoulder,d_p)):
                self.body_l_arm_collide_frame.append(frame)

            d = self.get_distance(frame,5,6)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(5,frame),self.get_body_vector(6,frame),r_elbow,l_elbow)
            if (self.is_collide(self.radius_arm,self.radius_arm,d,r_elbow,r_hand,l_shoulder,l_elbow,d_p)):
                self.r_arm_l_uarm_collide_frame.append(frame)

            d = self.get_distance(frame,4,7)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(4,frame),self.get_body_vector(7,frame),r_elbow,l_elbow)
            if (self.is_collide(self.radius_arm,self.radius_arm,d,r_elbow,r_hand,l_shoulder,l_elbow,d_p)):
                self.r_uarm_l_arm_collide_frame.append(frame)

            d = self.get_distance(frame,5,7)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(5,frame),self.get_body_vector(7,frame),r_elbow,l_elbow)
            if (self.is_collide(self.radius_arm,self.radius_arm,d,r_elbow,r_hand,l_elbow,l_hand,d_p)):
                self.r_arm_l_arm_collide_frame.append(frame)

            l_hip = self.get_data(frame,12)
            l_knee = self.get_data(frame,13)
            l_foot = self.get_data(frame,14)
            r_hip = self.get_data(frame,9)
            r_knee = self.get_data(frame,10)
            r_foot = self.get_data(frame,11)
            l_foot = list(l_foot)

            d = self.get_distance(frame,5,8)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(5,frame),self.get_body_vector(8,frame),r_elbow,r_hip)
            if (self.is_collide(self.radius_arm,self.radius_gap,d,r_elbow,r_hand,u_body,r_hip,d_p)):
                self.r_arm_gap_collide_frame.append(frame)

            d = self.get_distance(frame,7,11)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(7,frame),self.get_body_vector(11,frame),l_elbow,l_hip)
            if (self.is_collide(self.radius_arm,self.radius_gap,d,l_elbow,l_hand,u_body,l_hip,d_p)):
                self.l_arm_gap_collide_frame.append(frame)

            d = self.get_distance(frame,5,9)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(5,frame),self.get_body_vector(9,frame),r_elbow,r_knee)
            if (self.is_collide(self.radius_arm,self.radius_leg,d,r_elbow,r_hand,r_hip,r_knee,d_p)):
                self.r_arm_r_uleg_collide_frame.append(frame)

            d = self.get_distance(frame,7,13)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(7,frame),self.get_body_vector(13,frame),l_elbow,l_knee)
            if (self.is_collide(self.radius_arm,self.radius_leg,d,l_elbow,l_hand,l_hip,l_knee,d_p)):
                self.l_arm_l_uleg_collide_frame.append(frame)

            d = self.get_distance(frame,9,13)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(9,frame),self.get_body_vector(13,frame),r_knee,l_knee)
            if (self.is_collide(self.radius_leg,self.radius_leg,d,l_knee,l_foot,r_hip,r_knee,d_p)):
                self.r_uleg_l_leg_collide_frame.append(frame)

            d = self.get_distance(frame,10,13)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(10,frame),self.get_body_vector(13,frame),r_knee,l_knee)
            if (self.is_collide(self.radius_leg,self.radius_leg,d,r_knee,r_foot,l_knee,l_foot,d_p)):
                self.r_leg_l_leg_collide_frame.append(frame)

            d = self.get_distance(frame,9,12)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(9,frame),self.get_body_vector(12,frame),r_knee,l_knee)
            if (self.is_collide(self.radius_leg,self.radius_leg,d,r_hip,r_knee,l_hip,l_knee,d_p)):
                self.r_uleg_l_uleg_collide_frame.append(frame)

            d = self.get_distance(frame,10,12)
            d_p = self.line_to_line_shortest_point(self.get_body_vector(10,frame),self.get_body_vector(12,frame),r_knee,l_knee)
            if (self.is_collide(self.radius_leg,self.radius_leg,d,r_hip,r_knee,l_knee,l_foot,d_p)):
                self.r_leg_l_uleg_collide_frame.append(frame)


        for i in self.body_r_arm_collide_frame:
            self.move(i,3,'r')

        for i in self.body_l_arm_collide_frame:
            self.move(i,6,'l')

        for i in self.body_r_hand_collide_frame:
            self.move(i,3,'r')
            self.move(i,4,'r')

        for i in self.body_l_hand_collide_frame:
            self.move(i,6,'l')
            self.move(i,7,'l')

        for i in self.r_arm_l_uarm_collide_frame:
            self.move(i,3,'r')
            self.move(i,4,'r')

        for i in self.r_uarm_l_arm_collide_frame:
            self.move(i,6,'l')
            self.move(i,7,'l')

        for i in self.r_arm_l_arm_collide_frame:
            self.move(i,7,'l')
            self.move(i,4,'r')

        for i in self.r_arm_gap_collide_frame:
            self.move(i,3,'r')
            self.move(i,4,'r') 

        for i in self.l_arm_gap_collide_frame:
            self.move(i,6,'l')                   
            self.move(i,7,'l')

        for i in self.r_arm_r_uleg_collide_frame:
            self.move(i,3,'r')
            self.move(i,4,'r')

        for i in self.l_arm_l_uleg_collide_frame:
            self.move(i,6,'l')
            self.move(i,7,'l')

        for i in self.r_uleg_l_leg_collide_frame:
            self.move(i,10,'r')
            self.move(i,11,'r')

        for i in self.r_leg_l_leg_collide_frame:
            self.move(i,14,'l')
            self.move(i,11,'r')

        for i in self.r_uleg_l_uleg_collide_frame:
            self.move(i,10,'r')
            self.move(i,11,'r')
            self.move(i,13,'l')
            self.move(i,14,'l')

        for i in self.r_leg_l_uleg_collide_frame:
            self.move(i,13,'l')
            self.move(i,14,'l')
        
        for frame in range(len(self.data)):
            self.scale(frame)

        print('process end')
        self.dump()

    def read_dir(self):

        self.DATA_PATH = sys.argv[2]
        self.num = 0
        self.files = os.listdir(self.DATA_PATH)
        for _file in self.files:
            self.data = []
            self.out_path = os.path.join(self.outdir, _file)
            self.convert(os.path.join((self.DATA_PATH),_file))

nor = Normalize()
nor.read_dir()
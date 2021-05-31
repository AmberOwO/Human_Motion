import math
import numpy as np
import sys
import os

class Coordinate_Verified:
    def __init__(self, coordinate_file):
        self.input = coordinate_file
        self.tpose = [0, 1.596, -0.0905, 0, 1.407, -0.0905, 0.115, 1.407, -0.0905,0.376,1.396, -0.0905,0.56,1.38, -0.0905,-0.115,1.407, -0.0905,-0.4,1.4, -0.0905,-0.56,1.38, -0.0905,0,1.157, -0.0905,0.063,0.993, -0.0905,0.087,0.595, -0.0905,0.109,0.064, -0.0905,-0.064,0.994, -0.0905,-0.087,0.595, -0.0905,-0.107,0.064, -0.0905]
        self.output = 'new_coordinate.output'
        self.hip_flextion = [-30,120]
        self.hip_adduction = [-50 , 30]
        self.hip_rotation = [-40 , 40]
        self.knee_angle = [0,120]
        self.lumbar_extension = [-90, 90]
        self.lumbar_bending = [-90,90]
        self.lumbar_rotation = [-90,90]
        self.arm_flextion = [-90,90]
        self.arm_adduction = [-30,180]
        self.arm_rotation = [-180,0]
        self.elbow_flextion = [0,150]
        
        self.cur_hip_flextion = 0
        self.cur_hip_adduction = 0
        self.cur_hip_rotation = 0
        self.cur_knee_angle = 0
        self.cur_lumbar_extension = 0
        self.cur_lumbar_bending = 0 
        self.cur_lumbar_rotation = 0
        self.cur_arm_flextion = 0
        self.cur_arm_adduction = 0
        self.cur_arm_rotation = 0
        self.cur_elbow_flextion = 0
        self.max_coor = []
        self.min_coor = []
        self.uarm_length = 0
        self.larm_length = 0
        self.body_length = 0
        self.uleg_length = 0
        self.lleg_length = 0
        self.data = []

    def get_length(self,data,point1,point2):
        return ((data[3*point1] - data[3*point2])**2 + (data[3*point1+1] - data[3*point2+1])**2 + (data[3*point1+2] - data[3*point2+2])**2)**0.5

    def get_length_2d(self,data,point1,point2,d1,d2):
        if (d1 == 'x' and d2 == 'y') or (d1 == 'y' and d2 =='x'):
            return ((data[3*point1] - data[3*point2])**2 + (data[3*point1+1] - data[3*point2+1])**2)**0.5
        elif (d1 == 'y' and d2 == 'z') or (d1 == 'z' and d2 == 'y'):
            return ((data[3*point1+1] - data[3*point2+1])**2 + (data[3*point1+2] - data[3*point2+2])**2)**0.5
        elif (d1 == 'x' and d2 == 'z') or (d1 == 'z' and d2 =='x'):
            return ((data[3*point1] - data[3*point2])**2 + (data[3*point1+2] - data[3*point2+2])**2)**0.5

    def get_length_data(self):
        self.uarm_length = self.get_length(5,6)
        self.larm_length = self.get_length(6,7)
        self.body_length = self.get_length(1,8)
        self.uleg_length = self.get_length(12, 13)
        self.lleg_length = self.get_length(13,14)

    def rotation(self,x1,y1,x2,y2,degree):
        x1=x2*math.cos(degree)+y2*math.sin(degree)
        y1=y2*math.cos(degree)-x2*math.sin(degree)
        return x1,y1

    def get_angle(self,v1,v2):
        unit_vector_1 = v1 / np.linalg.norm(v1)
        unit_vector_2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)
        angle = np.degrees(angle)
        return angle

    def get_angle_3d(self,v1,v2):
        v1 = np.array(v1)
        v2 = np.array(v2)
        l_v1=np.sqrt(v1.dot(v1))
        l_v2=np.sqrt(v2.dot(v2))

        dian=v1.dot(v2)
        cos_=dian/(l_v1*l_v2)
        angle_hu=np.arccos(cos_)
        angle_d=angle_hu*180/np.pi
        return angle_d
        
    def angle_verify(self,data,angle):
        if angle <= data[0] :
            return 1
        elif angle >= data[1]:
            return 2
        else:
            return 0            

    def read_coordinate(self):
        self.input = sys.argv[1]
        with open(self.input, 'r') as f:           
            lines = f.readlines()
            for line in lines:
                line = line[1:-2]
                data = []
                data = line.split(',')
                for d in  range(len(data)):
                    data[d] = float(data[d])
                
                self.data.append(data)

    def read_coordinate(self, _file):
        self.input = _file
        self.data = []
        with open(self.input, 'r') as f:            
            lines = f.readlines()
            for line in lines:
                line = line[1:-2]
                data = []
                data = line.split(',')
                for d in  range(len(data)):
                    data[d] = float(data[d])
                
                self.data.append(data)

    def cut(self, output, start_line, end_line):
        with open(output, 'w') as f:
            for i in range(start_line, end_line):
                f.write(str(self.data[i]) + '\n')

    def coor_ver(self,_file):
        line_num = 0
        out_num = 0
        np.seterr(divide='ignore',invalid='ignore')
        start_line = 0
        for data in self.data:
            err = 0
            line_num += 1
            deg_lumbar_ext = self.get_angle([data[3*1+2] - data[3*8+2], data[3*1+1] - data[3*8+1]],[self.tpose[3*1+2] - self.tpose[3*8+2], self.tpose[3*1+1] - self.tpose[3*8+1]]) # lumbar_ext [1],[8] z,y
            if deg_lumbar_ext > 90:
                err = 1

            deg_lumbar_bend = self.get_angle([data[3*1] - data[3*8], data[3*1+2] - data[3*8+2]],[self.tpose[3*1] - self.tpose[3*8], self.tpose[3*1+2] - self.tpose[3*8+2]]) # lumbar_bend [1],[8] x,z
            if deg_lumbar_bend > 90:
                err = 1

            if data[6*3] > data[5*3]+1 and data[6*3+2] < data[5*3+2] :
                err = 1

            if data[3*3] < data[2*3]-1 and data[3*3+2] < data[2*3+2] :
                err = 1

            deg_lelbow_flex = self.get_angle_3d([data[3*7+2] - data[3*6+2], data[3*7] - data[3*6], data[3*7+1] - data[3*6+1]],[data[3*6+2] - data[3*5+2], data[3*6] - data[3*5], data[3*6+1] - data[3*5+1]]) #lelbow_flex [7],[6] [6],[5] x,y,z
            if deg_lelbow_flex > 150:
                err = 1
            else:
                vec = [data[3*6+2] - data[3*5+2], data[3*6+1] - data[3*5+1]]
                if (data[3*7+2]-data[3*6+2]/data[3*7+1] - data[3*6+1]) > (vec[0]/vec[1]):
                    err = 1

            deg_relbow_flex = self.get_angle_3d([data[3*4+2] - data[3*3+2], data[3*4] - data[3*3], data[3*4+1] - data[3*3+1]],[data[3*3+2] - data[3*2+2], data[3*3] - data[3*2], data[3*3+1] - data[3*2+1]]) #relbow_flex [4],[3] [3],[2] x,y,z
            if deg_relbow_flex > 150:
                err = 1
            else:
                vec = [data[3*3+2] - data[3*2+2], data[3*3+1] - data[3*2+1]]
                if (data[3*4+2]-data[3*3+2]/data[3*4+1] - data[3*3+1]) > (vec[0]/vec[1]):
                    err = 1

            deg_lhip_addc = self.get_angle([data[3*13] - data[3*12], data[3*13+2] - data[3*12+2]],[self.tpose[3*13] - self.tpose[3*12], self.tpose[3*13+2] - self.tpose[3*12+2]]) # deg_lhip_addc [13],[12] x,z
            if deg_lhip_addc > 30:
                err = 1

            deg_lhip_flex = self.get_angle([data[3*13+2] - data[3*12+2], data[3*13+1] - data[3*12+1]],[self.tpose[3*13+2] - self.tpose[3*12+2], self.tpose[3*13+1] - self.tpose[3*12+1]]) # deg_lhip_flex [13],[12] z,y
            if deg_lhip_flex > 90:
                err = 1

            deg_rhip_addc = self.get_angle([data[3*10] - data[3*9], data[3*10+2] - data[3*9+2]],[self.tpose[3*10] - self.tpose[3*9], self.tpose[3*10+2] - self.tpose[3*9+2]]) # deg_rhip_addc [10],[9] x,z
            if deg_rhip_addc > 90:
                err = 1

            deg_rhip_flex = self.get_angle([data[3*10+2] - data[3*9+2], data[3*10+1] - data[3*9+1]],[self.tpose[3*10+2] - self.tpose[3*9+2], self.tpose[3*10+1] - self.tpose[3*9+1]]) # deg_lhip_flex [13],[12] z,y
            if deg_rhip_flex > 90:
                err = 1

            deg_lknee_angle = self.get_angle_3d([data[3*14+2] - data[3*13+2], data[3*14] - data[3*13], data[3*14+1] - data[3*13+1]],[data[3*13+2] - data[3*12+2], data[3*13] - data[3*12], data[3*13+1] - data[3*12+1]]) #lknee_angle [14],[13] [13],[12] x,y,z
            if deg_lknee_angle > 120 or (data[3*14+1] > data[3*13+1] and deg_lhip_flex < 90 ):
                err = 1

            deg_rknee_angle = self.get_angle_3d([data[3*11+2] - data[3*10+2], data[3*11] - data[3*10], data[3*11+1] - data[3*10+1]],[data[3*10+2] - data[3*9+2], data[3*10] - data[3*9], data[3*10+1] - data[3*9+1]]) #rknee_angle [11],[10] [10],[9] x,y,z
            if deg_rknee_angle > 120 or (data[3*11+1] > data[3*10+1] and deg_rhip_flex < 90 ) :
                err = 1
            
            if err == 1:
                end_line = line_num
                if end_line - start_line >= 10 :
                    outfile = os.path.join((self.DATA_PATH_OUT),_file[:-4]) + "_" + str(out_num) + '.out'
                    self.cut(outfile,start_line,end_line)
                    out_num +=1
                    start_line = end_line+1

    def read_dir(self):
        self. data = []
        self.DATA_PATH = 'input'
        self.DATA_PATH_OUT = 'output'
        self.num = 0
        self.files = os.listdir(self.DATA_PATH)
        for _file in self.files:
            self.read_coordinate(os.path.join((self.DATA_PATH),_file))
            self.coor_ver(_file)

verify = Coordinate_Verified('walk__1.txt')
verify.read_dir()
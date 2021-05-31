import numpy as np 
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import random
import sys

def motion(part,point,angle,aix):
    output = part.copy()
    for r,row in enumerate(output):
        for frame in range(0,len(row),12):
            ori = row[frame+point*3:frame+point*3+3].copy()
            for coor in range(point*3+3,12,3):
                if aix == 'x':
                    row[frame+coor+1] = round((row[frame+coor+1]-ori[1])*math.cos(math.radians(angle)) - (row[frame+coor+2]-ori[2])*math.sin(math.radians(angle)),6)+ori[1]
                    row[frame+coor+2] = round((row[frame+coor+1]-ori[1])*math.sin(math.radians(angle)) + (row[frame+coor+2]-ori[2])*math.cos(math.radians(angle)),6)+ori[2]
                elif aix == 'y':
                    row[frame+coor] = round((row[frame+coor]-ori[0])*math.cos(math.radians(angle)) - (row[frame+coor+2]-ori[2])*math.sin(math.radians(angle)),6)+ori[0]
                    row[frame+coor+2] = round((row[frame+coor]-ori[0])*math.sin(math.radians(angle)) + (row[frame+coor+2]-ori[2])*math.cos(math.radians(angle)),6)+ori[2]                    
                elif aix == 'z':
                    row[frame+coor] = round((row[frame+coor]-ori[0])*math.cos(math.radians(angle)) - (row[frame+coor+1]-ori[1])*math.sin(math.radians(angle)),6)+ori[0]
                    row[frame+coor+1] = round((row[frame+coor]-ori[0])*math.sin(math.radians(angle)) + (row[frame+coor+1]-ori[1])*math.cos(math.radians(angle)),6)+ori[1]                  
   
    return output

def classify(file_name):
    body = []
    larm = []
    lleg = []
    rarm = []
    rleg = []
    body_data = []
    larm_data = []
    lleg_data = []
    rarm_data = []
    rleg_data = []
    loaded = read_coordinate(file_name)
    for data_frame in loaded:
        body.append([data_frame[0],data_frame[1],data_frame[2],data_frame[3],data_frame[4],data_frame[5],data_frame[24],data_frame[25],data_frame[26]])
        rarm.append([data_frame[3],data_frame[4],data_frame[5],data_frame[6],data_frame[7],data_frame[8],data_frame[9],data_frame[10],data_frame[11],data_frame[12],data_frame[13],data_frame[14]])
        rleg.append([data_frame[24],data_frame[25],data_frame[26],data_frame[27],data_frame[28],data_frame[29],data_frame[30],data_frame[31],data_frame[32],data_frame[33],data_frame[34],data_frame[35]])
        larm.append([data_frame[3],data_frame[4],data_frame[5],data_frame[15],data_frame[16],data_frame[17],data_frame[18],data_frame[19],data_frame[20],data_frame[21],data_frame[22],data_frame[23]])
        lleg.append([data_frame[24],data_frame[25],data_frame[26],data_frame[36],data_frame[37],data_frame[38],data_frame[39],data_frame[40],data_frame[41],data_frame[42],data_frame[43],data_frame[44]])
    
    for frame in range(0,len(loaded),10):
        body_data.append(np.array(body[frame:frame+10]).flatten())
        larm_data.append(np.array(larm[frame:frame+10]).flatten())
        lleg_data.append(np.array(lleg[frame:frame+10]).flatten())
        rarm_data.append(np.array(rarm[frame:frame+10]).flatten())
        rleg_data.append(np.array(rleg[frame:frame+10]).flatten())

    return body_data,larm_data,lleg_data,rarm_data,rleg_data

def read_coordinate(_file):
    res = []
    with open(_file, 'r') as f:
        
        lines = f.readlines()
        for line in lines:
            line = line[1:-2]
            data = []
            data = line.split(',')
            for d in  range(len(data)):
                data[d] = float(data[d])
            res.append(data)
            
    return res

def load_data(file_name):
    with open(file_name,'rb') as fpick:
        data = pickle.load(fpick)
        position = []
        for frame in data:
            load = [frame['head.x'],frame['head.y'],frame['head.z'],frame['neck.x'],frame['neck.y'],frame['neck.z'],
                    frame['rarm.x'],frame['rarm.y'],frame['rarm.z'],frame['rfarm.x'],frame['rfarm.y'],frame['rfarm.z'],
                    frame['rhand.x'],frame['rhand.y'],frame['rhand.z'],frame['larm.x'],frame['larm.y'],frame['larm.z'],
                    frame['lfarm.x'],frame['lfarm.y'],frame['lfarm.z'],frame['lhand.x'],frame['lhand.y'],frame['lhand.z'],
                    frame['hips.x'],frame['hips.y'],frame['hips.z'],frame['ruleg.x'],frame['ruleg.y'],frame['ruleg.z'],
                    frame['rleg.x'],frame['rleg.y'],frame['rleg.z'],frame['rfoot.x'],frame['rfoot.y'],frame['rfoot.z'],
                    frame['luleg.x'],frame['luleg.y'],frame['luleg.z'],frame['lleg.x'],frame['lleg.y'],frame['lleg.z'],
                    frame['lfoot.x'],frame['lfoot.y'],frame['lfoot.z']]
            load = [float(i) for i in load]
            position.append(load)

    return position

def normalize(normal_data):
    for i,frame in enumerate(normal_data):
        hips = [normal_data[i][24],normal_data[i][25],normal_data[i][26]]
        for j in range(15):
            normal_data[i][3*j] = round(float(normal_data[i][3*j])-float(hips[0]),6)
            normal_data[i][3*j+1] = round(float(normal_data[i][3*j+1])-float(hips[1]),6)
            normal_data[i][3*j+2] = round(float(normal_data[i][3*j+2])-float(hips[2]),6)

    return normal_data

def dump(out_path, data):
    with open(out_path, 'w') as f:
        for d in data:
            f.write(str(d) + '\n')

def combine(body,rarm,larm,rleg,lleg,file_name):
    result = [] 
    with open(file_name, "w") as outFile:                      
        for i,row in enumerate(body):
            j = 0
            for frame in range(0,len(row)-6,9):
                part_i = frame+(j*3)
                frame_data = [0 for _ in range(45)]
                frame_data[0:6] = body[i][frame+0:frame+6]
                frame_data[6:15] = rarm[i][part_i+3:part_i+12]
                frame_data[15:24] = larm[i][part_i+3:part_i+12]
                frame_data[24:27] = body[i][frame+6:frame+9]
                frame_data[27:36] = rleg[i][part_i+3:part_i+12]
                frame_data[36:45] = lleg[i][part_i+3:part_i+12] 
                result.append(frame_data)
                j += 1

        for d in result:
            outFile.write(str(d) + '\n')
          
    return result
            
def plot(final_plt):

    fig = plt.figure()
    ax = Axes3D(fig)

    for frame in final_plt:
        #body
        x=np.array([frame[0],frame[3],frame[24]],dtype='float32')
        y=np.array([frame[1],frame[4],frame[25]],dtype='float32')
        z=np.array([frame[2],frame[5],frame[26]],dtype='float32')
        ax.scatter(x, y, z,color='red')
        ax.plot(x,y,z,color='red')

        #rightarm
        x=np.array([frame[3],frame[6],frame[9],frame[12]],dtype='float32')
        y=np.array([frame[4],frame[7],frame[10],frame[13]],dtype='float32')
        z=np.array([frame[5],frame[8],frame[11],frame[14]],dtype='float32')
        ax.scatter(x, y, z,color='blue')
        ax.plot(x,y,z,color='blue')

        #rightleg
        x=np.array([frame[3],frame[15],frame[18],frame[21]],dtype='float32')
        z=np.array([frame[5],frame[17],frame[20],frame[23]],dtype='float32')
        ax.scatter(x, y, z,color='blue')
        ax.plot(x,y,z,color='blue')

        #leftarm
        x=np.array([frame[24],frame[27],frame[30],frame[33]],dtype='float32')
        y=np.array([frame[25],frame[28],frame[31],frame[34]],dtype='float32')
        z=np.array([frame[26],frame[29],frame[32],frame[35]],dtype='float32')
        ax.scatter(x, y, z,color='green')
        ax.plot(x,y,z,color='green')

        #leftleg
        x=np.array([frame[24],frame[36],frame[39],frame[42]],dtype='float32')
        y=np.array([frame[25],frame[37],frame[40],frame[43]],dtype='float32')
        z=np.array([frame[26],frame[38],frame[41],frame[44]],dtype='float32')
        ax.scatter(x, y, z,color='green')
        ax.plot(x,y,z,color='green')
         
        ax.set_xticks([-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5])
        ax.set_yticks([-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5])
        ax.set_zticks([-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        plt.draw()
        plt.pause(0.001)
        plt.cla()

if __name__ == "__main__":
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if len(sys.argv) < 3:
        print('no argument')
        sys.exit()
        
    body_data,larm_data,lleg_data,rarm_data,rleg_data = classify(input_file)
    larm_data = motion(larm_data,random.randint(0,3),random.randint(0,360),random.choice(['x','y','z']))
    lleg_data = motion(lleg_data,random.randint(0,3),random.randint(0,360),random.choice(['x','y','z']))
    rarm_data = motion(rarm_data,random.randint(0,3),random.randint(0,360),random.choice(['x','y','z']))
    rleg_data = motion(rleg_data,random.randint(0,3),random.randint(0,360),random.choice(['x','y','z']))
    final = combine(body_data,rarm_data,larm_data,rleg_data,lleg_data,output_file)
    plot(final)
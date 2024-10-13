#!/usr/bin/env python3
import numpy as np
import sys
sys.path.append('./walking_packet')
from thmos_walk_engine_pos import *
from random import random 
import scipy.io
from scipy.spatial.transform import Rotation as R

def RPYtoR(rpy):
    '''RPY angles to rotate matrix'''
    a = rpy[0]
    b = rpy[1]
    c = rpy[2]

    sinA = np.sin(a)
    cosA = np.cos(a)
    sinB = np.sin(b)
    cosB = np.cos(b)
    sinC = np.sin(c)
    cosC = np.cos(c)

    R = np.array([[cosB*cosC,  cosC*sinA*sinB - cosA*sinC,  sinA*sinC + cosA*cosC*sinB],
          [cosB*sinC,  cosA*cosC + sinA*sinB*sinC, cosA*sinB*sinC - cosC*sinA],
          [-sinB, cosB*sinA,  cosA*cosB]])
    R_axis = np.array([[0 ,-1, 0],
              [0 , 0, 1],
              [-1, 0, 0]])
    return np.dot(R_axis , R)

# 使用示例
      
if __name__ == '__main__':
  TIME_STEP = 0.001
  # control box ----
  sys.path.append(sys.path[0] + '/param.txt')
  param_path=sys.path[-1]		
  param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=38,invalid_raise=False)
  Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'walking_period' : param[7],
            'both_foot_support_time' : param[8],
            'dt' : param[9],
            'max_vx' : param[10],
            'max_vy': param[11],
            'max_vth' : param[12],
            'k_x_offset':param[13],#ex_com_x_offset k
            'k_y_offset':param[14],#ex_com_y_offset k
            'trunk_pitch':param[15],
            'way_left' : [1,-1,-1,-1,-1,-1],
            'way_right' : [1,1,-1,1,1,-1],
            'leg_rod_length' : [0.156,0.12,0.045]
            }

  walk = walking(**Params)
  step_num = 0
  j = 0
  n = 0
  k = 0 
  nk = 0
  roll_ang = 0
  pitch_ang = 0
  size_log = 1000 + 1000

  pos_mat = np.zeros((size_log,12))
  T_l_mat = np.zeros((4,4,size_log))
  T_r_mat = np.zeros((4,4,size_log))
  time_mat = np.zeros((size_log,1))
 
  for j in range(500):
    joint_pos = [Params["ex_foot_width"], 0, - 0.5 + j / 1000 * (-Params["trunk_height"] + 0.5),
                 0,0,0,
                 - Params["ex_foot_width"],  0, - 0.5 + j / 1000 * (-Params["trunk_height"] + 0.5),
                 0,0,0]
    pos_mat[step_num,:] = joint_pos
    T_l_mat[:3,:3,step_num] = RPYtoR(joint_pos[3:6])
    T_l_mat[:3, 3,step_num] = joint_pos[0:3]
    T_l_mat[ 3, 3,step_num] = 1
    T_r_mat[:3,:3,step_num] = RPYtoR(joint_pos[9:12])
    T_r_mat[:3, 3,step_num] = joint_pos[6:9]
    T_r_mat[ 3, 3,step_num] = 1   
    time_mat[step_num] = step_num * Params["dt"]
    #print(joint_pos)
    step_num += 1

  while step_num < size_log:
    #else:
    if n == 0:
      if nk < 8:
        walk.setGoalVel([0.05, 0.0, 0.0])
        nk = nk + 1
      elif nk < 12:
        walk.setGoalVel([0.05, 0.0, 0.0])
        nk = nk + 1
      else:
        walk.setGoalVel([0.05, 0.0, 0.0])
        nk = 0
    joint_pos,n = walk.getNextPos()
    pos_mat[step_num,:] = joint_pos
    T_l_mat[:3,:3,step_num] = RPYtoR(joint_pos[3:6])
    T_l_mat[:3, 3,step_num] = joint_pos[0:3]
    T_l_mat[ 3, 3,step_num] = 1
    T_r_mat[:3,:3,step_num] = RPYtoR(joint_pos[9:12])
    T_r_mat[:3, 3,step_num] = joint_pos[6:9]
    T_r_mat[ 3, 3,step_num] = 1   
    time_mat[step_num] = step_num * Params["dt"]
    #print(joint_pos)
    step_num += 1
  print(T_l_mat[:,:,0])
  #print(time_mat)
  np.savez('data.npz', time=time_mat, pos=pos_mat, lposT = T_l_mat, rposT = T_r_mat)


# 加载.mat文件
data = scipy.io.loadmat('defaultsimulationinputs.mat', struct_as_record=False, squeeze_me=True)

# 获取 siminR 和 siminL
siminR = data['siminR']
siminL = data['siminL']

# 访问 siminR 的 time 和 signals 字段
siminR_time = siminR.time
siminR_signals = siminR.signals

# 访问 signals 的 values 和 dimensions
siminR_values = siminR_signals.values
siminR_dimensions = siminR_signals.dimensions

# 打印输出以验证数据
print('Time:', siminR_time)
print('Time Shape:', siminR_time.shape)
print('Signals Values Shape:', siminR_values.shape)
print('Signals Dimensions:', siminR_dimensions)

print(siminR_values[:, :, 0])

# ----------------------------------------------------------------------
data['siminR'].time = time_mat
data['siminL'].time = time_mat

data['siminL'].signals.values = T_l_mat
data['siminL'].signals.dimensions = [4., 4.]

data['siminR'].signals.values = T_r_mat
data['siminR'].signals.dimensions = [4., 4.]


print(data['siminR'].signals.values.shape)
scipy.io.savemat('modifiedsimulationinputs.mat', data)


import scipy.io
import numpy as np

# 加载.mat文件
data = scipy.io.loadmat('defaultsimulationinputs.mat', struct_as_record=False, squeeze_me=True)
# data = scipy.io.loadmat('modifiedsimulationinputs.mat', struct_as_record=False, squeeze_me=True)

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
new_data = np.load('data.npz')
print(list(new_data.keys()))
data['siminR'].time = new_data['time']
data['siminL'].time = new_data['time']
print(new_data['pos'].shape)
poses = new_data['pos'].reshape(12, 1000)
T_l_mat = new_data['lposT']
T_r_mat = new_data['rposT']
# fixed_matrix = np.array([
#     [0., -1., 0., 0.],
#     [0., 0., 1., 0.],
#     [-1., 0., 0., 0.],
#     [0., 0., 0., 1.]
# ])
# 重复矩阵以创建4x4x1000的数组
# repeated_matrix = np.tile(fixed_matrix[:, :, np.newaxis], (1, 1, 1000))
# repeated_matrix[:3, 3, :] = poses[:3, :]
repeated_matrix = T_l_mat
data['siminL'].signals.values = repeated_matrix
data['siminL'].signals.dimensions = [4, 4]


# repeated_matrix = np.tile(fixed_matrix[:, :, np.newaxis], (1, 1, 1000))
# repeated_matrix[:3, 3, :] = poses[6:9, :]
repeated_matrix = T_r_mat
data['siminR'].signals.values = repeated_matrix
data['siminR'].signals.dimensions = [4, 4]


print(data['siminR'].signals.values.shape)

scipy.io.savemat('modifiedsimulationinputs.mat', data)

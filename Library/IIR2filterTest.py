import mat4py
from Library.IIR2Filter import IIR2Filter as IIR
import os
import scipy.io as sio
import numpy as np

FilterMains = IIR.IIR2Filter(5,[17],'lowpass',design='cheby2',rs=80,fs=200)


MAT_Trajectory = mat4py.loadmat(os.path.abspath(os.path.join(os.getcwd(), ".."))+'/Library/IIR2Filter/matlab.mat')
Trajectory_X=MAT_Trajectory['Data_R31']

# print(Trajectory_X)
mySignal=Trajectory_X

mySignalFiltered = []
for i in range(len(mySignal)):
    mySignalFiltered.append(FilterMains.filter(mySignal[i]))

save_fn = 'IIR2Filter/xxx.mat'
sio.savemat(save_fn, {'mySignalFiltered': mySignalFiltered}) #和上面的一样，存在了array变量的第一行



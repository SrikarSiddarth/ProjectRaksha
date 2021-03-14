#!/usr/bin/env python
import pandas as pd
import numpy as np
# import scipy as sp
import matplotlib.pyplot as plt

data = np.load('dataset.npy')
data = np.transpose(data)

df = pd.DataFrame(data,columns=['x','y','z','x_target','y_target','z_target'])
df.dropna(inplace=True)
# print(df.head(5))
# plt.plot(df.iloc[:,0])
# plt.show()
# print(df.describe())

# read a specific position (r,c)
print(df['y_target'][0] + df['x'][0])
# for i in range(6):
# 	plt.plot(df.iloc[:,i],label=df.columns[i])
# plt.legend()
# plt.show()
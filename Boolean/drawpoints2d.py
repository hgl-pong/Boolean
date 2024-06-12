# coding=utf-8
import matplotlib.pyplot as plt
import pandas as pd
 
data=pd.read_csv("triangle2d.csv",header=None)
x=data.iloc[:,0]
y=data.iloc[:,1]
print(data)
plt.scatter(x, y)
plt.show()
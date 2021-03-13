import pandas as pd
import numpy as np

df = pd.read_csv('../aruco_pose_estimation_rolling_statistics.txt', delimiter = " ")

# some sample data
ts = pd.Series(np.random.randn(1000), index=pd.date_range('1/1/2000', periods=1000)).cumsum()

#plot the time series
ts.plot(style='k--')

# calculate a 60 day rolling mean and plot
#pd.rolling_mean(ts, 60).plot(style='k')

ax = ts.rolling(60).mean().plot(style='k')
# add the 20 day rolling variance:

ax = ts.rolling(20).std().plot(style='b')

ax.figure.savefig('test.png')

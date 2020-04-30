import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv("pid_resp3.csv")
data = data[0: 70]
data['ticks'] = data.index

data.plot(kind="scatter",
          x="ticks", 
          y="err",
          title="Response of PID, Kp=0.1, Ki=0.02, Kd=0.01"
          )

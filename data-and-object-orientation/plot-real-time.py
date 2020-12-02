import matplotlib.pyplot as plt
import matplotlib.animation as anim
import pandas as pd
import sys
from itertools import count

plt.style.use("fivethirtyeight")
fig = plt.figure()
fig.set_size_inches(15, 7)
ax = plt.gca()

def callback(i):
    data = pd.read_csv("/opt/nordic/nRF5_SDK_17.0.0/examples/personal/tese-eduardo/data-and-object-orientation/data/combined.csv", header=None)
    
    data0 = [0.05*a for a in range(len(data[0]))]
    plt.cla()
    plt.plot(data0, data[6], label = "Pitch - ComplementaryFilter")
    plt.plot(data0, data[7], label = "Roll - ComplementaryFilter")
    
    plt.xlabel("Time(s)")
    plt.ylabel("Degrees")
    
    text = f"{data[8][len(data[8])-1]}"
    print(text) 
    plt.title(text)
    plt.legend(loc = "upper right")
    #plt.tight_layout()

ani = anim.FuncAnimation(fig, callback, interval = 50, frames = 10000)
#plt.figure(figsize=(15,5))
plt.show()
#plt.tight_layout()
ani.save('simulation.mp4', dpi=80, fps=10, writer='ffmpeg', extra_args=['-vcodec', 'libx264'])

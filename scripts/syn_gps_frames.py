import pandas as pd
import numpy as np

gps = pd.read_csv("gps_utm.csv")
timestamps = np.loadtxt("timestamps.txt")

synced = []

for i, t in enumerate(timestamps):
    idx = (gps["time"] - t).abs().idxmin()
    synced.append([i, t, gps.loc[idx,"x"], gps.loc[idx,"y"]])

df = pd.DataFrame(synced,
                  columns=["frame","time","x","y"])
df.to_csv("gps_synced.csv", index=False)


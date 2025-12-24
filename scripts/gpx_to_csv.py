import gpxpy
import pandas as pd

with open("vedio.gpx") as f:
    gpx = gpxpy.parse(f)

rows = []
for track in gpx.tracks:
    for seg in track.segments:
        for p in seg.points:
            rows.append([
                p.time.timestamp(),
                p.latitude,
                p.longitude
            ])

df = pd.DataFrame(rows, columns=["time","lat","lon"])
df.to_csv("gps.csv", index=False)


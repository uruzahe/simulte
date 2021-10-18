import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import sys
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

df = pd.read_csv(sys.argv[1], converters = {
'attrvalue': parse_if_number,
'binedges': parse_ndarray,
'binvalues': parse_ndarray,
'vectime': parse_ndarray,
'vecvalue': parse_ndarray})

pdr_vector = 'tbDecoded:vector'
pdr_dist_vector = 'txRxDistanceTB:vector'

distances = df[(df["name"] == pdr_dist_vector) & (df["vectime"].notnull())]
decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]
distances = distances[["module", "vecvalue"]]
distances.rename(columns={"vecvalue": "distance"}, inplace=True)
decoded = decoded[["module", "vecvalue"]]
decoded.rename(columns={"vecvalue": "decode"}, inplace=True)
new_df = pd.merge(distances, decoded, on='module', how='inner')

bins = []
dist = 150
for i in range(dist):
    bins.append({"count": 0, "success": 0})

for row in new_df.itertuples():
    for i in range(len(row.distance)):
        print(row.distance[i])
        if row.distance[i] < dist * 10:
            # Ensures that we have everything in 10m chunks
            remainder = int(row.distance[i] // 10)
            if row.decode[i] >= 0:
                # Only count TBs sent i.e. -1 will be ignored in result
                bins[remainder]["count"] += 1
                bins[remainder]["success"] += row.decode[i]

pdrs = []
distances = []
distance = 0
for dictionary in bins:
    pdrs.append((dictionary["success"] / dictionary["count"] * 100))
    distances.append(distance)
    print(f"{distance}: {pdrs[-1]}")
    distance += 10

fig, ax = plt.subplots()
ax.plot(distances, pdrs, label="PDR")
ax.set(xlabel='Distance (m)', ylabel="Packet Delivery Ratio %")
ax.legend(loc="lower left")
ax.tick_params(direction='in')
ax.set_xlim([0, (max(distances) + 1)])
ax.set_ylim([0, 101])
plt.xticks(np.arange(0, (max(distances))+50, step=50))
plt.yticks(np.arange(0, (101), step=10))
plt.show()
plt.savefig("test.png", dpi=300)
plt.close(fig)

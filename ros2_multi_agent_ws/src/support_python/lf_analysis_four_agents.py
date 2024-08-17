import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
import os
import sys
import datetime

curr_time = datetime.datetime.now().replace(microsecond=0)
formation_control_gain = str(20)
leader_gain = "0-7"
max_angular_velocity = "1-57"
title = f"leader_follower_kf{formation_control_gain}_lg{leader_gain}_am{max_angular_velocity}"

df00 = pd.read_csv("lf_data00.csv")
df01 = pd.read_csv("lf_data01.csv")
df02 = pd.read_csv("lf_data02.csv")
df03 = pd.read_csv("lf_data03.csv")

df = pd.concat([df00, df01, df02, df03], axis=1)

l0, l1, l2, l3 = len(df00), len(df01), len(df02), len(df03)

max_len = max(l0, l1, l2, l3)

if max_len == l0:
    df["time_stamp01"] = df["time_stamp01"].fillna(df["time_stamp00"])
    df["time_stamp02"] = df["time_stamp02"].fillna(df["time_stamp00"])
    df["time_stamp03"] = df["time_stamp03"].fillna(df["time_stamp00"])
elif max_len == l1:
    df["time_stamp00"] = df["time_stamp00"].fillna(df["time_stamp01"])
    df["time_stamp02"] = df["time_stamp02"].fillna(df["time_stamp01"])
    df["time_stamp03"] = df["time_stamp03"].fillna(df["time_stamp01"])
elif max_len == l2:
    df["time_stamp01"] = df["time_stamp01"].fillna(df["time_stamp02"])
    df["time_stamp00"] = df["time_stamp00"].fillna(df["time_stamp02"])
    df["time_stamp03"] = df["time_stamp03"].fillna(df["time_stamp02"])
else:
    df["time_stamp02"] = df["time_stamp02"].fillna(df["time_stamp03"])
    df["time_stamp01"] = df["time_stamp01"].fillna(df["time_stamp03"])
    df["time_stamp00"] = df["time_stamp00"].fillna(df["time_stamp03"])

df["Time (sec)"] = df[["time_stamp00", "time_stamp01", "time_stamp02", "time_stamp03"]].mean(axis=1)
df = df.drop([
    "time_stamp00", "time_stamp01", "time_stamp02", "time_stamp03"],
    axis=1)

df['diff'] = df['Time (sec)'].diff()
df.loc[0, "diff"] = 0
df['cumulative_sum'] = df['diff'].cumsum()
df['Time (sec)'] = df['cumulative_sum']
df = df.drop(columns=['diff', 'cumulative_sum'])

df[[
    "x00", "y00", "x01", "y01", "x02", "y02", "x03", "y03", 
    "curr00_01", "curr01_02", "curr01_03", "curr02_03", "des00_01",
    'des01_02', 'des01_03', 'des02_03'
]] = df[[
    "x00", "y00", "x01", "y01", "x02", "y02", "x03", "y03", 
    "curr00_01", "curr01_02", "curr01_03", "curr02_03", "des00_01",
    'des01_02', 'des01_03', 'des02_03'
]].ffill()

df = df.fillna(0.0)
fig, ax = plt.subplots(2,2, figsize=(16, 12))
ax = ax.flatten()

fig.suptitle(f"{curr_time} {title}", fontweight="bold", fontsize=20)
plt.subplots_adjust(hspace=20, wspace=25)

colors = {"v_input00": plt.cm.autumn_r(0.8), "v_input01": plt.cm.winter_r(0.8), 
          "v_input02": plt.cm.Greens(0.8), "v_input03": plt.cm.cool(0.8)}
cols = ["v_input00", "v_input01", "v_input02", "v_input03"]
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols],ax = ax[0])
ax[0].set_title("Linear velocity over time", fontweight="bold")
ax[0].set_ylabel("Linear velocity (m/s)")
ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.85))
ax[0].grid(True)

colors = {"omega_input00": plt.cm.autumn_r(0.8), "omega_input01": plt.cm.winter_r(0.8), 
          "omega_input02": plt.cm.Greens(0.8), "omega_input03": plt.cm.cool(0.8)}
cols = ["omega_input00", "omega_input01", "omega_input02", "omega_input03"]
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols], ax=ax[1])
ax[1].set_title("Angular velocity over time", fontweight="bold")
ax[1].set_ylabel("Angular velocity (rad/s)")
ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.85))
ax[1].grid(True)

colors = {
    'curr00_01': plt.cm.autumn_r(0.8), 'des00_01' : plt.cm.autumn_r(0.8),
    'curr01_02': plt.cm.Greens(0.8), 'des01_02' : plt.cm.Greens(0.8),
    'curr01_03': plt.cm.winter_r(0.8), 'des01_03' : plt.cm.winter_r(0.8),
    'curr02_03': plt.cm.cool(0.8), 'des02_03' : plt.cm.cool(0.8),
}
linestyles = {
    'curr00_01': '--', 'curr01_02': '--', 'curr01_03': '--', 'curr02_03': '--',
    'des00_01': '-', 'des01_02': '-', 'des01_03': '-', 'des02_03': '-'
}
cols = ["curr00_01", "curr01_02", "curr01_03", "curr02_03", "des00_01", 'des01_02', 'des01_03', 'des02_03']
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols], ax=ax[2])
for line in ax[2].get_lines():
    label = line.get_label()
    line.set_linestyle(linestyles[label])
ax[2].set_title("Current and desired distance between agents", fontweight="bold")
ax[2].set_ylabel("Distance (m)")
ax[2].legend(loc='center left', bbox_to_anchor=(1, 0.85))
ax[2].grid(True)

initial_xs = np.array([df.loc[df["x00"].index[0], "x00"], df.loc[df["x01"].index[0], "x01"], 
                       df.loc[df["x02"].index[0], "x02"], df.loc[df["x03"].index[0], "x03"]])
initial_ys = np.array([df.loc[df["y00"].index[0], "y00"], df.loc[df["y01"].index[0], "y01"], 
                       df.loc[df["y02"].index[0], "y02"], df.loc[df["y03"].index[0], "y03"]])
final_xs = np.array([df.loc[df["x00"].index[-1], "x00"], df.loc[df["x01"].index[-1], "x01"], 
              df.loc[df["x02"].index[-1], "x02"], df.loc[df["x03"].index[-1], "x03"]])
final_ys = np.array([df.loc[df["y00"].index[-1], "y00"], df.loc[df["y01"].index[-1], "y01"], 
              df.loc[df["y02"].index[-1], "y02"], df.loc[df["y03"].index[-1], "y03"]])

targets_in_map = np.array([[-2,0,0], [-.5,-1,0], [2.5,-1,0], [2.5,1,0], [-.5,1,0]])

ax[3].scatter(targets_in_map[:,0], targets_in_map[:,1], s=60, c= "black", marker="^")

df.plot(x="x00", y="y00", kind="scatter", s=14, colormap="autumn_r", c="Time (sec)", ax=ax[3], label='agent00', colorbar=False)
df.plot(x="x01", y="y01", kind="scatter", s=10, colormap="winter_r", c="Time (sec)", ax=ax[3], label='agent01', colorbar=False)
df.plot(x="x02", y="y02", kind="scatter", s=10, colormap="Greens", c="Time (sec)", ax=ax[3], label='agent02', colorbar=False)
df.plot(x="x03", y="y03", kind="scatter", s=10, colormap="cool", c="Time (sec)",  ax=ax[3], label='agent03', colorbar=False)

ax[3].scatter(initial_xs, initial_ys, s=20, c="dimgrey")
ax[3].scatter(final_xs[1:], final_ys[1:], s=30, c="darkred")
ax[3].scatter(final_xs[0], final_ys[0], s=45, c="black")

autumn_patch = mlines.Line2D([], [], color=plt.cm.autumn_r(0.8), marker='o', linestyle='None', markersize=10, label='agent00')
winter_patch = mlines.Line2D([], [], color=plt.cm.winter_r(0.8), marker='o', linestyle='None', markersize=10, label='agent01')
summer_patch = mlines.Line2D([], [], color=plt.cm.Greens(0.8), marker='o', linestyle='None', markersize=10, label='agent02')
spring_patch = mlines.Line2D([], [], color=plt.cm.cool(0.8), marker='o', linestyle='None', markersize=10, label='agent03')

ax[3].legend(handles=[autumn_patch, winter_patch, summer_patch, spring_patch], loc="center left", bbox_to_anchor=(1,0.85))
ax[3].set_ylabel("y (m)")
ax[3].set_xlabel("x (m)")
ax[3].set_title("Agent trajectories over time", fontweight="bold")
ax[3].grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(f"{curr_time} {title}")
plt.show()
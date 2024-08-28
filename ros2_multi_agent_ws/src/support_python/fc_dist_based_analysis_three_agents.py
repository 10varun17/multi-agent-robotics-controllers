import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
import datetime

curr_time = datetime.datetime.now().replace(microsecond=0) # for title and image name
title = "dist-based-formation-control"

df00 = pd.read_csv("fc_data00.csv")
df01 = pd.read_csv("fc_data01.csv")
df02 = pd.read_csv("fc_data02.csv")

df = pd.concat([df00, df01, df02], axis=1)

l0, l1, l2 = len(df00), len(df01), len(df02)

max_len = max(l0, l1, l2)

if max_len == l0:
    df["time_stamp01"] = df["time_stamp01"].fillna(df["time_stamp00"])
    df["time_stamp02"] = df["time_stamp02"].fillna(df["time_stamp00"])
elif max_len == l1:
    df["time_stamp00"] = df["time_stamp00"].fillna(df["time_stamp01"])
    df["time_stamp02"] = df["time_stamp02"].fillna(df["time_stamp01"])
else:
    df["time_stamp01"] = df["time_stamp01"].fillna(df["time_stamp02"])
    df["time_stamp00"] = df["time_stamp00"].fillna(df["time_stamp02"])

df["Time (sec)"] = df[["time_stamp00", "time_stamp01", "time_stamp02"]].mean(axis=1)
df = df.drop([
    "time_stamp00", "time_stamp01", "time_stamp02"],
    axis=1)

df['diff'] = df['Time (sec)'].diff()
df.loc[0, "diff"] = 0
df['cumulative_sum'] = df['diff'].cumsum()
df['Time (sec)'] = df['cumulative_sum']
df = df.drop(columns=['diff', 'cumulative_sum'])

df = df.drop(["curr01_00", "des01_00", "curr02_00", 
    "des02_00", "curr02_01", "des02_01"],
    axis=1)

df[[
    "x00", "y00", "x01", "y01", "x02", "y02",
    "curr00_01", "curr00_02", "curr01_02", "des00_01", "des00_02", "des01_02"
]] = df[[
    "x00", "y00", "x01", "y01", "x02", "y02",
    "curr00_01", "curr00_02", "curr01_02", "des00_01", "des00_02", "des01_02"
]].ffill()

df = df.fillna(0.0)
fig, ax = plt.subplots(2,2, figsize=(16, 12))
ax = ax.flatten()

fig.suptitle(f"{curr_time} {title}", fontweight="bold", fontsize=20)
plt.subplots_adjust(hspace=40, wspace=15)

colors = {"v_input00": plt.cm.autumn_r(0.8), "v_input01": plt.cm.winter_r(0.8), "v_input02": plt.cm.cool(0.8)}
cols = ["v_input00", "v_input01", "v_input02"]
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols],ax = ax[0])
ax[0].set_title("Linear velocity over time", fontweight="bold")
ax[0].set_ylabel("Linear velocity (m/s)")
ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.85))
ax[0].grid(True)

colors = {"omega_input00": plt.cm.autumn_r(0.8), "omega_input01": plt.cm.winter_r(0.8), "omega_input02": plt.cm.cool(0.8)}
cols = ["omega_input00", "omega_input01", "omega_input02"]
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols], ax=ax[1])
ax[1].set_title("Angular velocity over time", fontweight="bold")
ax[1].set_ylabel("Angular velocity (rad/s)")
ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.85))
ax[1].grid(True)

colors = {
    'curr00_01': plt.cm.autumn_r(0.8), 'des00_01': plt.cm.autumn_r(0.8),
    'curr00_02': plt.cm.winter_r(0.8), 'des00_02': plt.cm.winter_r(0.8),
    'curr01_02': plt.cm.cool(0.8), 'des01_02': plt.cm.cool(0.8)
}
linestyles = {
    'curr00_01': '--', 'curr00_02': '--', 'curr01_02': '--',
    'des00_01': '-', 'des00_02': '-', 'des01_02': '-'
}
cols = ["curr00_01", "curr00_02", "curr01_02", "des00_01", "des00_02", "des01_02"]
df.plot(x="Time (sec)", y=cols, color=[colors[col] for col in cols], ax=ax[2])
for line in ax[2].get_lines():
    label = line.get_label()
    line.set_linestyle(linestyles[label])
ax[2].set_title("Current and desired distance between agents", fontweight="bold")
ax[2].set_ylabel("Distance (m)")
ax[2].legend(loc='center left', bbox_to_anchor=(1, 0.75))
ax[2].grid(True)

df.plot(x="x00", y="y00", kind="scatter", c = "Time (sec)", colormap="autumn_r",colorbar=False, ax=ax[3])
df.plot(x="x01", y="y01", kind="scatter", c = "Time (sec)", colormap="winter_r",colorbar=False, ax=ax[3])
df.plot(x="x02", y="y02", kind="scatter", c = "Time (sec)", colormap="cool",colorbar=False, ax=ax[3])

initial_xs = np.array([df.loc[df["x00"].index[0], "x00"], df.loc[df["x01"].index[0], "x01"], 
                       df.loc[df["x02"].index[0], "x02"]])
initial_ys = np.array([df.loc[df["y00"].index[0], "y00"], df.loc[df["y01"].index[0], "y01"], 
                       df.loc[df["y02"].index[0], "y02"]])
final_xs = np.array([df.loc[df["x00"].index[-1], "x00"], df.loc[df["x01"].index[-1], "x01"], 
              df.loc[df["x02"].index[-1], "x02"]])
final_ys = np.array([df.loc[df["y00"].index[-1], "y00"], df.loc[df["y01"].index[-1], "y01"], 
              df.loc[df["y02"].index[-1], "y02"]])

ci = np.array([initial_xs.mean(), initial_ys.mean()])
cf = np.array([final_xs.mean(), final_ys.mean()])

# ax[3].scatter(ci[0], ci[1], s=60, c="cyan")
# ax[3].scatter(cf[0], cf[1], s=60, c="magenta")
# ax[3].annotate('ci', (ci[0], ci[1]), textcoords="offset points", xytext=(15,-3), ha='center')
# ax[3].annotate('cf', (cf[0], cf[1]), textcoords="offset points", xytext=(-15,-3), ha='center')

# ax[3].scatter(initial_xs, initial_ys, s=20, c="dimgrey")
ax[3].scatter(final_xs, final_ys, s=30, c="black")
ax[3].annotate('00', (final_xs[0], final_ys[0]), textcoords="offset points", xytext=(-15,-4), ha='center')
ax[3].annotate('01', (final_xs[1], final_ys[1]), textcoords="offset points", xytext=(15,-4), ha='center')
ax[3].annotate('02', (final_xs[2], final_ys[2]), textcoords="offset points", xytext=(15,-4), ha='center')

spring_patch = mlines.Line2D([], [], color=plt.cm.autumn(0.8), marker='o', linestyle='None', markersize=10, label='agent00')
winter_r_patch = mlines.Line2D([], [], color=plt.cm.winter(0.8), marker='o', linestyle='None', markersize=10, label='agent01')
cool_patch = mlines.Line2D([], [], color=plt.cm.cool_r(0.8), marker='o', linestyle='None', markersize=10, label='agent02')
ax[3].legend(handles=[spring_patch, winter_r_patch, cool_patch], loc="center left", bbox_to_anchor=(1,0.85))
ax[3].set_ylabel("y (m)")
ax[3].set_xlabel("x (m)")
ax[3].set_title("Agent trajectories over time", fontweight="bold")
ax[3].grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(f"{curr_time} {title}")
plt.show()
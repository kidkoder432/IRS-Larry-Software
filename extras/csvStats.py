import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("data.csv")

print(f"Average Loop Time: {df['Delta Time'].mean()}")
print(f"Max Loop Time: {df['Delta Time'].max()}")
print(f"Min Loop Time: {df['Delta Time'].min()}")
print(f"Total Loops: {len(df['Delta Time'])}")
print(f"Average Loop Rate: {1 / df['Delta Time'].where(df['State'] == 76).mean()}")
print(f"Max Loop Rate: {1 / df['Delta Time'].min()}")
print(f"Min Loop Rate: {1 / df['Delta Time'].max()}")

dt_write = df["Delta Time"].where(df["Delta Time"] > 0.03)
dt_loop = df["Delta Time"].where(df["Delta Time"] < 0.03)

print(f"Average Write Time: {dt_write.mean()}")
print(f"Average Calculation Time: {dt_loop.mean()}")

ax = plt.gca()

m, s, b = ax.stem(range(len(df)), (df["Delta Time"]))
plt.setp(m, markersize=0.5)
plt.setp(s, linewidth=0.5)
plt.show()

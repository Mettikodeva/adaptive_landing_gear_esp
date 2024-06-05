import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot_log(log_file, save_path):
    df = pd.read_csv(log_file)
    df = df.dropna()
    time = np.linspace(0, len(df),len(df))
    df["time"] = time
    fig, ax = plt.subplots()
    ax.plot(df["time"], df[" yaw"], label="yaw")
    ax.plot(df["time"], df[" pitch"], label="pitch")
    ax.plot(df["time"], df[" roll"], label="roll")
    plt.savefig(save_path)
    plt.show()
    fig2 , ax2 = plt.subplots()
    ax2.plot(df["time"], df[" m1"], label="m1")
    ax2.plot(df["time"], df[" m2"], label="m2")
    ax2.plot(df["time"], df[" m3"], label="m3")
    ax2.legend()
    plt.savefig(save_path)
    plt.show()


if __name__ == "__main__":
    plot_log("outputSat May  4 00:38:53 2024_clean.csv", "output_clean.png")
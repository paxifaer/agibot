import pandas as pd
import matplotlib.pyplot as plt

def analyze(file):

    df = pd.read_csv(file)

    latency = df["latency_ms"]

    print("File:", file)
    print("Mean latency:", latency.mean())
    print("Max latency:", latency.max())
    print("Min latency:", latency.min())
    print("Std latency:", latency.std())

    plt.hist(latency, bins=30)
    plt.title(file)
    plt.xlabel("Latency ms")
    plt.ylabel("count")

    plt.savefig(file + ".png")
    plt.clf()


if __name__ == "__main__":

    analyze("nav_latency.csv")
    analyze("observe_latency.csv")
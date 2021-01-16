#!/usr/bin/env python
import csv
import matplotlib.pyplot as plt
import pandas as pd

with open('bist_data.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile)
    data_list = []
    for row in spamreader:
        data_list.append(row)

columns = data_list[0]
df = pd.DataFrame(data_list[1:], columns=columns)
df=df.astype(float)
df.plot(x="Threshold", y=["Seed","FBG","Output"])
plt.savefig("bist_data")
plt.show()
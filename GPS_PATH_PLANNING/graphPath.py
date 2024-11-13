import matplotlib.pyplot as plt
import csv

def graphPath(csvFile):
    x = []
    y = []

    with open(csvFile, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            x.append(float(row[1]))
            y.append(float(row[0]))
    plt.plot(x, y, marker='o')
    plt.axhline(min(y), color='black', linewidth=0.5)
    plt.axvline(min(x), color='black', linewidth=0.5)
    plt.grid(color='gray', linestyle='--', linewidth=0.5)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


graphPath('./static/path.csv')
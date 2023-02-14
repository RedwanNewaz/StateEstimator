#!/usr/bin/python3
import matplotlib.pyplot as plt
import csv
from collections import defaultdict
from argparse import ArgumentParser

def read_csv_file(filename, keys):

    history = defaultdict(list)
    # opening the CSV file
    with open(filename, mode ='r') as file:

        # reading the CSV file
        csvFile = csv.DictReader(file)

        # displaying the contents of the CSV file

        for lines in csvFile:
            for key, value in lines.items():
                key = key.strip()
                if key in keys:
                    data = float(value)
                    history[key].append(data)
    return history


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--file', type=str, default='../build/ekf/logger_2023-02-13_21:39:25.csv',
                        help='csv file path')

    args = parser.parse_args()

    keys = ['filter_x', 'filter_y']
    data = read_csv_file(args.file, keys)
    plt.plot(data['filter_x'], data['filter_y'])

    # noisy observations
    keys = ['raw_x', 'raw_y']
    data = read_csv_file(args.file, keys)
    plt.scatter(data['raw_x'], data['raw_y'], s=5, color='red')

    plt.show()
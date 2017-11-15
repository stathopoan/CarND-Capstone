import matplotlib
import numpy as np

matplotlib.use('TkAgg')  # Needs to be done before importing pyplot

import csv
from matplotlib import pyplot as plt
import os


# 0)Iteration 1)wanted_velocity 2)throttle 3)brake 4)steer 5)linear_v_error 6)angular_v_error 7)cte 8)delta_t 9)processing_time 10)avg_proc_time

def main():
    f_name = os.path.expanduser('~/.ros/chart_data0002.txt')

    do = (1, 2, 3, 4, 5, 6, 7, 9)  # List here the indices of the parameters to be plotted
    # 0)Iteration 1)wanted_velocity 2)throttle 3)brake 4)steer 5)linear_v_error 6)angular_v_error 7)cte 8)delta_t 9)processing_time 10)avg_proc_time

    max_entries = 150000 # Process at most these many entries from the log file 175000
    skip_entries = 1500  # Skip these many entries from the beginning of the log file

    # Read the log file
    count = 0
    print 'Reading data from file', f_name
    print
    with open(f_name, 'r') as csvfile:
        file_reader = csv.reader(csvfile, delimiter=' ')
        is_header = True
        first_line_skipped = False
        for line in file_reader:
            if not first_line_skipped:
                first_line = line
                first_line_skipped =  True
                continue
            if is_header:
                header = line
                y = []
                for i in xrange(len(header)):
                    y.append([])
                is_header = False
                continue
            if count + 1 <= skip_entries:
                count += 1
                continue
            for i in xrange(len(header)):
                y[i].append(float(line[i]))
            count += 1
            if count == max_entries:
                break

    # Plot the values and compute various metrics
    n_categories = len(do)
    total_n_categories = len(header)
    n_entries = len(y[0])

    mean = [.0] * total_n_categories
    std_dev = [.0] * total_n_categories
    the_max = [.0] * total_n_categories
    the_min = [.0] * total_n_categories
    positive_percentage = [.0] * total_n_categories

    for i_category in xrange(total_n_categories):
        mean[i_category] = np.mean(y[i_category])
        std_dev[i_category] = np.std(y[i_category])
        the_max[i_category] = max(y[i_category])
        the_min[i_category] = min(y[i_category])
        positive_percentage[i_category] = (np.array(y[i_category]) >= 0).sum()

    for i_category in xrange(total_n_categories):
        positive_percentage[i_category] /= float(n_entries)

    print first_line
    print

    for i_category in xrange(1, total_n_categories):
        print header[i_category]
        print '   Mean      ', mean[i_category]
        print '   Std. Dev. ', std_dev[i_category]
        print '   Max.      ', the_max[i_category]
        print '   Min.      ', the_min[i_category]
        print '   Non-neg. %', positive_percentage[i_category]*100
        print

    plt.ion()
    fig, axes = plt.subplots(nrows=n_categories)
    fig.canvas.set_window_title(f_name)
    x = list(xrange(skip_entries, n_entries + skip_entries))
    for i in xrange(n_categories):
        category_i = do[i]
        axes[i].plot(x, y[category_i])
        y_bottom, y_top = axes[i].get_ylim()
        axes[i].text(skip_entries, y_top - (y_top - y_bottom) / 5, header[category_i], fontdict={'weight': 'bold'})
        axes[i].grid()
    plt.show()
    while True:
        plt.pause(1)


if __name__ == '__main__':
    main()

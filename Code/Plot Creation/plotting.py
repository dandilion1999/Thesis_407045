import matplotlib.pyplot as plt
import numpy as np


def create_user_plot(values, test_attribute, user):
    # Coordinates of the points
    x = [0, 45, 55, 95, 105, 145, 155]
    off_colour = "lightred"
    on_colour = "lightgreen"

    # Create the plot
    plt.figure(figsize=(8, 6))
    plt.scatter(x[0], values[0], color='black')

    plt.scatter(x[1], values[1], color='red', label='CTRL OFF')
    plt.scatter(x[3], values[3], color='red')
    plt.scatter(x[5], values[5], color='red')

    plt.scatter(x[2], values[2], color='green', label='CTRL ON')
    plt.scatter(x[4], values[4], color='green')
    plt.scatter(x[6], values[6], color='green')

    # Label the axes
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title(f"{user}, {test_attribute}")
    plt.xticks(ticks=[0, 50, 100, 150], labels=['0', '200', '300', '400'])

    # Show a legend
    plt.legend()

    # Show the plot
    plt.show()


def create_plot(title, categories, means, std_devs):
    # Set the positions and width for the bars
    positions = np.arange(len(categories))
    width = 0.5  # Width of the bars

    # Plotting the bar graph
    plt.bar(positions, means, width, yerr=std_devs, capsize=5, alpha=0.7, label='Means with SD')

    # Adding some labels and a title
    plt.xlabel('Category')
    plt.ylabel('Lane deviation')
    plt.title(title)
    # plt.xticks(positions, categories)  # Set the positions and labels of the x-axis

    # Display the legend and show the plot
    plt.legend()
    plt.show()


def create_error_box_plots(data, title):
    baseline, two_off, two_on, three_off, three_on, four_off, four_on = data

    off_colour = "peachpuff"
    on_colour = "lightgreen"

    fig, ax = plt.subplots()
    baseline_box = ax.boxplot(baseline, widths=5, positions=[0], patch_artist=True, boxprops=dict(facecolor='yellow', color='black'))

    two_off_box = ax.boxplot(two_off, widths=5, positions=[45], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    two_on_box = ax.boxplot(two_on, widths=5, positions=[55], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    three_off_box = ax.boxplot(three_off, widths=5, positions=[95], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    three_on_box = ax.boxplot(three_on, widths=5, positions=[105], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    four_off_box = ax.boxplot(four_off, widths=5, positions=[145], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    four_on_box = ax.boxplot(four_on, widths=5, positions=[155], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    # Set the labels for x-axis and y-axis
    ax.set_ylabel('Lane deviation (m)')
    ax.set_xlabel('Latency (ms)')
    plt.grid()
    ax.legend([baseline_box["boxes"][0], two_off_box["boxes"][0], two_on_box["boxes"][0]], ['Baseline', 'Controller OFF', 'Controller ON'])

    # plt.title("Median Lane Deviation")
    plt.xticks(ticks=[0, 50, 100, 150], labels=['0', '200', '300', '400'])

    plt.show()


def create_side_by_side_error_box_plots(data, data_2):

    baseline, two_off, two_on, three_off, three_on, four_off, four_on = data
    baseline_2, two_off_2, two_on_2, three_off_2, three_on_2, four_off_2, four_on_2 = data_2

    off_colour = "peachpuff"
    on_colour = "lightgreen"

    fig, ax = plt.subplots(1, 2, figsize=(12, 6))
    baseline_box = ax[0].boxplot(baseline, widths=5, positions=[0], patch_artist=True, boxprops=dict(facecolor='yellow', color='black'))

    two_off_box = ax[0].boxplot(two_off, widths=5, positions=[45], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    two_on_box = ax[0].boxplot(two_on, widths=5, positions=[55], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    three_off_box = ax[0].boxplot(three_off, widths=5, positions=[95], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    three_on_box = ax[0].boxplot(three_on, widths=5, positions=[105], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    four_off_box = ax[0].boxplot(four_off, widths=5, positions=[145], patch_artist=True, boxprops=dict(facecolor=off_colour, color='black'))
    four_on_box = ax[0].boxplot(four_on, widths=5, positions=[155], patch_artist=True, boxprops=dict(facecolor=on_colour, color='black'))

    # Set the labels for x-axis and y-axis
    ax[0].set_ylabel('Steering actions')
    ax[0].set_xlabel('Latency (ms)')
    ax[0].grid()
    ax[0].legend([baseline_box["boxes"][0], two_off_box["boxes"][0], two_on_box["boxes"][0]], ['Baseline', 'Controller OFF', 'Controller ON'])
    # plt.title("Median Lane Deviation")
    # ax[0].xticks(ticks=[0, 50, 100, 150], labels=['0', '200', '300', '400'])
    ax[0].set_xticks([0, 50, 100, 150])
    ax[0].set_xticklabels(['0', '200', '300', '400'])


    baseline_box_2 = ax[1].boxplot(baseline_2, widths=5, positions=[0], patch_artist=True,
                                 boxprops=dict(facecolor='yellow', color='black'))

    two_off_box_2 = ax[1].boxplot(two_off_2, widths=5, positions=[45], patch_artist=True,
                                boxprops=dict(facecolor=off_colour, color='black'))
    two_on_box_2 = ax[1].boxplot(two_on_2, widths=5, positions=[55], patch_artist=True,
                               boxprops=dict(facecolor=on_colour, color='black'))

    three_off_box_2 = ax[1].boxplot(three_off_2, widths=5, positions=[95], patch_artist=True,
                                  boxprops=dict(facecolor=off_colour, color='black'))
    three_on_box_2 = ax[1].boxplot(three_on_2, widths=5, positions=[105], patch_artist=True,
                                 boxprops=dict(facecolor=on_colour, color='black'))

    four_off_box_2 = ax[1].boxplot(four_off_2, widths=5, positions=[145], patch_artist=True,
                                 boxprops=dict(facecolor=off_colour, color='black'))
    four_on_box_2 = ax[1].boxplot(four_on_2, widths=5, positions=[155], patch_artist=True,
                                boxprops=dict(facecolor=on_colour, color='black'))

    # Set the labels for x-axis and y-axis
    ax[1].set_ylabel('Steering input')
    ax[1].set_xlabel('Latency (ms)')
    ax[1].grid()
    ax[1].legend([baseline_box["boxes"][0], two_off_box["boxes"][0], two_on_box["boxes"][0]],
                 ['Baseline', 'Controller OFF', 'Controller ON'])

    # ax[1].xticks(ticks=[0, 50, 100, 150], labels=['0', '200', '300', '400'])
    ax[1].set_xticks([0, 50, 100, 150])
    ax[1].set_xticklabels(['0', '200', '300', '400'])

    plt.show()



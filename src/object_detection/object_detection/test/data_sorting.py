# import re
# from math import sqrt

# def parse_pothole_data(line):
#     match = re.search(r'Pothole (\d+) - Odom Coordinates: geometry_msgs.msg.Point\(x=([-.\d]+), y=([-.\d]+), z=([-.\d]+)\), Size: (\d+)', line)
#     if match:
#         pothole_id, x, y, _, size = match.groups()  # Ignore z coordinate
#         return int(pothole_id), float(x), float(y), int(size)
#     return None

# def read_pothole_data(filename):
#     pothole_data = []
#     with open(filename, 'r') as file:
#         for line in file:
#             parsed_data = parse_pothole_data(line)
#             if parsed_data:
#                 pothole_data.append(parsed_data)
#     return pothole_data

# def distance(point1, point2):
#     return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# def remove_close_coordinates(pothole_data, threshold_distance):
#     unique_coordinates = set()
#     filtered_potholes = []

#     for pothole in pothole_data:
#         coordinates = (pothole[1], pothole[2])

#         is_close = False
#         for unique_coord in unique_coordinates:
#             if distance(coordinates, unique_coord) < threshold_distance:
#                 is_close = True
#                 break

#         if not is_close:
#             unique_coordinates.add(coordinates)
#             filtered_potholes.append(pothole)

#     return filtered_potholes

# def write_filtered_data(filtered_potholes, output_filename):
#     with open(output_filename, 'w') as file:
#         for pothole in filtered_potholes:
#             file.write(f'Pothole {pothole[0]} - Odom Coordinates: geometry_msgs.msg.Point(x={pothole[1]}, y={pothole[2]}), Size: {pothole[3]}\n')

# if __name__ == "__main__":
#     input_filename = "src/object_detection/collect_pothole_data.txt"
#     output_filename = "src/object_detection/object_detection/test/filtered_pothole_data.txt"
#     distance_threshold = 0.3  # Adjust this threshold as needed

#     pothole_data = read_pothole_data(input_filename)
#     filtered_potholes = remove_close_coordinates(pothole_data, distance_threshold)
#     write_filtered_data(filtered_potholes, output_filename)

#     print("Filtered data has been written to", output_filename)



################################################################

# from geometry_msgs.msg import Point
import geometry_msgs.msg
from math import sqrt
import matplotlib.pyplot as plt

def filter_points(input_list, distance_threshold):
    filtered_list = []

    for point in input_list:
        x = point['odom_coords'].x
        y = point['odom_coords'].y

        # Condition 1: -2 < x < 2, Condition 2: -2 < y < 1
        if -2 < x < 2 and -2 < y < 1:
            # Ignore z value (Condition 3)
            point_copy = point.copy()
            point_copy['odom_coords'] = geometry_msgs.msg.Point(x=x, y=y, z=float(0))

            # Check for duplicate/similar points (Condition 4)
            is_duplicate = any(
                sqrt((x - fp['odom_coords'].x)**2 + (y - fp['odom_coords'].y)**2) < distance_threshold
                for fp in filtered_list
            )

            # If not a duplicate, add to the filtered list
            if not is_duplicate:
                filtered_list.append(point_copy)

    return filtered_list

def write_to_file(file_path, data):
    try:
        with open(file_path, 'w') as file:
            for point in data:
                # Convert each dictionary to a string and write to the file
                line = ', '.join([f"'{key}': {value}" for key, value in point.items()])
                file.write(f"{{{line}}}\n")
    except Exception as e:
        print(f"Error writing to file: {e}")

def read_from_file(file_path):
    data = []

    try:
        with open(file_path, 'r') as file:
            for line in file:
                # Evaluate each line as a dictionary and append to the list
                point = eval(line.strip())
                data.append(point)
    except Exception as e:
        print(f"Error reading from file: {e}")

    return data

def plot_points(input_list):
    x_values = [point['odom_coords'].x for point in input_list]
    y_values = [point['odom_coords'].y for point in input_list]

    plt.scatter(x_values, y_values, marker='o', label='Filtered Points')
    plt.xlabel('X Values')
    plt.ylabel('Y Values')
    plt.title('Odom Coordinates in X-Y Plane')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_map_coordinates(input_list):
    x_values = [point['odom_coords'].x for point in input_list]
    y_values = [point['odom_coords'].y for point in input_list]

    plt.scatter(x_values, y_values, marker='o', label='Map Coordinates')
    plt.xlabel('X Values')
    plt.ylabel('Y Values')
    plt.title('Map Coordinates in X-Y Plane')
    plt.legend()
    plt.grid(True)
    plt.show()
    
def plot_both(input_list):
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))

    x_values_filtered = [point['odom_coords'].x for point in input_list]
    y_values_filtered = [point['odom_coords'].y for point in input_list]

    axs[0].scatter(x_values_filtered, y_values_filtered, marker='o', label='Filtered Points')
    axs[0].set_xlabel('X Values')
    axs[0].set_ylabel('Y Values')
    axs[0].set_title('Filtered Points in X-Y Plane')
    axs[0].legend()
    axs[0].grid(True)

    x_values_map = [point['x_map'] for point in input_list]
    y_values_map = [point['y_map'] for point in input_list]

    axs[1].scatter(x_values_map, y_values_map, marker='o', label='Map Coordinates')
    axs[1].set_xlabel('X Values')
    axs[1].set_ylabel('Y Values')
    axs[1].set_title('Map Coordinates in X-Y Plane')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()
    
    
file_path = "src/object_detection/object_detection/test/detected_potholes.txt"
distance_threshold = 0.3  # Adjust as needed

input_data = read_from_file(file_path)
if input_data:
    filtered_data = filter_points(input_data, distance_threshold)

    file_path_filtered = "src/object_detection/object_detection/test/sorted_potholes.txt"
    write_to_file(file_path_filtered, filtered_data)
    plot_both(filtered_data)
    # plot_points(filtered_data)
    # plot_map_coordinates(filtered_data)

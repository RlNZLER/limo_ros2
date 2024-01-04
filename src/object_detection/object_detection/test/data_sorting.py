import re
from math import sqrt

def parse_pothole_data(line):
    match = re.search(r'Pothole (\d+) - Odom Coordinates: geometry_msgs.msg.Point\(x=([-.\d]+), y=([-.\d]+), z=([-.\d]+)\), Size: (\d+)', line)
    if match:
        pothole_id, x, y, _, size = match.groups()  # Ignore z coordinate
        return int(pothole_id), float(x), float(y), int(size)
    return None

def read_pothole_data(filename):
    pothole_data = []
    with open(filename, 'r') as file:
        for line in file:
            parsed_data = parse_pothole_data(line)
            if parsed_data:
                pothole_data.append(parsed_data)
    return pothole_data

def distance(point1, point2):
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def remove_close_coordinates(pothole_data, threshold_distance):
    unique_coordinates = set()
    filtered_potholes = []

    for pothole in pothole_data:
        coordinates = (pothole[1], pothole[2])

        is_close = False
        for unique_coord in unique_coordinates:
            if distance(coordinates, unique_coord) < threshold_distance:
                is_close = True
                break

        if not is_close:
            unique_coordinates.add(coordinates)
            filtered_potholes.append(pothole)

    return filtered_potholes

def write_filtered_data(filtered_potholes, output_filename):
    with open(output_filename, 'w') as file:
        for pothole in filtered_potholes:
            file.write(f'Pothole {pothole[0]} - Odom Coordinates: geometry_msgs.msg.Point(x={pothole[1]}, y={pothole[2]}), Size: {pothole[3]}\n')

if __name__ == "__main__":
    input_filename = "src/object_detection/collect_pothole_data.txt"
    output_filename = "src/object_detection/object_detection/test/filtered_pothole_data.txt"
    distance_threshold = 0.3  # Adjust this threshold as needed

    pothole_data = read_pothole_data(input_filename)
    filtered_potholes = remove_close_coordinates(pothole_data, distance_threshold)
    write_filtered_data(filtered_potholes, output_filename)

    print("Filtered data has been written to", output_filename)

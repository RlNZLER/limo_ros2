import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import geometry_msgs.msg
from math import sqrt
import matplotlib.pyplot as plt

DISTANCE_THRESHOLD = 0.3

class ReportGenerator(Node):
    
    def __init__(self):
        super().__init__('report_generator')
        self.object_detection_status_sub = self.create_subscription(String, '/object_detection/status', self.report_generator_callback, 10)
        self.start_report_generation = False
            
    def filter_points(self, input_list):
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
                    sqrt((x - fp['odom_coords'].x)**2 + (y - fp['odom_coords'].y)**2) < DISTANCE_THRESHOLD
                    for fp in filtered_list
                )

                # If not a duplicate, add to the filtered list
                if not is_duplicate:
                    filtered_list.append(point_copy)

        return filtered_list

    def write_to_file(self, file_path, data):
        try:
            with open(file_path, 'w') as file:
                for point in data:
                    # Convert each dictionary to a string and write to the file
                    line = ', '.join([f"'{key}': {value}" for key, value in point.items()])
                    file.write(f"{{{line}}}\n")
        except Exception as e:
            print(f"Error writing to file: {e}")

    def read_from_file(self, file_path):
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

    def plot_points(self, input_list):
        x_values = [point['odom_coords'].x for point in input_list]
        y_values = [point['odom_coords'].y for point in input_list]

        plt.scatter(x_values, y_values, marker='o', label='Filtered Points')
        plt.xlabel('X Values')
        plt.ylabel('Y Values')
        plt.title('Odom Coordinates in X-Y Plane')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_map_coordinates(self, input_list):
        x_values = [point['odom_coords'].x for point in input_list]
        y_values = [point['odom_coords'].y for point in input_list]

        plt.scatter(x_values, y_values, marker='o', label='Map Coordinates')
        plt.xlabel('X Values')
        plt.ylabel('Y Values')
        plt.title('Map Coordinates in X-Y Plane')
        plt.legend()
        plt.grid(True)
        plt.show()
        
    def plot_both(self, input_list):
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
        
    def report_generator_callback(self, msg):
        if not msg.data == "Object detection node stopped!":
            return
        file_path = "src/object_detection/object_detection/test/detected_potholes.txt"
          # Adjust as needed
        input_data = self.read_from_file(file_path)  
        filtered_data = self.filter_points(input_data)
        file_path_filtered = "src/object_detection/object_detection/test/sorted_potholes.txt"
        self.write_to_file(file_path_filtered, filtered_data)
        self.plot_both(filtered_data)
        # self.plot_points(filtered_data)
        # self.plot_map_coordinates(filtered_data)
        
def main():
    # --- Init
    rclpy.init()
    report = ReportGenerator()
    
    rclpy.spin(report)
    report.destroy_node()

    # --- Shut down
    rclpy.shutdown()

if __name__ == "__main__":
    main()
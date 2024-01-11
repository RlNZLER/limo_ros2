"""
Pothole Report Generator Node

This code represents a ROS (Robot Operating System) implementation for a Pothole Detection and Reporting System.
The system includes ReportGenerator for filtering, mapping, and generating reports.

ReportGenerator:
- Subscribes to the status of the ObjectDetector.
- Filters detected potholes based on specific conditions.
- Draws markers on a map and generates a plot of filtered potholes.
- Creates a PDF report with pothole details and visualizations.

Author: Amel Varghese
Date: 11/01/2024
Github: https://github.com/RlNZLER/limo_ros2.git

"""


# ROS imports.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import geometry_msgs.msg

# Mathematical operations.
from math import sqrt

# Plotting and visualization.
import matplotlib.pyplot as plt

# PDF generation.
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas

# Image processing.
from PIL import Image
from io import BytesIO

# External processes and utilities.
import subprocess

# Numerical and image processing libraries.
import numpy as np
import cv2

# Node responsible for filtering, mapping, and generating reports based on detected potholes.
class ReportGenerator(Node):
    DISTANCE_THRESHOLD = 0.20
    DETECTED_POTHOLE_FILE_PATH = "src/object_detection/data/detected_potholes.txt"
    FILTERED_FILE_PATH = "src/report_generator/data/filtered_potholes.txt"
    PLOT_FILE_PATH = "src/report_generator/data/filtered_pothole_map.png"
    MAP_DIMENSIONS = (500, 500)  # Adjust as needed
    MAP_FILE_PATH = "src/report_generator/report_generator/potholes_20mm.pgm"
    POTHOLE_MAP_PATH = "src/report_generator/data/pothole_map.pgm"
    filtered_data = []
    
    
    def __init__(self):
        # Initializes the ReportGenerator node.
        super().__init__('report_generator')
        # Load map image
        self.map = cv2.imread(self.MAP_FILE_PATH, cv2.IMREAD_GRAYSCALE)

        # Map metadata
        self.resolution = 0.02
        self.origin = np.array([-1.51, -1.32, 0])
        self.origin_pixel = self.world_to_pixel(self.origin[0],self.origin[1])
        self.pothole_map = np.zeros(self.MAP_DIMENSIONS, dtype=np.uint8)
        self.object_detection_status_sub = self.create_subscription(String, '/object_detection/status', self.report_generator_callback, 10)
        
    # Filters detected potholes based on specific conditions.
    def filter_points(self, input_list):

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
                    sqrt((x - fp['odom_coords'].x)**2 + (y - fp['odom_coords'].y)**2) < self.DISTANCE_THRESHOLD
                    for fp in self.filtered_data
                )

                # If not a duplicate, add to the filtered list
                if not is_duplicate:
                    self.filtered_data.append(point_copy)

    # Writes filtered pothole data to a file.
    def write_to_file(self):
        try:
            with open(self.FILTERED_FILE_PATH, 'w') as file:
                for point in self.filtered_data:
                    # Convert each dictionary to a string and write to the file
                    line = ', '.join([f"'{key}': {value}" for key, value in point.items()])
                    file.write(f"{{{line}}}\n")
        except Exception as e:
            print(f"Error writing to file: {e}")

    # Reads detected pothole data from a file. Returns: List of detected potholes with coordinates.
    def read_from_file(self):
        data = []

        try:
            with open(self.DETECTED_POTHOLE_FILE_PATH, 'r') as file:
                for line in file:
                    # Evaluate each line as a dictionary and append to the list
                    point = eval(line.strip())
                    data.append(point)
        except Exception as e:
            print(f"Error reading from file: {e}")

        return data
    
    # Converts world coordinates to pixel coordinates.
    def world_to_pixel(self, x_map, y_map):
        # Convert world coordinates to pixel coordinates
        pixel_x = int((x_map - self.origin[0]) / self.resolution)
        pixel_y = int((y_map - self.origin[1]) / self.resolution)
        return pixel_x, pixel_y
    
    # Draws a marker on the map at specified coordinates.
    def draw_marker(self, x_map, y_map, marker_size=1):
        # Convert world coordinates to pixel coordinates
        pixel_coords = self.world_to_pixel(x_map*-1, y_map)

        # Ensure the pixel coordinates are within the map dimensions
        pixel_x = max(0, min(pixel_coords[0], self.map.shape[1] - 1))
        pixel_y = max(0, min(pixel_coords[1], self.map.shape[0] - 1))

        # Draw a filled circle around the specified pixel
        cv2.circle(self.map, (pixel_x, pixel_y), marker_size, 0, thickness=-1)

    # Saves the map image with markers.
    def save_map_with_markers(self):
        rotated_image = cv2.rotate(self.map, cv2.ROTATE_180)
        cv2.imwrite(self.POTHOLE_MAP_PATH, rotated_image)
        
    # Plots filtered pothole odom coordinates in the X-Y plane.
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

    # Plots map coordinates in the X-Y plane.
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
        
    # Plots both odom and map coordinates in the X-Y plane.
    def plot_both(self, save_path=None):
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))

        x_values_filtered = [point['odom_coords'].x for point in self.filtered_data]
        y_values_filtered = [point['odom_coords'].y for point in self.filtered_data]

        axs[0].scatter(x_values_filtered, y_values_filtered, marker='o', label='Odom Coordinates')
        axs[0].set_xlabel('X Values')
        axs[0].set_ylabel('Y Values')
        axs[0].set_title('Odom Coordinates in X-Y Plane')
        axs[0].legend()
        axs[0].grid(True)
        for idx, point in enumerate(self.filtered_data):
            axs[0].annotate(idx + 1, (point['odom_coords'].x, point['odom_coords'].y), textcoords="offset points", xytext=(5,5), ha='center')
            
        x_values_map = [point['x_map'] for point in self.filtered_data]
        y_values_map = [point['y_map'] for point in self.filtered_data]

        axs[1].scatter(x_values_map, y_values_map, marker='o', label='Map Coordinates')
        axs[1].set_xlabel('X Values')
        axs[1].set_ylabel('Y Values')
        axs[1].set_title('Map Coordinates in X-Y Plane')
        axs[1].legend()
        axs[1].grid(True)
        for idx, point in enumerate(self.filtered_data):
            axs[1].annotate(idx + 1, (point['x_map'], point['y_map']), textcoords="offset points", xytext=(5,5), ha='center')
            
        plt.tight_layout()
        plt.savefig(self.PLOT_FILE_PATH)

    # Generates a PDF report with pothole details and visualizations.
    def generate_pdf_report(self):
        # Create PDF
        pdf_buffer = BytesIO()
        pdf = canvas.Canvas(pdf_buffer, pagesize=letter)
        
        # Write report content
        pdf.setFont("Helvetica", 12)
        pdf.drawString(100, 750, "Report Summary")
        pdf.drawString(100, 730, f"Number of Potholes: {len(self.filtered_data)}")

        # Iterate over filtered data
        pdf.drawString(100, 710, "Pothole Details:")
        y_position = 690
        page_height = 750
        potholes_per_page = 7

        for idx, point in enumerate(self.filtered_data):
            odom_coords = f"Odom Coordinates: ({round(point['odom_coords'].x,3)}, {round(point['odom_coords'].y,3)})"
            map_coords = f"Map Coordinates: ({round(point['x_map'],3)}, {round(point['y_map'],3)})"
            pothole_size = f"Pothole Size: {round(point['box_size'],3)} sq. meters"

            pdf.drawString(120, y_position, f"Pothole {idx + 1}:")
            pdf.drawString(140, y_position - 20, odom_coords)
            pdf.drawString(140, y_position - 40, map_coords)
            pdf.drawString(140, y_position - 60, pothole_size)

            y_position -= 80

            if (idx + 1) % potholes_per_page == 0 and idx + 1 != len(self.filtered_data):
                # Create a new page
                pdf.showPage()
                page_height = 750
                y_position = page_height

        # Open the image using PIL
        img = Image.open(self.PLOT_FILE_PATH)

        # Calculate scaling factors to fit the image within the page.
        image_width, image_height = img.size
        max_width, max_height = letter
        scale_x = max_width / image_width
        scale_y = max_height / image_height
        scale = min(scale_x, scale_y) - 0.05

        # Calculate the new dimensions for the scaled image.
        new_width = image_width * scale
        new_height = image_height * scale

        # Calculate the position to center the scaled image on a new page.
        x_position = (max_width - new_width) / 2
        y_position = (max_height - new_height) / 2

        # Draw the heading for the page with the plots.
        pdf.showPage()
        pdf.drawString(100, max_height - 50, "Pothole Position Plots")

        # Calculate the position for the first half of the page.
        plots_y_position = max_height - 300

        # Draw the plots on the first half of the page.
        pdf.drawInlineImage(Image.open(self.PLOT_FILE_PATH), x_position, plots_y_position, width=new_width, height=new_height)

        # Draw the heading for the map section.
        pdf.drawString(100, max_height - 350, "Pothole Position on Map")

        # Calculate the position for the second half of the page.
        map_y_position = max_height - 600

        # Draw the map on the second half of the page.
        map_image_path = "src/report_generator/data/pothole_map.pgm"
        pdf.drawInlineImage(Image.open(map_image_path), x_position, map_y_position, width=new_width, height=new_height)

        pdf.save()
        pdf_buffer.seek(0)

        return pdf_buffer
    
    # Callback function to check if the object detection node is stopped to start report generation.
    def report_generator_callback(self, msg):
        # Check if the object detection node is stopped.
        if not msg.data == "Object detection node stopped!":
            return

        # Read the data from the object detection node.
        input_data = self.read_from_file()  
        self.filter_points(input_data)
        self.write_to_file()
        
        # Plot both odom coordinates and map coordinates.
        self.plot_both()
        # self.plot_points(filtered_data)
        # self.plot_map_coordinates(filtered_data)
        
        # Iterate through filtered data and draw markers on the PGM map
        for point in self.filtered_data:
            self.draw_marker(point['x_map'], point['y_map'])
            
        self.save_map_with_markers()
        # Generate PDF report
        pdf_report = self.generate_pdf_report()

        # Save the PDF to a file.
        with open("Report Summary.pdf", "wb") as f:
            f.write(pdf_report.read())
        
        # Open the generated "Report Summary.pdf".
        subprocess.run(["xdg-open", "Report Summary.pdf"])

# Main function to initialize and run the report generator node.
def main():
    # --- Initialize ROS
    rclpy.init()
    report = ReportGenerator()
    
    rclpy.spin(report)
    report.destroy_node()

    # --- Shut down ROS
    rclpy.shutdown()

if __name__ == "__main__":
    main()
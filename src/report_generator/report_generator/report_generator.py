import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import geometry_msgs.msg
from math import sqrt
import matplotlib.pyplot as plt
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from PIL import Image
from io import BytesIO
import subprocess

class ReportGenerator(Node):
    DISTANCE_THRESHOLD = 0.3
    DETECTED_POTHOLE_FILE_PATH = "src/object_detection/data/detected_potholes.txt"
    FILTERED_FILE_PATH = "src/report_generator/data/filtered_potholes.txt"
    PLOT_FILE_PATH = "src/report_generator/data/filtered_pothole_map.png"
    filtered_data = []
    
    def __init__(self):
        super().__init__('report_generator')
        self.object_detection_status_sub = self.create_subscription(String, '/object_detection/status', self.report_generator_callback, 10)
            
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

    def write_to_file(self):
        try:
            with open(self.FILTERED_FILE_PATH, 'w') as file:
                for point in self.filtered_data:
                    # Convert each dictionary to a string and write to the file
                    line = ', '.join([f"'{key}': {value}" for key, value in point.items()])
                    file.write(f"{{{line}}}\n")
        except Exception as e:
            print(f"Error writing to file: {e}")

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
        page_height = 750  # Initial page height, adjust as needed
        potholes_per_page = 7  # Adjust as needed

        for idx, point in enumerate(self.filtered_data):
            odom_coords = f"Odom Coordinates: ({point['odom_coords'].x}, {point['odom_coords'].y})"
            map_coords = f"Map Coordinates: ({point['x_map']}, {point['y_map']})"
            pothole_size = f"Pothole Size: {point['box_size']}"

            pdf.drawString(120, y_position, f"Pothole {idx + 1}:")
            pdf.drawString(140, y_position - 20, odom_coords)
            pdf.drawString(140, y_position - 40, map_coords)
            pdf.drawString(140, y_position - 60, pothole_size)

            y_position -= 80

            if (idx + 1) % potholes_per_page == 0 and idx + 1 != len(self.filtered_data):
                # Create a new page
                pdf.showPage()
                page_height = 750  # Reset page height for the new page
                y_position = page_height

        # Open the image using PIL
        img = Image.open(self.PLOT_FILE_PATH)

        # Calculate scaling factors to fit the image within the page
        image_width, image_height = img.size
        max_width, max_height = letter
        scale_x = max_width / image_width
        scale_y = max_height / image_height
        scale = min(scale_x, scale_y) - 0.05

        # Calculate the new dimensions for the scaled image
        new_width = image_width * scale
        new_height = image_height * scale

        # Calculate the position to center the scaled image on a new page
        x_position = (max_width - new_width) / 2
        y_position = (max_height - new_height) / 2

        # Draw the heading for the page with the image
        pdf.showPage()
        pdf.drawString(100, max_height - 50, "Pothole Position on Map")

        # Draw the scaled image on a new page
        pdf.drawInlineImage(img, x_position, y_position, width=new_width, height=new_height)

        pdf.save()
        pdf_buffer.seek(0)

        return pdf_buffer
    
    def report_generator_callback(self, msg):
        if not msg.data == "Object detection node stopped!":
            return

          # Adjust as needed
        input_data = self.read_from_file()  
        self.filter_points(input_data)
        self.write_to_file()
        
        plot_file_path = "src/report_generator/data/filtered_pothole_map.png"
        self.plot_both()
        # self.plot_points(filtered_data)
        # self.plot_map_coordinates(filtered_data)

        # Generate PDF report
        pdf_report = self.generate_pdf_report()

        # Now, you can save the PDF to a file or send it through ROS, depending on your needs.
        with open("Report Summary.pdf", "wb") as f:
            f.write(pdf_report.read())
            
        subprocess.run(["xdg-open", "Report Summary.pdf"])

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
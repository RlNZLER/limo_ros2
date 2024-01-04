import geometry_msgs.msg
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from PIL import Image
from io import BytesIO
import subprocess


def generate_pdf_report(filtered_data, image_path):
    # Create PDF
    pdf_buffer = BytesIO()
    pdf = canvas.Canvas(pdf_buffer, pagesize=letter)
    
    # Write report content
    pdf.setFont("Helvetica", 12)
    pdf.drawString(100, 750, "Report Summary")
    pdf.drawString(100, 730, f"Number of Potholes: {len(filtered_data)}")

    # Iterate over filtered data
    pdf.drawString(100, 710, "Pothole Details:")
    y_position = 690
    page_height = 750  # Initial page height, adjust as needed
    potholes_per_page = 7  # Adjust as needed

    for idx, point in enumerate(filtered_data):
        odom_coords = f"Odom Coordinates: ({point['odom_coords'].x}, {point['odom_coords'].y})"
        map_coords = f"Map Coordinates: ({point['x_map']}, {point['y_map']})"
        pothole_size = f"Pothole Size: {point['box_size']}"

        pdf.drawString(120, y_position, f"Pothole {idx + 1}:")
        pdf.drawString(140, y_position - 20, odom_coords)
        pdf.drawString(140, y_position - 40, map_coords)
        pdf.drawString(140, y_position - 60, pothole_size)

        y_position -= 80

        if (idx + 1) % potholes_per_page == 0 and idx + 1 != len(filtered_data):
            # Create a new page
            pdf.showPage()
            page_height = 750  # Reset page height for the new page
            y_position = page_height

    # Open the image using PIL
    img = Image.open(image_path)

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

filtered_data = []
file_path = "src/report_generator/data/filtered_potholes.txt"
with open(file_path, 'r') as file:
    for line in file:
        point = eval(line.strip())
        filtered_data.append(point)

# Path to the static image
image_path = "src/report_generator/data/filtered_pothole_map.png"

# Generate PDF report
pdf_report = generate_pdf_report(filtered_data, image_path)

# Now, you can save the PDF to a file or send it through ROS, depending on your needs.
with open("Report Summary.pdf", "wb") as f:
    f.write(pdf_report.read())
    
subprocess.run(["xdg-open", "Report Summary.pdf"])
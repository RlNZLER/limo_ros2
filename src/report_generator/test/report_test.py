import geometry_msgs.msg
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from PIL import Image
from io import BytesIO

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
    for idx, point in enumerate(filtered_data):
        odom_coords = f"Odom Coordinates: ({point['odom_coords'].x}, {point['odom_coords'].y})"
        map_coords = f"Map Coordinates: ({point['x_map']}, {point['y_map']})"
        pothole_size = f"Pothole Size: {point['box_size']}"

        pdf.drawString(120, y_position, f"Pothole {idx + 1}:")
        pdf.drawString(140, y_position - 20, odom_coords)
        pdf.drawString(140, y_position - 40, map_coords)
        pdf.drawString(140, y_position - 60, pothole_size)

        y_position -= 80

    # Open the image using PIL
    img = Image.open(image_path)

    # Draw the image using drawInlineImage
    pdf.drawString(100, y_position, "Generated Plot:")
    pdf.drawInlineImage(img, 100, y_position - 20, width=400, height=300)

    pdf.save()
    pdf_buffer.seek(0)

    return pdf_buffer

filtered_data = []
file_path = "src/object_detection/object_detection/test/sorted_potholes.txt"
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

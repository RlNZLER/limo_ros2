import cv2
import numpy as np

class MarkerOverlay:
    def __init__(self, map_info):
        # Load map image
        self.map = cv2.imread(map_info["image"], cv2.IMREAD_GRAYSCALE)

        # Map metadata
        self.resolution = map_info["resolution"]
        self.origin = np.array(map_info["origin"])
        self.origin_pixel = self.world_to_pixel(self.origin)

    def world_to_pixel(self, world_coords):
        # Convert world coordinates to pixel coordinates
        pixel_x = int((world_coords[0] - self.origin[0]) / self.resolution)
        pixel_y = int((world_coords[1] - self.origin[1]) / self.resolution)
        return pixel_x, pixel_y

    def draw_marker(self, world_coords, marker_size=1):
        # Convert world coordinates to pixel coordinates
        pixel_coords = self.world_to_pixel(world_coords)

        # Ensure the pixel coordinates are within the map dimensions
        pixel_x = max(0, min(pixel_coords[0], self.map.shape[1] - 1))
        pixel_y = max(0, min(pixel_coords[1], self.map.shape[0] - 1))

        # Draw a filled circle around the specified pixel
        cv2.circle(self.map, (pixel_x, pixel_y), marker_size, 0, thickness=-1)

    def save_map_with_markers(self, output_file_path):
        cv2.imwrite(output_file_path, self.map)

# Example usage
map_info = {
    "image": "src/limo_navigation/maps/potholes_20mm.pgm",
    "resolution": 0.02,
    "origin": [-1.51, -1.32, 0],
}

output_file_path = "src/report_generator/data/pothole_map_test.pgm"

marker_overlay = MarkerOverlay(map_info)

# Add markers to the map (replace these coordinates with your desired marker locations)
marker_overlay.draw_marker([0.0, 0.0, 0])
marker_overlay.draw_marker([0.5, 0.5, 0])

# Save the map with markers
marker_overlay.save_map_with_markers(output_file_path)

import os
import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def process_directory(label_data_dir, lidar_data_dir, save_data_path):
    # Get lists of label and lidar files in the specified directories
    label_files = [f for f in os.listdir(label_data_dir) if f.endswith('.json')]
    lidar_files = [f for f in os.listdir(lidar_data_dir) if f.endswith('.bin')]

    # Iterate through label files
    for label_file in label_files:
        label_path = os.path.join(label_data_dir, label_file)

        # Find corresponding lidar file for the current label file
        lidar_file = find_lidar_file(label_file, lidar_files)
        if lidar_file:
            lidar_path = os.path.join(lidar_data_dir, lidar_file)
            open_and_save_dir(label_path, lidar_path, save_data_path)

def find_lidar_file(label_file, lidar_files):
    base_name = os.path.splitext(label_file)[0]
    for lidar_file in lidar_files:
        if base_name in lidar_file:
            return lidar_file
    return None

def read_lidar_data(file_path):
    # Read lidar data from binary file
    lidar_data = np.fromfile(file_path, dtype=np.float32)
    lidar_data = lidar_data[:len(lidar_data)//3*3].reshape(-1,3)
    return lidar_data

def open_and_save_dir(label_path, lidar_path, output_dir):
    lidar_data = read_lidar_data(lidar_path)

    # Read label data from JSON file
    with open(label_path, 'r') as json_file:
        labels_data = json.load(json_file)
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    frame_number = os.path.splitext(os.path.basename(label_path))[0]
    
    # Iterate through objects in the label data
    for label in labels_data:
        extracted_points_data(label, lidar_data, output_dir, frame_number)

def extracted_points_data(label, lidar_data, output_dir, frame_number):
    object_id = label['obj_id']
    category = label['obj_type']

    print(f"Object ID: {object_id}")
    print(f"Category: {category}")

    if category:
        psr = label.get('psr')
        if psr is not None:
            obj_dir = os.path.join(output_dir, f"{object_id}")
            # frame_name = os.path.splitext(os.path.basename(label_path))[0]
            frame_dir = os.path.join(obj_dir, frame_number)

            os.makedirs(obj_dir, exist_ok=True)
            os.makedirs(frame_dir, exist_ok=True)

            position, rotation, scale = get_psr_values(psr)
            rotated_lidar_data = apply_rotation(lidar_data, psr)

            min_bbox, max_bbox = get_bounding_box(position, scale)

            mask = np.all((rotated_lidar_data >= min_bbox) & (rotated_lidar_data <= max_bbox), axis=1)
            filtered_lidar_data = lidar_data[mask]

            # visualize_lidar_data(filtered_lidar_data)

            bbox_data = {
                "position": position.tolist(),
                "rotation": rotation.tolist(),
                "scale": scale.tolist()
            }

            save_data_to_files(object_id, frame_dir, filtered_lidar_data, bbox_data, category)

def get_psr_values(psr):
    # Extract position, rotation, and scale values from psr
    position = np.array([psr["position"]["x"], psr["position"]["y"], psr["position"]["z"]])
    rotation = np.array([psr["rotation"]["x"], psr["rotation"]["y"], psr["rotation"]["z"]])
    scale = np.array([psr["scale"]["x"], psr["scale"]["y"], psr["scale"]["z"]])
    return position, rotation, scale

def apply_rotation(lidar_data, psr):
    # Apply rotation to lidar data based on psr values
    rotation_matrix = Rotation.from_euler('zyx', [psr["rotation"]["z"], psr["rotation"]["y"], psr["rotation"]["x"]], degrees=True).as_matrix()
    return lidar_data @ rotation_matrix.T

def get_bounding_box(position, scale):
    # Calculate min and max bounding box values
    min_bbox = position - 0.5 * scale
    max_bbox = position + 0.5 * scale
    return min_bbox, max_bbox

def visualize_lidar_data(lidar_data):
    # Visualize lidar data 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(lidar_data[:, 0], lidar_data[:, 1], lidar_data[:, 2], s=1, c='b', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def save_data_to_files(object_id, frame_dir, filtered_lidar_data, bbox_data, category):
    # Save filtered lidar data and bounding box data to files
    i = 0
    while os.path.exists(os.path.join(frame_dir, f"points_{object_id}_{i}.bin")):
        i += 1

    points_path = os.path.join(frame_dir, f"points_{object_id}_{i}.bin")
    filtered_lidar_data.tofile(points_path)
    bbox_path = os.path.join(frame_dir, f"bbox_{object_id}_{i}.json")
    with open(bbox_path, 'w') as bbox_file:
        json.dump(bbox_data, bbox_file)
    
    print(f"Filtered lidar data for object {category} has been saved.")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    label_dir = 'data/labeled_lidar/label'
    lidar_dir = 'data/labeled_lidar/lidar'
    save_dir = 'data/labeled_lidar' 
    parser.add_argument('--label_dir', type=str, default=label_dir, help='Directory labeling result file (.json)')
    parser.add_argument('--lidar_dir', type=str, default=lidar_dir, help='Directory LiDAR data file (.bin)')
    parser.add_argument('--save_dir', type=str, default=save_dir, help='Directory to save points.bin and bbox.json')
    args = parser.parse_args()

    # Process the provided directories
    process_directory(args.label_dir, args.lidar_dir, args.save_dir)

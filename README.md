# LiDAR-Point-Extraction

## Introduction
This project focuses on extracting valuable information from labeled LiDAR frames. Preceded by a labeling process, where LiDAR frames are annotated with bounding boxes defining objects, this extraction tool dives into the labeled data to derive insights. Leveraging JSON files containing position, scale, and rotation attributes for each bounding box, the script filters corresponding LiDAR point cloud data (.bin files) based on the provided parameters. The primary aim is to isolate and extract points within each bounding box, offering precise object-specific data for further analysis and application.

## Steps
1. **Labeling Process**: Before using the extraction script, ensure that your LiDAR frames have undergone a labeling process where objects of interest are annotated with bounding boxes.
2. **Data Preparation**: Organize your labeled LiDAR data, including JSON files containing bounding box annotations and corresponding binary (.bin) files containing point cloud data from each frames.
3. **Script Execution**: Run the Python script, providing the path to the labeled data. The script will parse the JSON files, extract bounding box information, and filter the point cloud data accordingly.
4. **Output**: Upon execution, the script generates object-specific data files containing points within each bounding box, ready for further analysis and application.

By automating the extraction of points within bounding boxes and generating object-specific data, the script facilitates advanced analysis and applications in domains such as autonomous driving, robotics, and environmental monitoring. With its efficiency and scalability, this tool serves as a valuable asset in the LiDAR data processing pipeline.

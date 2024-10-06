import numpy as np
import open3d as o3d
import argparse
import os

def read_matrices(file_path):
    matrices = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        values = []
        for line in lines:
            if line.strip():  # Ignore empty lines
                values.extend(map(float, line.strip().split()))
        
        # Print total number of values read
        print(f"Total values read: {len(values)}")
        
        # Check if the number of values is a multiple of 12
        if len(values) % 12 != 0:
            raise ValueError(f"Number of values in the file is not a multiple of 12.")
        
        # Split values into matrices
        num_matrices = len(values) // 12
        print(f"Expected number of matrices: {num_matrices}")
        
        for i in range(num_matrices):
            matrix_values = values[i*12:(i+1)*12]
            matrix = np.array(matrix_values).reshape(3, 4)
            matrices.append(matrix)
            print(f"Matrix {i+1}: \n{matrix}\n")
    
    return matrices

def apply_transformation(pcd, transformation_matrix):
    pcd.transform(transformation_matrix)

def save_trajectory(matrices, file_path):
    # Create a list of points for the trajectory
    trajectory_points = []
    for matrix in matrices:
        # Extract the translation vector from the 3x4 matrix
        translation = matrix[:, 3]
        trajectory_points.append(translation)
    
    # Convert to numpy array
    trajectory_points = np.array(trajectory_points)
    
    # Create a PointCloud object to save trajectory
    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(trajectory_points)
    
    # Save trajectory to PLY file
    o3d.io.write_point_cloud(file_path, trajectory_pcd)

def process_pcd_files(folder_path, matrices, trajectory_ply_path, transformed_folder):
    pcd_files = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.pcd')])
    
    if len(pcd_files) != len(matrices):
        raise ValueError("The number of PCD files does not match the number of matrices.")
    
    if not os.path.exists(transformed_folder):
        os.makedirs(transformed_folder)  # Create the folder if it doesn't exist

    for i, pcd_file in enumerate(pcd_files):
        # Read the PCD file
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        # Get the corresponding transformation matrix
        matrix = matrices[i]
        
        # Create a 4x4 transformation matrix from the 3x4 matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :4] = matrix
        
        # Apply the transformation to the PCD
        apply_transformation(pcd, transformation_matrix)
        
        # Save the transformed PCD to the output folder with zero-padded filenames
        transformed_pcd_file = os.path.join(transformed_folder, f"{i:05d}.pcd")
        o3d.io.write_point_cloud(transformed_pcd_file, pcd)

    # Save the trajectory to a PLY file
    if trajectory_ply_path:
        save_trajectory(matrices, trajectory_ply_path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Transform PCD files with given matrices and save them with zero-padded names.")
    parser.add_argument('matrix_file_path', type=str, help='Path to the text file containing the transformation matrices.')
    parser.add_argument('folder_path', type=str, help='Path to the folder containing the PCD files.')
    parser.add_argument('trajectory_ply_path', type=str, default=None, help='Path to save the trajectory PLY file.')
    parser.add_argument('transformed_folder', type=str, help='Folder to save each transformed PCD file.')
    
    args = parser.parse_args()
    
    # Read the matrices from the text file
    matrices = read_matrices(args.matrix_file_path)
    
    # Process each PCD file with the corresponding matrix and save the transformed PCDs
    process_pcd_files(args.folder_path, matrices, args.trajectory_ply_path, args.transformed_folder)

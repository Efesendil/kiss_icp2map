import open3d as o3d
import os
import argparse

def combine_transformed_pcds(folder_path, output_pcd_path=None):
    # Get all the PCD files in the folder, sorted in ascending order
    pcd_files = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.pcd')])
    
    if not pcd_files:
        raise ValueError(f"No PCD files found in folder: {folder_path}")
    
    combined_pcd = o3d.geometry.PointCloud()  # Initialize an empty PointCloud object

    for i, pcd_file in enumerate(pcd_files):
        print(f"Loading PCD file {i+1}/{len(pcd_files)}: {pcd_file}")
        
        # Read the PCD file
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        # Add it to the combined point cloud
        combined_pcd += pcd

    # Visualize the combined point cloud
    o3d.visualization.draw_geometries([combined_pcd], window_name="Combined Transformed PCDs")

    # Optionally, save the combined point cloud to a file
    if output_pcd_path:
        o3d.io.write_point_cloud(output_pcd_path, combined_pcd)
        print(f"Combined point cloud saved to: {output_pcd_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Combine transformed PCD files into a single point cloud.")
    parser.add_argument('folder_path', type=str, help='Path to the folder containing the transformed PCD files.')
    parser.add_argument('--output_pcd_path', type=str, default=None, help='Optional path to save the combined point cloud.')

    args = parser.parse_args()

    # Combine the transformed PCDs and optionally save the result
    combine_transformed_pcds(args.folder_path, args.output_pcd_path)


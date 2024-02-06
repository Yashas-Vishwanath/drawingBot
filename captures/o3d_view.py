import open3d as o3d

# Load PCD file
pcd = o3d.io.read_point_cloud("scan03.ply")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])

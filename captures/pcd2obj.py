# import open3d as o3d

# def convert_pcd_to_ply_or_obj(input_file, output_file, format="ply"):
#     # Load PCD file
#     pcd = o3d.io.read_point_cloud(input_file)

#     # Save to PLY or OBJ
#     if format.lower() == "ply":
#         o3d.io.write_point_cloud(output_file, pcd)
#     else:
#         print("Invalid output format. Supported formats: PLY")

# # Example usage:
# input_file = "/home/yashas/mapping_tbot/test_01.pcd"
# output_file = "/home/yashas/mapping_tbot/test_01.ply"  # Change the extension to .ply for PLY format

# convert_pcd_to_ply_or_obj(input_file, output_file)



import open3d as o3d

def convert_pcd_to_ply(input_file, output_ply):
    # Load PCD file
    pcd = o3d.io.read_point_cloud(input_file)

    # Save as PLY
    o3d.io.write_point_cloud(output_ply, pcd)

def convert_ply_to_obj(output_ply, output_obj):
    # Load PLY file
    ply = o3d.io.read_point_cloud(output_ply)

    # Convert to OBJ
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(ply)

    # Save as OBJ
    o3d.io.write_triangle_mesh(output_obj, mesh)

# Example usage:
input_file = "/home/yashas/mapping_tbot/test_01.pcd"
output_ply = "/home/yashas/mapping_tbot/test_01.ply"
output_obj = "/home/yashas/mapping_tbot/test_01.obj"

# Convert PCD to PLY
convert_pcd_to_ply(input_file, output_ply)

# Convert PLY to OBJ
convert_ply_to_obj(output_ply, output_obj)

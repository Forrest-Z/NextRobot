import time
import open3d as o3d
import numpy as np
# import matplotlib.pyplot as plt

import argparse

# Construct the argument parser and parse the arguments
arg_desc = '''\
        Let's load a PCD filename from the command line!
        --------------------------------
            This program loads a PCD file
        '''
parser = argparse.ArgumentParser(formatter_class = argparse.RawDescriptionHelpFormatter,
                                 description= arg_desc)

parser.add_argument("-i", "--input", metavar="INPUT_PCD", required=True, help = "Path to your input PCD")
parser.add_argument("-o", "--output", metavar="OUTPUT_PCD", required=True, help = "Path to your output PCD")
args = vars(parser.parse_args())

#打印姓名

if (__name__ == "__main__"):
    print("Testing IO for point cloud ...")
    pcd = o3d.io.read_point_cloud(args["input"])

    m = np.asarray(pcd.points)
    if m.size == 0:
        print("Exit with empty file")
        exit(0)
    print(pcd)
    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    picked_points = vis.get_picked_points()
    print(picked_points)
    index = [p.index for p in picked_points]
    pcd2 = pcd.select_by_index(index)
    m = np.asarray(pcd2.points)
    if m.size == 0:
        print("Exit with empty file")
        exit(0)
    o3d.visualization.draw_geometries([pcd2])
    o3d.io.write_point_cloud(args["output"], pcd2, True)
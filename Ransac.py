#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import math
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d


class RANSAC:
    """
    RANSAC Class
    """
    def __init__(self, point_cloud, max_iterations, distance_ratio_threshold):
        self.point_cloud = point_cloud
        self.max_iterations = max_iterations
        self.distance_ratio_threshold = distance_ratio_threshold

    def run(self):
        """
        method for running the class directly
        :return:
        """
        inliers, outliers = self._ransac_algorithm(self.max_iterations, self.distance_ratio_threshold)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(inliers.X, inliers.Y,  inliers.Z,  c="green")
        ax.scatter(outliers.X, outliers.Y, outliers.Z, c="red")
        plt.show()

    def _visualize_point_cloud(self):
        """
        Visualize the 3D data
        :return: None
        """
        # Visualize the point cloud data
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(self.point_cloud.X, self.point_cloud.Y, self.point_cloud.Z)		
        plt.show()
		
    def _ransac_algorithm(self, max_iterations, distance_ratio_threshold):
        """
        Implementation of the RANSAC logic
        :return: inliers(Dataframe), outliers(Dataframe)
        
        """
        point_cloud['Distance'] = 0
        inliers_result = set()
        while max_iterations:
                max_iterations -= 1
                # Add 3 random indexes
                random.seed()
                inliers = []
                inliers= random.sample(range(0,len(point_cloud.X)-1),3)
                try:
                    # In case of *.xyz data
                    x1, y1, z1, _, _, _ = point_cloud.loc[inliers[0]]
                    x2, y2, z2, _, _, _ = point_cloud.loc[inliers[1]]
                    x3, y3, z3, _, _, _ = point_cloud.loc[inliers[2]]

                except:
                    
                    # In case of *.pcd data
                    x1, y1, z1, _= point_cloud.loc[inliers[0]]
                    x2, y2, z2, _= point_cloud.loc[inliers[1]]
                    x3, y3, z3, _ = point_cloud.loc[inliers[2]]
                    
                # Plane Equation --> ax + by + cz + d = 0
                # Value of Constants for inlier plane
                a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
                b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
                c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
                d = -(a*x1 + b*y1 + c*z1)
                plane_lenght = max(0.1, math.sqrt(a*a + b*b + c*c))
                point_cloud['Distance'] = point_cloud.apply(lambda row: math.fabs(a*row.X + b*row.Y + c*row.Z +d)/plane_lenght, axis = 1)
                point_cloud_filtered=point_cloud
                
                point_cloud_filtered['Distance'][inliers] = np.nan
                point_cloud_filtered.dropna()
                point_cloud_filtered = point_cloud_filtered[point_cloud_filtered['Distance'] <= distance_ratio_threshold]
                inliers = inliers + point_cloud_filtered.index.values.tolist()
                
                if len(inliers) > len(inliers_result):                    
                    inliers_result.clear()
                    inliers_result = inliers
                
                inlier_points = pd.DataFrame(columns={"X", "Y", "Z"})
                outlier_points = pd.DataFrame(columns={"X", "Y", "Z"})
                inlier_points = point_cloud.loc[inliers_result, ['X', 'Y','Z']] 
                outlier_points = point_cloud.loc[(point_cloud[~point_cloud.index.isin(inliers_result)].index.values.tolist()),['X', 'Y','Z']]                           
                    
            
        

        return inlier_points, outlier_points
    
if __name__ == "__main__":
    # Read the point cloud data
    point_cloudd = pd.read_csv("point_cloud_data_sample.xyz", delimiter=" ", nrows=500)
    
    point_cloud= pd.DataFrame(point_cloudd, columns={"X" ,"Y" ,"Z"})
    
    APPLICATION = RANSAC(point_cloud, max_iterations=50, distance_ratio_threshold=1)
    APPLICATION.run()

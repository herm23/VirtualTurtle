#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cmath>
#include <iostream>

namespace utils {

    /**********/
    /* BASICS */
    /**********/

    /* Custom data structore representing a cluster
     * namely a set of points, with a centroid.
     * Type represents the classification assigned with PCA.
     */
    struct Cluster {
        std::vector<cv::Point2f> points;
        cv::Point2f centroid;       // this is the centre if the cluster is a circle
        char type='u';              //'c'=circle, 'l'=line, 'u'=unassigned 
        float radius=0.0f;
    };

    /* Function that converts data from laserscans
     * into a set of 2D points in a cartesian frame
     */
    void lidar2pts(const sensor_msgs::msg::LaserScan& scan, std::vector<cv::Point2f>& points);

    /* Function that converts a cluster into a matrix of points
     * specifically an Nx2 one to work with opencv linalg routines.
     * The matrix is NOT a binary image.
     */
    void cluster2mat(const Cluster& cls, cv::Mat& mat);


    /**************/
    /* CLUSTERING */
    /**************/
    
    /* Function that performs simple, distance based clustering
     * using O(N^2) cv::partition on a point array
     */
    std::vector<Cluster> cluster_points(const std::vector<cv::Point2f>& points, const float threshold=0.05f);

    /* Function that computes cluster centroids
     * given a vector of clusters using smart matrix operations
     */
    void compute_centroids(std::vector<Cluster>& clusters);

    /* Function that refines clusters removing noise
     * and small clusters, or those whose centroid
     * is too close (inside the robot)
     */
    void refine_clusters(std::vector<Cluster>& clusters, const float min_points = 3, const float min_distance = 0.1f);

    /* Function that performs circle detection by fitting
     * a circle with least squares, rejecting data if bad fit
    */
    void detect_circles(std::vector<Cluster>& clusters, float max_radius = 1.0f, float max_residual = 0.02f);

    /* Function that chains the pipeline together
     * to fully process a single scan
     */
    std::vector<Cluster> process_scan(const sensor_msgs::msg::LaserScan& scan,
                                      const float cluster_threshold = 0.3f,
                                      const int min_points = 3,
                                      const float min_distance = 0.1f,
                                      const float max_radius = 1.0f,
                                      const float max_residual = 0.05f);

    /* Semi-horrible function used for debug only to show clusters
     * on an image, each with its own color;
     */
    void visualize_clusters(const std::vector<Cluster>& clusters, cv::Mat& output, int image_size = 800, float scale = 100.0f);

}  // namespace utils

#endif  // UTILITIES_HPP

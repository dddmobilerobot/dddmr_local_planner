/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef BASE_TRAJECTORY_H_
#define BASE_TRAJECTORY_H_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace base_trajectory {

  typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> cuboid_min_max_t;
  /**
   * @class Trajectory
   * @brief Holds a trajectory generated by considering an x, y, and theta velocity
   */
  class Trajectory {
    public:
      /**
       * @brief  Default constructor
       */
      Trajectory();

      /**
       * @brief  Constructs a trajectory
       * @param xv The x velocity used to seed the trajectory
       * @param thetav The theta velocity used to seed the trajectory
       * @param num_pts The expected number of points for a trajectory
       */
      Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts);

      double xv_, yv_, thetav_; ///< @brief The x, y, and theta velocities of the trajectory

      double cost_; ///< @brief The cost/score of the trajectory

      double time_delta_; ///< @brief The time gap between points

      /**
       * @brief  Get a point within the trajectory
       * @param index The index of the point to get
       * @param x Will be set to the x position of the point
       * @param y Will be set to the y position of the point
       * @param th Will be set to the theta position of the point
       */
      geometry_msgs::msg::PoseStamped getPoint(unsigned int index) const;
      pcl::PointXYZI getPCLPoint(unsigned int index) const;

      /**
       * @brief  Set a point within the trajectory
       * @param index The index of the point to set
       * @param x The x position
       * @param y The y position
       * @param th The theta position
       */
      void setPoint(unsigned int index, double x, double y, double th);

      /**
       * @brief  Add a point to the end of a trajectory
       */
      bool addPoint(const geometry_msgs::msg::PoseStamped& pos, 
                    const pcl::PointCloud<pcl::PointXYZ>& cuboid,
                    const cuboid_min_max_t& cuboid_min_max);

      pcl::PointCloud<pcl::PointXYZ> getCuboid(unsigned int index) const;

      cuboid_min_max_t getCuboidMinMax(unsigned int index) const;

      /**
       * @brief  Get the last point of the trajectory
       * @param x Will be set to the x position of the point
       * @param y Will be set to the y position of the point
       * @param th Will be set to the theta position of the point
       */
      void getEndpoint(double& x, double& y, double& th) const;

      /**
       * @brief  Clear the trajectory's points
       */
      void resetPoints();

      /**
       * @brief  Return the number of points in the trajectory
       * @return The number of points in the trajectory
       */
      unsigned int getPointsSize() const;

    private:
      nav_msgs::msg::Path trajectory_path_;
      // cuboid of trajectory pose
      std::vector<pcl::PointCloud<pcl::PointXYZ>> cuboids_;
      //min max value for collision model
      std::vector<cuboid_min_max_t> cuboids_min_max_;

      pcl::PointCloud<pcl::PointXYZI> pcl_trajectory_path_;

  };
};
#endif

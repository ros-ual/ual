//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef UAL_CORE_BACKEND_H
#define UAL_CORE_BACKEND_H

#include <thread>
#include <atomic>
#include <mutex>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ual_core/State.h>
#include <sensor_msgs/BatteryState.h>
#include <ros/ros.h>

namespace ual {

typedef geometry_msgs::PoseStamped      Pose;
typedef geometry_msgs::PoseStamped      Waypoint;
typedef geographic_msgs::GeoPoseStamped PoseGeo;
typedef geographic_msgs::GeoPose        WaypointGeo;
typedef geometry_msgs::TwistStamped     Velocity;
typedef nav_msgs::Odometry              Odometry;
typedef geometry_msgs::TransformStamped Transform;
typedef uint8_t                         State;

/// Common interface for back-end implementations of ual
class Backend {
public:

    /// Wrap a Backend function to make it thread-safe
    template <typename Callable, typename ... Args>
    inline bool threadSafeCall(Callable&& _fn, Args&& ... _args) {
        // Only one thread can lock
        if (running_mutex_.try_lock()) {
            running_task_ = true;  // set running after locking
            std::bind(_fn, this, std::forward<Args>(_args)...)();
            running_mutex_.unlock();
            running_task_ = false;  // reset it after unlocking
            return true;  // Call succeeded
        } else {
            return false;  // Call failed
        }
    }

    /// Constructor inits node
    Backend();

    /// Backend is initialized and ready to run tasks?
    virtual bool     isReady() const = 0;
    /// Is it idle?
    bool             isIdle();
    /// Latest pose estimation of the robot
    virtual Pose     pose() = 0;
    /// Latest velocity estimation of the robot
    virtual Velocity velocity() const = 0;
    /// Latest odometry estimation of the robot
    virtual Odometry odometry() const = 0;
    /// Latest transform estimation of the robot
    virtual Transform transform() const = 0;
    /// Latest pose estimation of the robot in geodesic coordinates
    virtual PoseGeo  globalPose() const { return PoseGeo(); }
    /// Current robot state
    inline State state() { return this->state_; }
    /// Current autopilot custom flight mode
    inline std::string flightMode() { return this->flight_mode_; }
    /// Current battery state
    inline sensor_msgs::BatteryState battery() { return this->battery_; }
    /// Current reference pose
    virtual Pose referencePose() = 0;

    /// Set pose
    /// \param _pose target pose
    virtual void    setPose(const geometry_msgs::PoseStamped& _pose) = 0;

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    virtual void	goToWaypoint(const Waypoint& _wp) = 0;

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    virtual void	goToWaypointGeo(const WaypointGeo& _wp)
    {
        ROS_ERROR("UAL: This backend is not supporting function goToWaypointGeo.");
    }

    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    virtual void    takeOff(double _height) = 0;
    /// Land on the current position.
    virtual void	land() = 0;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    virtual void    setVelocity(const Velocity& _vel) = 0;
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    virtual void    recoverFromManual() = 0;
    /// Set home position
    virtual void    setHome(bool set_z) = 0;

    /// Cancel execution of the current task
    void	        abort(bool _freeze = true);

    virtual ~Backend();  // Ensure proper destructor calling for derived classes

protected:
    /// Abort flag
    /// If you want your task to be abortable, check its value periodically
    std::atomic<bool> abort_ = {false};

    /// Freeze flag
    /// When aborting a task, freezes the platform if it is true
    std::atomic<bool> freeze_ = {false};

    /// Simplest state-machine model: idle/running
    /// Implemented via mutex-locking
    std::mutex running_mutex_;
    std::atomic<bool> running_task_ = {false};

    // Ros spinning thread
    std::thread spin_thread_;

    std::atomic<State> state_ = {ual_core::State::UNINITIALIZED};
    std::string flight_mode_;
    sensor_msgs::BatteryState battery_;
};

}	// namespace ual

#endif // UAL_CORE_BACKEND_H

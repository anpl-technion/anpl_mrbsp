/**
 * @file: conversion.h
 * @brief: convert library pose objects to one to another.
 * @author: Tal Regev
 */


#ifndef CONVERSION_H
#define CONVERSION_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <octomap/octomap_types.h>
#include <fcl/math/transform.h>

#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>


namespace ob = ompl::base;
namespace bg = boost::geometry;

typedef bg::model::point<double , 3, bg::cs::cartesian> BoostPoint;

/**
 * Covert from and to (ompl, octomap, fcl, ros, boost) <-> gtsam Pose3
 * @tparam T_Conversion - class to convert to
 */
template<class T_Conversion>
struct Conversion;

/**
 * convert to Point3
 */
template<>
struct Conversion<gtsam::Point3>
{
    /**
     * convert from ompl State to gtsam Point3
     * @param state - ompl State
     * @return gtsam Point3
     */
    static gtsam::Point3 as(const ob::State *state) {
        const ob::SE3StateSpace::StateType* se3State = state->as<ob::SE3StateSpace::StateType>();
        return gtsam::Point3(se3State->getX(),se3State->getY(),se3State->getZ());
    }

    /**
     * convert octomap Vector3 to gtsam Point3
     * @param vector3 - octomap Vector3
     * @return gtsam Point3
     */
    static gtsam::Point3 as(const octomath::Vector3& vector3) {
        return gtsam::Point3(vector3.x(), vector3.y(), vector3.z());
    }

    /**
     * convert fcl Vec3f to gtsam Point3
     * @param vec3f - fcl Vec3f
     * @return gtsam Point3
     */
    static gtsam::Point3 as(const fcl::Vec3f& vec3f) {
        return gtsam::Point3(vec3f[0], vec3f[1], vec3f[2]);
    }

    /**
     * convert ros Point to gtsam Point3
     * @param point - ros Point
     * @return gtsam Point3
     */
    static gtsam::Point3 as(const geometry_msgs::Point point) {
        return gtsam::Point3(point.x, point.y, point.z);
    }

    /**
     * convert BoostPoint to gtsam Point3
     * @param boostPoint - BoostPoint
     * @return gtsam Point3
     */
    static gtsam::Point3 as(const BoostPoint &boostPoint) {
        return gtsam::Point3(boostPoint.get<0>(), boostPoint.get<1>(), boostPoint.get<2>());
    }
};

/**
 * convert to gtsam Rot3
 */
template<>
struct Conversion<gtsam::Rot3>
{
    /**
     * convert ompl State to gtsam Rot3
     * @param state - ompl State
     * @return gtsam Rot3
     */
    static gtsam::Rot3 as(const ob::State *state) {
        const ob::SE3StateSpace::StateType* se3State = state->as<ob::SE3StateSpace::StateType>();
        const ob::SO3StateSpace::StateType& rotation = se3State->rotation();
        gtsam::Quaternion quaternion(rotation.w,rotation.x,rotation.y,rotation.z);
        return gtsam::Rot3(quaternion);
    }

    /**
     * Convert octomap Quaternion to gtsam Rot3
     * @param quaternion - octomap Quaternion
     * @return gtsam Rot3
     */
    static gtsam::Rot3 as(const octomath::Quaternion &quaternion) {
        return gtsam::Rot3::quaternion(quaternion.u(), quaternion.x(), quaternion.y(), quaternion.z());
    }

    /**
     * Convert fcl Quaternion3f to gtsam Rot3
     * @param quaternion3f - fcl Quaternion3f
     * @return gtsam Rot3
     */
    static gtsam::Rot3 as(const fcl::Quaternion3f& quaternion3f) {
        return gtsam::Rot3::quaternion(quaternion3f.getW(), quaternion3f.getX(), quaternion3f.getY(), quaternion3f.getZ());
    }

    /**
     * convert ros Quaternion to gtsam Rot3
     * @param quaternion - ros Quaternion
     * @return gtsam Rot3
     */
    static gtsam::Rot3 as(const geometry_msgs::Quaternion& quaternion) {
        return gtsam::Rot3::quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }
};

/**
 * Convert to gtsam Point2
 */
template<>
struct Conversion<gtsam::Point2>
{
    /**
     * Convert ompl State to gtsam Point2
     * @param state - ompl State
     * @return gtsam Point2
     */
    static gtsam::Point2 as(const ob::State *state) {
        const ob::SE2StateSpace::StateType* se2State = state->as<ob::SE2StateSpace::StateType>();
        return gtsam::Point2(se2State->getX(),se2State->getY());
    }
};

/**
 * Convert to Rot2
 */
template<>
struct Conversion<gtsam::Rot2>
{
    /**
     * Convert ompl State to gtsam Rot2
     * @param state - ompl State
     * @return gtsam Rot2
     */
    static gtsam::Rot2 as(const ob::State *state) {
        const ob::SE2StateSpace::StateType* se2State = state->as<ob::SE2StateSpace::StateType>();
        return gtsam::Rot2(se2State->getYaw());
    }
};

/**
 * Convert to gtsam Pose2
 */
template<>
struct Conversion<gtsam::Pose2>
{
    /**
     * Convert ompl State to gtsam Pose2
     * @param state - omple State
     * @return gtsam Pose2
     */
    static gtsam::Pose2 as(const ob::State *state) {
        return gtsam::Pose2(Conversion<gtsam::Rot2>::as(state), Conversion<gtsam::Point2>::as(state));
    }
};

/**
 * Convert to octomap Quaternion
 */
template<>
struct Conversion<octomath::Quaternion>
{
    /**
     * Convert from gtsam Rot3 to octomap Quaternion
     * @param rot3 - gtsam Rot3
     * @return octomap Quaternion
     */
    static octomath::Quaternion as(const gtsam::Rot3& rot3) {
        gtsam::Quaternion q = rot3.toQuaternion();
        return octomath::Quaternion(q.w(), q.x(), q.y(), q.z());
    }
};


/**
 * Convert to octomap Vector3
 */
template<>
struct Conversion<octomath::Vector3>
{
    /**
     * Convert from gtsam Point3 to octomap Vector3
     * @param point3 - gtsam Point3
     * @return octomap Vector3
     */
    static octomath::Vector3 as(const gtsam::Point3& point3) {
        return octomath::Vector3(point3.x(), point3.y(), point3.z());
    }
};

/**
 * Convert to octomap pose6d
 */
template<>
struct Conversion<octomap::pose6d>
{
    /**
     * Convert from gtsam Pose3 to octomap pose6d
     * @param pose3 - gtsam Pose3
     * @return octomap pose6d
     */
    static octomap::pose6d as(const gtsam::Pose3& pose3) {
        return octomap::pose6d(Conversion<octomath::Vector3>::as(pose3.translation()), Conversion<octomath::Quaternion>::as(pose3.rotation()));
    }
};


/**
 * Convert to fcl Quaternion3f
 */
template<>
struct Conversion<fcl::Quaternion3f>
{
    /**
     * Convert from gtsam Rot3 to fcl Quaternion3f
     * @param rot3 - gtsam Rot3
     * @return fcl Quaternion3f
     */
    static fcl::Quaternion3f as(const gtsam::Rot3& rot3) {
        gtsam::Quaternion q = rot3.toQuaternion();
        return fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z());
    }
};

/**
 * Convert to fcl Vec3f
 */
template<>
struct Conversion<fcl::Vec3f>
{
    /**
     * Convert from gtsam Point3 to fcl Vec3f
     * @param point3 - gtsam Point3
     * @return fcl Vec3f
     */
    static fcl::Vec3f as(const gtsam::Point3& point3) {
        return fcl::Vec3f(point3.x(), point3.y(), point3.z());
    }
};

/**
 * Convert to fcl Transform3f
 */
template<>
struct Conversion<fcl::Transform3f>
{
    /**
     * Convert from gtsam Pose3 to fcl Transform3f
     * @param pose3 - gtsam Pose3
     * @return fcl Transform3f
     */
    static fcl::Transform3f as(const gtsam::Pose3& pose3) {
        return fcl::Transform3f(Conversion<fcl::Quaternion3f>::as(pose3.rotation()), Conversion<fcl::Vec3f>::as(pose3.translation()));
    }
};

/**
 * Convert to ros Point
 */
template<>
struct Conversion<geometry_msgs::Point>
{
    /**
     * Convert from gtsam Point3 to ros Point
     * @param point3 - gtsam Point3
     * @return ros Point
     */
    static geometry_msgs::Point as(const gtsam::Point3& point3) {
        geometry_msgs::Point point;
        point.x = point3.x();
        point.y = point3.y();
        point.z = point3.z();
        return point;
    }

    /**
     * Convert from gtsam Point2 to ros Point
     * @param point2 - gtsam Point2
     * @return ros Point
     */
    static geometry_msgs::Point as(const gtsam::Point2& point2) {
        geometry_msgs::Point point;
        point.x = point2.x();
        point.y = point2.y();
        return point;
    }
};

/**
 * Convert to ros Quaternion
 */
template<>
struct Conversion<geometry_msgs::Quaternion>
{
    /**
     * Convert from gtsam Rot3 to ros Quaternion
     * @param rot3 - gtsam Rot3
     * @return ros Quaternion
     */
    static geometry_msgs::Quaternion as(const gtsam::Rot3& rot3) {
        gtsam::Quaternion quaternion_gtsam = rot3.toQuaternion();
        geometry_msgs::Quaternion quaternion;
        quaternion.x = quaternion_gtsam.x();
        quaternion.y = quaternion_gtsam.y();
        quaternion.z = quaternion_gtsam.z();
        quaternion.w = quaternion_gtsam.w();
        return quaternion;
    }
};

/**
 * Convert to ros Pose
 */
template<>
struct Conversion<geometry_msgs::Pose>
{
    /**
     * Convert from gtsam Pose3 to ros Pose
     * @param pose3 - gtsam Pose3
     * @return ros Pose
     */
    static geometry_msgs::Pose as(const gtsam::Pose3& pose3) {
        geometry_msgs::Pose pose;
        pose.position    = Conversion<geometry_msgs::Point>::as(pose3.translation());
        pose.orientation = Conversion<geometry_msgs::Quaternion>::as(pose3.rotation());
        return pose;
    }
};

/**
 * Convert to gtsam Pose3
 * @tparam T_Conversion - class to convert to
 */
template<class T_Conversion>
struct Conversion
{
    /**
     * Convert from ompl State to gtsam Pose3
     * @param state - ompl State
     * @return gtsam Pose3
     */
    static gtsam::Pose3 as(const ob::State *state) {
        return gtsam::Pose3(Conversion<gtsam::Rot3>::as(state), Conversion<gtsam::Point3>::as(state));
    }

    /**
     * Convert from octomap pose6d to gtsam Pose3
     * @param pose6d - octomap pose6d
     * @return gtsam Pose3
     */
    static gtsam::Pose3 as(const octomap::pose6d &pose6d) {
        return gtsam::Pose3(Conversion<gtsam::Rot3>::as(pose6d.rot()), Conversion<gtsam::Point3>::as(pose6d.trans()));
    }

    /**
     * Convert from fcl Transform3f to gtsam Pose3
     * @param transform3f - fcl Transform3f
     * @return gtsam Pose3
     */
    static gtsam::Pose3 as(const fcl::Transform3f &transform3f) {
        return gtsam::Pose3(Conversion<gtsam::Rot3>::as(transform3f.getQuatRotation()), Conversion<gtsam::Point3>::as(transform3f.getTranslation()));
    }

    /**
     * Convert from ros Pose to gtsam Pose3
     * @param pose - ros Pose
     * @return gtsam Pose3
     */
    static gtsam::Pose3 as(const geometry_msgs::Pose& pose) {
        return gtsam::Pose3(Conversion<gtsam::Rot3>::as(pose.orientation), Conversion<gtsam::Point3>::as(pose.position));
    }
};

#endif //CONVERSION_H

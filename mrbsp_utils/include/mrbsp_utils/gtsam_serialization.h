/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file: gtsam_serialization.h
 * @brief: for serialize gtsam objects
 * @author: Tal Regev
 */

#ifndef GTSAM_SERIALIZATION_H
#define GTSAM_SERIALIZATION_H

/* ************************************************************************ */

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <fstream>
#include <gtsam/base/serialization.h>

// ... Includes for your values and factors:
#include <gtsam/base/GenericValue.h> // GTSAM_VALUE_EXPORT
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
//#include <gtsam/slam/AntiFactor.h>
//#include <gtsam/sam/BearingRangeFactor.h>
//#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/slam/GeneralSFMFactor.h>
//#include <gtsam/slam/PartialPriorFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/StereoCamera.h>
// ...

// Define the Boost export macros:
#include <boost/serialization/export.hpp> // BOOST_CLASS_EXPORT_GUID
#include <boost/serialization/serialization.hpp>

namespace boost {
namespace serialization {
template <class Archive, typename Derived>
void serialize(Archive &ar, Eigen::EigenBase<Derived> &g,
               const unsigned int version) {
  ar &boost::serialization::make_array(g.derived().data(), g.size());
}
} // namespace serialization
} // namespace boost


// Creating as many permutations of factors as possible
typedef gtsam::PriorFactor<gtsam::Point2>            PriorFactorPoint2;
typedef gtsam::PriorFactor<gtsam::StereoPoint2>      PriorFactorStereoPoint2;
typedef gtsam::PriorFactor<gtsam::Point3>            PriorFactorPoint3;
typedef gtsam::PriorFactor<gtsam::Rot2>              PriorFactorRot2;
typedef gtsam::PriorFactor<gtsam::Rot3>              PriorFactorRot3;
typedef gtsam::PriorFactor<gtsam::Pose2>             PriorFactorPose2;
typedef gtsam::PriorFactor<gtsam::Pose3>             PriorFactorPose3;
typedef gtsam::PriorFactor<gtsam::Cal3_S2>           PriorFactorCal3_S2;
typedef gtsam::PriorFactor<gtsam::Cal3DS2>           PriorFactorCal3DS2;
typedef gtsam::PriorFactor<gtsam::CalibratedCamera>  PriorFactorCalibratedCamera;
typedef gtsam::PriorFactor<gtsam::SimpleCamera>      PriorFactorSimpleCamera;
typedef gtsam::PriorFactor<gtsam::StereoCamera>      PriorFactorStereoCamera;

typedef gtsam::BetweenFactor<gtsam::Point2>          BetweenFactorPoint2;
typedef gtsam::BetweenFactor<gtsam::Point3>          BetweenFactorPoint3;
typedef gtsam::BetweenFactor<gtsam::Rot2>            BetweenFactorRot2;
typedef gtsam::BetweenFactor<gtsam::Rot3>            BetweenFactorRot3;
typedef gtsam::BetweenFactor<gtsam::Pose2>           BetweenFactorPose2;
typedef gtsam::BetweenFactor<gtsam::Pose3>           BetweenFactorPose3;

typedef gtsam::NonlinearEquality<gtsam::Point2>            NonlinearEqualityPoint2;
typedef gtsam::NonlinearEquality<gtsam::StereoPoint2>      NonlinearEqualityStereoPoint2;
typedef gtsam::NonlinearEquality<gtsam::Point3>            NonlinearEqualityPoint3;
typedef gtsam::NonlinearEquality<gtsam::Rot2>              NonlinearEqualityRot2;
typedef gtsam::NonlinearEquality<gtsam::Rot3>              NonlinearEqualityRot3;
typedef gtsam::NonlinearEquality<gtsam::Pose2>             NonlinearEqualityPose2;
typedef gtsam::NonlinearEquality<gtsam::Pose3>             NonlinearEqualityPose3;
typedef gtsam::NonlinearEquality<gtsam::Cal3_S2>           NonlinearEqualityCal3_S2;
typedef gtsam::NonlinearEquality<gtsam::Cal3DS2>           NonlinearEqualityCal3DS2;
typedef gtsam::NonlinearEquality<gtsam::CalibratedCamera>  NonlinearEqualityCalibratedCamera;
typedef gtsam::NonlinearEquality<gtsam::SimpleCamera>      NonlinearEqualitySimpleCamera;
typedef gtsam::NonlinearEquality<gtsam::StereoCamera>      NonlinearEqualityStereoCamera;

//typedef RangeFactor<Pose2, Point2>                      RangeFactorPosePoint2;
//typedef RangeFactor<Pose3, Point3>                      RangeFactorPosePoint3;
//typedef RangeFactor<Pose2, Pose2>                       RangeFactorPose2;
//typedef RangeFactor<Pose3, Pose3>                       RangeFactorPose3;
//typedef RangeFactor<CalibratedCamera, Point3>           RangeFactorCalibratedCameraPoint;
//typedef RangeFactor<SimpleCamera, Point3>               RangeFactorSimpleCameraPoint;
//typedef RangeFactor<CalibratedCamera, CalibratedCamera> RangeFactorCalibratedCamera;
//typedef RangeFactor<SimpleCamera, SimpleCamera>         RangeFactorSimpleCamera;
//
//typedef BearingRangeFactor<Pose2, Point2>  BearingRangeFactor2D;
//typedef BearingRangeFactor<Pose3, Point3>  BearingRangeFactor3D;

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> GenericProjectionFactorCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> GenericProjectionFactorCal3DS2;

//typedef gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3> GeneralSFMFactorCal3_S2;
//typedef gtsam::GeneralSFMFactor<gtsam::PinholeCameraCal3DS2, gtsam::Point3> GeneralSFMFactorCal3DS2;

//typedef gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> GeneralSFMFactor2Cal3_S2;

typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> GenericStereoFactor3D;

/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null,"gtsam_noiseModel_mEstimator_Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam_noiseModel_mEstimator_Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber,"gtsam_noiseModel_mEstimator_Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");
// clang-format on

/* Create GUIDs for geometry */
/* ************************************************************************* */
GTSAM_VALUE_EXPORT(gtsam::Point2);
GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Rot2);
GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::Pose2);
GTSAM_VALUE_EXPORT(gtsam::Pose3);

BOOST_CLASS_EXPORT_GUID(gtsam::Cal3_S2, "gtsam::Cal3_S2");
BOOST_CLASS_EXPORT_GUID(gtsam::Cal3DS2, "gtsam::Cal3DS2");
BOOST_CLASS_EXPORT_GUID(gtsam::Cal3_S2Stereo, "gtsam::Cal3_S2Stereo");
BOOST_CLASS_EXPORT_GUID(gtsam::CalibratedCamera, "gtsam::CalibratedCamera");
BOOST_CLASS_EXPORT_GUID(gtsam::SimpleCamera, "gtsam::SimpleCamera");
BOOST_CLASS_EXPORT_GUID(gtsam::StereoCamera, "gtsam::StereoCamera");


/* Create GUIDs for belief representations */
/* ************************************************************************* */
//BOOST_CLASS_EXPORT_GUID(gtsam::BayesTree<gtsam::ISAM2Clique>, "gtsam::BayesTree<gtsam::ISAM2Clique>");


/* Create GUIDs for factors */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor");

BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Point3>, "gtsam::ExpressionFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Rot3>, "gtsam::ExpressionFactor<gtsam::Rot3>");

// Add your custom factors, if any.

BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Point3>, "gtsam::PriorFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Rot3>, "gtsam::PriorFactor<gtsam::Rot3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose2>, "gtsam::PriorFactor<gtsam::Pose2>");

BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Point3>, "gtsam::BetweenFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Rot3>, "gtsam::BetweenFactor<gtsam::Rot3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose2>, "gtsam::BetweenFactor<gtsam::Pose2>");


BOOST_CLASS_EXPORT_GUID(PriorFactorPoint2, "gtsamPriorFactorPoint2");
BOOST_CLASS_EXPORT_GUID(PriorFactorRot2, "gtsamPriorFactorRot2");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoPoint2, "gtsamPriorFactorStereoPoint2");


BOOST_CLASS_EXPORT_GUID(PriorFactorCal3_S2, "gtsamPriorFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3DS2, "gtsamPriorFactorCal3DS2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCalibratedCamera, "gtsamPriorFactorCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(PriorFactorSimpleCamera, "gtsamPriorFactorSimpleCamera");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoCamera, "gtsamPriorFactorStereoCamera");

BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint2, "gtsamBetweenFactorPoint2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot2, "gtsamBetweenFactorRot2");

BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint2, "gtsamNonlinearEqualityPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoPoint2, "gtsamNonlinearEqualityStereoPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint3, "gtsamNonlinearEqualityPoint3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot2, "gtsamNonlinearEqualityRot2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot3, "gtsamNonlinearEqualityRot3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose2, "gtsamNonlinearEqualityPose2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose3, "gtsamNonlinearEqualityPose3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3_S2, "gtsamNonlinearEqualityCal3_S2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3DS2, "gtsamNonlinearEqualityCal3DS2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCalibratedCamera, "gtsamNonlinearEqualityCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualitySimpleCamera, "gtsamNonlinearEqualitySimpleCamera");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoCamera, "gtsamNonlinearEqualityStereoCamera");

//BOOST_CLASS_EXPORT_GUID(RangeFactorPosePoint2, "gtsamRangeFactorPosePoint2");
//BOOST_CLASS_EXPORT_GUID(RangeFactorPosePoint3, "gtsamRangeFactorPosePoint3");
//BOOST_CLASS_EXPORT_GUID(RangeFactorPose2, "gtsamRangeFactorPose2");
//BOOST_CLASS_EXPORT_GUID(RangeFactorPose3, "gtsamRangeFactorPose3");
//BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCameraPoint, "gtsamRangeFactorCalibratedCameraPoint");
//BOOST_CLASS_EXPORT_GUID(RangeFactorSimpleCameraPoint, "gtsamRangeFactorSimpleCameraPoint");
//BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCamera, "gtsamRangeFactorCalibratedCamera");
//BOOST_CLASS_EXPORT_GUID(RangeFactorSimpleCamera, "gtsamRangeFactorSimpleCamera");
//
//BOOST_CLASS_EXPORT_GUID(BearingRangeFactor2D, "gtsamBearingRangeFactor2D");

BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3_S2, "gtsamGenericProjectionFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3DS2, "gtsamGenericProjectionFactorCal3DS2");

//BOOST_CLASS_EXPORT_GUID(GeneralSFMFactorCal3_S2, "gtsamGeneralSFMFactorCal3_S2");
//BOOST_CLASS_EXPORT_GUID(GeneralSFMFactorCal3DS2, "gtsamGeneralSFMFactorCal3DS2");

BOOST_CLASS_EXPORT_GUID(GenericStereoFactor3D, "gtsamGenericStereoFactor3D");


void saveBinary(const std::string &outFileName, const gtsam::NonlinearFactorGraph &f, const gtsam::Values &v) 
{
  std::ofstream ofs(outFileName);
  boost::archive::binary_oarchive oa(ofs);
  oa << f << v;
}

void loadBinary(const std::string &inFileName, gtsam::NonlinearFactorGraph &f, gtsam::Values &v) 
{
  std::ifstream ifs(inFileName);
  if (!ifs.is_open())
    throw std::runtime_error("Error opening file");

  boost::archive::binary_iarchive ia(ifs);
  ia >> f >> v;
}

#endif //GTSAM_SERIALIZATION_H

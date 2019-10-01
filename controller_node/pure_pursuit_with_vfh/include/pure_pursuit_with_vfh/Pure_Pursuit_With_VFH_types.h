/*
 * Pure_Pursuit_With_VFH_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Pure_Pursuit_With_VFH".
 *
 * Model version              : 1.146
 * Simulink Coder version : 8.13 (R2017b) 24-Jul-2017
 * C++ source code generated on : Mon Dec 10 12:36:04 2018
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Pure_Pursuit_With_VFH_types_h_
#define RTW_HEADER_Pure_Pursuit_With_VFH_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool_

typedef struct {
  boolean_T Data;
} SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_Pure_Pursuit_With_VFH_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[720];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_Pure_Pursuit_With_VFH_ros_time_Time Stamp;
} SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan_

typedef struct {
  real32_T AngleMin;
  real32_T AngleMax;
  real32_T AngleIncrement;
  real32_T TimeIncrement;
  real32_T ScanTime;
  real32_T RangeMin;
  real32_T RangeMax;
  real32_T Ranges[721];
  SL_Bus_ROSVariableLengthArrayInfo Ranges_SL_Info;
  real32_T Intensities[721];
  SL_Bus_ROSVariableLengthArrayInfo Intensities_SL_Info;
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Header Header;
} SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Quaternion_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Quaternion;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Pose_

typedef struct {
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point Position;
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Quaternion Orientation;
} SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Pose;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_PoseWithCovariance_fw236l_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_PoseWithCovariance_fw236l_

typedef struct {
  real_T Covariance[36];
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Pose Pose;
} SL_Bus_Pure_Pursuit_With_VFH_PoseWithCovariance_fw236l;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Vector3_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist_

typedef struct {
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Vector3 Linear;
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Vector3 Angular;
} SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_TwistWithCovariance_uwyk8x_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_TwistWithCovariance_uwyk8x_

typedef struct {
  real_T Covariance[36];
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist Twist;
} SL_Bus_Pure_Pursuit_With_VFH_TwistWithCovariance_uwyk8x;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry_
#define DEFINED_TYPEDEF_FOR_SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry_

typedef struct {
  uint8_T ChildFrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo ChildFrameId_SL_Info;
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Header Header;
  SL_Bus_Pure_Pursuit_With_VFH_PoseWithCovariance_fw236l Pose;
  SL_Bus_Pure_Pursuit_With_VFH_TwistWithCovariance_uwyk8x Twist;
} SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry;

#endif

#ifndef struct_tag_sSnSp9bfvpJuZkKgamaX3qB
#define struct_tag_sSnSp9bfvpJuZkKgamaX3qB

struct tag_sSnSp9bfvpJuZkKgamaX3qB
{
  uint32_T RangeLimits;
  uint32_T AngleLimits;
};

#endif                                 /*struct_tag_sSnSp9bfvpJuZkKgamaX3qB*/

#ifndef typedef_sSnSp9bfvpJuZkKgamaX3qB_Pure__T
#define typedef_sSnSp9bfvpJuZkKgamaX3qB_Pure__T

typedef struct tag_sSnSp9bfvpJuZkKgamaX3qB sSnSp9bfvpJuZkKgamaX3qB_Pure__T;

#endif                                 /*typedef_sSnSp9bfvpJuZkKgamaX3qB_Pure__T*/

#ifndef struct_tag_sKDUzAdQGZWzDaLo6BY8rKD
#define struct_tag_sKDUzAdQGZWzDaLo6BY8rKD

struct tag_sKDUzAdQGZWzDaLo6BY8rKD
{
  real_T RangeLimits;
  real_T AngleLimits;
};

#endif                                 /*struct_tag_sKDUzAdQGZWzDaLo6BY8rKD*/

#ifndef typedef_sKDUzAdQGZWzDaLo6BY8rKD_Pure__T
#define typedef_sKDUzAdQGZWzDaLo6BY8rKD_Pure__T

typedef struct tag_sKDUzAdQGZWzDaLo6BY8rKD sKDUzAdQGZWzDaLo6BY8rKD_Pure__T;

#endif                                 /*typedef_sKDUzAdQGZWzDaLo6BY8rKD_Pure__T*/

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_T;

#endif                                 /*typedef_robotics_slros_internal_block_T*/

#ifndef typedef_robotics_slros_internal_blo_n_T
#define typedef_robotics_slros_internal_blo_n_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_n_T;

#endif                                 /*typedef_robotics_slros_internal_blo_n_T*/

#ifndef struct_tag_so4pBuofLXIy3ixtLNBoZq
#define struct_tag_so4pBuofLXIy3ixtLNBoZq

struct tag_so4pBuofLXIy3ixtLNBoZq
{
  char_T PartialMatching[6];
};

#endif                                 /*struct_tag_so4pBuofLXIy3ixtLNBoZq*/

#ifndef typedef_so4pBuofLXIy3ixtLNBoZq_Pure_P_T
#define typedef_so4pBuofLXIy3ixtLNBoZq_Pure_P_T

typedef struct tag_so4pBuofLXIy3ixtLNBoZq so4pBuofLXIy3ixtLNBoZq_Pure_P_T;

#endif                                 /*typedef_so4pBuofLXIy3ixtLNBoZq_Pure_P_T*/

#ifndef typedef_cell_wrap_Pure_Pursuit_With_V_T
#define typedef_cell_wrap_Pure_Pursuit_With_V_T

typedef struct {
  real_T f1[2];
} cell_wrap_Pure_Pursuit_With_V_T;

#endif                                 /*typedef_cell_wrap_Pure_Pursuit_With_V_T*/

#ifndef typedef_cell_wrap_Pure_Pursuit_With_n_T
#define typedef_cell_wrap_Pure_Pursuit_With_n_T

typedef struct {
  uint32_T f1[8];
} cell_wrap_Pure_Pursuit_With_n_T;

#endif                                 /*typedef_cell_wrap_Pure_Pursuit_With_n_T*/

#ifndef typedef_robotics_slalgs_internal_Vect_T
#define typedef_robotics_slalgs_internal_Vect_T

typedef struct {
  int32_T isInitialized;
  cell_wrap_Pure_Pursuit_With_n_T inputVarSize[3];
  real_T NarrowOpeningThreshold;
  real_T DistanceLimits[2];
  real_T RobotRadius;
  real_T SafetyDistance;
  real_T MinTurningRadius;
  real_T TargetDirectionWeight;
  real_T CurrentDirectionWeight;
  real_T PreviousDirectionWeight;
  real_T HistogramThresholds[2];
  real_T PolarObstacleDensity[240];
  boolean_T BinaryHistogram[240];
  boolean_T MaskedHistogram[240];
  real_T PreviousDirection;
  real_T AngularSectorMidPoints[240];
  real_T AngularDifference;
  real_T AngularSectorStartPoints[240];
  real_T AngularLimits[2];
} robotics_slalgs_internal_Vect_T;

#endif                                 /*typedef_robotics_slalgs_internal_Vect_T*/

#ifndef typedef_robotics_core_internal_codege_T
#define typedef_robotics_core_internal_codege_T

typedef struct {
  cell_wrap_Pure_Pursuit_With_V_T Defaults[2];
  cell_wrap_Pure_Pursuit_With_V_T ParsedResults[2];
} robotics_core_internal_codege_T;

#endif                                 /*typedef_robotics_core_internal_codege_T*/

#ifndef typedef_robotics_slalgs_internal_Pure_T
#define typedef_robotics_slalgs_internal_Pure_T

typedef struct {
  int32_T isInitialized;
  cell_wrap_Pure_Pursuit_With_n_T inputVarSize[2];
  real_T MaxAngularVelocity;
  real_T LookaheadDistance;
  real_T DesiredLinearVelocity;
  real_T ProjectionPoint[2];
  real_T ProjectionLineIndex;
  real_T LookaheadPoint[2];
  real_T LastPose[3];
  real_T WaypointsInternal[2];
} robotics_slalgs_internal_Pure_T;

#endif                                 /*typedef_robotics_slalgs_internal_Pure_T*/

/* Parameters for system: '<S8>/Enabled Subsystem' */
typedef struct P_EnabledSubsystem_Pure_Pursu_T_ P_EnabledSubsystem_Pure_Pursu_T;

/* Parameters (auto storage) */
typedef struct P_Pure_Pursuit_With_VFH_T_ P_Pure_Pursuit_With_VFH_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Pure_Pursuit_With_VFH_T RT_MODEL_Pure_Pursuit_With_VF_T;

#endif                                 /* RTW_HEADER_Pure_Pursuit_With_VFH_types_h_ */

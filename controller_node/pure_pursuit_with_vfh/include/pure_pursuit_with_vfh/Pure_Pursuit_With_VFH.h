/*
 * Pure_Pursuit_With_VFH.h
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

#ifndef RTW_HEADER_Pure_Pursuit_With_VFH_h_
#define RTW_HEADER_Pure_Pursuit_With_VFH_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef Pure_Pursuit_With_VFH_COMMON_INCLUDES_
# define Pure_Pursuit_With_VFH_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 /* Pure_Pursuit_With_VFH_COMMON_INCLUDES_ */

#include "Pure_Pursuit_With_VFH_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

/* Block signals for system '<S8>/Enabled Subsystem' */
typedef struct {
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool In1;/* '<S11>/In1' */
} B_EnabledSubsystem_Pure_Pursu_T;

/* Block signals (auto storage) */
typedef struct {
  real_T candToSectDiff_data[87840];
  real_T tmp_data[87840];
  boolean_T validWeights_data[173040];
  boolean_T nearIdx_data[87840];
  real_T lowerVec_data[2163];
  real_T higherVec_data[2163];
  real_T lh_data[2163];
  real_T kalphaVec_data[2163];
  real_T lk_data[2163];
  real_T kh_data[2163];
  SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan In1;/* '<S28>/In1' */
  SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan b_varargout_2;
  real_T Angles[721];                  /* '<S3>/MATLAB Function' */
  real_T dv0[721];
  real_T theta_data[721];
  real_T b_z1_data[721];
  real_T scan_InternalRanges_data[721];
  real_T scan_InternalAngles_data[721];
  real_T weightedRanges_data[721];
  real_T sinOfEnlargement_data[721];
  real_T enlargementAngle_data[721];
  real_T lowerAng_data[721];
  real_T tmp_data_m[721];
  real_T tmp_data_c[721];
  real_T tmp_data_k[721];
  real_T b_x_data[721];
  real_T b_validAngles_data[721];
  real_T validScan_InternalRanges_data[721];
  real_T validScan_InternalAngles_data[721];
  real_T DXj_data[721];
  real_T DYj_data[721];
  real_T obj_data[721];
  real_T tmp_data_cx[721];
  real_T b_z1_data_b[721];
  real_T thetaWrap[721];
  real_T kalpha[720];
  real_T candidateDirs_data[366];
  real_T costValues_data[366];
  real_T cDiff_data[366];
  real_T candidateDirs_data_p[366];
  real_T b_data[366];
  real_T prevDir_data[366];
  real_T tmp_data_cv[366];
  real_T targetDir_data[366];
  real_T delta_data[366];
  real_T thetaWrap_data[366];
  real_T theta_data_f[366];
  real_T b_z1_data_g[366];
  real_T theta_data_g[366];
  real_T tmp_data_me[366];
  real_T thetaWrap_data_n[366];
  real_T theta_data_p[366];
  int32_T m_data[721];
  real_T sectors_data[242];
  real_T angles_data[242];
  real_T nonNarrowDirs_data[242];
  real_T dv1[242];
  real_T changes[241];
  real_T obstacleDensity[240];
  real_T dv2[240];
  real_T dv3[240];
  SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry In1_m;/* '<S29>/In1' */
  SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry b_varargout_2_l;
  int32_T qd_data[366];
  int32_T sd_data[366];
  int32_T td_data[366];
  real_T sectorSizes_data[121];
  real_T narrowDirs_data[121];
  real_T angles_data_j[121];
  real_T tmp_data_d[121];
  real_T b_y1_data[121];
  boolean_T tmp_data_g[721];
  boolean_T validIndices_data[721];
  boolean_T validRangeLimitIndices_data[721];
  boolean_T validAngleLimitIndices_data[721];
  boolean_T g_data[721];
  boolean_T validIndices_data_l[721];
  boolean_T b_data_d[721];
  boolean_T blockedR_data[721];
  boolean_T blockedL_data[721];
  int32_T kd_data[121];
  int32_T pd_data[121];
  boolean_T freeDirs_data[366];
  boolean_T pos_data[366];
  boolean_T b_data_dy[366];
  boolean_T pos_data_l[366];
  boolean_T b_data_o[366];
  boolean_T pos_data_b[366];
  boolean_T b_data_n[366];
  uint8_T foundSectors_data[241];
  uint8_T ii_data[241];
  uint8_T ii_data_b[241];
  uint8_T rd_data[240];
  uint8_T b_data_h[240];
  uint8_T c_data[240];
  boolean_T obj_data_l[240];
  boolean_T d[240];
  robotics_core_internal_codege_T parser;
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist Constant;/* '<S31>/Constant' */
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist BusAssignment1;/* '<S4>/Bus Assignment1' */
  cell_wrap_Pure_Pursuit_With_V_T parsedResults[2];
  char_T cv0[31];
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point In1_p;/* '<S30>/In1' */
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point b_varargout_2_b;
  real_T TmpSignalConversionAtPurePu[3];
  real_T kalpha_d[3];
  char_T cv1[19];
  char_T cv2[16];
  real_T Goal[2];                      /* '<S2>/ Extract Goal' */
  real_T VectorConcatenate[2];         /* '<S3>/Vector Concatenate' */
  real_T obj[2];
  real_T parsedRangeLimits[2];
  real_T parsedAngleLimits[2];
  real_T obj_e[2];
  real_T waypoints_data[2];
  real_T lookaheadStartPt[2];
  real_T waypoints[2];
  real_T waypoints_b[2];
  real_T refPt[2];
  char_T cv3[15];
  char_T cv4[14];
  real_T Add1;                         /* '<S2>/Add1' */
  real_T Merge;                        /* '<S15>/Merge' */
  real_T Merge1;                       /* '<S15>/Merge1' */
  real_T Merge_e;                      /* '<S1>/Merge' */
  real_T Merge1_k;                     /* '<S1>/Merge1' */
  real_T In;                           /* '<S16>/In' */
  real_T Constant_j;                   /* '<S19>/Constant' */
  real_T Constant_k;                   /* '<S20>/Constant' */
  real_T Constant_n;                   /* '<S17>/Constant' */
  real_T Constant1;                    /* '<S17>/Constant1' */
  real_T Constant2;                    /* '<S17>/Constant2' */
  real_T Constant_c;                   /* '<S22>/Constant' */
  real_T Constant_cw;                  /* '<S21>/Constant' */
  real_T Gain;                         /* '<S17>/Gain' */
  real_T PurePursuit_o1;               /* '<S2>/Pure Pursuit' */
  real_T PurePursuit_o2;               /* '<S2>/Pure Pursuit' */
  real_T aSinInput;
  real_T VectorFieldHistogram;         /* '<S1>/Vector Field Histogram' */
  real_T Switch;                       /* '<S1>/Switch' */
  real_T rtb_Subtract_idx_1;
  real_T rtb_SaturationOmega_i;
  real_T target;
  real_T mtmp;
  real_T totalWeight;
  real_T q;
  real_T x;
  real_T phiR_data;
  real_T phiL_data;
  real_T obj_tmp;
  real_T minDistance;
  real_T dist;
  real_T lookaheadIdx;
  real_T b;
  real_T lookaheadEndPt_idx_0;
  real_T lookaheadEndPt_idx_1;
  real_T alpha;
  real_T d0;
  real_T closestPoint;
  int32_T sectors_size[2];
  int32_T candidateDirs_size[2];
  int32_T angles_size[2];
  int32_T angles_size_j[2];
  int32_T tmp_size[2];
  int32_T obj_size[2];
  int32_T b_size[2];
  int32_T prevDir_size[2];
  int32_T tmp_size_f[2];
  int32_T targetDir_size[2];
  int32_T thetaWrap_size[2];
  int32_T theta_size[2];
  int32_T theta_size_a[2];
  int32_T tmp_size_j[2];
  int32_T tmp_size_jz[2];
  int32_T thetaWrap_size_o[2];
  int32_T theta_size_n[2];
  int32_T lowerVec_size[2];
  int32_T lh_size[2];
  int32_T kalphaVec_size[2];
  int32_T lk_size[2];
  int32_T kh_size[2];
  int32_T k;
  int32_T scan_InternalRanges_size;
  int32_T scan_InternalAngles_size;
  int32_T idx;
  int32_T calclen;
  int32_T vstride;
  int32_T ixstop;
  int32_T loop_ub;
  int32_T end;
  int32_T trueCount;
  int32_T loop_ub_o;
  int32_T loop_ub_n;
  int32_T sinOfEnlargement_size;
  int32_T enlargementAngle_size;
  int32_T lowerAng_size;
  int32_T tmp_size_m;
  int32_T tmp_size_c;
  int32_T lh_size_m;
  int32_T m_size_idx_0;
  int32_T i0;
  int32_T loop_ub_m;
  int32_T ii_data_j;
  int32_T k_h;
  int32_T idx_c;
  int32_T xtmp;
  int32_T validScan_InternalAngles_size;
  int32_T tmp_size_ct;
  int32_T obj_size_p;
  int32_T tmp_size_p;
  int32_T tmp_size_a;
  int32_T obj_size_e;
  int32_T tmp_size_ax;
  int32_T tmp_size_as;
  int32_T DXj_size_idx_0;
  int32_T DYj_size_idx_0;
  int32_T ii_data_tmp;
  boolean_T Constant1_f;               /* '<S15>/Constant1' */
  boolean_T LogicalOperator1;          /* '<S3>/Logical Operator1' */
  boolean_T LogicalOperator;           /* '<S4>/Logical Operator' */
  boolean_T LogicalOperator1_m;        /* '<S4>/Logical Operator1' */
  boolean_T D;                         /* '<S9>/D' */
  B_EnabledSubsystem_Pure_Pursu_T EnabledSubsystem_d;/* '<S33>/Enabled Subsystem' */
  B_EnabledSubsystem_Pure_Pursu_T EnabledSubsystem;/* '<S8>/Enabled Subsystem' */
} B_Pure_Pursuit_With_VFH_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  robotics_slalgs_internal_Vect_T obj; /* '<S1>/Vector Field Histogram' */
  robotics_slalgs_internal_Pure_T obj_a;/* '<S2>/Pure Pursuit' */
  real_T DelayInput1_DSTATE[2];        /* '<S14>/Delay Input1' */
  robotics_slros_internal_block_T obj_e;/* '<S33>/SourceBlock' */
  robotics_slros_internal_block_T obj_b;/* '<S26>/SourceBlock' */
  robotics_slros_internal_block_T obj_n;/* '<S25>/SourceBlock' */
  robotics_slros_internal_block_T obj_m;/* '<S24>/SourceBlock' */
  robotics_slros_internal_block_T obj_p;/* '<S8>/SourceBlock' */
  robotics_slros_internal_blo_n_T obj_l;/* '<S32>/SinkBlock' */
  int8_T If1_ActiveSubsystem;          /* '<S15>/If1' */
  int8_T If_ActiveSubsystem;           /* '<S1>/If' */
  boolean_T Memory_PreviousInput;      /* '<S3>/Memory' */
  boolean_T objisempty;                /* '<S33>/SourceBlock' */
  boolean_T objisempty_p;              /* '<S32>/SinkBlock' */
  boolean_T objisempty_o;              /* '<S26>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S25>/SourceBlock' */
  boolean_T objisempty_b;              /* '<S24>/SourceBlock' */
  boolean_T objisempty_d;              /* '<S2>/Pure Pursuit' */
  boolean_T objisempty_a;              /* '<S8>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S1>/Vector Field Histogram' */
  boolean_T Outputs_MODE;              /* '<Root>/Outputs' */
} DW_Pure_Pursuit_With_VFH_T;

/* Continuous states (auto storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S17>/Integrator' */
} X_Pure_Pursuit_With_VFH_T;

/* State derivatives (auto storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S17>/Integrator' */
} XDot_Pure_Pursuit_With_VFH_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S17>/Integrator' */
} XDis_Pure_Pursuit_With_VFH_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState SampleandHold_Trig_ZCE[2];/* '<S2>/Sample and Hold' */
} PrevZCX_Pure_Pursuit_With_VFH_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters for system: '<S8>/Enabled Subsystem' */
struct P_EnabledSubsystem_Pure_Pursu_T_ {
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool Out1_Y0;/* Computed Parameter: Out1_Y0
                                                      * Referenced by: '<S11>/Out1'
                                                      */
};

/* Parameters (auto storage) */
struct P_Pure_Pursuit_With_VFH_T_ {
  real_T LinVelocityControl_a;         /* Mask Parameter: LinVelocityControl_a
                                        * Referenced by:
                                        *   '<S17>/Constant'
                                        *   '<S17>/Gain1'
                                        */
  real_T LinVelocityControl_c;         /* Mask Parameter: LinVelocityControl_c
                                        * Referenced by:
                                        *   '<S19>/Constant'
                                        *   '<S20>/Constant'
                                        */
  real_T LinVelocityControl_k;         /* Mask Parameter: LinVelocityControl_k
                                        * Referenced by: '<S17>/Gain'
                                        */
  real_T LinVelocityControl_omega_max; /* Mask Parameter: LinVelocityControl_omega_max
                                        * Referenced by:
                                        *   '<S17>/SaturationOmega'
                                        *   '<S21>/Constant'
                                        *   '<S22>/Constant'
                                        */
  real_T LinVelocityControl_v_max;     /* Mask Parameter: LinVelocityControl_v_max
                                        * Referenced by:
                                        *   '<S17>/Constant1'
                                        *   '<S17>/Saturation'
                                        */
  real_T LinVelocityControl_v_min;     /* Mask Parameter: LinVelocityControl_v_min
                                        * Referenced by: '<S17>/Constant2'
                                        */
  boolean_T LinVelocityControl_enable; /* Mask Parameter: LinVelocityControl_enable
                                        * Referenced by: '<S15>/Constant1'
                                        */
  boolean_T DetectChange_vinit;        /* Mask Parameter: DetectChange_vinit
                                        * Referenced by: '<S14>/Delay Input1'
                                        */
  SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan Out1_Y0;/* Computed Parameter: Out1_Y0
                                                              * Referenced by: '<S28>/Out1'
                                                              */
  SL_Bus_Pure_Pursuit_With_VFH_sensor_msgs_LaserScan Constant_Value;/* Computed Parameter: Constant_Value
                                                                     * Referenced by: '<S24>/Constant'
                                                                     */
  SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry Out1_Y0_g;/* Computed Parameter: Out1_Y0_g
                                                            * Referenced by: '<S29>/Out1'
                                                            */
  SL_Bus_Pure_Pursuit_With_VFH_nav_msgs_Odometry Constant_Value_m;/* Computed Parameter: Constant_Value_m
                                                                   * Referenced by: '<S25>/Constant'
                                                                   */
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Twist Constant_Value_e;/* Computed Parameter: Constant_Value_e
                                                                     * Referenced by: '<S31>/Constant'
                                                                     */
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point Out1_Y0_m;/* Computed Parameter: Out1_Y0_m
                                                              * Referenced by: '<S30>/Out1'
                                                              */
  SL_Bus_Pure_Pursuit_With_VFH_geometry_msgs_Point Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                                     * Referenced by: '<S26>/Constant'
                                                                     */
  real_T Constant1_Value;              /* Expression: -1
                                        * Referenced by: '<S1>/Constant1'
                                        */
  real_T Constant_Value_p;             /* Expression: 1
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T VectorFieldHistogram_DistanceLi[2];/* Expression: [ 0.05, 2.0 ]
                                             * Referenced by: '<S1>/Vector Field Histogram'
                                             */
  real_T VectorFieldHistogram_HistogramT[2];/* Expression: [ 3, 6 ]
                                             * Referenced by: '<S1>/Vector Field Histogram'
                                             */
  real_T VectorFieldHistogram_RobotRadiu;/* Expression: 0.3
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T VectorFieldHistogram_SafetyDist;/* Expression: 0.1
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T VectorFieldHistogram_MinTurning;/* Expression: 0.3
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T VectorFieldHistogram_TargetDire;/* Expression: 5
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T VectorFieldHistogram_CurrentDir;/* Expression: 2
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T VectorFieldHistogram_PreviousDi;/* Expression: 2
                                          * Referenced by: '<S1>/Vector Field Histogram'
                                          */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S7>/Gain'
                                        */
  real_T OmegaRec_Value;               /* Expression: 0.2
                                        * Referenced by: '<S7>/OmegaRec'
                                        */
  real_T Constant_Value_lm;            /* Expression: 0
                                        * Referenced by: '<S6>/Constant'
                                        */
  real_T MaxAngularVelocity_Value;     /* Expression: 1
                                        * Referenced by: '<S6>/MaxAngularVelocity'
                                        */
  real_T PurePursuit_DesiredLinearVeloci;/* Expression: 0.3
                                          * Referenced by: '<S2>/Pure Pursuit'
                                          */
  real_T PurePursuit_MaxAngularVelocity;/* Expression: 1.0
                                         * Referenced by: '<S2>/Pure Pursuit'
                                         */
  real_T PurePursuit_LookaheadDistance;/* Expression: 1.0
                                        * Referenced by: '<S2>/Pure Pursuit'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S17>/Integrator'
                                        */
  real_T _Y0;                          /* Expression: initCond
                                        * Referenced by: '<S16>/ '
                                        */
  real_T Constant_Value_o;             /* Expression: 0
                                        * Referenced by: '<S13>/Constant'
                                        */
  real_T GoalRadius_Value;             /* Expression: 0.1
                                        * Referenced by: '<S2>/GoalRadius'
                                        */
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool Constant_Value_pg;/* Computed Parameter: Constant_Value_pg
                                                                * Referenced by: '<S8>/Constant'
                                                                */
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool Constant_Value_oo;/* Computed Parameter: Constant_Value_oo
                                                                * Referenced by: '<S33>/Constant'
                                                                */
  boolean_T Q_Y0;                      /* Computed Parameter: Q_Y0
                                        * Referenced by: '<S9>/Q'
                                        */
  boolean_T Q_Y0_j;                    /* Computed Parameter: Q_Y0_j
                                        * Referenced by: '<S9>/!Q'
                                        */
  boolean_T Memory_InitialCondition;   /* Computed Parameter: Memory_InitialCondition
                                        * Referenced by: '<S3>/Memory'
                                        */
  P_EnabledSubsystem_Pure_Pursu_T EnabledSubsystem_d;/* '<S33>/Enabled Subsystem' */
  P_EnabledSubsystem_Pure_Pursu_T EnabledSubsystem;/* '<S8>/Enabled Subsystem' */
};

/* Real-time Model Data Structure */
struct tag_RTM_Pure_Pursuit_With_VFH_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_Pure_Pursuit_With_VFH_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[1];
  real_T odeF[3][1];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Class declaration for model Pure_Pursuit_With_VFH */
class Pure_Pursuit_With_VFHModelClass {
  /* public data and function members */
 public:
  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  Pure_Pursuit_With_VFHModelClass();

  /* Destructor */
  ~Pure_Pursuit_With_VFHModelClass();

  /* Real-Time Model get method */
  RT_MODEL_Pure_Pursuit_With_VF_T * getRTM();

  /* private data and function members */
 private:
  /* Tunable parameters */
  P_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_P;

  /* Block signals */
  B_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_B;

  /* Block states */
  DW_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_DW;
  X_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_X;/* Block continuous states */
  PrevZCX_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_PrevZCX;/* Triggered events */

  /* Real-Time Model */
  RT_MODEL_Pure_Pursuit_With_VF_T Pure_Pursuit_With_VFH_M;

  /* private member function(s) for subsystem '<Root>'*/
  void VectorFieldHistogramBase_set_Di(robotics_slalgs_internal_Vect_T *obj,
    const real_T val[2]);
  void VectorFieldHistogramBase_set_Hi(robotics_slalgs_internal_Vect_T *obj,
    const real_T val[2]);
  void Pure_Pursuit_With_VFH_linspace(real_T d1, real_T d2, real_T y[240]);
  real_T Pure_Pursuit_With_VFH_floatmod(real_T x);
  real_T Pure_Pursuit_With_VFH_angdiff(real_T x, real_T y);
  void Pure_Pursuit_With_VFH_mod_l(const real_T x[721], real_T r[721]);
  void Pure_Pursuit_With_VF_wrapToPi_b(real_T theta[721]);
  boolean_T Pure_Pursuit_With_V_isequal_n1q(real_T varargin_1, real_T varargin_2);
  real_T Pure_Pursuit_With_VFH_norm(const real_T x[2]);
  void Pure_Pursuit_closestPointOnLine(const real_T pt1[2], const real_T pt2[2],
    const real_T refPt[2], real_T closestPoint[2], real_T *distance);
  void Pu_PurePursuitBase_stepInternal(robotics_slalgs_internal_Pure_T *obj,
    const real_T currentPose[3], const real_T wayptsIn[2], real_T *v, real_T *w,
    real_T lookaheadPoint[2], real_T *targetDir);
  boolean_T Pure_Pursuit_With_VFH_isequal(const real_T varargin_1[2], const
    real_T varargin_2[2]);
  boolean_T Pure_Pursuit_With_VFH_isequal_n(real_T varargin_1, real_T varargin_2);
  void Pure_Pursuit_With_VFH_mod(const real_T x_data[], const int32_T *x_size,
    real_T r_data[], int32_T *r_size);
  void Pure_Pursuit_With_VFH_wrapToPi(const real_T theta_data[], const int32_T
    *theta_size, real_T b_theta_data[], int32_T *b_theta_size);
  void VectorFieldHistogram_parseAndVa(const real_T ranges[721], const real_T
    angles[721], real_T target, real_T scan_InternalRanges_data[], int32_T
    *scan_InternalRanges_size, real_T scan_InternalAngles_data[], int32_T
    *scan_InternalAngles_size, boolean_T *scan_ContainsOnlyFiniteData, real_T
    *b_target);
  real_T Pure_Pursuit_With_VF_wrapToPi_n(real_T theta);
  void Pure_Pursuit_With_VFH_isfinite(const real_T x_data[], boolean_T b_data[],
    int32_T *b_size);
  robotics_core_internal_codege_T *NameValueParser_NameValueParser
    (robotics_core_internal_codege_T *obj);
  void Pure_Pursuit_With_V_wrapToPi_n1(const real_T theta[2], real_T b_theta[2]);
  void Pure_Pursuit_With_VFH_sort(const real_T x[2], real_T b_x[2]);
  boolean_T Pure_Pursuit_With_VFH_all(const boolean_T x_data[], const int32_T
    *x_size);
  void Pure_Pursuit_Wit_nullAssignment(const real_T x_data[], const int32_T
    *x_size, const boolean_T idx_data[], const int32_T *idx_size, real_T
    b_x_data[], int32_T *b_x_size);
  void Pure_Pursuit__validateLaserScan(const real_T inRanges_data[], const
    int32_T *inRanges_size, const real_T inAngles_data[], const int32_T
    *inAngles_size, real_T validRanges_data[], int32_T *validRanges_size, real_T
    validAngles_data[], int32_T *validAngles_size);
  void Pure_Pursui_lidarScan_lidarScan(const real_T varargin_1_data[], const
    int32_T *varargin_1_size, const real_T varargin_2_data[], const int32_T
    *varargin_2_size, real_T obj_InternalRanges_data[], int32_T
    *obj_InternalRanges_size, real_T obj_InternalAngles_data[], int32_T
    *obj_InternalAngles_size, boolean_T *obj_ContainsOnlyFiniteData);
  void Pure_lidarScan_extractValidData(const real_T objIn_InternalRanges_data[],
    const int32_T *objIn_InternalRanges_size, const real_T
    objIn_InternalAngles_data[], const int32_T *objIn_InternalAngles_size, const
    boolean_T validIndices_data[], real_T objOut_InternalRanges_data[], int32_T *
    objOut_InternalRanges_size, real_T objOut_InternalAngles_data[], int32_T
    *objOut_InternalAngles_size, boolean_T *objOut_ContainsOnlyFiniteData);
  void Pur_lidarScan_removeInvalidData(const real_T objIn_InternalRanges_data[],
    const int32_T *objIn_InternalRanges_size, const real_T
    objIn_InternalAngles_data[], const int32_T *objIn_InternalAngles_size,
    boolean_T objIn_ContainsOnlyFiniteData, const real_T varargin_2[2], real_T
    objOut_InternalRanges_data[], int32_T *objOut_InternalRanges_size, real_T
    objOut_InternalAngles_data[], int32_T *objOut_InternalAngles_size, boolean_T
    *objOut_ContainsOnlyFiniteData);
  boolean_T Pure_Pursuit_With_VF_isequal_n1(const real_T
    varargin_1_InternalRanges_data[], const int32_T
    *varargin_1_InternalRanges_size, const real_T
    varargin_1_InternalAngles_data[], const int32_T
    *varargin_1_InternalAngles_size, boolean_T varargin_1_ContainsOnlyFiniteDa,
    const real_T varargin_2_InternalRanges_data[], const int32_T
    *varargin_2_InternalRanges_size, const real_T
    varargin_2_InternalAngles_data[], const int32_T
    *varargin_2_InternalAngles_size, boolean_T varargin_2_ContainsOnlyFiniteDa);
  void Pure_Pursuit_With_VFH_asin(const real_T x_data[], const int32_T *x_size,
    real_T b_x_data[], int32_T *b_x_size);
  void Pure_Pursuit_With_VFH_cos(const real_T x_data[], const int32_T *x_size,
    real_T b_x_data[], int32_T *b_x_size);
  void Pure_Pursuit_With_VFH_sin(const real_T x_data[], const int32_T *x_size,
    real_T b_x_data[], int32_T *b_x_size);
  void Pure_Pursuit_With_VFH_cross(const real_T a_data[], const int32_T a_size[2],
    const real_T b_data[], real_T c_data[], int32_T c_size[2]);
  void Pure_Pursuit_With_VFH_cos_n(const real_T x[240], real_T b_x[240]);
  void Pure_Pursuit_With_VFH_sin_n(const real_T x[240], real_T b_x[240]);
  void Pure_Pursuit_With_VFH_repmat(const real_T a[3], real_T varargin_1, real_T
    b_data[], int32_T b_size[2]);
  void Pure_Pursuit_With_VFH_sign(const real_T x_data[], const int32_T *x_size,
    real_T b_x_data[], int32_T *b_x_size);
  void Pure_Pursuit_With_VFH_abs(const real_T x_data[], const int32_T *x_size,
    real_T y_data[], int32_T *y_size);
  void Pure_Pursuit_With_VFH_histc(const real_T X_data[], const int32_T *X_size,
    const real_T edges[240], real_T N[240], real_T BIN_data[], int32_T *BIN_size);
  void VectorFieldHistogramBase_buildP(robotics_slalgs_internal_Vect_T *obj,
    const real_T scan_InternalRanges_data[], const int32_T
    *scan_InternalRanges_size, const real_T scan_InternalAngles_data[], const
    int32_T *scan_InternalAngles_size, boolean_T scan_ContainsOnlyFiniteData);
  void VectorFieldHistogramBase_buildB(robotics_slalgs_internal_Vect_T *obj);
  void Pure_Pursuit_With_VFH_power(const real_T a_data[], const int32_T *a_size,
    real_T y_data[], int32_T *y_size);
  void Pure_Pursuit_With_VFH_sqrt(const real_T x_data[], const int32_T *x_size,
    real_T b_x_data[], int32_T *b_x_size);
  void VectorFieldHistogramBase_buildM(robotics_slalgs_internal_Vect_T *obj,
    const real_T scan_InternalRanges_data[], const int32_T
    *scan_InternalRanges_size, const real_T scan_InternalAngles_data[], const
    int32_T *scan_InternalAngles_size, boolean_T scan_ContainsOnlyFiniteData);
  void Pure_Pursuit_With_VFH_diff(const real_T x[242], real_T y[241]);
  boolean_T Pure_Pursuit_With_VFH_any(const real_T x[241]);
  void Pure_Pursuit_With_VFH_diff_n(const real_T x_data[], const int32_T x_size
    [2], real_T y_data[], int32_T y_size[2]);
  void Pure_Pursuit_With_VFH_mod_n(const real_T x_data[], const int32_T x_size[2],
    real_T r_data[], int32_T r_size[2]);
  void Pure_Pursuit_With__wrapToPi_n1q(const real_T theta_data[], const int32_T
    theta_size[2], real_T b_theta_data[], int32_T b_theta_size[2]);
  void Pure_Pursuit_With__bisectAngles(real_T theta1_data[], int32_T
    theta1_size[2], real_T theta2_data[], int32_T theta2_size[2], real_T
    bisect_data[], int32_T bisect_size[2]);
  void Pure_Pursuit_With_VFH_bsxfun(const real_T a[240], const real_T b_data[],
    const int32_T *b_size, real_T c_data[], int32_T c_size[2]);
  void Pure_Pursuit_With_VFH_abs_n(const real_T x_data[], const int32_T x_size[2],
    real_T y_data[], int32_T y_size[2]);
  void Pure_Pursuit_With_VFH_bsxfun_n(const real_T a_data[], const int32_T
    a_size[2], const real_T b_data[], const int32_T *b_size, real_T c_data[],
    int32_T c_size[2]);
  void VectorFieldHistogramBase_localC(const real_T candidateDir_data[], const
    real_T selectDir_data[], const int32_T selectDir_size[2], real_T cost_data[],
    int32_T cost_size[2]);
  void VectorFieldHistogramBase_comput(const robotics_slalgs_internal_Vect_T
    *obj, const real_T c_data[], const int32_T c_size[2], real_T targetDir,
    real_T prevDir, real_T cost_data[], int32_T cost_size[2]);
  boolean_T Pure_Pursuit_With_VFH_any_n(const boolean_T x_data[], const int32_T
    x_size[2]);
  real_T VectorFieldHistogramBase_select(robotics_slalgs_internal_Vect_T *obj,
    real_T targetDir);
  real_T VectorFieldHistogramBase_stepIm(robotics_slalgs_internal_Vect_T *obj,
    const real_T varargin_1[721], const real_T varargin_2[721], real_T
    varargin_3);

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void Pure_Pursuit_With_VFH_derivatives();
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S9>/Logic' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Pure_Pursuit_With_VFH'
 * '<S1>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles'
 * '<S2>'   : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following'
 * '<S3>'   : 'Pure_Pursuit_With_VFH/Inputs'
 * '<S4>'   : 'Pure_Pursuit_With_VFH/Outputs'
 * '<S5>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/D Latch'
 * '<S6>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Path Following'
 * '<S7>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Recovery'
 * '<S8>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Subscribe'
 * '<S9>'   : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/D Latch/D Latch'
 * '<S10>'  : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Path Following/Compute Angular Velocity'
 * '<S11>'  : 'Pure_Pursuit_With_VFH/Adjust Velocities to Avoid Obstacles/Subscribe/Enabled Subsystem'
 * '<S12>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/ Extract Goal'
 * '<S13>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/Compare To Zero'
 * '<S14>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/Detect Change'
 * '<S15>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl'
 * '<S16>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/Sample and Hold'
 * '<S17>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Active'
 * '<S18>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Passthrough'
 * '<S19>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Active/Compare To Constant'
 * '<S20>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Active/Compare To Constant1'
 * '<S21>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Active/CorrectionInTheWLimit'
 * '<S22>'  : 'Pure_Pursuit_With_VFH/Compute Velocity and Heading for Path following/LinVelocityControl/Active/CorrectionInTheWLimit/Compare To Constant2'
 * '<S23>'  : 'Pure_Pursuit_With_VFH/Inputs/MATLAB Function'
 * '<S24>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe'
 * '<S25>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe1'
 * '<S26>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe2'
 * '<S27>'  : 'Pure_Pursuit_With_VFH/Inputs/quad2eul'
 * '<S28>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe/Enabled Subsystem'
 * '<S29>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe1/Enabled Subsystem'
 * '<S30>'  : 'Pure_Pursuit_With_VFH/Inputs/Subscribe2/Enabled Subsystem'
 * '<S31>'  : 'Pure_Pursuit_With_VFH/Outputs/Blank Message'
 * '<S32>'  : 'Pure_Pursuit_With_VFH/Outputs/Publish2'
 * '<S33>'  : 'Pure_Pursuit_With_VFH/Outputs/Subscribe'
 * '<S34>'  : 'Pure_Pursuit_With_VFH/Outputs/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_Pure_Pursuit_With_VFH_h_ */

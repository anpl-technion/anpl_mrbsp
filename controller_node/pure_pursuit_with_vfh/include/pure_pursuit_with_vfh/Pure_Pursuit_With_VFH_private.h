/*
 * Pure_Pursuit_With_VFH_private.h
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

#ifndef RTW_HEADER_Pure_Pursuit_With_VFH_private_h_
#define RTW_HEADER_Pure_Pursuit_With_VFH_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "Pure_Pursuit_With_VFH.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void Pure_Purs_EnabledSubsystem_Init(B_EnabledSubsystem_Pure_Pursu_T
  *localB, P_EnabledSubsystem_Pure_Pursu_T *localP);
extern void Pure_Pursuit_W_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool *rtu_In1,
  B_EnabledSubsystem_Pure_Pursu_T *localB);

/* private model entry point functions */
extern void Pure_Pursuit_With_VFH_derivatives();

#endif                                 /* RTW_HEADER_Pure_Pursuit_With_VFH_private_h_ */

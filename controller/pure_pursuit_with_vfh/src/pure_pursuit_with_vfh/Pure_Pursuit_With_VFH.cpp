/*
 * Pure_Pursuit_With_VFH.cpp
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

#include "Pure_Pursuit_With_VFH.h"
#include "Pure_Pursuit_With_VFH_private.h"
#define Pure_Pursuit_Wi_MessageQueueLen (1)

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
void Pure_Pursuit_With_VFHModelClass::rt_ertODEUpdateContinuousStates
  (RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Pure_Pursuit_With_VFH_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  Pure_Pursuit_With_VFH_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  Pure_Pursuit_With_VFH_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S8>/Enabled Subsystem'
 *    '<S33>/Enabled Subsystem'
 */
void Pure_Purs_EnabledSubsystem_Init(B_EnabledSubsystem_Pure_Pursu_T *localB,
  P_EnabledSubsystem_Pure_Pursu_T *localP)
{
  /* SystemInitialize for Outport: '<S11>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Output and update for enable system:
 *    '<S8>/Enabled Subsystem'
 *    '<S33>/Enabled Subsystem'
 */
void Pure_Pursuit_W_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool *rtu_In1,
  B_EnabledSubsystem_Pure_Pursu_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S11>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S11>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S8>/Enabled Subsystem' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_set_Di
  (robotics_slalgs_internal_Vect_T *obj, const real_T val[2])
{
  real_T mtmp;
  int32_T ixstart;
  int32_T ix;
  real_T mtmp_0;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  ixstart = 1;
  mtmp = val[0];
  if (rtIsNaN(val[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 < 3)) {
      ixstart = ix + 1;
      if (!rtIsNaN(val[ix])) {
        mtmp = val[ix];
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 2) {
    while (ixstart + 1 < 3) {
      if (val[ixstart] < mtmp) {
        mtmp = val[ixstart];
      }

      ixstart++;
    }
  }

  ixstart = 1;
  mtmp_0 = val[0];
  if (rtIsNaN(val[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 < 3)) {
      ixstart = ix + 1;
      if (!rtIsNaN(val[ix])) {
        mtmp_0 = val[ix];
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 2) {
    while (ixstart + 1 < 3) {
      if (val[ixstart] > mtmp_0) {
        mtmp_0 = val[ixstart];
      }

      ixstart++;
    }
  }

  obj->DistanceLimits[0] = mtmp;
  obj->DistanceLimits[1] = mtmp_0;

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_set_Hi
  (robotics_slalgs_internal_Vect_T *obj, const real_T val[2])
{
  real_T mtmp;
  int32_T ixstart;
  int32_T ix;
  real_T mtmp_0;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  ixstart = 1;
  mtmp = val[0];
  if (rtIsNaN(val[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 < 3)) {
      ixstart = ix + 1;
      if (!rtIsNaN(val[ix])) {
        mtmp = val[ix];
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 2) {
    while (ixstart + 1 < 3) {
      if (val[ixstart] < mtmp) {
        mtmp = val[ixstart];
      }

      ixstart++;
    }
  }

  ixstart = 1;
  mtmp_0 = val[0];
  if (rtIsNaN(val[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 < 3)) {
      ixstart = ix + 1;
      if (!rtIsNaN(val[ix])) {
        mtmp_0 = val[ix];
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 2) {
    while (ixstart + 1 < 3) {
      if (val[ixstart] > mtmp_0) {
        mtmp_0 = val[ixstart];
      }

      ixstart++;
    }
  }

  obj->HistogramThresholds[0] = mtmp;
  obj->HistogramThresholds[1] = mtmp_0;

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_linspace(real_T d1,
  real_T d2, real_T y[240])
{
  real_T RMD2;
  real_T delta2;
  int32_T c_k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  y[239] = d2;
  y[0] = d1;
  if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307) ||
       (fabs(d2) > 8.9884656743115785E+307))) {
    RMD2 = d1 / 239.0;
    delta2 = d2 / 239.0;
    for (c_k = 0; c_k < 238; c_k++) {
      y[c_k + 1] = ((1.0 + (real_T)c_k) * delta2 + d1) - (1.0 + (real_T)c_k) *
        RMD2;
    }
  } else {
    RMD2 = (d2 - d1) / 239.0;
    for (c_k = 0; c_k < 238; c_k++) {
      y[c_k + 1] = (1.0 + (real_T)c_k) * RMD2 + d1;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

real_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_floatmod(real_T x)
{
  real_T r;
  boolean_T rEQ0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = fmod(x, 6.2831853071795862);
      rEQ0 = (r == 0.0);
      if (!rEQ0) {
        Pure_Pursuit_With_VFH_B.q = fabs(x / 6.2831853071795862);
        rEQ0 = (fabs(Pure_Pursuit_With_VFH_B.q - floor(Pure_Pursuit_With_VFH_B.q
                  + 0.5)) <= 2.2204460492503131E-16 * Pure_Pursuit_With_VFH_B.q);
      }

      if (rEQ0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += 6.2831853071795862;
        }
      }
    }
  } else {
    r = (rtNaN);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return r;
}

real_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_angdiff(real_T x,
  real_T y)
{
  real_T delta;
  real_T d;
  boolean_T pos;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  d = y - x;
  pos = (d + 3.1415926535897931 > 0.0);
  d = Pure_Pursuit_With_VFH_floatmod(d + 3.1415926535897931);
  if ((d == 0.0) && pos) {
    d = 6.2831853071795862;
  }

  delta = d - 3.1415926535897931;

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return delta;
}

/* Function for MATLAB Function: '<S3>/MATLAB Function' */
void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_mod_l(const real_T
  x[721], real_T r[721])
{
  int32_T k;
  real_T b_r;
  boolean_T rEQ0;
  real_T q;
  for (k = 0; k < 721; k++) {
    if ((!rtIsInf(x[k])) && (!rtIsNaN(x[k]))) {
      if (x[k] == 0.0) {
        b_r = 0.0;
      } else {
        b_r = fmod(x[k], 6.2831853071795862);
        rEQ0 = (b_r == 0.0);
        if (!rEQ0) {
          q = fabs(x[k] / 6.2831853071795862);
          rEQ0 = (fabs(q - floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
        }

        if (rEQ0) {
          b_r = 0.0;
        } else {
          if (x[k] < 0.0) {
            b_r += 6.2831853071795862;
          }
        }
      }
    } else {
      b_r = (rtNaN);
    }

    r[k] = b_r;
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function' */
void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VF_wrapToPi_b(real_T
  theta[721])
{
  int32_T i;
  real_T thetaWrap;
  for (i = 0; i < 721; i++) {
    theta[i] += 3.1415926535897931;
  }

  Pure_Pursuit_With_VFH_mod_l(theta, Pure_Pursuit_With_VFH_B.thetaWrap);
  for (i = 0; i < 721; i++) {
    thetaWrap = Pure_Pursuit_With_VFH_B.thetaWrap[i];
    if ((Pure_Pursuit_With_VFH_B.thetaWrap[i] == 0.0) && (theta[i] > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }

    theta[i] = thetaWrap - 3.1415926535897931;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_V_isequal_n1q
  (real_T varargin_1, real_T varargin_2)
{
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  p = false;
  p_0 = true;
  if (!(varargin_1 == varargin_2)) {
    p_0 = false;
  }

  if (p_0) {
    p = true;
  }

  /* End of Start for MATLABSystem: '<S2>/Pure Pursuit' */
  return p;
}

real_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_norm(const real_T
  x[2])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);

  /* End of Start for MATLABSystem: '<S2>/Pure Pursuit' */
  return y;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_closestPointOnLine(const
  real_T pt1[2], const real_T pt2[2], const real_T refPt[2], real_T
  closestPoint[2], real_T *distance)
{
  boolean_T p;
  boolean_T p_0;
  int32_T b_k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    closestPoint[0] = pt1[0];
    Pure_Pursuit_With_VFH_B.refPt[0] = refPt[0] - pt1[0];
    closestPoint[1] = pt1[1];
    Pure_Pursuit_With_VFH_B.refPt[1] = refPt[1] - pt1[1];
    *distance = Pure_Pursuit_With_VFH_norm(Pure_Pursuit_With_VFH_B.refPt);
  } else {
    Pure_Pursuit_With_VFH_B.alpha = pt2[0] - pt1[0];
    Pure_Pursuit_With_VFH_B.closestPoint = (pt2[0] - refPt[0]) *
      Pure_Pursuit_With_VFH_B.alpha;
    Pure_Pursuit_With_VFH_B.d0 = Pure_Pursuit_With_VFH_B.alpha *
      Pure_Pursuit_With_VFH_B.alpha;
    Pure_Pursuit_With_VFH_B.alpha = pt2[1] - pt1[1];
    Pure_Pursuit_With_VFH_B.closestPoint += (pt2[1] - refPt[1]) *
      Pure_Pursuit_With_VFH_B.alpha;
    Pure_Pursuit_With_VFH_B.d0 += Pure_Pursuit_With_VFH_B.alpha *
      Pure_Pursuit_With_VFH_B.alpha;
    Pure_Pursuit_With_VFH_B.alpha = Pure_Pursuit_With_VFH_B.closestPoint /
      Pure_Pursuit_With_VFH_B.d0;
    p = (Pure_Pursuit_With_VFH_B.alpha > 1.0);
    p_0 = (Pure_Pursuit_With_VFH_B.alpha < 0.0);
    if (p) {
      Pure_Pursuit_With_VFH_B.closestPoint = pt1[0];
    } else if (p_0) {
      Pure_Pursuit_With_VFH_B.closestPoint = pt2[0];
    } else {
      Pure_Pursuit_With_VFH_B.closestPoint = (1.0 -
        Pure_Pursuit_With_VFH_B.alpha) * pt2[0] + Pure_Pursuit_With_VFH_B.alpha *
        pt1[0];
    }

    Pure_Pursuit_With_VFH_B.refPt[0] = refPt[0] -
      Pure_Pursuit_With_VFH_B.closestPoint;
    closestPoint[0] = Pure_Pursuit_With_VFH_B.closestPoint;
    if (p) {
      Pure_Pursuit_With_VFH_B.closestPoint = pt1[1];
    } else if (p_0) {
      Pure_Pursuit_With_VFH_B.closestPoint = pt2[1];
    } else {
      Pure_Pursuit_With_VFH_B.closestPoint = (1.0 -
        Pure_Pursuit_With_VFH_B.alpha) * pt2[1] + Pure_Pursuit_With_VFH_B.alpha *
        pt1[1];
    }

    Pure_Pursuit_With_VFH_B.refPt[1] = refPt[1] -
      Pure_Pursuit_With_VFH_B.closestPoint;
    closestPoint[1] = Pure_Pursuit_With_VFH_B.closestPoint;
    *distance = Pure_Pursuit_With_VFH_norm(Pure_Pursuit_With_VFH_B.refPt);
  }

  /* End of Start for MATLABSystem: '<S2>/Pure Pursuit' */
}

void Pure_Pursuit_With_VFHModelClass::Pu_PurePursuitBase_stepInternal
  (robotics_slalgs_internal_Pure_T *obj, const real_T currentPose[3], const
   real_T wayptsIn[2], real_T *v, real_T *w, real_T lookaheadPoint[2], real_T
   *targetDir)
{
  int32_T trueCount;
  boolean_T searchFlag;
  int32_T i;
  int32_T i_0;
  boolean_T b_idx_0;
  boolean_T b_idx_1;
  int32_T obj_tmp;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  b_idx_0 = !rtIsNaN(wayptsIn[0]);
  b_idx_1 = !rtIsNaN(wayptsIn[1]);
  trueCount = 0;
  if (b_idx_0 && b_idx_1) {
    trueCount = 1;
  }

  if (trueCount == 0) {
    *v = 0.0;
    *w = 0.0;
    *targetDir = 0.0;
    lookaheadPoint[0] = currentPose[0];
    lookaheadPoint[1] = currentPose[1];
  } else {
    trueCount = 0;
    if (b_idx_0 && b_idx_1) {
      trueCount = 1;
    }

    for (obj_tmp = 0; obj_tmp < trueCount; obj_tmp++) {
      Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp] = wayptsIn[0];
    }

    for (obj_tmp = 0; obj_tmp < trueCount; obj_tmp++) {
      Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp + trueCount] = wayptsIn[1];
    }

    searchFlag = false;
    if (obj->ProjectionLineIndex == 0.0) {
      searchFlag = true;
      obj->ProjectionPoint[0] = Pure_Pursuit_With_VFH_B.waypoints_data[0];
      obj->ProjectionPoint[1] = Pure_Pursuit_With_VFH_B.waypoints_data[trueCount];
      obj->ProjectionLineIndex = 1.0;
    }

    if (trueCount == 1) {
      obj->ProjectionPoint[0] = Pure_Pursuit_With_VFH_B.waypoints_data[0];
      obj->ProjectionPoint[1] = Pure_Pursuit_With_VFH_B.waypoints_data[trueCount];
    } else {
      obj_tmp = (int32_T)(obj->ProjectionLineIndex + 1.0);
      Pure_Pursuit_With_VFH_B.waypoints[0] =
        Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp - 1];
      Pure_Pursuit_With_VFH_B.waypoints[1] =
        Pure_Pursuit_With_VFH_B.waypoints_data[(obj_tmp + trueCount) - 1];
      Pure_Pursuit_closestPointOnLine(obj->ProjectionPoint,
        Pure_Pursuit_With_VFH_B.waypoints, &currentPose[0],
        Pure_Pursuit_With_VFH_B.lookaheadStartPt,
        &Pure_Pursuit_With_VFH_B.minDistance);
      obj->ProjectionPoint[0] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[0];
      Pure_Pursuit_With_VFH_B.waypoints[0] = obj->ProjectionPoint[0] -
        Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp - 1];
      obj->ProjectionPoint[1] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[1];
      Pure_Pursuit_With_VFH_B.waypoints[1] = obj->ProjectionPoint[1] -
        Pure_Pursuit_With_VFH_B.waypoints_data[(obj_tmp + trueCount) - 1];
      Pure_Pursuit_With_VFH_B.dist = Pure_Pursuit_With_VFH_norm
        (Pure_Pursuit_With_VFH_B.waypoints);
      Pure_Pursuit_With_VFH_B.lookaheadIdx = obj->ProjectionLineIndex + 1.0;
      obj_tmp = 0;
      exitg1 = false;
      while ((!exitg1) && (obj_tmp <= (int32_T)((1.0 -
                Pure_Pursuit_With_VFH_B.lookaheadIdx) + -1.0) - 1)) {
        Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 =
          Pure_Pursuit_With_VFH_B.lookaheadIdx + (real_T)obj_tmp;
        if ((!searchFlag) && (Pure_Pursuit_With_VFH_B.dist >
                              obj->LookaheadDistance)) {
          exitg1 = true;
        } else {
          i = (int32_T)Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0;
          i_0 = (int32_T)(Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 + 1.0);
          Pure_Pursuit_With_VFH_B.waypoints[0] =
            Pure_Pursuit_With_VFH_B.waypoints_data[i - 1] -
            Pure_Pursuit_With_VFH_B.waypoints_data[i_0 - 1];
          Pure_Pursuit_With_VFH_B.waypoints[1] =
            Pure_Pursuit_With_VFH_B.waypoints_data[(i + trueCount) - 1] -
            Pure_Pursuit_With_VFH_B.waypoints_data[(i_0 + trueCount) - 1];
          Pure_Pursuit_With_VFH_B.dist += Pure_Pursuit_With_VFH_norm
            (Pure_Pursuit_With_VFH_B.waypoints);
          i = (int32_T)Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0;
          i_0 = (int32_T)(Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 + 1.0);
          Pure_Pursuit_With_VFH_B.waypoints[0] =
            Pure_Pursuit_With_VFH_B.waypoints_data[i - 1];
          Pure_Pursuit_With_VFH_B.waypoints_b[0] =
            Pure_Pursuit_With_VFH_B.waypoints_data[i_0 - 1];
          Pure_Pursuit_With_VFH_B.waypoints[1] =
            Pure_Pursuit_With_VFH_B.waypoints_data[(i + trueCount) - 1];
          Pure_Pursuit_With_VFH_B.waypoints_b[1] =
            Pure_Pursuit_With_VFH_B.waypoints_data[(i_0 + trueCount) - 1];
          Pure_Pursuit_closestPointOnLine(Pure_Pursuit_With_VFH_B.waypoints,
            Pure_Pursuit_With_VFH_B.waypoints_b, &currentPose[0],
            Pure_Pursuit_With_VFH_B.lookaheadStartPt,
            &Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1);
          if (Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1 <
              Pure_Pursuit_With_VFH_B.minDistance) {
            Pure_Pursuit_With_VFH_B.minDistance =
              Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1;
            obj->ProjectionPoint[0] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[0];
            obj->ProjectionPoint[1] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[1];
            obj->ProjectionLineIndex =
              Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0;
          }

          obj_tmp++;
        }
      }
    }

    trueCount = 0;
    if (b_idx_0 && b_idx_1) {
      trueCount = 1;
    }

    for (obj_tmp = 0; obj_tmp < trueCount; obj_tmp++) {
      Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp] = wayptsIn[0];
    }

    for (obj_tmp = 0; obj_tmp < trueCount; obj_tmp++) {
      Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp + trueCount] = wayptsIn[1];
    }

    if (trueCount == 1) {
      Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] =
        Pure_Pursuit_With_VFH_B.waypoints_data[0];
      Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] =
        Pure_Pursuit_With_VFH_B.waypoints_data[trueCount];
    } else {
      obj_tmp = (int32_T)(obj->ProjectionLineIndex + 1.0);
      Pure_Pursuit_With_VFH_B.waypoints[0] = obj->ProjectionPoint[0] -
        Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp - 1];
      Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] = obj->ProjectionPoint[0];
      Pure_Pursuit_With_VFH_B.waypoints[1] = obj->ProjectionPoint[1] -
        Pure_Pursuit_With_VFH_B.waypoints_data[(obj_tmp + trueCount) - 1];
      Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] = obj->ProjectionPoint[1];
      Pure_Pursuit_With_VFH_B.dist = Pure_Pursuit_With_VFH_norm
        (Pure_Pursuit_With_VFH_B.waypoints);
      obj_tmp = (int32_T)(obj->ProjectionLineIndex + 1.0);
      Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 =
        Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp - 1];
      Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1 =
        Pure_Pursuit_With_VFH_B.waypoints_data[(obj_tmp + trueCount) - 1];
      Pure_Pursuit_With_VFH_B.minDistance = Pure_Pursuit_With_VFH_B.dist -
        obj->LookaheadDistance;
      Pure_Pursuit_With_VFH_B.lookaheadIdx = obj->ProjectionLineIndex;
      while ((Pure_Pursuit_With_VFH_B.minDistance < 0.0) &&
             (Pure_Pursuit_With_VFH_B.lookaheadIdx < -1.0)) {
        Pure_Pursuit_With_VFH_B.lookaheadIdx++;
        obj_tmp = (int32_T)Pure_Pursuit_With_VFH_B.lookaheadIdx;
        i = (int32_T)(Pure_Pursuit_With_VFH_B.lookaheadIdx + 1.0);
        i_0 = (int32_T)Pure_Pursuit_With_VFH_B.lookaheadIdx;
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] =
          Pure_Pursuit_With_VFH_B.waypoints_data[obj_tmp - 1];
        Pure_Pursuit_With_VFH_B.waypoints[0] =
          Pure_Pursuit_With_VFH_B.waypoints_data[i_0 - 1] -
          Pure_Pursuit_With_VFH_B.waypoints_data[i - 1];
        Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 =
          Pure_Pursuit_With_VFH_B.waypoints_data[i - 1];
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] =
          Pure_Pursuit_With_VFH_B.waypoints_data[(obj_tmp + trueCount) - 1];
        Pure_Pursuit_With_VFH_B.waypoints[1] =
          Pure_Pursuit_With_VFH_B.waypoints_data[(i_0 + trueCount) - 1] -
          Pure_Pursuit_With_VFH_B.waypoints_data[(i + trueCount) - 1];
        Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1 =
          Pure_Pursuit_With_VFH_B.waypoints_data[(i + trueCount) - 1];
        Pure_Pursuit_With_VFH_B.dist += Pure_Pursuit_With_VFH_norm
          (Pure_Pursuit_With_VFH_B.waypoints);
        Pure_Pursuit_With_VFH_B.minDistance = Pure_Pursuit_With_VFH_B.dist -
          obj->LookaheadDistance;
      }

      Pure_Pursuit_With_VFH_B.waypoints_data[0] =
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] -
        Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0;
      Pure_Pursuit_With_VFH_B.waypoints_data[1] =
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] -
        Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1;
      Pure_Pursuit_With_VFH_B.dist = Pure_Pursuit_With_VFH_B.minDistance /
        Pure_Pursuit_With_VFH_norm(Pure_Pursuit_With_VFH_B.waypoints_data);
      if (Pure_Pursuit_With_VFH_B.dist > 0.0) {
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] = (1.0 -
          Pure_Pursuit_With_VFH_B.dist) *
          Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0 +
          Pure_Pursuit_With_VFH_B.dist *
          Pure_Pursuit_With_VFH_B.lookaheadStartPt[0];
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] = (1.0 -
          Pure_Pursuit_With_VFH_B.dist) *
          Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1 +
          Pure_Pursuit_With_VFH_B.dist *
          Pure_Pursuit_With_VFH_B.lookaheadStartPt[1];
      } else {
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[0] =
          Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_0;
        Pure_Pursuit_With_VFH_B.lookaheadStartPt[1] =
          Pure_Pursuit_With_VFH_B.lookaheadEndPt_idx_1;
      }
    }

    obj->LookaheadPoint[0] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[0];
    obj->LookaheadPoint[1] = Pure_Pursuit_With_VFH_B.lookaheadStartPt[1];
    Pure_Pursuit_With_VFH_B.dist = rt_atan2d_snf(obj->LookaheadPoint[1] -
      currentPose[1], obj->LookaheadPoint[0] - currentPose[0]) - currentPose[2];
    if ((!rtIsInf(Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931)) &&
        (!rtIsNaN(Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931))) {
      if (Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931 == 0.0) {
        Pure_Pursuit_With_VFH_B.minDistance = 0.0;
      } else {
        Pure_Pursuit_With_VFH_B.minDistance = fmod(Pure_Pursuit_With_VFH_B.dist
          + 3.1415926535897931, 6.2831853071795862);
        b_idx_0 = (Pure_Pursuit_With_VFH_B.minDistance == 0.0);
        if (!b_idx_0) {
          Pure_Pursuit_With_VFH_B.lookaheadIdx = fabs
            ((Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931) /
             6.2831853071795862);
          b_idx_0 = (fabs(Pure_Pursuit_With_VFH_B.lookaheadIdx - floor
                          (Pure_Pursuit_With_VFH_B.lookaheadIdx + 0.5)) <=
                     2.2204460492503131E-16 *
                     Pure_Pursuit_With_VFH_B.lookaheadIdx);
        }

        if (b_idx_0) {
          Pure_Pursuit_With_VFH_B.minDistance = 0.0;
        } else {
          if (Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931 < 0.0) {
            Pure_Pursuit_With_VFH_B.minDistance += 6.2831853071795862;
          }
        }
      }
    } else {
      Pure_Pursuit_With_VFH_B.minDistance = (rtNaN);
    }

    Pure_Pursuit_With_VFH_B.b = Pure_Pursuit_With_VFH_B.minDistance;
    if ((Pure_Pursuit_With_VFH_B.minDistance == 0.0) &&
        (Pure_Pursuit_With_VFH_B.dist + 3.1415926535897931 > 0.0)) {
      Pure_Pursuit_With_VFH_B.b = 6.2831853071795862;
    }

    *w = sin(Pure_Pursuit_With_VFH_B.b - 3.1415926535897931) * 2.0 /
      obj->LookaheadDistance;
    if (fabs(fabs(Pure_Pursuit_With_VFH_B.b - 3.1415926535897931) -
             3.1415926535897931) < 1.4901161193847656E-8) {
      if (*w < 0.0) {
        *w = -1.0;
      } else if (*w > 0.0) {
        *w = 1.0;
      } else if (*w == 0.0) {
        *w = 0.0;
      } else {
        *w = (rtNaN);
      }
    }

    if (fabs(*w) > obj->MaxAngularVelocity) {
      if (*w < 0.0) {
        *w = -1.0;
      } else if (*w > 0.0) {
        *w = 1.0;
      } else if (*w == 0.0) {
        *w = 0.0;
      } else {
        *w = (rtNaN);
      }

      *w *= obj->MaxAngularVelocity;
    }

    *v = obj->DesiredLinearVelocity;
    lookaheadPoint[0] = obj->LookaheadPoint[0];
    lookaheadPoint[1] = obj->LookaheadPoint[1];
    obj->LastPose[0] = currentPose[0];
    obj->LastPose[1] = currentPose[1];
    obj->LastPose[2] = currentPose[2];
    *targetDir = Pure_Pursuit_With_VFH_B.b - 3.1415926535897931;
    if (rtIsNaN(Pure_Pursuit_With_VFH_B.b - 3.1415926535897931)) {
      *targetDir = 0.0;
    }
  }

  /* End of Start for MATLABSystem: '<S2>/Pure Pursuit' */
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_isequal(const
  real_T varargin_1[2], const real_T varargin_2[2])
{
  boolean_T p;
  boolean_T p_0;
  int32_T b_k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return p;
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_isequal_n
  (real_T varargin_1, real_T varargin_2)
{
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  p = false;
  p_0 = true;
  if (!(varargin_1 == varargin_2)) {
    p_0 = false;
  }

  if (p_0) {
    p = true;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return p;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_mod(const real_T
  x_data[], const int32_T *x_size, real_T r_data[], int32_T *r_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if (0 <= *x_size - 1) {
    memcpy(&Pure_Pursuit_With_VFH_B.b_z1_data[0], &r_data[0], *x_size * sizeof
           (real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    Pure_Pursuit_With_VFH_B.b_z1_data[loop_ub] = Pure_Pursuit_With_VFH_floatmod
      (x_data[loop_ub]);
  }

  *r_size = *x_size;
  if (0 <= *x_size - 1) {
    memcpy(&r_data[0], &Pure_Pursuit_With_VFH_B.b_z1_data[0], *x_size * sizeof
           (real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_wrapToPi(const
  real_T theta_data[], const int32_T *theta_size, real_T b_theta_data[], int32_T
  *b_theta_size)
{
  int32_T i;
  int32_T loop_ub;
  loop_ub = *theta_size;
  for (i = 0; i < loop_ub; i++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.theta_data[i] = theta_data[i] + 3.1415926535897931;
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_mod(Pure_Pursuit_With_VFH_B.theta_data, theta_size,
    b_theta_data, b_theta_size);
  loop_ub = *b_theta_size;
  for (i = 0; i < loop_ub; i++) {
    Pure_Pursuit_With_VFH_B.b_data_d[i] = (b_theta_data[i] == 0.0);
  }

  loop_ub = *b_theta_size - 1;
  for (i = 0; i <= loop_ub; i++) {
    if (Pure_Pursuit_With_VFH_B.b_data_d[i] &&
        (Pure_Pursuit_With_VFH_B.theta_data[i] > 0.0)) {
      b_theta_data[i] = 6.2831853071795862;
    }
  }

  loop_ub = *b_theta_size;
  for (i = 0; i < loop_ub; i++) {
    b_theta_data[i] -= 3.1415926535897931;
  }
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogram_parseAndVa(const
  real_T ranges[721], const real_T angles[721], real_T target, real_T
  scan_InternalRanges_data[], int32_T *scan_InternalRanges_size, real_T
  scan_InternalAngles_data[], int32_T *scan_InternalAngles_size, boolean_T
  *scan_ContainsOnlyFiniteData, real_T *b_target)
{
  static int32_T tmp = 721;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_target = target;
  *scan_ContainsOnlyFiniteData = false;
  *scan_InternalRanges_size = 721;
  memcpy(&scan_InternalRanges_data[0], &ranges[0], 721U * sizeof(real_T));
  Pure_Pursuit_With_VFH_wrapToPi(angles, &tmp, scan_InternalAngles_data,
    scan_InternalAngles_size);
}

real_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VF_wrapToPi_n(real_T
  theta)
{
  real_T b_theta;
  real_T thetaWrap;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  thetaWrap = Pure_Pursuit_With_VFH_floatmod(theta + 3.1415926535897931);
  if ((thetaWrap == 0.0) && (theta + 3.1415926535897931 > 0.0)) {
    thetaWrap = 6.2831853071795862;
  }

  b_theta = thetaWrap - 3.1415926535897931;

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return b_theta;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_isfinite(const
  real_T x_data[], boolean_T b_data[], int32_T *b_size)
{
  int32_T i;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_size = 721;
  for (i = 0; i < 721; i++) {
    b_data[i] = ((!rtIsInf(x_data[i])) && (!rtIsNaN(x_data[i])));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

robotics_core_internal_codege_T *Pure_Pursuit_With_VFHModelClass::
  NameValueParser_NameValueParser(robotics_core_internal_codege_T *obj)
{
  robotics_core_internal_codege_T *b_obj;
  cell_wrap_Pure_Pursuit_With_V_T c;
  cell_wrap_Pure_Pursuit_With_V_T d;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  c.f1[0] = 0.0;
  d.f1[0] = -3.1415926535897931;
  c.f1[1] = (rtInf);
  d.f1[1] = 3.1415926535897931;
  b_obj = obj;
  obj->Defaults[0] = c;
  obj->Defaults[1] = d;
  return b_obj;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_V_wrapToPi_n1(const
  real_T theta[2], real_T b_theta[2])
{
  real_T theta_0[2];
  int32_T i;
  real_T b_theta_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  theta_0[0] = theta[0] + 3.1415926535897931;
  b_theta[0] = Pure_Pursuit_With_VFH_floatmod(theta[0] + 3.1415926535897931);
  theta_0[1] = theta[1] + 3.1415926535897931;
  b_theta[1] = Pure_Pursuit_With_VFH_floatmod(theta[1] + 3.1415926535897931);
  for (i = 0; i < 2; i++) {
    b_theta_0 = b_theta[i];
    if ((b_theta[i] == 0.0) && (theta_0[i] > 0.0)) {
      b_theta_0 = 6.2831853071795862;
    }

    b_theta[i] = b_theta_0;
  }

  b_theta[0] -= 3.1415926535897931;
  b_theta[1] -= 3.1415926535897931;

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_sort(const real_T x
  [2], real_T b_x[2])
{
  boolean_T b_p;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  b_x[0] = x[0];
  b_x[1] = x[1];
  if ((x[0] <= x[1]) || rtIsNaN(x[1])) {
    b_p = true;
  } else {
    b_p = false;
  }

  if (!b_p) {
    b_x[0] = x[1];
    b_x[1] = x[0];
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_all(const
  boolean_T x_data[], const int32_T *x_size)
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= *x_size)) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_Wit_nullAssignment(const
  real_T x_data[], const int32_T *x_size, const boolean_T idx_data[], const
  int32_T *idx_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T nxout;
  int32_T k0;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  nxout = *x_size;
  if (0 <= nxout - 1) {
    memcpy(&b_x_data[0], &x_data[0], nxout * sizeof(real_T));
  }

  nxout = 0;
  for (k = 0; k + 1 <= *idx_size; k++) {
    nxout += idx_data[k];
  }

  nxout = *x_size - nxout;
  k0 = -1;
  for (k = 0; k + 1 <= *x_size; k++) {
    if ((k + 1 > *idx_size) || (!idx_data[k])) {
      k0++;
      b_x_data[k0] = b_x_data[k];
    }
  }

  if (1 > nxout) {
    nxout = 0;
  }

  if (0 <= nxout - 1) {
    memcpy(&Pure_Pursuit_With_VFH_B.b_x_data[0], &b_x_data[0], nxout * sizeof
           (real_T));
  }

  *b_x_size = nxout;
  if (0 <= nxout - 1) {
    memcpy(&b_x_data[0], &Pure_Pursuit_With_VFH_B.b_x_data[0], nxout * sizeof
           (real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit__validateLaserScan(const
  real_T inRanges_data[], const int32_T *inRanges_size, const real_T
  inAngles_data[], const int32_T *inAngles_size, real_T validRanges_data[],
  int32_T *validRanges_size, real_T validAngles_data[], int32_T
  *validAngles_size)
{
  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *validRanges_size = *inRanges_size;
  if (0 <= *inRanges_size - 1) {
    memcpy(&validRanges_data[0], &inRanges_data[0], *inRanges_size * sizeof
           (real_T));
  }

  *validAngles_size = *inAngles_size;
  if (0 <= *inAngles_size - 1) {
    memcpy(&validAngles_data[0], &inAngles_data[0], *inAngles_size * sizeof
           (real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursui_lidarScan_lidarScan(const
  real_T varargin_1_data[], const int32_T *varargin_1_size, const real_T
  varargin_2_data[], const int32_T *varargin_2_size, real_T
  obj_InternalRanges_data[], int32_T *obj_InternalRanges_size, real_T
  obj_InternalAngles_data[], int32_T *obj_InternalAngles_size, boolean_T
  *obj_ContainsOnlyFiniteData)
{
  int32_T b_validAngles_size;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *obj_ContainsOnlyFiniteData = false;
  Pure_Pursuit__validateLaserScan(varargin_1_data, varargin_1_size,
    varargin_2_data, varargin_2_size, obj_InternalRanges_data,
    obj_InternalRanges_size, Pure_Pursuit_With_VFH_B.b_validAngles_data,
    &b_validAngles_size);
  Pure_Pursuit_With_VFH_wrapToPi(Pure_Pursuit_With_VFH_B.b_validAngles_data,
    &b_validAngles_size, obj_InternalAngles_data, obj_InternalAngles_size);
}

void Pure_Pursuit_With_VFHModelClass::Pure_lidarScan_extractValidData(const
  real_T objIn_InternalRanges_data[], const int32_T *objIn_InternalRanges_size,
  const real_T objIn_InternalAngles_data[], const int32_T
  *objIn_InternalAngles_size, const boolean_T validIndices_data[], real_T
  objOut_InternalRanges_data[], int32_T *objOut_InternalRanges_size, real_T
  objOut_InternalAngles_data[], int32_T *objOut_InternalAngles_size, boolean_T
  *objOut_ContainsOnlyFiniteData)
{
  int32_T i;
  int32_T validIndices_size;
  int32_T tmp_size;
  int32_T validIndices_size_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  validIndices_size = 721;
  for (i = 0; i < 721; i++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.validIndices_data_l[i] = !validIndices_data[i];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_Wit_nullAssignment(objIn_InternalRanges_data,
    objIn_InternalRanges_size, Pure_Pursuit_With_VFH_B.validIndices_data_l,
    &validIndices_size, Pure_Pursuit_With_VFH_B.tmp_data_c, &tmp_size);
  validIndices_size_0 = 721;
  for (i = 0; i < 721; i++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.validIndices_data_l[i] = !validIndices_data[i];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_Wit_nullAssignment(objIn_InternalAngles_data,
    objIn_InternalAngles_size, Pure_Pursuit_With_VFH_B.validIndices_data_l,
    &validIndices_size_0, Pure_Pursuit_With_VFH_B.tmp_data_k, &validIndices_size);
  Pure_Pursui_lidarScan_lidarScan(Pure_Pursuit_With_VFH_B.tmp_data_c, &tmp_size,
    Pure_Pursuit_With_VFH_B.tmp_data_k, &validIndices_size,
    objOut_InternalRanges_data, objOut_InternalRanges_size,
    objOut_InternalAngles_data, objOut_InternalAngles_size,
    objOut_ContainsOnlyFiniteData);
}

void Pure_Pursuit_With_VFHModelClass::Pur_lidarScan_removeInvalidData(const
  real_T objIn_InternalRanges_data[], const int32_T *objIn_InternalRanges_size,
  const real_T objIn_InternalAngles_data[], const int32_T
  *objIn_InternalAngles_size, boolean_T objIn_ContainsOnlyFiniteData, const
  real_T varargin_2[2], real_T objOut_InternalRanges_data[], int32_T
  *objOut_InternalRanges_size, real_T objOut_InternalAngles_data[], int32_T
  *objOut_InternalAngles_size, boolean_T *objOut_ContainsOnlyFiniteData)
{
  static real_T tmp[2] = { 0.0, 0.0 };

  int32_T tmp_size;
  tmp[1U] = (rtInf);

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if (!objIn_ContainsOnlyFiniteData) {
    Pure_Pursuit_With_VFH_isfinite(objIn_InternalRanges_data,
      Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data, &tmp_size);
    Pure_Pursuit_With_VFH_isfinite(objIn_InternalAngles_data,
      Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data, &tmp_size);
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 < 721;
         Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validIndices_data[Pure_Pursuit_With_VFH_B.i0] =
        (Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
         &&
         Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]);
    }
  } else {
    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalRanges_size;
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 <
         Pure_Pursuit_With_VFH_B.loop_ub_m; Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validIndices_data[Pure_Pursuit_With_VFH_B.i0] =
        true;
    }
  }

  for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 < 721;
       Pure_Pursuit_With_VFH_B.i0++) {
    Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
      = true;
    Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
      = true;
  }

  NameValueParser_NameValueParser(&Pure_Pursuit_With_VFH_B.parser);
  Pure_Pursuit_With_VFH_B.parsedResults[0].f1[0] = varargin_2[0];
  Pure_Pursuit_With_VFH_B.parsedResults[0].f1[1] = varargin_2[1];
  Pure_Pursuit_With_VFH_B.parsedRangeLimits[0] =
    Pure_Pursuit_With_VFH_B.parser.Defaults[1].f1[0];
  Pure_Pursuit_With_VFH_B.parsedRangeLimits[1] =
    Pure_Pursuit_With_VFH_B.parser.Defaults[1].f1[1];
  Pure_Pursuit_With_VFH_B.parsedResults[1].f1[0] =
    Pure_Pursuit_With_VFH_B.parsedRangeLimits[0];
  Pure_Pursuit_With_VFH_B.parsedResults[1].f1[1] =
    Pure_Pursuit_With_VFH_B.parsedRangeLimits[1];
  Pure_Pursuit_With_VFH_B.parser.ParsedResults[0] =
    Pure_Pursuit_With_VFH_B.parsedResults[0];
  Pure_Pursuit_With_VFH_B.parser.ParsedResults[1] =
    Pure_Pursuit_With_VFH_B.parsedResults[1];
  Pure_Pursuit_With_VFH_B.parsedRangeLimits[0] =
    Pure_Pursuit_With_VFH_B.parser.ParsedResults[0].f1[0];
  Pure_Pursuit_With_VFH_B.parsedRangeLimits[1] =
    Pure_Pursuit_With_VFH_B.parser.ParsedResults[0].f1[1];
  Pure_Pursuit_With_VFH_B.parsedAngleLimits[0] =
    Pure_Pursuit_With_VFH_B.parser.ParsedResults[1].f1[0];
  Pure_Pursuit_With_VFH_B.parsedAngleLimits[1] =
    Pure_Pursuit_With_VFH_B.parser.ParsedResults[1].f1[1];
  if (!Pure_Pursuit_With_VFH_isequal(Pure_Pursuit_With_VFH_B.parsedRangeLimits,
       tmp)) {
    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalRanges_size;
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 <
         Pure_Pursuit_With_VFH_B.loop_ub_m; Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
        = (objIn_InternalRanges_data[Pure_Pursuit_With_VFH_B.i0] >=
           Pure_Pursuit_With_VFH_B.parsedRangeLimits[0]);
    }

    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalRanges_size;
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 <
         Pure_Pursuit_With_VFH_B.loop_ub_m; Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.g_data[Pure_Pursuit_With_VFH_B.i0] =
        (objIn_InternalRanges_data[Pure_Pursuit_With_VFH_B.i0] <=
         Pure_Pursuit_With_VFH_B.parsedRangeLimits[1]);
    }

    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 < 721;
         Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
        =
        (Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
         && Pure_Pursuit_With_VFH_B.g_data[Pure_Pursuit_With_VFH_B.i0]);
    }
  }

  Pure_Pursuit_With_VFH_B.parsedRangeLimits[0] = -3.1415926535897931;
  Pure_Pursuit_With_VFH_B.parsedRangeLimits[1] = 3.1415926535897931;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if (!Pure_Pursuit_With_VFH_isequal(Pure_Pursuit_With_VFH_B.parsedAngleLimits,
       Pure_Pursuit_With_VFH_B.parsedRangeLimits)) {
    Pure_Pursuit_With_V_wrapToPi_n1(Pure_Pursuit_With_VFH_B.parsedAngleLimits,
      Pure_Pursuit_With_VFH_B.parsedRangeLimits);
    Pure_Pursuit_With_VFH_sort(Pure_Pursuit_With_VFH_B.parsedRangeLimits,
      Pure_Pursuit_With_VFH_B.parsedAngleLimits);
    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalAngles_size;
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 <
         Pure_Pursuit_With_VFH_B.loop_ub_m; Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
        = (objIn_InternalAngles_data[Pure_Pursuit_With_VFH_B.i0] >=
           Pure_Pursuit_With_VFH_B.parsedAngleLimits[0]);
    }

    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalAngles_size;
    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 <
         Pure_Pursuit_With_VFH_B.loop_ub_m; Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.g_data[Pure_Pursuit_With_VFH_B.i0] =
        (objIn_InternalAngles_data[Pure_Pursuit_With_VFH_B.i0] <=
         Pure_Pursuit_With_VFH_B.parsedAngleLimits[1]);
    }

    for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 < 721;
         Pure_Pursuit_With_VFH_B.i0++) {
      Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
        =
        (Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
         && Pure_Pursuit_With_VFH_B.g_data[Pure_Pursuit_With_VFH_B.i0]);
    }
  }

  tmp_size = 721;
  for (Pure_Pursuit_With_VFH_B.i0 = 0; Pure_Pursuit_With_VFH_B.i0 < 721;
       Pure_Pursuit_With_VFH_B.i0++) {
    Pure_Pursuit_With_VFH_B.validIndices_data[Pure_Pursuit_With_VFH_B.i0] =
      (Pure_Pursuit_With_VFH_B.validIndices_data[Pure_Pursuit_With_VFH_B.i0] &&
       Pure_Pursuit_With_VFH_B.validRangeLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]
       &&
       Pure_Pursuit_With_VFH_B.validAngleLimitIndices_data[Pure_Pursuit_With_VFH_B.i0]);
  }

  if (!Pure_Pursuit_With_VFH_all(Pure_Pursuit_With_VFH_B.validIndices_data,
       &tmp_size)) {
    Pure_lidarScan_extractValidData(objIn_InternalRanges_data,
      objIn_InternalRanges_size, objIn_InternalAngles_data,
      objIn_InternalAngles_size, Pure_Pursuit_With_VFH_B.validIndices_data,
      objOut_InternalRanges_data, objOut_InternalRanges_size,
      objOut_InternalAngles_data, objOut_InternalAngles_size,
      objOut_ContainsOnlyFiniteData);
    *objOut_ContainsOnlyFiniteData = true;
  } else {
    *objOut_InternalRanges_size = *objIn_InternalRanges_size;
    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalRanges_size;
    if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_m - 1) {
      memcpy(&objOut_InternalRanges_data[0], &objIn_InternalRanges_data[0],
             Pure_Pursuit_With_VFH_B.loop_ub_m * sizeof(real_T));
    }

    *objOut_InternalAngles_size = *objIn_InternalAngles_size;
    Pure_Pursuit_With_VFH_B.loop_ub_m = *objIn_InternalAngles_size;
    if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_m - 1) {
      memcpy(&objOut_InternalAngles_data[0], &objIn_InternalAngles_data[0],
             Pure_Pursuit_With_VFH_B.loop_ub_m * sizeof(real_T));
    }

    *objOut_ContainsOnlyFiniteData = true;
  }
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VF_isequal_n1(const
  real_T varargin_1_InternalRanges_data[], const int32_T
  *varargin_1_InternalRanges_size, const real_T varargin_1_InternalAngles_data[],
  const int32_T *varargin_1_InternalAngles_size, boolean_T
  varargin_1_ContainsOnlyFiniteDa, const real_T varargin_2_InternalRanges_data[],
  const int32_T *varargin_2_InternalRanges_size, const real_T
  varargin_2_InternalAngles_data[], const int32_T
  *varargin_2_InternalAngles_size, boolean_T varargin_2_ContainsOnlyFiniteDa)
{
  boolean_T p;
  boolean_T p_0;
  int32_T b_k;
  boolean_T p_1;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  p = false;
  p_0 = true;
  p_1 = false;
  if (*varargin_1_InternalRanges_size == *varargin_2_InternalRanges_size) {
    p_1 = true;
  }

  if (p_1 && (!(*varargin_1_InternalRanges_size == 0)) &&
      (!(*varargin_2_InternalRanges_size == 0))) {
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k <= *varargin_2_InternalRanges_size - 1)) {
      if (!(varargin_1_InternalRanges_data[b_k] ==
            varargin_2_InternalRanges_data[b_k])) {
        p_1 = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }

  if (p_1) {
    p_1 = false;
    if (*varargin_1_InternalAngles_size == *varargin_2_InternalAngles_size) {
      p_1 = true;
    }

    if (p_1 && (!(*varargin_1_InternalAngles_size == 0)) &&
        (!(*varargin_2_InternalAngles_size == 0))) {
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= *varargin_2_InternalAngles_size - 1)) {
        if (!(varargin_1_InternalAngles_data[b_k] ==
              varargin_2_InternalAngles_data[b_k])) {
          p_1 = false;
          exitg1 = true;
        } else {
          b_k++;
        }
      }
    }

    if (p_1 && (varargin_1_ContainsOnlyFiniteDa !=
                varargin_2_ContainsOnlyFiniteDa)) {
      p_1 = false;
    }
  }

  if (!p_1) {
    p_0 = false;
  }

  if (p_0) {
    p = true;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return p;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_asin(const real_T
  x_data[], const int32_T *x_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_x_size = *x_size;
  loop_ub = *x_size;
  if (0 <= loop_ub - 1) {
    memcpy(&b_x_data[0], &x_data[0], loop_ub * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    b_x_data[loop_ub] = asin(b_x_data[loop_ub]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_cos(const real_T
  x_data[], const int32_T *x_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_x_size = *x_size;
  loop_ub = *x_size;
  if (0 <= loop_ub - 1) {
    memcpy(&b_x_data[0], &x_data[0], loop_ub * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    b_x_data[loop_ub] = cos(b_x_data[loop_ub]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_sin(const real_T
  x_data[], const int32_T *x_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_x_size = *x_size;
  loop_ub = *x_size;
  if (0 <= loop_ub - 1) {
    memcpy(&b_x_data[0], &x_data[0], loop_ub * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    b_x_data[loop_ub] = sin(b_x_data[loop_ub]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_cross(const real_T
  a_data[], const int32_T a_size[2], const real_T b_data[], real_T c_data[],
  int32_T c_size[2])
{
  int32_T stride;
  int32_T stridem1;
  int32_T iNext;
  int32_T iEnd;
  int32_T i2;
  int32_T i3;
  int32_T dim;
  int32_T i1;
  int32_T k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  c_size[0] = a_size[0];
  c_size[1] = 3;
  if (a_size[0] != 0) {
    dim = 0;
    stride = 1;
    exitg1 = false;
    while ((!exitg1) && (stride < 3)) {
      if (a_size[stride - 1] == 3) {
        dim = stride;
        exitg1 = true;
      } else {
        stride++;
      }
    }

    if (dim >= 2) {
      stride = 1;
      k = 1;
      while (k <= 1) {
        stride *= a_size[0];
        k = 2;
      }

      stridem1 = stride - 1;
    } else {
      stride = 1;
      stridem1 = 0;
    }

    iNext = stride * 3;
    if (dim >= 2) {
      dim = 1;
    } else {
      iEnd = 1;
      for (k = dim; k + 1 < 3; k++) {
        if (k + 1 <= 2) {
          dim = a_size[k];
        } else {
          dim = 1;
        }

        iEnd *= dim;
      }

      dim = (iEnd - 1) * iNext + 1;
    }

    k = 1;
    while (((iNext > 0) && (k <= dim)) || ((iNext < 0) && (k >= dim))) {
      iEnd = k + stridem1;
      for (i1 = k - 1; i1 + 1 <= iEnd; i1++) {
        i2 = i1 + stride;
        i3 = i2 + stride;
        c_data[i1] = a_data[i2] * b_data[i3] - a_data[i3] * b_data[i2];
        c_data[i2] = a_data[i3] * b_data[i1] - a_data[i1] * b_data[i3];
        c_data[i3] = a_data[i1] * b_data[i2] - a_data[i2] * b_data[i1];
      }

      k += iNext;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_cos_n(const real_T
  x[240], real_T b_x[240])
{
  int32_T k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  for (k = 0; k < 240; k++) {
    b_x[k] = cos(x[k]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_sin_n(const real_T
  x[240], real_T b_x[240])
{
  int32_T k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  for (k = 0; k < 240; k++) {
    b_x[k] = sin(x[k]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_repmat(const real_T
  a[3], real_T varargin_1, real_T b_data[], int32_T b_size[2])
{
  int32_T ibmat;
  int32_T itilerow;
  int32_T outsize_idx_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  outsize_idx_0 = (int32_T)varargin_1;
  b_size[0] = outsize_idx_0;
  b_size[1] = 3;
  if (!(outsize_idx_0 == 0)) {
    outsize_idx_0 = (int32_T)varargin_1;
    for (itilerow = 1; itilerow <= outsize_idx_0; itilerow++) {
      b_data[-1 + itilerow] = a[0];
    }

    ibmat = outsize_idx_0 - 1;
    for (itilerow = 1; itilerow <= outsize_idx_0; itilerow++) {
      b_data[ibmat + itilerow] = a[1];
    }

    ibmat = (outsize_idx_0 << 1) - 1;
    for (itilerow = 1; itilerow <= outsize_idx_0; itilerow++) {
      b_data[ibmat + itilerow] = a[2];
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_sign(const real_T
  x_data[], const int32_T *x_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_x_size = *x_size;
  loop_ub = *x_size;
  if (0 <= loop_ub - 1) {
    memcpy(&b_x_data[0], &x_data[0], loop_ub * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    if (b_x_data[loop_ub] < 0.0) {
      b_x_data[loop_ub] = -1.0;
    } else if (b_x_data[loop_ub] > 0.0) {
      b_x_data[loop_ub] = 1.0;
    } else if (b_x_data[loop_ub] == 0.0) {
      b_x_data[loop_ub] = 0.0;
    } else {
      b_x_data[loop_ub] = (rtNaN);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_abs(const real_T
  x_data[], const int32_T *x_size, real_T y_data[], int32_T *y_size)
{
  int32_T k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *y_size = *x_size;
  for (k = 0; k + 1 <= *x_size; k++) {
    y_data[k] = fabs(x_data[k]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_histc(const real_T
  X_data[], const int32_T *X_size, const real_T edges[240], real_T N[240],
  real_T BIN_data[], int32_T *BIN_size)
{
  boolean_T eok;
  int32_T xind;
  int32_T k;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  int32_T b_idx_0;
  int32_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  memset(&N[0], 0, 240U * sizeof(real_T));
  b_idx_0 = *X_size;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *BIN_size = *X_size;
  if (0 <= b_idx_0 - 1) {
    memset(&BIN_data[0], 0, b_idx_0 * sizeof(real_T));
  }

  xind = 0;
  do {
    exitg1 = 0;
    if (xind + 2 < 241) {
      if (!(edges[xind + 1] >= edges[xind])) {
        eok = false;
        exitg1 = 1;
      } else {
        xind++;
      }
    } else {
      eok = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (!eok) {
    for (xind = 0; xind < 240; xind++) {
      N[xind] = (rtNaN);
    }

    *BIN_size = *X_size;
    for (xind = 0; xind < b_idx_0; xind++) {
      BIN_data[xind] = (rtNaN);
    }
  } else {
    xind = 0;
    b_idx_0 = *X_size - 1;
    for (k = 0; k <= b_idx_0; k++) {
      low_i = 0;
      if (!rtIsNaN(X_data[xind])) {
        if ((X_data[xind] >= edges[0]) && (X_data[xind] < edges[239])) {
          low_i = 1;
          low_ip1 = 1;
          high_i = 240;
          while (high_i > low_ip1 + 1) {
            mid_i = (low_i + high_i) >> 1;
            if (X_data[xind] >= edges[mid_i - 1]) {
              low_i = mid_i;
              low_ip1 = mid_i;
            } else {
              high_i = mid_i;
            }
          }
        }

        if (X_data[xind] == edges[239]) {
          low_i = 240;
        }
      }

      if (low_i > 0) {
        N[low_i - 1]++;
      }

      BIN_data[xind] = low_i;
      xind++;
    }
  }
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_buildP
  (robotics_slalgs_internal_Vect_T *obj, const real_T scan_InternalRanges_data[],
   const int32_T *scan_InternalRanges_size, const real_T
   scan_InternalAngles_data[], const int32_T *scan_InternalAngles_size,
   boolean_T scan_ContainsOnlyFiniteData)
{
  boolean_T validScan_ContainsOnlyFiniteDat;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.obj[0] = obj->DistanceLimits[0];
  Pure_Pursuit_With_VFH_B.obj[1] = obj->DistanceLimits[1];
  Pur_lidarScan_removeInvalidData(scan_InternalRanges_data,
    scan_InternalRanges_size, scan_InternalAngles_data, scan_InternalAngles_size,
    scan_ContainsOnlyFiniteData, Pure_Pursuit_With_VFH_B.obj,
    Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
    &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
    Pure_Pursuit_With_VFH_B.lowerAng_data,
    &Pure_Pursuit_With_VFH_B.lowerAng_size, &validScan_ContainsOnlyFiniteDat);
  Pure_Pursuit_With_VFH_B.loop_ub_o =
    Pure_Pursuit_With_VFH_B.sinOfEnlargement_size;
  for (Pure_Pursuit_With_VFH_B.trueCount = 0; Pure_Pursuit_With_VFH_B.trueCount <
       Pure_Pursuit_With_VFH_B.loop_ub_o; Pure_Pursuit_With_VFH_B.trueCount++) {
    Pure_Pursuit_With_VFH_B.weightedRanges_data[Pure_Pursuit_With_VFH_B.trueCount]
      = obj->DistanceLimits[1] -
      Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount];
  }

  if (Pure_Pursuit_With_VF_isequal_n1
      (Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
       &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
       Pure_Pursuit_With_VFH_B.lowerAng_data,
       &Pure_Pursuit_With_VFH_B.lowerAng_size, validScan_ContainsOnlyFiniteDat,
       scan_InternalRanges_data, scan_InternalRanges_size,
       scan_InternalAngles_data, scan_InternalAngles_size,
       scan_ContainsOnlyFiniteData)) {
    memset(&obj->PolarObstacleDensity[0], 0, 240U * sizeof(real_T));
  } else {
    Pure_Pursuit_With_VFH_B.x = obj->RobotRadius + obj->SafetyDistance;
    if (Pure_Pursuit_With_VFH_B.x == 0.0) {
      Pure_Pursuit_With_VFH_histc(Pure_Pursuit_With_VFH_B.lowerAng_data,
        &Pure_Pursuit_With_VFH_B.lowerAng_size, obj->AngularSectorMidPoints,
        Pure_Pursuit_With_VFH_B.obstacleDensity,
        Pure_Pursuit_With_VFH_B.enlargementAngle_data,
        &Pure_Pursuit_With_VFH_B.enlargementAngle_size);
      memset(&Pure_Pursuit_With_VFH_B.obstacleDensity[0], 0, 240U * sizeof
             (real_T));
      Pure_Pursuit_With_VFH_B.end =
        Pure_Pursuit_With_VFH_B.enlargementAngle_size - 1;
      Pure_Pursuit_With_VFH_B.trueCount = 0;
      while (Pure_Pursuit_With_VFH_B.trueCount <= Pure_Pursuit_With_VFH_B.end) {
        Pure_Pursuit_With_VFH_B.m_size_idx_0 = (int32_T)
          Pure_Pursuit_With_VFH_B.enlargementAngle_data[Pure_Pursuit_With_VFH_B.trueCount]
          - 1;
        Pure_Pursuit_With_VFH_B.obstacleDensity[Pure_Pursuit_With_VFH_B.m_size_idx_0]
          +=
          Pure_Pursuit_With_VFH_B.weightedRanges_data[Pure_Pursuit_With_VFH_B.trueCount];
        Pure_Pursuit_With_VFH_B.trueCount++;
      }

      memcpy(&obj->PolarObstacleDensity[0],
             &Pure_Pursuit_With_VFH_B.obstacleDensity[0], 240U * sizeof(real_T));
    } else {
      Pure_Pursuit_With_VFH_B.loop_ub_o =
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_size;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount]
          = Pure_Pursuit_With_VFH_B.x /
          Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount];
      }

      Pure_Pursuit_With_VFH_B.end =
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_size - 1;
      Pure_Pursuit_With_VFH_B.trueCount = 0;
      for (Pure_Pursuit_With_VFH_B.loop_ub_o = 0;
           Pure_Pursuit_With_VFH_B.loop_ub_o <= Pure_Pursuit_With_VFH_B.end;
           Pure_Pursuit_With_VFH_B.loop_ub_o++) {
        if (Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.loop_ub_o]
            >= 1.0) {
          Pure_Pursuit_With_VFH_B.trueCount++;
        }
      }

      Pure_Pursuit_With_VFH_B.m_size_idx_0 = Pure_Pursuit_With_VFH_B.trueCount;
      Pure_Pursuit_With_VFH_B.trueCount = 0;
      for (Pure_Pursuit_With_VFH_B.loop_ub_o = 0;
           Pure_Pursuit_With_VFH_B.loop_ub_o <= Pure_Pursuit_With_VFH_B.end;
           Pure_Pursuit_With_VFH_B.loop_ub_o++) {
        if (Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.loop_ub_o]
            >= 1.0) {
          Pure_Pursuit_With_VFH_B.m_data[Pure_Pursuit_With_VFH_B.trueCount] =
            Pure_Pursuit_With_VFH_B.loop_ub_o + 1;
          Pure_Pursuit_With_VFH_B.trueCount++;
        }
      }

      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount <
           Pure_Pursuit_With_VFH_B.m_size_idx_0;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.m_data
          [Pure_Pursuit_With_VFH_B.trueCount] - 1] = 0.99999999999999978;
      }

      Pure_Pursuit_With_VFH_asin(Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
        &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
        Pure_Pursuit_With_VFH_B.enlargementAngle_data,
        &Pure_Pursuit_With_VFH_B.enlargementAngle_size);
      Pure_Pursuit_With_VFH_B.sinOfEnlargement_size =
        Pure_Pursuit_With_VFH_B.lowerAng_size;
      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.lowerAng_size;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount]
          =
          Pure_Pursuit_With_VFH_B.lowerAng_data[Pure_Pursuit_With_VFH_B.trueCount]
          + Pure_Pursuit_With_VFH_B.enlargementAngle_data[Pure_Pursuit_With_VFH_B.trueCount];
      }

      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.lowerAng_size;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.lowerAng_data[Pure_Pursuit_With_VFH_B.trueCount]
          -=
          Pure_Pursuit_With_VFH_B.enlargementAngle_data[Pure_Pursuit_With_VFH_B.trueCount];
      }

      Pure_Pursuit_With_VFH_cos(Pure_Pursuit_With_VFH_B.lowerAng_data,
        &Pure_Pursuit_With_VFH_B.lowerAng_size,
        Pure_Pursuit_With_VFH_B.enlargementAngle_data,
        &Pure_Pursuit_With_VFH_B.enlargementAngle_size);
      Pure_Pursuit_With_VFH_sin(Pure_Pursuit_With_VFH_B.lowerAng_data,
        &Pure_Pursuit_With_VFH_B.lowerAng_size,
        Pure_Pursuit_With_VFH_B.tmp_data_m, &Pure_Pursuit_With_VFH_B.tmp_size_c);
      Pure_Pursuit_With_VFH_B.lowerVec_size[0] =
        Pure_Pursuit_With_VFH_B.enlargementAngle_size;
      Pure_Pursuit_With_VFH_B.lowerVec_size[1] = 3;
      Pure_Pursuit_With_VFH_B.loop_ub_o =
        Pure_Pursuit_With_VFH_B.enlargementAngle_size;
      if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_o - 1) {
        memcpy(&Pure_Pursuit_With_VFH_B.lowerVec_data[0],
               &Pure_Pursuit_With_VFH_B.enlargementAngle_data[0],
               Pure_Pursuit_With_VFH_B.loop_ub_o * sizeof(real_T));
      }

      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.tmp_size_c;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.lowerVec_data[Pure_Pursuit_With_VFH_B.trueCount
          + Pure_Pursuit_With_VFH_B.enlargementAngle_size] =
          Pure_Pursuit_With_VFH_B.tmp_data_m[Pure_Pursuit_With_VFH_B.trueCount];
      }

      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.lowerAng_size;
      if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_o - 1) {
        memset
          (&Pure_Pursuit_With_VFH_B.lowerVec_data[Pure_Pursuit_With_VFH_B.enlargementAngle_size
           + Pure_Pursuit_With_VFH_B.tmp_size_c], 0,
           ((((Pure_Pursuit_With_VFH_B.loop_ub_o +
               Pure_Pursuit_With_VFH_B.enlargementAngle_size) +
              Pure_Pursuit_With_VFH_B.tmp_size_c) -
             Pure_Pursuit_With_VFH_B.enlargementAngle_size) -
            Pure_Pursuit_With_VFH_B.tmp_size_c) * sizeof(real_T));
      }

      Pure_Pursuit_With_VFH_cos(Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
        &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
        Pure_Pursuit_With_VFH_B.enlargementAngle_data,
        &Pure_Pursuit_With_VFH_B.tmp_size_c);
      Pure_Pursuit_With_VFH_sin(Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
        &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
        Pure_Pursuit_With_VFH_B.tmp_data_m, &Pure_Pursuit_With_VFH_B.tmp_size_m);
      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.tmp_size_c;
      if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_o - 1) {
        memcpy(&Pure_Pursuit_With_VFH_B.higherVec_data[0],
               &Pure_Pursuit_With_VFH_B.enlargementAngle_data[0],
               Pure_Pursuit_With_VFH_B.loop_ub_o * sizeof(real_T));
      }

      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.tmp_size_m;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.higherVec_data[Pure_Pursuit_With_VFH_B.trueCount
          + Pure_Pursuit_With_VFH_B.tmp_size_c] =
          Pure_Pursuit_With_VFH_B.tmp_data_m[Pure_Pursuit_With_VFH_B.trueCount];
      }

      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.lowerAng_size;
      if (0 <= Pure_Pursuit_With_VFH_B.loop_ub_o - 1) {
        memset
          (&Pure_Pursuit_With_VFH_B.higherVec_data[Pure_Pursuit_With_VFH_B.tmp_size_c
           + Pure_Pursuit_With_VFH_B.tmp_size_m], 0,
           ((((Pure_Pursuit_With_VFH_B.loop_ub_o +
               Pure_Pursuit_With_VFH_B.tmp_size_c) +
              Pure_Pursuit_With_VFH_B.tmp_size_m) -
             Pure_Pursuit_With_VFH_B.tmp_size_c) -
            Pure_Pursuit_With_VFH_B.tmp_size_m) * sizeof(real_T));
      }

      Pure_Pursuit_With_VFH_B.m_size_idx_0 =
        Pure_Pursuit_With_VFH_B.enlargementAngle_size;
      Pure_Pursuit_With_VFH_B.loop_ub_o = 240 *
        Pure_Pursuit_With_VFH_B.enlargementAngle_size;
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < Pure_Pursuit_With_VFH_B.loop_ub_o;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.validWeights_data[Pure_Pursuit_With_VFH_B.trueCount]
          = true;
      }

      Pure_Pursuit_With_VFH_cross(Pure_Pursuit_With_VFH_B.lowerVec_data,
        Pure_Pursuit_With_VFH_B.lowerVec_size,
        Pure_Pursuit_With_VFH_B.higherVec_data, Pure_Pursuit_With_VFH_B.lh_data,
        Pure_Pursuit_With_VFH_B.lh_size);
      Pure_Pursuit_With_VFH_cos_n(obj->AngularSectorMidPoints,
        Pure_Pursuit_With_VFH_B.obstacleDensity);
      Pure_Pursuit_With_VFH_sin_n(obj->AngularSectorMidPoints,
        Pure_Pursuit_With_VFH_B.dv2);
      Pure_Pursuit_With_VFH_B.loop_ub_o = Pure_Pursuit_With_VFH_B.lh_size[0];
      Pure_Pursuit_With_VFH_B.lh_size_m = Pure_Pursuit_With_VFH_B.lh_size[0];
      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount <
           Pure_Pursuit_With_VFH_B.m_size_idx_0;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        Pure_Pursuit_With_VFH_B.m_data[Pure_Pursuit_With_VFH_B.trueCount] =
          Pure_Pursuit_With_VFH_B.trueCount;
      }

      for (Pure_Pursuit_With_VFH_B.end = 0; Pure_Pursuit_With_VFH_B.end < 240;
           Pure_Pursuit_With_VFH_B.end++) {
        Pure_Pursuit_With_VFH_B.kalpha[Pure_Pursuit_With_VFH_B.end] =
          Pure_Pursuit_With_VFH_B.obstacleDensity[Pure_Pursuit_With_VFH_B.end];
        Pure_Pursuit_With_VFH_B.kalpha[240 + Pure_Pursuit_With_VFH_B.end] =
          Pure_Pursuit_With_VFH_B.dv2[Pure_Pursuit_With_VFH_B.end];
        Pure_Pursuit_With_VFH_B.kalpha[480 + Pure_Pursuit_With_VFH_B.end] = 0.0;
        Pure_Pursuit_With_VFH_B.kalpha_d[0] =
          Pure_Pursuit_With_VFH_B.kalpha[Pure_Pursuit_With_VFH_B.end];
        Pure_Pursuit_With_VFH_B.kalpha_d[1] =
          Pure_Pursuit_With_VFH_B.kalpha[Pure_Pursuit_With_VFH_B.end + 240];
        Pure_Pursuit_With_VFH_B.kalpha_d[2] =
          Pure_Pursuit_With_VFH_B.kalpha[Pure_Pursuit_With_VFH_B.end + 480];
        Pure_Pursuit_With_VFH_repmat(Pure_Pursuit_With_VFH_B.kalpha_d, (real_T)
          Pure_Pursuit_With_VFH_B.lowerVec_size[0],
          Pure_Pursuit_With_VFH_B.kalphaVec_data,
          Pure_Pursuit_With_VFH_B.kalphaVec_size);
        Pure_Pursuit_With_VFH_cross(Pure_Pursuit_With_VFH_B.lowerVec_data,
          Pure_Pursuit_With_VFH_B.lowerVec_size,
          Pure_Pursuit_With_VFH_B.kalphaVec_data,
          Pure_Pursuit_With_VFH_B.lk_data, Pure_Pursuit_With_VFH_B.lk_size);
        Pure_Pursuit_With_VFH_cross(Pure_Pursuit_With_VFH_B.kalphaVec_data,
          Pure_Pursuit_With_VFH_B.kalphaVec_size,
          Pure_Pursuit_With_VFH_B.higherVec_data,
          Pure_Pursuit_With_VFH_B.kh_data, Pure_Pursuit_With_VFH_B.kh_size);
        Pure_Pursuit_With_VFH_B.loop_ub_n = Pure_Pursuit_With_VFH_B.lk_size[0];
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_size =
          Pure_Pursuit_With_VFH_B.lk_size[0];
        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.loop_ub_n;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.tmp_data_m[Pure_Pursuit_With_VFH_B.trueCount] =
            Pure_Pursuit_With_VFH_B.lk_data[(Pure_Pursuit_With_VFH_B.lk_size[0] <<
            1) + Pure_Pursuit_With_VFH_B.trueCount];
        }

        Pure_Pursuit_With_VFH_sign(Pure_Pursuit_With_VFH_B.tmp_data_m,
          &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
          Pure_Pursuit_With_VFH_B.enlargementAngle_data,
          &Pure_Pursuit_With_VFH_B.enlargementAngle_size);
        Pure_Pursuit_With_VFH_B.loop_ub_n = Pure_Pursuit_With_VFH_B.kh_size[0];
        Pure_Pursuit_With_VFH_B.sinOfEnlargement_size =
          Pure_Pursuit_With_VFH_B.kh_size[0];
        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.loop_ub_n;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.lowerAng_data[Pure_Pursuit_With_VFH_B.trueCount]
            = Pure_Pursuit_With_VFH_B.kh_data[(Pure_Pursuit_With_VFH_B.kh_size[0]
            << 1) + Pure_Pursuit_With_VFH_B.trueCount];
        }

        Pure_Pursuit_With_VFH_sign(Pure_Pursuit_With_VFH_B.lowerAng_data,
          &Pure_Pursuit_With_VFH_B.sinOfEnlargement_size,
          Pure_Pursuit_With_VFH_B.tmp_data_m,
          &Pure_Pursuit_With_VFH_B.tmp_size_c);
        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.loop_ub_o;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount]
            = Pure_Pursuit_With_VFH_B.lh_data[(Pure_Pursuit_With_VFH_B.lh_size[0]
            << 1) + Pure_Pursuit_With_VFH_B.trueCount];
        }

        Pure_Pursuit_With_VFH_sign(Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
          &Pure_Pursuit_With_VFH_B.lh_size_m,
          Pure_Pursuit_With_VFH_B.lowerAng_data,
          &Pure_Pursuit_With_VFH_B.tmp_size_c);
        Pure_Pursuit_With_VFH_B.tmp_size_c =
          Pure_Pursuit_With_VFH_B.enlargementAngle_size;
        Pure_Pursuit_With_VFH_B.loop_ub_n =
          Pure_Pursuit_With_VFH_B.enlargementAngle_size;
        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.loop_ub_n;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.sinOfEnlargement_data[Pure_Pursuit_With_VFH_B.trueCount]
            =
            (Pure_Pursuit_With_VFH_B.enlargementAngle_data[Pure_Pursuit_With_VFH_B.trueCount]
             + Pure_Pursuit_With_VFH_B.tmp_data_m[Pure_Pursuit_With_VFH_B.trueCount])
            + Pure_Pursuit_With_VFH_B.lowerAng_data[Pure_Pursuit_With_VFH_B.trueCount];
        }

        Pure_Pursuit_With_VFH_abs(Pure_Pursuit_With_VFH_B.sinOfEnlargement_data,
          &Pure_Pursuit_With_VFH_B.tmp_size_c,
          Pure_Pursuit_With_VFH_B.enlargementAngle_data,
          &Pure_Pursuit_With_VFH_B.enlargementAngle_size);
        Pure_Pursuit_With_VFH_B.loop_ub_n =
          Pure_Pursuit_With_VFH_B.enlargementAngle_size;
        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.loop_ub_n;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.tmp_data_g[Pure_Pursuit_With_VFH_B.trueCount] =
            (Pure_Pursuit_With_VFH_B.enlargementAngle_data[Pure_Pursuit_With_VFH_B.trueCount]
             > 1.0);
        }

        for (Pure_Pursuit_With_VFH_B.trueCount = 0;
             Pure_Pursuit_With_VFH_B.trueCount <
             Pure_Pursuit_With_VFH_B.m_size_idx_0;
             Pure_Pursuit_With_VFH_B.trueCount++) {
          Pure_Pursuit_With_VFH_B.validWeights_data[Pure_Pursuit_With_VFH_B.end
            + 240 *
            Pure_Pursuit_With_VFH_B.m_data[Pure_Pursuit_With_VFH_B.trueCount]] =
            Pure_Pursuit_With_VFH_B.tmp_data_g[Pure_Pursuit_With_VFH_B.trueCount];
        }
      }

      for (Pure_Pursuit_With_VFH_B.trueCount = 0;
           Pure_Pursuit_With_VFH_B.trueCount < 240;
           Pure_Pursuit_With_VFH_B.trueCount++) {
        obj->PolarObstacleDensity[Pure_Pursuit_With_VFH_B.trueCount] = 0.0;
        for (Pure_Pursuit_With_VFH_B.end = 0; Pure_Pursuit_With_VFH_B.end <
             Pure_Pursuit_With_VFH_B.m_size_idx_0; Pure_Pursuit_With_VFH_B.end++)
        {
          obj->PolarObstacleDensity[Pure_Pursuit_With_VFH_B.trueCount] +=
            (real_T)Pure_Pursuit_With_VFH_B.validWeights_data[240 *
            Pure_Pursuit_With_VFH_B.end + Pure_Pursuit_With_VFH_B.trueCount] *
            Pure_Pursuit_With_VFH_B.weightedRanges_data[Pure_Pursuit_With_VFH_B.end];
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_buildB
  (robotics_slalgs_internal_Vect_T *obj)
{
  int32_T trueCount;
  int32_T i;
  int32_T b_size_idx_1;
  boolean_T d;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  trueCount = 0;
  for (i = 0; i < 240; i++) {
    d = (obj->PolarObstacleDensity[i] > obj->HistogramThresholds[1]);
    if (d) {
      trueCount++;
    }

    Pure_Pursuit_With_VFH_B.d[i] = d;
  }

  b_size_idx_1 = trueCount;
  trueCount = 0;
  for (i = 0; i < 240; i++) {
    if (Pure_Pursuit_With_VFH_B.d[i]) {
      Pure_Pursuit_With_VFH_B.b_data_h[trueCount] = (uint8_T)(i + 1);
      trueCount++;
    }
  }

  for (trueCount = 0; trueCount < b_size_idx_1; trueCount++) {
    obj->BinaryHistogram[Pure_Pursuit_With_VFH_B.b_data_h[trueCount] - 1] = true;
  }

  trueCount = 0;
  for (i = 0; i < 240; i++) {
    d = (obj->PolarObstacleDensity[i] < obj->HistogramThresholds[0]);
    if (d) {
      trueCount++;
    }

    Pure_Pursuit_With_VFH_B.d[i] = d;
  }

  b_size_idx_1 = trueCount;
  trueCount = 0;
  for (i = 0; i < 240; i++) {
    if (Pure_Pursuit_With_VFH_B.d[i]) {
      Pure_Pursuit_With_VFH_B.c_data[trueCount] = (uint8_T)(i + 1);
      trueCount++;
    }
  }

  for (trueCount = 0; trueCount < b_size_idx_1; trueCount++) {
    obj->BinaryHistogram[Pure_Pursuit_With_VFH_B.c_data[trueCount] - 1] = false;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_power(const real_T
  a_data[], const int32_T *a_size, real_T y_data[], int32_T *y_size)
{
  int32_T loop_ub;
  int32_T b_z1_size_idx_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  b_z1_size_idx_0 = *a_size;
  loop_ub = *a_size;
  if (0 <= loop_ub - 1) {
    memcpy(&Pure_Pursuit_With_VFH_B.b_z1_data_b[0], &y_data[0], loop_ub * sizeof
           (real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *a_size; loop_ub++) {
    Pure_Pursuit_With_VFH_B.b_z1_data_b[loop_ub] = a_data[loop_ub] *
      a_data[loop_ub];
  }

  *y_size = *a_size;
  if (0 <= b_z1_size_idx_0 - 1) {
    memcpy(&y_data[0], &Pure_Pursuit_With_VFH_B.b_z1_data_b[0], b_z1_size_idx_0 *
           sizeof(real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_sqrt(const real_T
  x_data[], const int32_T *x_size, real_T b_x_data[], int32_T *b_x_size)
{
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  *b_x_size = *x_size;
  loop_ub = *x_size;
  if (0 <= loop_ub - 1) {
    memcpy(&b_x_data[0], &x_data[0], loop_ub * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= *x_size; loop_ub++) {
    b_x_data[loop_ub] = sqrt(b_x_data[loop_ub]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_buildM
  (robotics_slalgs_internal_Vect_T *obj, const real_T scan_InternalRanges_data[],
   const int32_T *scan_InternalRanges_size, const real_T
   scan_InternalAngles_data[], const int32_T *scan_InternalAngles_size,
   boolean_T scan_ContainsOnlyFiniteData)
{
  boolean_T expl_temp;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.obj_e[0] = obj->DistanceLimits[0];
  Pure_Pursuit_With_VFH_B.obj_e[1] = obj->DistanceLimits[1];
  Pur_lidarScan_removeInvalidData(scan_InternalRanges_data,
    scan_InternalRanges_size, scan_InternalAngles_data, scan_InternalAngles_size,
    scan_ContainsOnlyFiniteData, Pure_Pursuit_With_VFH_B.obj_e,
    Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.ii_data_j,
    Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data,
    &Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size, &expl_temp);
  Pure_Pursuit_With_VFH_cos
    (Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data,
     &Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
     Pure_Pursuit_With_VFH_B.tmp_data_cx, &Pure_Pursuit_With_VFH_B.tmp_size_as);
  Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = Pure_Pursuit_With_VFH_B.ii_data_j;
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.DXj_data[Pure_Pursuit_With_VFH_B.k_h] =
      Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data[Pure_Pursuit_With_VFH_B.k_h]
      * Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h];
  }

  Pure_Pursuit_With_VFH_sin
    (Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data,
     &Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
     Pure_Pursuit_With_VFH_B.tmp_data_cx, &Pure_Pursuit_With_VFH_B.tmp_size_as);
  Pure_Pursuit_With_VFH_B.DYj_size_idx_0 = Pure_Pursuit_With_VFH_B.ii_data_j;
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.DYj_data[Pure_Pursuit_With_VFH_B.k_h] =
      Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data[Pure_Pursuit_With_VFH_B.k_h]
      * Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h];
  }

  Pure_Pursuit_With_VFH_B.tmp_size_a = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DXj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h] = 0.0 -
      Pure_Pursuit_With_VFH_B.DXj_data[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_power(Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_a,
    Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.tmp_size_as);
  Pure_Pursuit_With_VFH_B.obj_size_e = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DYj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.obj_data[Pure_Pursuit_With_VFH_B.k_h] =
      -obj->MinTurningRadius -
      Pure_Pursuit_With_VFH_B.DYj_data[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_power(Pure_Pursuit_With_VFH_B.obj_data,
    &Pure_Pursuit_With_VFH_B.obj_size_e, Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_a);
  Pure_Pursuit_With_VFH_B.tmp_size_ax = Pure_Pursuit_With_VFH_B.tmp_size_as;
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.tmp_size_as;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.obj_data[Pure_Pursuit_With_VFH_B.k_h] =
      Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data[Pure_Pursuit_With_VFH_B.k_h]
      + Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_sqrt(Pure_Pursuit_With_VFH_B.obj_data,
    &Pure_Pursuit_With_VFH_B.tmp_size_ax, Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_as);
  Pure_Pursuit_With_VFH_B.obj_tmp = (obj->MinTurningRadius + obj->RobotRadius) +
    obj->SafetyDistance;
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.tmp_size_as;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.blockedR_data[Pure_Pursuit_With_VFH_B.k_h] =
      ((Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h] <
        Pure_Pursuit_With_VFH_B.obj_tmp) &&
       (Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data[Pure_Pursuit_With_VFH_B.k_h]
        <= 0.0));
  }

  Pure_Pursuit_With_VFH_B.tmp_size_ct = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DXj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h] = 0.0 -
      Pure_Pursuit_With_VFH_B.DXj_data[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_power(Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_ct,
    Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.tmp_size_a);
  Pure_Pursuit_With_VFH_B.obj_size_p = Pure_Pursuit_With_VFH_B.ii_data_j;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DYj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.obj_data[Pure_Pursuit_With_VFH_B.k_h] =
      obj->MinTurningRadius -
      Pure_Pursuit_With_VFH_B.DYj_data[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_power(Pure_Pursuit_With_VFH_B.obj_data,
    &Pure_Pursuit_With_VFH_B.obj_size_p, Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_ax);
  Pure_Pursuit_With_VFH_B.tmp_size_p = Pure_Pursuit_With_VFH_B.tmp_size_a;
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.tmp_size_a;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.obj_data[Pure_Pursuit_With_VFH_B.k_h] =
      Pure_Pursuit_With_VFH_B.validScan_InternalRanges_data[Pure_Pursuit_With_VFH_B.k_h]
      + Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_sqrt(Pure_Pursuit_With_VFH_B.obj_data,
    &Pure_Pursuit_With_VFH_B.tmp_size_p, Pure_Pursuit_With_VFH_B.tmp_data_cx,
    &Pure_Pursuit_With_VFH_B.tmp_size_a);
  Pure_Pursuit_With_VFH_B.idx_c = Pure_Pursuit_With_VFH_B.tmp_size_a;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.idx_c; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.blockedL_data[Pure_Pursuit_With_VFH_B.k_h] =
      ((Pure_Pursuit_With_VFH_B.tmp_data_cx[Pure_Pursuit_With_VFH_B.k_h] <
        Pure_Pursuit_With_VFH_B.obj_tmp) &&
       (Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data[Pure_Pursuit_With_VFH_B.k_h]
        >= 0.0));
  }

  if (1 < Pure_Pursuit_With_VFH_B.tmp_size_as) {
    Pure_Pursuit_With_VFH_B.k_h = 1;
  } else {
    Pure_Pursuit_With_VFH_B.k_h = Pure_Pursuit_With_VFH_B.tmp_size_as;
  }

  Pure_Pursuit_With_VFH_B.idx_c = 0;
  Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = Pure_Pursuit_With_VFH_B.k_h;
  Pure_Pursuit_With_VFH_B.DYj_size_idx_0 = Pure_Pursuit_With_VFH_B.tmp_size_as;
  exitg1 = false;
  while ((!exitg1) && (Pure_Pursuit_With_VFH_B.DYj_size_idx_0 > 0)) {
    if (Pure_Pursuit_With_VFH_B.blockedR_data[Pure_Pursuit_With_VFH_B.DYj_size_idx_0
        - 1]) {
      Pure_Pursuit_With_VFH_B.idx_c++;
      Pure_Pursuit_With_VFH_B.ii_data_j = Pure_Pursuit_With_VFH_B.DYj_size_idx_0;
      if (Pure_Pursuit_With_VFH_B.idx_c >= Pure_Pursuit_With_VFH_B.k_h) {
        exitg1 = true;
      } else {
        Pure_Pursuit_With_VFH_B.DYj_size_idx_0--;
      }
    } else {
      Pure_Pursuit_With_VFH_B.DYj_size_idx_0--;
    }
  }

  if (Pure_Pursuit_With_VFH_B.k_h == 1) {
    if (Pure_Pursuit_With_VFH_B.idx_c == 0) {
      Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = 0;
    }
  } else {
    if (1 > Pure_Pursuit_With_VFH_B.idx_c) {
      Pure_Pursuit_With_VFH_B.idx_c = 0;
    }

    if (0 <= Pure_Pursuit_With_VFH_B.idx_c - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
             &Pure_Pursuit_With_VFH_B.ii_data_j, Pure_Pursuit_With_VFH_B.idx_c *
             sizeof(int32_T));
    }

    Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = Pure_Pursuit_With_VFH_B.idx_c;
    if (0 <= Pure_Pursuit_With_VFH_B.idx_c - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.ii_data_j,
             &Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
             Pure_Pursuit_With_VFH_B.idx_c * sizeof(int32_T));
    }

    Pure_Pursuit_With_VFH_B.idx_c--;
    Pure_Pursuit_With_VFH_B.k_h = Pure_Pursuit_With_VFH_B.DXj_size_idx_0 / 2;
    Pure_Pursuit_With_VFH_B.DYj_size_idx_0 = 1;
    while (Pure_Pursuit_With_VFH_B.DYj_size_idx_0 <= Pure_Pursuit_With_VFH_B.k_h)
    {
      Pure_Pursuit_With_VFH_B.xtmp = Pure_Pursuit_With_VFH_B.ii_data_j;
      Pure_Pursuit_With_VFH_B.ii_data_tmp = (Pure_Pursuit_With_VFH_B.idx_c -
        Pure_Pursuit_With_VFH_B.DYj_size_idx_0) + 1;
      Pure_Pursuit_With_VFH_B.ii_data_j = Pure_Pursuit_With_VFH_B.xtmp;
      Pure_Pursuit_With_VFH_B.DYj_size_idx_0++;
    }
  }

  Pure_Pursuit_With_VFH_B.xtmp = Pure_Pursuit_With_VFH_B.DXj_size_idx_0;
  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DXj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.phiR_data =
      Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data[Pure_Pursuit_With_VFH_B.ii_data_j
      - 1];
  }

  if (1 < Pure_Pursuit_With_VFH_B.tmp_size_a) {
    Pure_Pursuit_With_VFH_B.k_h = 1;
  } else {
    Pure_Pursuit_With_VFH_B.k_h = Pure_Pursuit_With_VFH_B.tmp_size_a;
  }

  Pure_Pursuit_With_VFH_B.idx_c = 0;
  Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = Pure_Pursuit_With_VFH_B.k_h;
  Pure_Pursuit_With_VFH_B.DYj_size_idx_0 = 1;
  exitg1 = false;
  while ((!exitg1) && (Pure_Pursuit_With_VFH_B.DYj_size_idx_0 <=
                       Pure_Pursuit_With_VFH_B.tmp_size_a)) {
    if (Pure_Pursuit_With_VFH_B.blockedL_data[Pure_Pursuit_With_VFH_B.DYj_size_idx_0
        - 1]) {
      Pure_Pursuit_With_VFH_B.idx_c++;
      Pure_Pursuit_With_VFH_B.ii_data_j = Pure_Pursuit_With_VFH_B.DYj_size_idx_0;
      if (Pure_Pursuit_With_VFH_B.idx_c >= Pure_Pursuit_With_VFH_B.k_h) {
        exitg1 = true;
      } else {
        Pure_Pursuit_With_VFH_B.DYj_size_idx_0++;
      }
    } else {
      Pure_Pursuit_With_VFH_B.DYj_size_idx_0++;
    }
  }

  if (Pure_Pursuit_With_VFH_B.k_h == 1) {
    if (Pure_Pursuit_With_VFH_B.idx_c == 0) {
      Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = 0;
    }
  } else {
    if (1 > Pure_Pursuit_With_VFH_B.idx_c) {
      Pure_Pursuit_With_VFH_B.idx_c = 0;
    }

    if (0 <= Pure_Pursuit_With_VFH_B.idx_c - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
             &Pure_Pursuit_With_VFH_B.ii_data_j, Pure_Pursuit_With_VFH_B.idx_c *
             sizeof(int32_T));
    }

    Pure_Pursuit_With_VFH_B.DXj_size_idx_0 = Pure_Pursuit_With_VFH_B.idx_c;
    if (0 <= Pure_Pursuit_With_VFH_B.idx_c - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.ii_data_j,
             &Pure_Pursuit_With_VFH_B.validScan_InternalAngles_size,
             Pure_Pursuit_With_VFH_B.idx_c * sizeof(int32_T));
    }
  }

  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h <
       Pure_Pursuit_With_VFH_B.DXj_size_idx_0; Pure_Pursuit_With_VFH_B.k_h++) {
    Pure_Pursuit_With_VFH_B.phiL_data =
      Pure_Pursuit_With_VFH_B.validScan_InternalAngles_data[Pure_Pursuit_With_VFH_B.ii_data_j
      - 1];
  }

  if (Pure_Pursuit_With_VFH_B.xtmp == 0) {
    Pure_Pursuit_With_VFH_B.phiR_data = obj->AngularSectorMidPoints[0];
  } else {
    if (Pure_Pursuit_With_VFH_B.phiR_data <= obj->AngularSectorMidPoints[0]) {
      Pure_Pursuit_With_VFH_B.phiR_data = obj->AngularSectorMidPoints[1];
    }
  }

  if (Pure_Pursuit_With_VFH_B.DXj_size_idx_0 == 0) {
    Pure_Pursuit_With_VFH_B.phiL_data = obj->AngularSectorMidPoints[239];
  } else {
    if (Pure_Pursuit_With_VFH_B.phiL_data >= obj->AngularSectorMidPoints[239]) {
      Pure_Pursuit_With_VFH_B.phiL_data = obj->AngularSectorMidPoints[238];
    }
  }

  for (Pure_Pursuit_With_VFH_B.k_h = 0; Pure_Pursuit_With_VFH_B.k_h < 240;
       Pure_Pursuit_With_VFH_B.k_h++) {
    obj->MaskedHistogram[Pure_Pursuit_With_VFH_B.k_h] = (obj->
      BinaryHistogram[Pure_Pursuit_With_VFH_B.k_h] ||
      ((obj->AngularSectorMidPoints[Pure_Pursuit_With_VFH_B.k_h] <
        Pure_Pursuit_With_VFH_B.phiR_data) || (obj->
      AngularSectorMidPoints[Pure_Pursuit_With_VFH_B.k_h] >
      Pure_Pursuit_With_VFH_B.phiL_data)));
  }
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_diff(const real_T x
  [242], real_T y[241])
{
  real_T work;
  real_T tmp1;
  int32_T ixLead;
  int32_T iyLead;
  int32_T m;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  ixLead = 1;
  iyLead = 0;
  work = x[0];
  for (m = 0; m < 241; m++) {
    tmp1 = work;
    work = x[ixLead];
    tmp1 = x[ixLead] - tmp1;
    ixLead++;
    y[iyLead] = tmp1;
    iyLead++;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_any(const
  real_T x[241])
{
  boolean_T y;
  boolean_T b;
  int32_T b_k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 241)) {
    if ((x[b_k] == 0.0) || rtIsNaN(x[b_k])) {
      b = true;
    } else {
      b = false;
    }

    if (!b) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_diff_n(const real_T
  x_data[], const int32_T x_size[2], real_T y_data[], int32_T y_size[2])
{
  int32_T ixStart;
  int32_T iyStart;
  int32_T r;
  int32_T b_y1_size_idx_1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  b_y1_size_idx_1 = x_size[1];
  if (!(x_size[1] == 0)) {
    ixStart = 0;
    iyStart = 0;
    for (r = 1; r <= x_size[1]; r++) {
      Pure_Pursuit_With_VFH_B.b_y1_data[iyStart] = x_data[ixStart + 1] -
        x_data[ixStart];
      ixStart += 2;
      iyStart++;
    }
  }

  y_size[0] = 1;
  y_size[1] = x_size[1];
  if (0 <= b_y1_size_idx_1 - 1) {
    memcpy(&y_data[0], &Pure_Pursuit_With_VFH_B.b_y1_data[0], b_y1_size_idx_1 *
           sizeof(real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_mod_n(const real_T
  x_data[], const int32_T x_size[2], real_T r_data[], int32_T r_size[2])
{
  int32_T loop_ub;
  int32_T b_z1_size_idx_1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  b_z1_size_idx_1 = x_size[1];
  loop_ub = x_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&Pure_Pursuit_With_VFH_B.b_z1_data_g[0], &r_data[0], loop_ub * sizeof
           (real_T));
  }

  for (loop_ub = 0; loop_ub + 1 <= x_size[1]; loop_ub++) {
    Pure_Pursuit_With_VFH_B.b_z1_data_g[loop_ub] =
      Pure_Pursuit_With_VFH_floatmod(x_data[loop_ub]);
  }

  r_size[0] = 1;
  r_size[1] = x_size[1];
  if (0 <= b_z1_size_idx_1 - 1) {
    memcpy(&r_data[0], &Pure_Pursuit_With_VFH_B.b_z1_data_g[0], b_z1_size_idx_1 *
           sizeof(real_T));
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With__wrapToPi_n1q(const
  real_T theta_data[], const int32_T theta_size[2], real_T b_theta_data[],
  int32_T b_theta_size[2])
{
  int32_T end;
  int32_T i;
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.theta_size_n[0] = 1;
  Pure_Pursuit_With_VFH_B.theta_size_n[1] = theta_size[1];
  loop_ub = theta_size[0] * theta_size[1];
  for (end = 0; end < loop_ub; end++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.theta_data_p[end] = theta_data[end] +
      3.1415926535897931;
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  loop_ub = theta_size[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.pos_data_b[end] =
      (Pure_Pursuit_With_VFH_B.theta_data_p[end] > 0.0);
  }

  Pure_Pursuit_With_VFH_mod_n(Pure_Pursuit_With_VFH_B.theta_data_p,
    Pure_Pursuit_With_VFH_B.theta_size_n,
    Pure_Pursuit_With_VFH_B.thetaWrap_data_n,
    Pure_Pursuit_With_VFH_B.thetaWrap_size_o);
  loop_ub = Pure_Pursuit_With_VFH_B.thetaWrap_size_o[0] *
    Pure_Pursuit_With_VFH_B.thetaWrap_size_o[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.b_data_n[end] =
      (Pure_Pursuit_With_VFH_B.thetaWrap_data_n[end] == 0.0);
  }

  end = Pure_Pursuit_With_VFH_B.thetaWrap_size_o[1] - 1;
  for (i = 0; i <= end; i++) {
    if (Pure_Pursuit_With_VFH_B.b_data_n[i] &&
        Pure_Pursuit_With_VFH_B.pos_data_b[i]) {
      Pure_Pursuit_With_VFH_B.thetaWrap_data_n[i] = 6.2831853071795862;
    }
  }

  b_theta_size[0] = 1;
  b_theta_size[1] = Pure_Pursuit_With_VFH_B.thetaWrap_size_o[1];
  for (end = 0; end < loop_ub; end++) {
    b_theta_data[end] = Pure_Pursuit_With_VFH_B.thetaWrap_data_n[end] -
      3.1415926535897931;
  }
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With__bisectAngles(real_T
  theta1_data[], int32_T theta1_size[2], real_T theta2_data[], int32_T
  theta2_size[2], real_T bisect_data[], int32_T bisect_size[2])
{
  int32_T end;
  int32_T i;
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With__wrapToPi_n1q(theta1_data, theta1_size,
    Pure_Pursuit_With_VFH_B.tmp_data_me, Pure_Pursuit_With_VFH_B.tmp_size_jz);
  loop_ub = Pure_Pursuit_With_VFH_B.tmp_size_jz[0] *
    Pure_Pursuit_With_VFH_B.tmp_size_jz[1];
  if (0 <= loop_ub - 1) {
    memcpy(&theta1_data[0], &Pure_Pursuit_With_VFH_B.tmp_data_me[0], loop_ub *
           sizeof(real_T));
  }

  Pure_Pursuit_With__wrapToPi_n1q(theta2_data, theta2_size,
    Pure_Pursuit_With_VFH_B.tmp_data_me, Pure_Pursuit_With_VFH_B.tmp_size_j);
  loop_ub = Pure_Pursuit_With_VFH_B.tmp_size_j[0] *
    Pure_Pursuit_With_VFH_B.tmp_size_j[1];
  if (0 <= loop_ub - 1) {
    memcpy(&theta2_data[0], &Pure_Pursuit_With_VFH_B.tmp_data_me[0], loop_ub *
           sizeof(real_T));
  }

  Pure_Pursuit_With_VFH_B.theta_size_a[0] = 1;
  Pure_Pursuit_With_VFH_B.theta_size_a[1] = Pure_Pursuit_With_VFH_B.tmp_size_jz
    [1];
  loop_ub = Pure_Pursuit_With_VFH_B.tmp_size_jz[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.theta_data_g[end] = (theta1_data[end] -
      (theta1_data[end] - theta2_data[end]) / 2.0) + 3.1415926535897931;
  }

  loop_ub = Pure_Pursuit_With_VFH_B.tmp_size_jz[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.pos_data_l[end] =
      (Pure_Pursuit_With_VFH_B.theta_data_g[end] > 0.0);
  }

  Pure_Pursuit_With_VFH_mod_n(Pure_Pursuit_With_VFH_B.theta_data_g,
    Pure_Pursuit_With_VFH_B.theta_size_a, Pure_Pursuit_With_VFH_B.tmp_data_me,
    Pure_Pursuit_With_VFH_B.tmp_size_jz);
  loop_ub = Pure_Pursuit_With_VFH_B.tmp_size_jz[0] *
    Pure_Pursuit_With_VFH_B.tmp_size_jz[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.b_data_o[end] =
      (Pure_Pursuit_With_VFH_B.tmp_data_me[end] == 0.0);
  }

  end = Pure_Pursuit_With_VFH_B.tmp_size_jz[1] - 1;
  for (i = 0; i <= end; i++) {
    if (Pure_Pursuit_With_VFH_B.b_data_o[i] &&
        Pure_Pursuit_With_VFH_B.pos_data_l[i]) {
      Pure_Pursuit_With_VFH_B.tmp_data_me[i] = 6.2831853071795862;
    }
  }

  bisect_size[0] = 1;
  bisect_size[1] = Pure_Pursuit_With_VFH_B.tmp_size_jz[1];
  for (end = 0; end < loop_ub; end++) {
    bisect_data[end] = Pure_Pursuit_With_VFH_B.tmp_data_me[end] -
      3.1415926535897931;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_bsxfun(const real_T
  a[240], const real_T b_data[], const int32_T *b_size, real_T c_data[], int32_T
  c_size[2])
{
  int32_T k;
  int32_T bcoef;
  int32_T k_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  c_size[0] = *b_size;
  c_size[1] = 240;
  if (*b_size != 0) {
    bcoef = (*b_size != 1);
    for (k = 0; k < 240; k++) {
      for (k_0 = 0; k_0 < c_size[0]; k_0++) {
        c_data[k_0 + *b_size * k] = Pure_Pursuit_With_VFH_angdiff(a[k],
          b_data[bcoef * k_0]);
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_abs_n(const real_T
  x_data[], const int32_T x_size[2], real_T y_data[], int32_T y_size[2])
{
  int32_T nx;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  nx = x_size[0] * 240;
  y_size[0] = x_size[0];
  y_size[1] = 240;
  for (k = 0; k + 1 <= nx; k++) {
    y_data[k] = fabs(x_data[k]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_bsxfun_n(const
  real_T a_data[], const int32_T a_size[2], const real_T b_data[], const int32_T
  *b_size, real_T c_data[], int32_T c_size[2])
{
  int32_T k;
  int32_T acoef;
  int32_T bcoef;
  int32_T k_0;
  int32_T csz_idx_0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if (*b_size == 1) {
    csz_idx_0 = a_size[0];
  } else if (a_size[0] == 1) {
    csz_idx_0 = *b_size;
  } else if (a_size[0] == *b_size) {
    csz_idx_0 = a_size[0];
  } else if (*b_size < a_size[0]) {
    csz_idx_0 = *b_size;
  } else {
    csz_idx_0 = a_size[0];
  }

  c_size[0] = csz_idx_0;
  c_size[1] = 240;
  if (csz_idx_0 != 0) {
    acoef = (a_size[0] != 1);
    bcoef = (*b_size != 1);
    for (k = 0; k < 240; k++) {
      for (k_0 = 0; k_0 < csz_idx_0; k_0++) {
        c_data[k_0 + csz_idx_0 * k] = a_data[acoef * k_0 + a_size[0] * k] -
          b_data[bcoef * k_0];
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_localC(const
  real_T candidateDir_data[], const real_T selectDir_data[], const int32_T
  selectDir_size[2], real_T cost_data[], int32_T cost_size[2])
{
  int32_T end;
  int32_T i;
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.theta_size[0] = 1;
  Pure_Pursuit_With_VFH_B.theta_size[1] = selectDir_size[1];
  loop_ub = selectDir_size[0] * selectDir_size[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.theta_data_f[end] = (selectDir_data[end] -
      candidateDir_data[end]) + 3.1415926535897931;
  }

  loop_ub = selectDir_size[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.pos_data[end] =
      (Pure_Pursuit_With_VFH_B.theta_data_f[end] > 0.0);
  }

  Pure_Pursuit_With_VFH_mod_n(Pure_Pursuit_With_VFH_B.theta_data_f,
    Pure_Pursuit_With_VFH_B.theta_size, Pure_Pursuit_With_VFH_B.thetaWrap_data,
    Pure_Pursuit_With_VFH_B.thetaWrap_size);
  loop_ub = Pure_Pursuit_With_VFH_B.thetaWrap_size[0] *
    Pure_Pursuit_With_VFH_B.thetaWrap_size[1];
  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.b_data_dy[end] =
      (Pure_Pursuit_With_VFH_B.thetaWrap_data[end] == 0.0);
  }

  end = Pure_Pursuit_With_VFH_B.thetaWrap_size[1] - 1;
  for (i = 0; i <= end; i++) {
    if (Pure_Pursuit_With_VFH_B.b_data_dy[i] &&
        Pure_Pursuit_With_VFH_B.pos_data[i]) {
      Pure_Pursuit_With_VFH_B.thetaWrap_data[i] = 6.2831853071795862;
    }
  }

  for (end = 0; end < loop_ub; end++) {
    Pure_Pursuit_With_VFH_B.delta_data[end] =
      Pure_Pursuit_With_VFH_B.thetaWrap_data[end] - 3.1415926535897931;
  }

  cost_size[0] = 1;
  cost_size[1] = Pure_Pursuit_With_VFH_B.thetaWrap_size[1];
  for (end = 0; end + 1 <= Pure_Pursuit_With_VFH_B.thetaWrap_size[1]; end++) {
    cost_data[end] = fabs(Pure_Pursuit_With_VFH_B.delta_data[end]);
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

void Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_comput(const
  robotics_slalgs_internal_Vect_T *obj, const real_T c_data[], const int32_T
  c_size[2], real_T targetDir, real_T prevDir, real_T cost_data[], int32_T
  cost_size[2])
{
  int32_T i;
  int32_T loop_ub;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.totalWeight = (obj->TargetDirectionWeight +
    obj->CurrentDirectionWeight) + obj->PreviousDirectionWeight;
  Pure_Pursuit_With_VFH_B.targetDir_size[0] = 1;
  Pure_Pursuit_With_VFH_B.targetDir_size[1] = c_size[1];
  loop_ub = c_size[1];
  for (i = 0; i < loop_ub; i++) {
    Pure_Pursuit_With_VFH_B.targetDir_data[i] = targetDir;
  }

  VectorFieldHistogramBase_localC(c_data, Pure_Pursuit_With_VFH_B.targetDir_data,
    Pure_Pursuit_With_VFH_B.targetDir_size, Pure_Pursuit_With_VFH_B.b_data,
    Pure_Pursuit_With_VFH_B.b_size);
  Pure_Pursuit_With_VFH_B.tmp_size_f[0] = 1;
  Pure_Pursuit_With_VFH_B.tmp_size_f[1] = c_size[1];
  loop_ub = c_size[1];
  if (0 <= loop_ub - 1) {
    memset(&Pure_Pursuit_With_VFH_B.tmp_data_cv[0], 0, loop_ub * sizeof(real_T));
  }

  VectorFieldHistogramBase_localC(c_data, Pure_Pursuit_With_VFH_B.tmp_data_cv,
    Pure_Pursuit_With_VFH_B.tmp_size_f, Pure_Pursuit_With_VFH_B.targetDir_data,
    Pure_Pursuit_With_VFH_B.targetDir_size);
  Pure_Pursuit_With_VFH_B.prevDir_size[0] = 1;
  Pure_Pursuit_With_VFH_B.prevDir_size[1] = c_size[1];
  loop_ub = c_size[1];
  for (i = 0; i < loop_ub; i++) {
    Pure_Pursuit_With_VFH_B.prevDir_data[i] = prevDir;
  }

  VectorFieldHistogramBase_localC(c_data, Pure_Pursuit_With_VFH_B.prevDir_data,
    Pure_Pursuit_With_VFH_B.prevDir_size, Pure_Pursuit_With_VFH_B.tmp_data_cv,
    Pure_Pursuit_With_VFH_B.targetDir_size);
  cost_size[0] = 1;
  cost_size[1] = Pure_Pursuit_With_VFH_B.b_size[1];
  loop_ub = Pure_Pursuit_With_VFH_B.b_size[0] * Pure_Pursuit_With_VFH_B.b_size[1];
  for (i = 0; i < loop_ub; i++) {
    cost_data[i] = ((obj->TargetDirectionWeight *
                     Pure_Pursuit_With_VFH_B.b_data[i] +
                     obj->CurrentDirectionWeight *
                     Pure_Pursuit_With_VFH_B.targetDir_data[i]) +
                    obj->PreviousDirectionWeight *
                    Pure_Pursuit_With_VFH_B.tmp_data_cv[i]) / 3.0 *
      Pure_Pursuit_With_VFH_B.totalWeight;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
}

boolean_T Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_any_n(const
  boolean_T x_data[], const int32_T x_size[2])
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size[1])) {
    if (x_data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

real_T Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_select
  (robotics_slalgs_internal_Vect_T *obj, real_T targetDir)
{
  real_T thetaSteer;
  int32_T candidateDirs_size_idx_1;
  int32_T rd_size_idx_1;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.dv1[0] = 0.0;
  for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
       240; Pure_Pursuit_With_VFH_B.calclen++) {
    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_B.dv1[Pure_Pursuit_With_VFH_B.calclen + 1] =
      !obj->MaskedHistogram[Pure_Pursuit_With_VFH_B.calclen];
  }

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  Pure_Pursuit_With_VFH_B.dv1[241] = 0.0;
  Pure_Pursuit_With_VFH_diff(Pure_Pursuit_With_VFH_B.dv1,
    Pure_Pursuit_With_VFH_B.changes);
  if (!Pure_Pursuit_With_VFH_any(Pure_Pursuit_With_VFH_B.changes)) {
    thetaSteer = (rtNaN);
    obj->PreviousDirection = (rtNaN);
  } else {
    Pure_Pursuit_With_VFH_B.idx = 0;
    Pure_Pursuit_With_VFH_B.vstride = 1;
    exitg1 = false;
    while ((!exitg1) && (Pure_Pursuit_With_VFH_B.vstride < 242)) {
      if (Pure_Pursuit_With_VFH_B.changes[Pure_Pursuit_With_VFH_B.vstride - 1]
          != 0.0) {
        Pure_Pursuit_With_VFH_B.idx++;
        Pure_Pursuit_With_VFH_B.ii_data[Pure_Pursuit_With_VFH_B.idx - 1] =
          (uint8_T)Pure_Pursuit_With_VFH_B.vstride;
        if (Pure_Pursuit_With_VFH_B.idx >= 241) {
          exitg1 = true;
        } else {
          Pure_Pursuit_With_VFH_B.vstride++;
        }
      } else {
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    if (1 > Pure_Pursuit_With_VFH_B.idx) {
      Pure_Pursuit_With_VFH_B.vstride = 0;
      Pure_Pursuit_With_VFH_B.idx = 0;
    } else {
      Pure_Pursuit_With_VFH_B.vstride = 0;
    }

    if (0 <= Pure_Pursuit_With_VFH_B.idx - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.ii_data_b[0],
             &Pure_Pursuit_With_VFH_B.ii_data[0], Pure_Pursuit_With_VFH_B.idx *
             sizeof(uint8_T));
      memcpy(&Pure_Pursuit_With_VFH_B.ii_data[0],
             &Pure_Pursuit_With_VFH_B.ii_data_b[0], Pure_Pursuit_With_VFH_B.idx *
             sizeof(uint8_T));
      memcpy(&Pure_Pursuit_With_VFH_B.foundSectors_data[0],
             &Pure_Pursuit_With_VFH_B.ii_data[0], Pure_Pursuit_With_VFH_B.idx *
             sizeof(uint8_T));
    }

    Pure_Pursuit_With_VFH_B.sectors_size[0] = 2;
    Pure_Pursuit_With_VFH_B.sectors_size[1] = Pure_Pursuit_With_VFH_B.idx / 2;
    Pure_Pursuit_With_VFH_B.loop_ub = (Pure_Pursuit_With_VFH_B.idx / 2) << 1;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.sectors_data[Pure_Pursuit_With_VFH_B.calclen] =
        Pure_Pursuit_With_VFH_B.foundSectors_data[Pure_Pursuit_With_VFH_B.calclen];
    }

    if (1 > Pure_Pursuit_With_VFH_B.idx / 2) {
      Pure_Pursuit_With_VFH_B.idx = 0;
    } else {
      Pure_Pursuit_With_VFH_B.vstride = Pure_Pursuit_With_VFH_B.idx / 2;
      Pure_Pursuit_With_VFH_B.idx = 0;
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.vstride - 1;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.sectors_data[1 + (Pure_Pursuit_With_VFH_B.calclen <<
        1)] = (real_T)Pure_Pursuit_With_VFH_B.foundSectors_data
        [(Pure_Pursuit_With_VFH_B.calclen << 1) + 1] - 1.0;
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.sectors_size[1] <<
      1;
    if (0 <= Pure_Pursuit_With_VFH_B.loop_ub - 1) {
      memset(&Pure_Pursuit_With_VFH_B.angles_data[0], 0,
             Pure_Pursuit_With_VFH_B.loop_ub * sizeof(real_T));
    }

    if (!(1 > Pure_Pursuit_With_VFH_B.sectors_size[1])) {
      Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.sectors_size[1];
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.idx - 1;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.angles_data[Pure_Pursuit_With_VFH_B.calclen << 1] =
        obj->AngularSectorMidPoints[(int32_T)
        Pure_Pursuit_With_VFH_B.sectors_data[Pure_Pursuit_With_VFH_B.calclen <<
        1] - 1];
    }

    if (1 > Pure_Pursuit_With_VFH_B.sectors_size[1]) {
      Pure_Pursuit_With_VFH_B.calclen = 0;
    } else {
      Pure_Pursuit_With_VFH_B.calclen = Pure_Pursuit_With_VFH_B.sectors_size[1];
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.calclen - 1;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.angles_data[1 + (Pure_Pursuit_With_VFH_B.calclen <<
        1)] = obj->AngularSectorMidPoints[(int32_T)
        Pure_Pursuit_With_VFH_B.sectors_data[(Pure_Pursuit_With_VFH_B.calclen <<
        1) + 1] - 1];
    }

    Pure_Pursuit_With_VFH_diff_n(Pure_Pursuit_With_VFH_B.sectors_data,
      Pure_Pursuit_With_VFH_B.sectors_size, Pure_Pursuit_With_VFH_B.tmp_data_d,
      Pure_Pursuit_With_VFH_B.tmp_size);
    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[1];
    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.tmp_size[0] *
      Pure_Pursuit_With_VFH_B.tmp_size[1];
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.sectorSizes_data[Pure_Pursuit_With_VFH_B.calclen] =
        obj->AngularDifference *
        Pure_Pursuit_With_VFH_B.tmp_data_d[Pure_Pursuit_With_VFH_B.calclen];
    }

    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen]
        = obj->NarrowOpeningThreshold;
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[1] - 1;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.sectorSizes_data[Pure_Pursuit_With_VFH_B.calclen]
          <
          Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen])
      {
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.vstride;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.sectorSizes_data[Pure_Pursuit_With_VFH_B.calclen]
          <
          Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen])
      {
        Pure_Pursuit_With_VFH_B.kd_data[Pure_Pursuit_With_VFH_B.vstride] =
          Pure_Pursuit_With_VFH_B.calclen + 1;
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.angles_size[0] = 1;
    Pure_Pursuit_With_VFH_B.angles_size[1] = Pure_Pursuit_With_VFH_B.loop_ub;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.tmp_data_d[Pure_Pursuit_With_VFH_B.calclen] =
        Pure_Pursuit_With_VFH_B.angles_data
        [(Pure_Pursuit_With_VFH_B.kd_data[Pure_Pursuit_With_VFH_B.calclen] - 1) <<
        1];
    }

    Pure_Pursuit_With_VFH_B.angles_size_j[0] = 1;
    Pure_Pursuit_With_VFH_B.angles_size_j[1] = Pure_Pursuit_With_VFH_B.loop_ub;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.angles_data_j[Pure_Pursuit_With_VFH_B.calclen] =
        Pure_Pursuit_With_VFH_B.angles_data
        [((Pure_Pursuit_With_VFH_B.kd_data[Pure_Pursuit_With_VFH_B.calclen] - 1)
          << 1) + 1];
    }

    Pure_Pursuit_With__bisectAngles(Pure_Pursuit_With_VFH_B.tmp_data_d,
      Pure_Pursuit_With_VFH_B.angles_size, Pure_Pursuit_With_VFH_B.angles_data_j,
      Pure_Pursuit_With_VFH_B.angles_size_j,
      Pure_Pursuit_With_VFH_B.narrowDirs_data,
      Pure_Pursuit_With_VFH_B.sectors_size);
    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[1] - 1;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (!(Pure_Pursuit_With_VFH_B.sectorSizes_data[Pure_Pursuit_With_VFH_B.calclen]
            <
            Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen]))
      {
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.vstride;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (!(Pure_Pursuit_With_VFH_B.sectorSizes_data[Pure_Pursuit_With_VFH_B.calclen]
            <
            Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen]))
      {
        Pure_Pursuit_With_VFH_B.pd_data[Pure_Pursuit_With_VFH_B.vstride] =
          Pure_Pursuit_With_VFH_B.calclen + 1;
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.loop_ub +
      Pure_Pursuit_With_VFH_B.loop_ub;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.nonNarrowDirs_data[Pure_Pursuit_With_VFH_B.calclen]
        = Pure_Pursuit_With_VFH_B.angles_data
        [(Pure_Pursuit_With_VFH_B.pd_data[Pure_Pursuit_With_VFH_B.calclen] - 1) <<
        1] + obj->NarrowOpeningThreshold / 2.0;
    }

    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.nonNarrowDirs_data[Pure_Pursuit_With_VFH_B.calclen
        + Pure_Pursuit_With_VFH_B.loop_ub] =
        Pure_Pursuit_With_VFH_B.angles_data
        [((Pure_Pursuit_With_VFH_B.pd_data[Pure_Pursuit_With_VFH_B.calclen] - 1)
          << 1) + 1] - obj->NarrowOpeningThreshold / 2.0;
    }

    if (rtIsNaN(obj->PreviousDirection)) {
      obj->PreviousDirection = 0.0;
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.sectors_size[1];
    Pure_Pursuit_With_VFH_B.vstride = Pure_Pursuit_With_VFH_B.idx +
      Pure_Pursuit_With_VFH_B.sectors_size[1];
    candidateDirs_size_idx_1 = Pure_Pursuit_With_VFH_B.vstride + 3;
    if (0 <= Pure_Pursuit_With_VFH_B.idx - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.candidateDirs_data[0],
             &Pure_Pursuit_With_VFH_B.nonNarrowDirs_data[0],
             Pure_Pursuit_With_VFH_B.idx * sizeof(real_T));
    }

    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.calclen
        + Pure_Pursuit_With_VFH_B.idx] =
        Pure_Pursuit_With_VFH_B.narrowDirs_data[Pure_Pursuit_With_VFH_B.sectors_size
        [0] * Pure_Pursuit_With_VFH_B.calclen];
    }

    Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.vstride] =
      targetDir;
    Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.vstride +
      1] = 0.0;
    Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.vstride +
      2] = obj->PreviousDirection;
    if (0 <= candidateDirs_size_idx_1 - 1) {
      memcpy(&Pure_Pursuit_With_VFH_B.candidateDirs_data_p[0],
             &Pure_Pursuit_With_VFH_B.candidateDirs_data[0],
             candidateDirs_size_idx_1 * sizeof(real_T));
    }

    Pure_Pursuit_With_VFH_bsxfun(obj->AngularSectorMidPoints,
      Pure_Pursuit_With_VFH_B.candidateDirs_data_p, &candidateDirs_size_idx_1,
      Pure_Pursuit_With_VFH_B.tmp_data, Pure_Pursuit_With_VFH_B.tmp_size);
    Pure_Pursuit_With_VFH_abs_n(Pure_Pursuit_With_VFH_B.tmp_data,
      Pure_Pursuit_With_VFH_B.tmp_size,
      Pure_Pursuit_With_VFH_B.candToSectDiff_data,
      Pure_Pursuit_With_VFH_B.angles_size);
    candidateDirs_size_idx_1 = Pure_Pursuit_With_VFH_B.angles_size[0];
    Pure_Pursuit_With_VFH_B.vstride = Pure_Pursuit_With_VFH_B.angles_size[0];
    Pure_Pursuit_With_VFH_B.idx = 1;
    while (Pure_Pursuit_With_VFH_B.idx <= Pure_Pursuit_With_VFH_B.vstride) {
      Pure_Pursuit_With_VFH_B.calclen = Pure_Pursuit_With_VFH_B.idx;
      Pure_Pursuit_With_VFH_B.ixstop = 239 * Pure_Pursuit_With_VFH_B.vstride +
        Pure_Pursuit_With_VFH_B.idx;
      Pure_Pursuit_With_VFH_B.mtmp =
        Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.idx
        - 1];
      if (rtIsNaN
          (Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.idx
           - 1])) {
        Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.idx +
          Pure_Pursuit_With_VFH_B.vstride;
        exitg1 = false;
        while ((!exitg1) && (((Pure_Pursuit_With_VFH_B.vstride > 0) &&
                              (Pure_Pursuit_With_VFH_B.loop_ub <=
                               Pure_Pursuit_With_VFH_B.ixstop)) ||
                             ((Pure_Pursuit_With_VFH_B.vstride < 0) &&
                              (Pure_Pursuit_With_VFH_B.loop_ub >=
                               Pure_Pursuit_With_VFH_B.ixstop)))) {
          Pure_Pursuit_With_VFH_B.calclen = Pure_Pursuit_With_VFH_B.loop_ub;
          if (!rtIsNaN
              (Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.loop_ub
               - 1])) {
            Pure_Pursuit_With_VFH_B.mtmp =
              Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.loop_ub
              - 1];
            exitg1 = true;
          } else {
            Pure_Pursuit_With_VFH_B.loop_ub += Pure_Pursuit_With_VFH_B.vstride;
          }
        }
      }

      if (Pure_Pursuit_With_VFH_B.calclen < Pure_Pursuit_With_VFH_B.ixstop) {
        Pure_Pursuit_With_VFH_B.calclen += Pure_Pursuit_With_VFH_B.vstride;
        while (((Pure_Pursuit_With_VFH_B.vstride > 0) &&
                (Pure_Pursuit_With_VFH_B.calclen <=
                 Pure_Pursuit_With_VFH_B.ixstop)) ||
               ((Pure_Pursuit_With_VFH_B.vstride < 0) &&
                (Pure_Pursuit_With_VFH_B.calclen >=
                 Pure_Pursuit_With_VFH_B.ixstop))) {
          if (Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.calclen
              - 1] < Pure_Pursuit_With_VFH_B.mtmp) {
            Pure_Pursuit_With_VFH_B.mtmp =
              Pure_Pursuit_With_VFH_B.candToSectDiff_data[Pure_Pursuit_With_VFH_B.calclen
              - 1];
          }

          Pure_Pursuit_With_VFH_B.calclen += Pure_Pursuit_With_VFH_B.vstride;
        }
      }

      Pure_Pursuit_With_VFH_B.candidateDirs_data_p[Pure_Pursuit_With_VFH_B.idx -
        1] = Pure_Pursuit_With_VFH_B.mtmp;
      Pure_Pursuit_With_VFH_B.idx++;
    }

    Pure_Pursuit_With_VFH_bsxfun_n(Pure_Pursuit_With_VFH_B.candToSectDiff_data,
      Pure_Pursuit_With_VFH_B.angles_size,
      Pure_Pursuit_With_VFH_B.candidateDirs_data_p, &candidateDirs_size_idx_1,
      Pure_Pursuit_With_VFH_B.tmp_data, Pure_Pursuit_With_VFH_B.tmp_size);
    candidateDirs_size_idx_1 = Pure_Pursuit_With_VFH_B.tmp_size[0];
    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.tmp_size[0] *
      Pure_Pursuit_With_VFH_B.tmp_size[1];
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.nearIdx_data[Pure_Pursuit_With_VFH_B.calclen] =
        (Pure_Pursuit_With_VFH_B.tmp_data[Pure_Pursuit_With_VFH_B.calclen] <
         1.4901161193847656E-8);
    }

    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         candidateDirs_size_idx_1; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.freeDirs_data[Pure_Pursuit_With_VFH_B.calclen] =
        true;
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[0] - 1;
    if (0 <= Pure_Pursuit_With_VFH_B.idx) {
      Pure_Pursuit_With_VFH_B.obj_size[0] = 1;
    }

    Pure_Pursuit_With_VFH_B.vstride = 0;
    while (Pure_Pursuit_With_VFH_B.vstride <= Pure_Pursuit_With_VFH_B.idx) {
      Pure_Pursuit_With_VFH_B.loop_ub = 0;
      for (Pure_Pursuit_With_VFH_B.ixstop = 0; Pure_Pursuit_With_VFH_B.ixstop <
           240; Pure_Pursuit_With_VFH_B.ixstop++) {
        if (Pure_Pursuit_With_VFH_B.nearIdx_data[candidateDirs_size_idx_1 *
            Pure_Pursuit_With_VFH_B.ixstop + Pure_Pursuit_With_VFH_B.vstride]) {
          Pure_Pursuit_With_VFH_B.loop_ub++;
        }
      }

      rd_size_idx_1 = Pure_Pursuit_With_VFH_B.loop_ub;
      Pure_Pursuit_With_VFH_B.loop_ub = 0;
      for (Pure_Pursuit_With_VFH_B.ixstop = 0; Pure_Pursuit_With_VFH_B.ixstop <
           240; Pure_Pursuit_With_VFH_B.ixstop++) {
        if (Pure_Pursuit_With_VFH_B.nearIdx_data[candidateDirs_size_idx_1 *
            Pure_Pursuit_With_VFH_B.ixstop + Pure_Pursuit_With_VFH_B.vstride]) {
          Pure_Pursuit_With_VFH_B.rd_data[Pure_Pursuit_With_VFH_B.loop_ub] =
            (uint8_T)(Pure_Pursuit_With_VFH_B.ixstop + 1);
          Pure_Pursuit_With_VFH_B.loop_ub++;
        }
      }

      Pure_Pursuit_With_VFH_B.obj_size[1] = rd_size_idx_1;
      for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
           rd_size_idx_1; Pure_Pursuit_With_VFH_B.calclen++) {
        Pure_Pursuit_With_VFH_B.obj_data_l[Pure_Pursuit_With_VFH_B.calclen] =
          obj->
          MaskedHistogram[Pure_Pursuit_With_VFH_B.rd_data[Pure_Pursuit_With_VFH_B.calclen]
          - 1];
      }

      Pure_Pursuit_With_VFH_B.freeDirs_data[Pure_Pursuit_With_VFH_B.vstride] =
        !Pure_Pursuit_With_VFH_any_n(Pure_Pursuit_With_VFH_B.obj_data_l,
        Pure_Pursuit_With_VFH_B.obj_size);
      Pure_Pursuit_With_VFH_B.vstride++;
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[0] - 1;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.freeDirs_data[Pure_Pursuit_With_VFH_B.calclen])
      {
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.vstride;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.freeDirs_data[Pure_Pursuit_With_VFH_B.calclen])
      {
        Pure_Pursuit_With_VFH_B.qd_data[Pure_Pursuit_With_VFH_B.vstride] =
          Pure_Pursuit_With_VFH_B.calclen + 1;
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.candidateDirs_size[0] = 1;
    Pure_Pursuit_With_VFH_B.candidateDirs_size[1] =
      Pure_Pursuit_With_VFH_B.loop_ub;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.candidateDirs_data_p[Pure_Pursuit_With_VFH_B.calclen]
        =
        Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.qd_data
        [Pure_Pursuit_With_VFH_B.calclen] - 1];
    }

    VectorFieldHistogramBase_comput(obj,
      Pure_Pursuit_With_VFH_B.candidateDirs_data_p,
      Pure_Pursuit_With_VFH_B.candidateDirs_size, targetDir,
      obj->PreviousDirection, Pure_Pursuit_With_VFH_B.costValues_data,
      Pure_Pursuit_With_VFH_B.obj_size);
    Pure_Pursuit_With_VFH_B.calclen = 1;
    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.obj_size[1];
    Pure_Pursuit_With_VFH_B.mtmp = Pure_Pursuit_With_VFH_B.costValues_data[0];
    if (Pure_Pursuit_With_VFH_B.obj_size[1] > 1) {
      if (rtIsNaN(Pure_Pursuit_With_VFH_B.costValues_data[0])) {
        Pure_Pursuit_With_VFH_B.loop_ub = 1;
        exitg1 = false;
        while ((!exitg1) && (Pure_Pursuit_With_VFH_B.loop_ub + 1 <=
                             Pure_Pursuit_With_VFH_B.idx)) {
          Pure_Pursuit_With_VFH_B.calclen = Pure_Pursuit_With_VFH_B.loop_ub + 1;
          if (!rtIsNaN
              (Pure_Pursuit_With_VFH_B.costValues_data[Pure_Pursuit_With_VFH_B.loop_ub]))
          {
            Pure_Pursuit_With_VFH_B.mtmp =
              Pure_Pursuit_With_VFH_B.costValues_data[Pure_Pursuit_With_VFH_B.loop_ub];
            exitg1 = true;
          } else {
            Pure_Pursuit_With_VFH_B.loop_ub++;
          }
        }
      }

      if (Pure_Pursuit_With_VFH_B.calclen < Pure_Pursuit_With_VFH_B.obj_size[1])
      {
        while (Pure_Pursuit_With_VFH_B.calclen + 1 <=
               Pure_Pursuit_With_VFH_B.idx) {
          if (Pure_Pursuit_With_VFH_B.costValues_data[Pure_Pursuit_With_VFH_B.calclen]
              < Pure_Pursuit_With_VFH_B.mtmp) {
            Pure_Pursuit_With_VFH_B.mtmp =
              Pure_Pursuit_With_VFH_B.costValues_data[Pure_Pursuit_With_VFH_B.calclen];
          }

          Pure_Pursuit_With_VFH_B.calclen++;
        }
      }
    }

    Pure_Pursuit_With_VFH_B.loop_ub = Pure_Pursuit_With_VFH_B.obj_size[0] *
      Pure_Pursuit_With_VFH_B.obj_size[1];
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         Pure_Pursuit_With_VFH_B.loop_ub; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen] =
        Pure_Pursuit_With_VFH_B.costValues_data[Pure_Pursuit_With_VFH_B.calclen]
        - Pure_Pursuit_With_VFH_B.mtmp;
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.tmp_size[0] - 1;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.freeDirs_data[Pure_Pursuit_With_VFH_B.calclen])
      {
        Pure_Pursuit_With_VFH_B.sd_data[Pure_Pursuit_With_VFH_B.vstride] =
          Pure_Pursuit_With_VFH_B.calclen + 1;
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    Pure_Pursuit_With_VFH_B.idx = Pure_Pursuit_With_VFH_B.obj_size[1] - 1;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen] <
          1.4901161193847656E-8) {
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    candidateDirs_size_idx_1 = Pure_Pursuit_With_VFH_B.vstride;
    Pure_Pursuit_With_VFH_B.vstride = 0;
    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <=
         Pure_Pursuit_With_VFH_B.idx; Pure_Pursuit_With_VFH_B.calclen++) {
      if (Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen] <
          1.4901161193847656E-8) {
        Pure_Pursuit_With_VFH_B.td_data[Pure_Pursuit_With_VFH_B.vstride] =
          Pure_Pursuit_With_VFH_B.calclen + 1;
        Pure_Pursuit_With_VFH_B.vstride++;
      }
    }

    for (Pure_Pursuit_With_VFH_B.calclen = 0; Pure_Pursuit_With_VFH_B.calclen <
         candidateDirs_size_idx_1; Pure_Pursuit_With_VFH_B.calclen++) {
      Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen] =
        Pure_Pursuit_With_VFH_B.candidateDirs_data[Pure_Pursuit_With_VFH_B.sd_data
        [Pure_Pursuit_With_VFH_B.td_data[Pure_Pursuit_With_VFH_B.calclen] - 1] -
        1];
    }

    Pure_Pursuit_With_VFH_B.calclen = 1;
    thetaSteer = Pure_Pursuit_With_VFH_B.cDiff_data[0];
    if (candidateDirs_size_idx_1 > 1) {
      if (rtIsNaN(Pure_Pursuit_With_VFH_B.cDiff_data[0])) {
        Pure_Pursuit_With_VFH_B.loop_ub = 1;
        exitg1 = false;
        while ((!exitg1) && (Pure_Pursuit_With_VFH_B.loop_ub + 1 <=
                             candidateDirs_size_idx_1)) {
          Pure_Pursuit_With_VFH_B.calclen = Pure_Pursuit_With_VFH_B.loop_ub + 1;
          if (!rtIsNaN
              (Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.loop_ub]))
          {
            thetaSteer =
              Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.loop_ub];
            exitg1 = true;
          } else {
            Pure_Pursuit_With_VFH_B.loop_ub++;
          }
        }
      }

      if (Pure_Pursuit_With_VFH_B.calclen < candidateDirs_size_idx_1) {
        while (Pure_Pursuit_With_VFH_B.calclen + 1 <= candidateDirs_size_idx_1)
        {
          if (Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen]
              < thetaSteer) {
            thetaSteer =
              Pure_Pursuit_With_VFH_B.cDiff_data[Pure_Pursuit_With_VFH_B.calclen];
          }

          Pure_Pursuit_With_VFH_B.calclen++;
        }
      }
    }

    obj->PreviousDirection = thetaSteer;
  }

  return thetaSteer;
}

real_T Pure_Pursuit_With_VFHModelClass::VectorFieldHistogramBase_stepIm
  (robotics_slalgs_internal_Vect_T *obj, const real_T varargin_1[721], const
   real_T varargin_2[721], real_T varargin_3)
{
  real_T steeringDir;
  boolean_T scan_ContainsOnlyFiniteData;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  VectorFieldHistogram_parseAndVa(varargin_1, varargin_2, varargin_3,
    Pure_Pursuit_With_VFH_B.scan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalRanges_size,
    Pure_Pursuit_With_VFH_B.scan_InternalAngles_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalAngles_size,
    &scan_ContainsOnlyFiniteData, &Pure_Pursuit_With_VFH_B.target);
  if (fabs(Pure_Pursuit_With_VFH_B.target) > 3.1415926535897931) {
    Pure_Pursuit_With_VFH_B.target = Pure_Pursuit_With_VF_wrapToPi_n
      (Pure_Pursuit_With_VFH_B.target);
  }

  VectorFieldHistogramBase_buildP(obj,
    Pure_Pursuit_With_VFH_B.scan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalRanges_size,
    Pure_Pursuit_With_VFH_B.scan_InternalAngles_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalAngles_size,
    scan_ContainsOnlyFiniteData);
  VectorFieldHistogramBase_buildB(obj);
  VectorFieldHistogramBase_buildM(obj,
    Pure_Pursuit_With_VFH_B.scan_InternalRanges_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalRanges_size,
    Pure_Pursuit_With_VFH_B.scan_InternalAngles_data,
    &Pure_Pursuit_With_VFH_B.scan_InternalAngles_size,
    scan_ContainsOnlyFiniteData);
  steeringDir = VectorFieldHistogramBase_select(obj,
    Pure_Pursuit_With_VFH_B.target);

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  return steeringDir;
}

/* Model step function */
void Pure_Pursuit_With_VFHModelClass::step()
{
  /* local block i/o variables */
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool rtb_SourceBlock_o2_j;
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool rtb_SourceBlock_o2_d;
  boolean_T rtb_LogicalOperator_m;
  boolean_T rtb_SourceBlock_o1;
  boolean_T rtb_SourceBlock_o1_b;
  boolean_T rEQ0;
  SL_Bus_Pure_Pursuit_With_VFH_std_msgs_Bool b_varargout_2;
  boolean_T rtb_SourceBlock_o1_g;
  boolean_T rtb_SourceBlock_o1_i;
  boolean_T rtb_Memory;
  int8_T rtAction;
  boolean_T rtb_FixPtRelationalOperator_idx;
  boolean_T rtb_FixPtRelationalOperator_i_0;
  boolean_T exitg1;
  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    /* set solver stop time */
    if (!((&Pure_Pursuit_With_VFH_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                            (((&Pure_Pursuit_With_VFH_M)->Timing.clockTickH0 + 1)
        * (&Pure_Pursuit_With_VFH_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                            (((&Pure_Pursuit_With_VFH_M)->Timing.clockTick0 + 1)
        * (&Pure_Pursuit_With_VFH_M)->Timing.stepSize0 +
        (&Pure_Pursuit_With_VFH_M)->Timing.clockTickH0 *
        (&Pure_Pursuit_With_VFH_M)->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    (&Pure_Pursuit_With_VFH_M)->Timing.t[0] = rtsiGetT
      (&(&Pure_Pursuit_With_VFH_M)->solverInfo);
  }

  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    /* Outputs for Atomic SubSystem: '<S3>/Subscribe' */
    /* Start for MATLABSystem: '<S24>/SourceBlock' incorporates:
     *  Inport: '<S28>/In1'
     */
    rtb_SourceBlock_o1_i = Sub_Pure_Pursuit_With_VFH_54.getLatestMessage
      (&Pure_Pursuit_With_VFH_B.b_varargout_2);

    /* Outputs for Enabled SubSystem: '<S24>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S28>/Enable'
     */
    if (rtb_SourceBlock_o1_i) {
      Pure_Pursuit_With_VFH_B.In1 = Pure_Pursuit_With_VFH_B.b_varargout_2;
    }

    /* End of Start for MATLABSystem: '<S24>/SourceBlock' */
    /* End of Outputs for SubSystem: '<S24>/Enabled Subsystem' */
    /* End of Outputs for SubSystem: '<S3>/Subscribe' */

    /* MATLAB Function: '<S3>/MATLAB Function' */
    /* MATLAB Function 'Inputs/MATLAB Function': '<S23>:1' */
    /* '<S23>:1:13' */
    /* '<S23>:1:5' */
    /* '<S23>:1:9' */
    /* '<S23>:1:10' */
    /* '<S23>:1:16' */
    for (Pure_Pursuit_With_VFH_B.k = 0; Pure_Pursuit_With_VFH_B.k < 721;
         Pure_Pursuit_With_VFH_B.k++) {
      Pure_Pursuit_With_VFH_B.Angles[Pure_Pursuit_With_VFH_B.k] = (real_T)
        Pure_Pursuit_With_VFH_B.k * Pure_Pursuit_With_VFH_B.In1.AngleIncrement +
        Pure_Pursuit_With_VFH_B.In1.AngleMin;
    }

    Pure_Pursuit_With_VF_wrapToPi_b(Pure_Pursuit_With_VFH_B.Angles);

    /* Outputs for Atomic SubSystem: '<S3>/Subscribe1' */
    /* Start for MATLABSystem: '<S25>/SourceBlock' incorporates:
     *  Inport: '<S29>/In1'
     */
    rtb_SourceBlock_o1_g = Sub_Pure_Pursuit_With_VFH_55.getLatestMessage
      (&Pure_Pursuit_With_VFH_B.b_varargout_2_l);

    /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S29>/Enable'
     */
    if (rtb_SourceBlock_o1_g) {
      Pure_Pursuit_With_VFH_B.In1_m = Pure_Pursuit_With_VFH_B.b_varargout_2_l;
    }

    /* End of Start for MATLABSystem: '<S25>/SourceBlock' */
    /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem' */
    /* End of Outputs for SubSystem: '<S3>/Subscribe1' */

    /* MATLAB Function: '<S3>/quad2eul' */
    /* MATLAB Function 'Inputs/quad2eul': '<S27>:1' */
    /* '<S27>:1:4' */
    Pure_Pursuit_With_VFH_B.aSinInput = 1.0 / sqrt
      (((Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.W *
         Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.W +
         Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.X *
         Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.X) +
        Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Y *
        Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Y) +
       Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Z *
       Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Z);
    Pure_Pursuit_With_VFH_B.Switch =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.W *
      Pure_Pursuit_With_VFH_B.aSinInput;
    Pure_Pursuit_With_VFH_B.VectorFieldHistogram =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.X *
      Pure_Pursuit_With_VFH_B.aSinInput;
    Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Y *
      Pure_Pursuit_With_VFH_B.aSinInput;
    Pure_Pursuit_With_VFH_B.aSinInput *=
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Orientation.Z;

    /* SignalConversion: '<S2>/TmpSignal ConversionAtPure PursuitInport1' incorporates:
     *  MATLAB Function: '<S3>/quad2eul'
     */
    /* '<S27>:1:5' */
    Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu[0] =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.X;
    Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu[1] =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.Y;
    Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu[2] = rt_atan2d_snf
      ((Pure_Pursuit_With_VFH_B.VectorFieldHistogram *
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 +
        Pure_Pursuit_With_VFH_B.Switch * Pure_Pursuit_With_VFH_B.aSinInput) *
       2.0, ((Pure_Pursuit_With_VFH_B.Switch * Pure_Pursuit_With_VFH_B.Switch +
              Pure_Pursuit_With_VFH_B.VectorFieldHistogram *
              Pure_Pursuit_With_VFH_B.VectorFieldHistogram) -
             Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 *
             Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1) -
       Pure_Pursuit_With_VFH_B.aSinInput * Pure_Pursuit_With_VFH_B.aSinInput);

    /* Outputs for Atomic SubSystem: '<S3>/Subscribe2' */
    /* Start for MATLABSystem: '<S26>/SourceBlock' incorporates:
     *  Inport: '<S30>/In1'
     */
    rtb_SourceBlock_o1_g = Sub_Pure_Pursuit_With_VFH_87.getLatestMessage
      (&Pure_Pursuit_With_VFH_B.b_varargout_2_b);

    /* Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S30>/Enable'
     */
    if (rtb_SourceBlock_o1_g) {
      Pure_Pursuit_With_VFH_B.In1_p = Pure_Pursuit_With_VFH_B.b_varargout_2_b;
    }

    /* End of Start for MATLABSystem: '<S26>/SourceBlock' */
    /* End of Outputs for SubSystem: '<S26>/Enabled Subsystem' */
    /* End of Outputs for SubSystem: '<S3>/Subscribe2' */

    /* SignalConversion: '<S3>/ConcatBufferAtVector ConcatenateIn1' */
    Pure_Pursuit_With_VFH_B.VectorConcatenate[0] =
      Pure_Pursuit_With_VFH_B.In1_p.X;

    /* SignalConversion: '<S3>/ConcatBufferAtVector ConcatenateIn2' */
    Pure_Pursuit_With_VFH_B.VectorConcatenate[1] =
      Pure_Pursuit_With_VFH_B.In1_p.Y;

    /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
    if (!Pure_Pursuit_With_V_isequal_n1q
        (Pure_Pursuit_With_VFH_DW.obj_a.DesiredLinearVelocity,
         Pure_Pursuit_With_VFH_P.PurePursuit_DesiredLinearVeloci)) {
      Pure_Pursuit_With_VFH_DW.obj_a.DesiredLinearVelocity =
        Pure_Pursuit_With_VFH_P.PurePursuit_DesiredLinearVeloci;
    }

    if (!Pure_Pursuit_With_V_isequal_n1q
        (Pure_Pursuit_With_VFH_DW.obj_a.MaxAngularVelocity,
         Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity)) {
      Pure_Pursuit_With_VFH_DW.obj_a.MaxAngularVelocity =
        Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity;
    }

    if (!Pure_Pursuit_With_V_isequal_n1q
        (Pure_Pursuit_With_VFH_DW.obj_a.LookaheadDistance,
         Pure_Pursuit_With_VFH_P.PurePursuit_LookaheadDistance)) {
      Pure_Pursuit_With_VFH_DW.obj_a.LookaheadDistance =
        Pure_Pursuit_With_VFH_P.PurePursuit_LookaheadDistance;
    }

    rtb_Memory = false;
    rtb_FixPtRelationalOperator_idx = true;
    Pure_Pursuit_With_VFH_B.k = 0;
    exitg1 = false;
    while ((!exitg1) && (Pure_Pursuit_With_VFH_B.k < 2)) {
      if ((Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[Pure_Pursuit_With_VFH_B.k]
           ==
           Pure_Pursuit_With_VFH_B.VectorConcatenate[Pure_Pursuit_With_VFH_B.k])
          || (rtIsNaN
              (Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[Pure_Pursuit_With_VFH_B.k])
              && rtIsNaN
              (Pure_Pursuit_With_VFH_B.VectorConcatenate[Pure_Pursuit_With_VFH_B.k])))
      {
        rtb_FixPtRelationalOperator_i_0 = true;
      } else {
        rtb_FixPtRelationalOperator_i_0 = false;
      }

      if (!rtb_FixPtRelationalOperator_i_0) {
        rtb_FixPtRelationalOperator_idx = false;
        exitg1 = true;
      } else {
        Pure_Pursuit_With_VFH_B.k++;
      }
    }

    if (rtb_FixPtRelationalOperator_idx) {
      rtb_Memory = true;
    }

    if (!rtb_Memory) {
      Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[0] =
        Pure_Pursuit_With_VFH_B.VectorConcatenate[0];
      Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[1] =
        Pure_Pursuit_With_VFH_B.VectorConcatenate[1];
      Pure_Pursuit_With_VFH_DW.obj_a.ProjectionLineIndex = 0.0;
    }

    /* MATLABSystem: '<S2>/Pure Pursuit' */
    Pu_PurePursuitBase_stepInternal(&Pure_Pursuit_With_VFH_DW.obj_a,
      Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu,
      Pure_Pursuit_With_VFH_B.VectorConcatenate,
      &Pure_Pursuit_With_VFH_B.PurePursuit_o1,
      &Pure_Pursuit_With_VFH_B.PurePursuit_o2, Pure_Pursuit_With_VFH_B.Goal,
      &Pure_Pursuit_With_VFH_B.Switch);

    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    if (!Pure_Pursuit_With_VFH_isequal
        (Pure_Pursuit_With_VFH_DW.obj.DistanceLimits,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_DistanceLi)) {
      VectorFieldHistogramBase_set_Di(&Pure_Pursuit_With_VFH_DW.obj,
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_DistanceLi);
    }

    if (!Pure_Pursuit_With_VFH_isequal
        (Pure_Pursuit_With_VFH_DW.obj.HistogramThresholds,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_HistogramT)) {
      VectorFieldHistogramBase_set_Hi(&Pure_Pursuit_With_VFH_DW.obj,
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_HistogramT);
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.RobotRadius,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_RobotRadiu)) {
      Pure_Pursuit_With_VFH_DW.obj.RobotRadius =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_RobotRadiu;
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.SafetyDistance,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_SafetyDist)) {
      Pure_Pursuit_With_VFH_DW.obj.SafetyDistance =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_SafetyDist;
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.MinTurningRadius,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_MinTurning)) {
      Pure_Pursuit_With_VFH_DW.obj.MinTurningRadius =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_MinTurning;
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.TargetDirectionWeight,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_TargetDire)) {
      Pure_Pursuit_With_VFH_DW.obj.TargetDirectionWeight =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_TargetDire;
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.CurrentDirectionWeight,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_CurrentDir)) {
      Pure_Pursuit_With_VFH_DW.obj.CurrentDirectionWeight =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_CurrentDir;
    }

    if (!Pure_Pursuit_With_VFH_isequal_n
        (Pure_Pursuit_With_VFH_DW.obj.PreviousDirectionWeight,
         Pure_Pursuit_With_VFH_P.VectorFieldHistogram_PreviousDi)) {
      Pure_Pursuit_With_VFH_DW.obj.PreviousDirectionWeight =
        Pure_Pursuit_With_VFH_P.VectorFieldHistogram_PreviousDi;
    }

    /* MATLAB Function: '<S3>/MATLAB Function' */
    for (Pure_Pursuit_With_VFH_B.k = 0; Pure_Pursuit_With_VFH_B.k < 721;
         Pure_Pursuit_With_VFH_B.k++) {
      Pure_Pursuit_With_VFH_B.dv0[Pure_Pursuit_With_VFH_B.k] =
        Pure_Pursuit_With_VFH_B.In1.Ranges[Pure_Pursuit_With_VFH_B.k];
    }

    /* MATLABSystem: '<S1>/Vector Field Histogram' incorporates:
     *  MATLABSystem: '<S2>/Pure Pursuit'
     *  MATLABSystem: '<S2>/Pure Pursuit'
     */
    Pure_Pursuit_With_VFH_B.VectorFieldHistogram =
      VectorFieldHistogramBase_stepIm(&Pure_Pursuit_With_VFH_DW.obj,
      Pure_Pursuit_With_VFH_B.dv0, Pure_Pursuit_With_VFH_B.Angles,
      Pure_Pursuit_With_VFH_B.Switch);

    /* Outputs for Atomic SubSystem: '<S1>/Subscribe' */
    /* MATLABSystem: '<S8>/SourceBlock' */
    rtb_SourceBlock_o1_b = Sub_Pure_Pursuit_With_VFH_145.getLatestMessage
      (&b_varargout_2);
    rtb_SourceBlock_o2_d = b_varargout_2;

    /* Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' */
    Pure_Pursuit_W_EnabledSubsystem(rtb_SourceBlock_o1_b, &rtb_SourceBlock_o2_d,
      &Pure_Pursuit_With_VFH_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S8>/Enabled Subsystem' */

    /* End of Outputs for SubSystem: '<S1>/Subscribe' */

    /* Logic: '<S1>/Logical Operator1' incorporates:
     *  MATLABSystem: '<S1>/Vector Field Histogram'
     *  RelationalOperator: '<S1>/Relational Operator'
     */
    rtb_Memory = (rtIsNaN(Pure_Pursuit_With_VFH_B.VectorFieldHistogram) ||
                  Pure_Pursuit_With_VFH_B.EnabledSubsystem.In1.Data);

    /* Outputs for Enabled SubSystem: '<S5>/D Latch' incorporates:
     *  EnablePort: '<S9>/C'
     */
    /* Logic: '<S1>/Logical Operator' incorporates:
     *  Constant: '<S13>/Constant'
     *  Fcn: '<S2>/Fcn'
     *  Inport: '<S9>/D'
     *  MATLAB Function: '<S2>/ Extract Goal'
     *  RelationalOperator: '<S13>/Compare'
     *  SignalConversion: '<S2>/ConcatBufferAtVector ConcatenateIn1'
     *  SignalConversion: '<S2>/TmpSignal ConversionAtPure PursuitInport1'
     */
    /* MATLAB Function 'Compute Velocity and Heading for Path following/ Extract Goal': '<S12>:1' */
    /* '<S12>:1:9' */
    if (!rtb_Memory) {
      Pure_Pursuit_With_VFH_B.D =
        ((Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.X -
          Pure_Pursuit_With_VFH_B.VectorConcatenate[0]) * sin
         (Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu[2]) +
         (Pure_Pursuit_With_VFH_B.VectorConcatenate[1] -
          Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.Y) * cos
         (Pure_Pursuit_With_VFH_B.TmpSignalConversionAtPurePu[2]) >=
         Pure_Pursuit_With_VFH_P.Constant_Value_o);
    }

    /* End of Logic: '<S1>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S5>/D Latch' */

    /* Switch: '<S1>/Switch' incorporates:
     *  Constant: '<S1>/Constant'
     *  Constant: '<S1>/Constant1'
     */
    if (Pure_Pursuit_With_VFH_B.D) {
      Pure_Pursuit_With_VFH_B.Switch = Pure_Pursuit_With_VFH_P.Constant_Value_p;
    } else {
      Pure_Pursuit_With_VFH_B.Switch = Pure_Pursuit_With_VFH_P.Constant1_Value;
    }

    /* End of Switch: '<S1>/Switch' */

    /* Constant: '<S15>/Constant1' */
    Pure_Pursuit_With_VFH_B.Constant1_f =
      Pure_Pursuit_With_VFH_P.LinVelocityControl_enable;

    /* Sum: '<S2>/Subtract' incorporates:
     *  MATLAB Function: '<S2>/ Extract Goal'
     */
    Pure_Pursuit_With_VFH_B.aSinInput =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.X -
      Pure_Pursuit_With_VFH_B.VectorConcatenate[0];
    Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
      Pure_Pursuit_With_VFH_B.In1_m.Pose.Pose.Position.Y -
      Pure_Pursuit_With_VFH_B.VectorConcatenate[1];

    /* RelationalOperator: '<S14>/FixPt Relational Operator' incorporates:
     *  UnitDelay: '<S14>/Delay Input1'
     */
    rtb_FixPtRelationalOperator_idx =
      (Pure_Pursuit_With_VFH_B.VectorConcatenate[0] !=
       Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[0]);
    rtb_FixPtRelationalOperator_i_0 =
      (Pure_Pursuit_With_VFH_B.VectorConcatenate[1] !=
       Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[1]);

    /* Sqrt: '<S2>/Sqrt' incorporates:
     *  DotProduct: '<S2>/Dot Product'
     *  Sum: '<S2>/Subtract'
     */
    Pure_Pursuit_With_VFH_B.aSinInput = sqrt(Pure_Pursuit_With_VFH_B.aSinInput *
      Pure_Pursuit_With_VFH_B.aSinInput +
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 *
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1);

    /* Outputs for Triggered SubSystem: '<S2>/Sample and Hold' incorporates:
     *  TriggerPort: '<S16>/Trigger'
     */
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      if ((rtb_FixPtRelationalOperator_idx &&
           (Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[0] != POS_ZCSIG))
          || (rtb_FixPtRelationalOperator_i_0 &&
              (Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[1] !=
               POS_ZCSIG))) {
        /* Inport: '<S16>/In' */
        Pure_Pursuit_With_VFH_B.In = Pure_Pursuit_With_VFH_B.aSinInput;
      }

      Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[0] =
        rtb_FixPtRelationalOperator_idx;
      Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[1] =
        rtb_FixPtRelationalOperator_i_0;
    }

    /* End of Outputs for SubSystem: '<S2>/Sample and Hold' */

    /* Sum: '<S2>/Add1' */
    Pure_Pursuit_With_VFH_B.Add1 = Pure_Pursuit_With_VFH_B.In -
      Pure_Pursuit_With_VFH_B.aSinInput;
  }

  /* If: '<S15>/If1' incorporates:
   *  Constant: '<S17>/Constant'
   *  Constant: '<S17>/Constant1'
   *  Constant: '<S17>/Constant2'
   *  Constant: '<S19>/Constant'
   *  Constant: '<S20>/Constant'
   *  Constant: '<S21>/Constant'
   *  Constant: '<S22>/Constant'
   *  Inport: '<S18>/omega'
   *  Inport: '<S18>/v'
   */
  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    rtAction = (int8_T)!Pure_Pursuit_With_VFH_B.Constant1_f;
    Pure_Pursuit_With_VFH_DW.If1_ActiveSubsystem = rtAction;
  } else {
    rtAction = Pure_Pursuit_With_VFH_DW.If1_ActiveSubsystem;
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S15>/Active' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      Pure_Pursuit_With_VFH_B.Constant_j =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_c;
      Pure_Pursuit_With_VFH_B.Constant_k =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_c;
      Pure_Pursuit_With_VFH_B.Constant_n =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_a;
      Pure_Pursuit_With_VFH_B.Constant1 =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max;
      Pure_Pursuit_With_VFH_B.Constant2 =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_v_min;
    }

    /* Switch: '<S17>/Switch3' incorporates:
     *  Constant: '<S17>/Constant'
     *  Constant: '<S17>/Constant1'
     *  Constant: '<S17>/Constant2'
     *  Constant: '<S19>/Constant'
     *  Constant: '<S20>/Constant'
     *  Integrator: '<S17>/Integrator'
     *  Product: '<S17>/Divide'
     *  Product: '<S17>/Product'
     */
    if (Pure_Pursuit_With_VFH_B.PurePursuit_o1 != 0.0) {
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_X.Integrator_CSTATE /
        Pure_Pursuit_With_VFH_B.PurePursuit_o1 *
        Pure_Pursuit_With_VFH_B.PurePursuit_o2;
    } else {
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_B.PurePursuit_o2;
    }

    /* End of Switch: '<S17>/Switch3' */

    /* Saturate: '<S17>/SaturationOmega' */
    if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 >
        Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max) {
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max;
    } else {
      if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 <
          -Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max) {
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
          -Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max;
      }
    }

    /* End of Saturate: '<S17>/SaturationOmega' */
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      Pure_Pursuit_With_VFH_B.Constant_c =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max;
      Pure_Pursuit_With_VFH_B.Constant_cw =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max;
    }

    /* Switch: '<S21>/Switch5' incorporates:
     *  Abs: '<S21>/Abs'
     *  Constant: '<S21>/Constant'
     *  Constant: '<S22>/Constant'
     *  Integrator: '<S17>/Integrator'
     *  Math: '<S21>/Math Function'
     *  Product: '<S21>/Product'
     *  RelationalOperator: '<S22>/Compare'
     *
     * About '<S21>/Math Function':
     *  Operator: reciprocal
     */
    if (fabs(Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1) >=
        Pure_Pursuit_With_VFH_B.Constant_c) {
      Pure_Pursuit_With_VFH_B.Goal[0] = Pure_Pursuit_With_VFH_B.Constant_cw;

      /* Signum: '<S21>/Sign' */
      if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 < 0.0) {
        Pure_Pursuit_With_VFH_B.rtb_SaturationOmega_i = -1.0;
      } else if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 > 0.0) {
        Pure_Pursuit_With_VFH_B.rtb_SaturationOmega_i = 1.0;
      } else if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 == 0.0) {
        Pure_Pursuit_With_VFH_B.rtb_SaturationOmega_i = 0.0;
      } else {
        Pure_Pursuit_With_VFH_B.rtb_SaturationOmega_i = (rtNaN);
      }

      /* End of Signum: '<S21>/Sign' */
      Pure_Pursuit_With_VFH_B.Goal[1] = Pure_Pursuit_With_VFH_B.Constant_cw *
        Pure_Pursuit_With_VFH_X.Integrator_CSTATE * (1.0 /
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1) *
        Pure_Pursuit_With_VFH_B.rtb_SaturationOmega_i;
    } else {
      Pure_Pursuit_With_VFH_B.Goal[0] =
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1;
      Pure_Pursuit_With_VFH_B.Goal[1] =
        Pure_Pursuit_With_VFH_X.Integrator_CSTATE;
    }

    /* End of Switch: '<S21>/Switch5' */

    /* Switch: '<S17>/Switch1' incorporates:
     *  RelationalOperator: '<S19>/Compare'
     *  RelationalOperator: '<S20>/Compare'
     *  Sum: '<S17>/Sum'
     *  Switch: '<S17>/Switch2'
     */
    if (Pure_Pursuit_With_VFH_B.Add1 < Pure_Pursuit_With_VFH_B.Constant_j) {
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_B.Constant2;
    } else if (Pure_Pursuit_With_VFH_B.In - Pure_Pursuit_With_VFH_B.Add1 >
               Pure_Pursuit_With_VFH_B.Constant_k) {
      /* Switch: '<S17>/Switch4' incorporates:
       *  Gain: '<S17>/Gain1'
       *  Sqrt: '<S17>/Sqrt1'
       *  Switch: '<S17>/Switch2'
       */
      if (Pure_Pursuit_With_VFH_B.Constant_n != 0.0) {
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 = sqrt(2.0 *
          Pure_Pursuit_With_VFH_P.LinVelocityControl_a *
          Pure_Pursuit_With_VFH_B.Add1);
      } else {
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
          Pure_Pursuit_With_VFH_B.Constant1;
      }

      /* End of Switch: '<S17>/Switch4' */
    } else {
      /* Switch: '<S17>/Switch2' */
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_B.Constant2;
    }

    /* End of Switch: '<S17>/Switch1' */

    /* Saturate: '<S17>/Saturation' */
    if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 >
        Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max) {
      Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
        Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max;
    } else {
      if (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 <
          -Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max) {
        Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 =
          -Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max;
      }
    }

    /* End of Saturate: '<S17>/Saturation' */

    /* Gain: '<S17>/Gain' incorporates:
     *  Integrator: '<S17>/Integrator'
     *  Sum: '<S17>/Sum1'
     */
    Pure_Pursuit_With_VFH_B.Gain = (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 -
      Pure_Pursuit_With_VFH_X.Integrator_CSTATE) *
      Pure_Pursuit_With_VFH_P.LinVelocityControl_k;

    /* SignalConversion: '<S17>/OutportBufferForomega_new' */
    Pure_Pursuit_With_VFH_B.Merge1 = Pure_Pursuit_With_VFH_B.Goal[0];

    /* SignalConversion: '<S17>/OutportBufferForv_new' */
    Pure_Pursuit_With_VFH_B.Merge = Pure_Pursuit_With_VFH_B.Goal[1];

    /* End of Outputs for SubSystem: '<S15>/Active' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S15>/Passthrough' incorporates:
     *  ActionPort: '<S18>/Action Port'
     */
    Pure_Pursuit_With_VFH_B.Merge = Pure_Pursuit_With_VFH_B.PurePursuit_o1;
    Pure_Pursuit_With_VFH_B.Merge1 = Pure_Pursuit_With_VFH_B.PurePursuit_o2;

    /* End of Outputs for SubSystem: '<S15>/Passthrough' */
    break;
  }

  /* End of If: '<S15>/If1' */
  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    /* If: '<S1>/If' */
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      rtAction = (int8_T)!rtb_Memory;
      Pure_Pursuit_With_VFH_DW.If_ActiveSubsystem = rtAction;
    } else {
      rtAction = Pure_Pursuit_With_VFH_DW.If_ActiveSubsystem;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S1>/Recovery' incorporates:
       *  ActionPort: '<S7>/Action Port'
       */
      /* Gain: '<S7>/Gain' */
      Pure_Pursuit_With_VFH_B.Merge_e = Pure_Pursuit_With_VFH_P.Gain_Gain *
        Pure_Pursuit_With_VFH_B.Merge;

      /* Sum: '<S7>/Subtract' incorporates:
       *  Constant: '<S7>/OmegaRec'
       *  Product: '<S7>/Product'
       */
      Pure_Pursuit_With_VFH_B.Merge1_k = Pure_Pursuit_With_VFH_P.OmegaRec_Value *
        Pure_Pursuit_With_VFH_B.Switch - Pure_Pursuit_With_VFH_B.Merge1;

      /* End of Outputs for SubSystem: '<S1>/Recovery' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S1>/Path Following' incorporates:
       *  ActionPort: '<S6>/Action Port'
       */
      /* SignalConversion: '<S6>/OutportBufferFordV' incorporates:
       *  Constant: '<S6>/Constant'
       */
      Pure_Pursuit_With_VFH_B.Merge_e =
        Pure_Pursuit_With_VFH_P.Constant_Value_lm;

      /* MATLAB Function: '<S6>/Compute Angular Velocity' incorporates:
       *  Constant: '<S6>/MaxAngularVelocity'
       */
      /* MATLAB Function 'Adjust Velocities to Avoid Obstacles/Path Following/Compute Angular Velocity': '<S10>:1' */
      /* '<S10>:1:38' */
      /* '<S10>:1:39' */
      Pure_Pursuit_With_VFH_B.VectorFieldHistogram = rt_atan2d_snf(sin
        (Pure_Pursuit_With_VFH_B.VectorFieldHistogram), cos
        (Pure_Pursuit_With_VFH_B.VectorFieldHistogram));

      /* '<S10>:1:41' */
      if ((!rtIsInf(Pure_Pursuit_With_VFH_B.VectorFieldHistogram +
                    3.1415926535897931)) && (!rtIsNaN
           (Pure_Pursuit_With_VFH_B.VectorFieldHistogram + 3.1415926535897931)))
      {
        if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram + 3.1415926535897931 ==
            0.0) {
          Pure_Pursuit_With_VFH_B.Switch = 0.0;
        } else {
          Pure_Pursuit_With_VFH_B.Switch = fmod
            (Pure_Pursuit_With_VFH_B.VectorFieldHistogram + 3.1415926535897931,
             6.2831853071795862);
          rEQ0 = (Pure_Pursuit_With_VFH_B.Switch == 0.0);
          if (!rEQ0) {
            Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 = fabs
              ((Pure_Pursuit_With_VFH_B.VectorFieldHistogram +
                3.1415926535897931) / 6.2831853071795862);
            rEQ0 = (fabs(Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 - floor
                         (Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1 + 0.5)) <=
                    2.2204460492503131E-16 *
                    Pure_Pursuit_With_VFH_B.rtb_Subtract_idx_1);
          }

          if (rEQ0) {
            Pure_Pursuit_With_VFH_B.Switch = 0.0;
          } else {
            if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram +
                3.1415926535897931 < 0.0) {
              Pure_Pursuit_With_VFH_B.Switch += 6.2831853071795862;
            }
          }
        }
      } else {
        Pure_Pursuit_With_VFH_B.Switch = (rtNaN);
      }

      if ((Pure_Pursuit_With_VFH_B.Switch == 0.0) &&
          (Pure_Pursuit_With_VFH_B.VectorFieldHistogram + 3.1415926535897931 >
           0.0)) {
        Pure_Pursuit_With_VFH_B.Switch = 6.2831853071795862;
      }

      /* '<S10>:1:46' */
      Pure_Pursuit_With_VFH_B.VectorFieldHistogram = sin
        (Pure_Pursuit_With_VFH_B.Switch - 3.1415926535897931) * 2.0;
      if (fabs(fabs(Pure_Pursuit_With_VFH_B.Switch - 3.1415926535897931) -
               3.1415926535897931) < 1.0E-12) {
        /* '<S10>:1:50' */
        /* '<S10>:1:51' */
        if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram < 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = -1.0;
        } else if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram > 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = 1.0;
        } else if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram == 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = 0.0;
        } else {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = (rtNaN);
        }
      }

      if (fabs(Pure_Pursuit_With_VFH_B.VectorFieldHistogram) >
          Pure_Pursuit_With_VFH_P.MaxAngularVelocity_Value) {
        /* '<S10>:1:54' */
        /* '<S10>:1:55' */
        if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram < 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = -1.0;
        } else if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram > 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = 1.0;
        } else if (Pure_Pursuit_With_VFH_B.VectorFieldHistogram == 0.0) {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = 0.0;
        } else {
          Pure_Pursuit_With_VFH_B.VectorFieldHistogram = (rtNaN);
        }

        Pure_Pursuit_With_VFH_B.VectorFieldHistogram *=
          Pure_Pursuit_With_VFH_P.MaxAngularVelocity_Value;
      }

      /* End of MATLAB Function: '<S6>/Compute Angular Velocity' */

      /* Sum: '<S6>/Subtract' */
      Pure_Pursuit_With_VFH_B.Merge1_k =
        Pure_Pursuit_With_VFH_B.VectorFieldHistogram -
        Pure_Pursuit_With_VFH_B.Merge1;

      /* End of Outputs for SubSystem: '<S1>/Path Following' */
      break;
    }

    /* End of If: '<S1>/If' */

    /* RelationalOperator: '<S2>/Relational Operator' incorporates:
     *  Constant: '<S2>/GoalRadius'
     */
    rEQ0 = (Pure_Pursuit_With_VFH_B.aSinInput <=
            Pure_Pursuit_With_VFH_P.GoalRadius_Value);

    /* Logic: '<S3>/Logical Operator' incorporates:
     *  Memory: '<S3>/Memory'
     */
    rtb_LogicalOperator_m = (Pure_Pursuit_With_VFH_DW.Memory_PreviousInput ||
      rtb_SourceBlock_o1_g);

    /* Logic: '<S3>/Logical Operator1' */
    Pure_Pursuit_With_VFH_B.LogicalOperator1 = (rtb_LogicalOperator_m &&
      rtb_SourceBlock_o1_i);

    /* Outputs for Enabled SubSystem: '<Root>/Outputs' incorporates:
     *  EnablePort: '<S4>/Enable'
     */
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      if (Pure_Pursuit_With_VFH_B.LogicalOperator1) {
        if (!Pure_Pursuit_With_VFH_DW.Outputs_MODE) {
          Pure_Pursuit_With_VFH_DW.Outputs_MODE = true;
        }
      } else {
        if (Pure_Pursuit_With_VFH_DW.Outputs_MODE) {
          Pure_Pursuit_With_VFH_DW.Outputs_MODE = false;
        }
      }
    }

    /* End of Outputs for SubSystem: '<Root>/Outputs' */
  }

  /* Outputs for Enabled SubSystem: '<Root>/Outputs' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  if (Pure_Pursuit_With_VFH_DW.Outputs_MODE) {
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      /* Constant: '<S31>/Constant' */
      Pure_Pursuit_With_VFH_B.Constant =
        Pure_Pursuit_With_VFH_P.Constant_Value_e;

      /* Logic: '<S4>/Logical Operator' */
      Pure_Pursuit_With_VFH_B.LogicalOperator = !rEQ0;

      /* Outputs for Atomic SubSystem: '<S4>/Subscribe' */
      /* MATLABSystem: '<S33>/SourceBlock' */
      rtb_SourceBlock_o1 = Sub_Pure_Pursuit_With_VFH_119.getLatestMessage
        (&b_varargout_2);
      rtb_SourceBlock_o2_j = b_varargout_2;

      /* Outputs for Enabled SubSystem: '<S33>/Enabled Subsystem' */
      Pure_Pursuit_W_EnabledSubsystem(rtb_SourceBlock_o1, &rtb_SourceBlock_o2_j,
        &Pure_Pursuit_With_VFH_B.EnabledSubsystem_d);

      /* End of Outputs for SubSystem: '<S33>/Enabled Subsystem' */

      /* End of Outputs for SubSystem: '<S4>/Subscribe' */

      /* Logic: '<S4>/Logical Operator1' */
      Pure_Pursuit_With_VFH_B.LogicalOperator1_m =
        !Pure_Pursuit_With_VFH_B.EnabledSubsystem_d.In1.Data;
    }

    /* BusAssignment: '<S4>/Bus Assignment1' incorporates:
     *  Product: '<S4>/Product'
     *  Product: '<S4>/Product1'
     *  Sum: '<S4>/Add'
     *  Sum: '<S4>/Add1'
     */
    Pure_Pursuit_With_VFH_B.BusAssignment1 = Pure_Pursuit_With_VFH_B.Constant;
    Pure_Pursuit_With_VFH_B.BusAssignment1.Linear.X =
      (Pure_Pursuit_With_VFH_B.Merge_e + Pure_Pursuit_With_VFH_B.Merge) *
      (real_T)Pure_Pursuit_With_VFH_B.LogicalOperator * (real_T)
      Pure_Pursuit_With_VFH_B.LogicalOperator1_m;
    Pure_Pursuit_With_VFH_B.BusAssignment1.Angular.Z =
      (Pure_Pursuit_With_VFH_B.Merge1_k + Pure_Pursuit_With_VFH_B.Merge1) *
      (real_T)Pure_Pursuit_With_VFH_B.LogicalOperator * (real_T)
      Pure_Pursuit_With_VFH_B.LogicalOperator1_m;

    /* Outputs for Atomic SubSystem: '<S4>/Publish2' */
    /* Start for MATLABSystem: '<S32>/SinkBlock' */
    Pub_Pure_Pursuit_With_VFH_81.publish(&Pure_Pursuit_With_VFH_B.BusAssignment1);

    /* End of Outputs for SubSystem: '<S4>/Publish2' */
  }

  /* End of Outputs for SubSystem: '<Root>/Outputs' */
  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
      /* Update for UnitDelay: '<S14>/Delay Input1' */
      Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[0] =
        Pure_Pursuit_With_VFH_B.VectorConcatenate[0];
      Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[1] =
        Pure_Pursuit_With_VFH_B.VectorConcatenate[1];

      /* Update for Memory: '<S3>/Memory' */
      Pure_Pursuit_With_VFH_DW.Memory_PreviousInput = rtb_LogicalOperator_m;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&Pure_Pursuit_With_VFH_M))) {
    rt_ertODEUpdateContinuousStates(&(&Pure_Pursuit_With_VFH_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&Pure_Pursuit_With_VFH_M)->Timing.clockTick0)) {
      ++(&Pure_Pursuit_With_VFH_M)->Timing.clockTickH0;
    }

    (&Pure_Pursuit_With_VFH_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&Pure_Pursuit_With_VFH_M)->solverInfo);

    {
      /* Update absolute timer for sample time: [0.05s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.05, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&Pure_Pursuit_With_VFH_M)->Timing.clockTick1++;
      if (!(&Pure_Pursuit_With_VFH_M)->Timing.clockTick1) {
        (&Pure_Pursuit_With_VFH_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFH_derivatives()
{
  XDot_Pure_Pursuit_With_VFH_T *_rtXdot;
  _rtXdot = ((XDot_Pure_Pursuit_With_VFH_T *) (&Pure_Pursuit_With_VFH_M)->derivs);

  /* Derivatives for If: '<S15>/If1' */
  ((XDot_Pure_Pursuit_With_VFH_T *) (&Pure_Pursuit_With_VFH_M)->derivs)
    ->Integrator_CSTATE = 0.0;
  if (Pure_Pursuit_With_VFH_DW.If1_ActiveSubsystem == 0) {
    /* Derivatives for IfAction SubSystem: '<S15>/Active' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    /* Derivatives for Integrator: '<S17>/Integrator' */
    _rtXdot->Integrator_CSTATE = Pure_Pursuit_With_VFH_B.Gain;

    /* End of Derivatives for SubSystem: '<S15>/Active' */
  }

  /* End of Derivatives for If: '<S15>/If1' */
}

/* Model initialize function */
void Pure_Pursuit_With_VFHModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)(&Pure_Pursuit_With_VFH_M), 0,
                sizeof(RT_MODEL_Pure_Pursuit_With_VF_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                          &(&Pure_Pursuit_With_VFH_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo, &rtmGetTPtr
                ((&Pure_Pursuit_With_VFH_M)));
    rtsiSetStepSizePtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                       &(&Pure_Pursuit_With_VFH_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                 &(&Pure_Pursuit_With_VFH_M)->derivs);
    rtsiSetContStatesPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo, (real_T **) &(
      &Pure_Pursuit_With_VFH_M)->contStates);
    rtsiSetNumContStatesPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
      &(&Pure_Pursuit_With_VFH_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
      &(&Pure_Pursuit_With_VFH_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
      &(&Pure_Pursuit_With_VFH_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
      &(&Pure_Pursuit_With_VFH_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                          (&rtmGetErrorStatus((&Pure_Pursuit_With_VFH_M))));
    rtsiSetRTModelPtr(&(&Pure_Pursuit_With_VFH_M)->solverInfo,
                      (&Pure_Pursuit_With_VFH_M));
  }

  rtsiSetSimTimeStep(&(&Pure_Pursuit_With_VFH_M)->solverInfo, MAJOR_TIME_STEP);
  (&Pure_Pursuit_With_VFH_M)->intgData.y = (&Pure_Pursuit_With_VFH_M)->odeY;
  (&Pure_Pursuit_With_VFH_M)->intgData.f[0] = (&Pure_Pursuit_With_VFH_M)->odeF[0];
  (&Pure_Pursuit_With_VFH_M)->intgData.f[1] = (&Pure_Pursuit_With_VFH_M)->odeF[1];
  (&Pure_Pursuit_With_VFH_M)->intgData.f[2] = (&Pure_Pursuit_With_VFH_M)->odeF[2];
  getRTM()->contStates = ((X_Pure_Pursuit_With_VFH_T *) &Pure_Pursuit_With_VFH_X);
  rtsiSetSolverData(&(&Pure_Pursuit_With_VFH_M)->solverInfo, (void *)
                    &(&Pure_Pursuit_With_VFH_M)->intgData);
  rtsiSetSolverName(&(&Pure_Pursuit_With_VFH_M)->solverInfo,"ode3");
  rtmSetTPtr(getRTM(), &(&Pure_Pursuit_With_VFH_M)->Timing.tArray[0]);
  (&Pure_Pursuit_With_VFH_M)->Timing.stepSize0 = 0.05;

  /* block I/O */
  (void) memset(((void *) &Pure_Pursuit_With_VFH_B), 0,
                sizeof(B_Pure_Pursuit_With_VFH_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Pure_Pursuit_With_VFH_X, 0,
                  sizeof(X_Pure_Pursuit_With_VFH_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Pure_Pursuit_With_VFH_DW, 0,
                sizeof(DW_Pure_Pursuit_With_VFH_T));

  {
    robotics_slalgs_internal_Vect_T *obj;
    static const char_T tmp[30] = { '/', 'm', 'o', 'b', 'i', 'l', 'e', '_', 'b',
      'a', 's', 'e', '/', 'c', 'o', 'm', 'm', 'a', 'n', 'd', 's', '/', 'v', 'e',
      'l', 'o', 'c', 'i', 't', 'y' };

    static const char_T tmp_0[15] = { '/', 'e', 'm', 'e', 'r', 'g', 'e', 'n',
      'c', 'y', '_', 's', 't', 'o', 'p' };

    static const char_T tmp_1[18] = { '/', 'C', 'o', 'l', 'l', 'i', 's', 'i',
      'o', 'n', '_', 'w', 'a', 'r', 'n', 'i', 'n', 'g' };

    static const char_T tmp_2[14] = { '/', 'n', 'e', 'x', 't', '_', 'w', 'a',
      'y', 'p', 'o', 'i', 'n', 't' };

    static const char_T tmp_3[13] = { '/', 'c', 'u', 'r', 'r', 'e', 'n', 't',
      '_', 'p', 'o', 's', 'e' };

    static const char_T tmp_4[5] = { '/', 's', 'c', 'a', 'n' };

    char_T tmp_5[6];
    int32_T i;
    real_T angularLimits_idx_0;
    real_T angularLimits_idx_1;

    /* Start for Atomic SubSystem: '<S3>/Subscribe' */
    /* Start for MATLABSystem: '<S24>/SourceBlock' */
    Pure_Pursuit_With_VFH_DW.obj_m.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_b = true;
    Pure_Pursuit_With_VFH_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 5; i++) {
      tmp_5[i] = tmp_4[i];
    }

    tmp_5[5] = '\x00';
    Sub_Pure_Pursuit_With_VFH_54.createSubscriber(tmp_5,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S24>/SourceBlock' */
    /* End of Start for SubSystem: '<S3>/Subscribe' */

    /* Start for Atomic SubSystem: '<S3>/Subscribe1' */
    /* Start for MATLABSystem: '<S25>/SourceBlock' */
    Pure_Pursuit_With_VFH_DW.obj_n.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_l = true;
    Pure_Pursuit_With_VFH_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      Pure_Pursuit_With_VFH_B.cv4[i] = tmp_3[i];
    }

    Pure_Pursuit_With_VFH_B.cv4[13] = '\x00';
    Sub_Pure_Pursuit_With_VFH_55.createSubscriber(Pure_Pursuit_With_VFH_B.cv4,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S25>/SourceBlock' */
    /* End of Start for SubSystem: '<S3>/Subscribe1' */

    /* Start for Atomic SubSystem: '<S3>/Subscribe2' */
    /* Start for MATLABSystem: '<S26>/SourceBlock' */
    Pure_Pursuit_With_VFH_DW.obj_b.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_o = true;
    Pure_Pursuit_With_VFH_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      Pure_Pursuit_With_VFH_B.cv3[i] = tmp_2[i];
    }

    Pure_Pursuit_With_VFH_B.cv3[14] = '\x00';
    Sub_Pure_Pursuit_With_VFH_87.createSubscriber(Pure_Pursuit_With_VFH_B.cv3,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S26>/SourceBlock' */
    /* End of Start for SubSystem: '<S3>/Subscribe2' */

    /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
    Pure_Pursuit_With_VFH_DW.obj_a.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_d = true;
    Pure_Pursuit_With_VFH_DW.obj_a.DesiredLinearVelocity =
      Pure_Pursuit_With_VFH_P.PurePursuit_DesiredLinearVeloci;
    Pure_Pursuit_With_VFH_DW.obj_a.MaxAngularVelocity =
      Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity;
    Pure_Pursuit_With_VFH_DW.obj_a.LookaheadDistance =
      Pure_Pursuit_With_VFH_P.PurePursuit_LookaheadDistance;
    Pure_Pursuit_With_VFH_DW.obj_a.isInitialized = 1;
    Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[0] = (rtNaN);
    Pure_Pursuit_With_VFH_DW.obj_a.LookaheadPoint[0] = 0.0;
    Pure_Pursuit_With_VFH_DW.obj_a.WaypointsInternal[1] = (rtNaN);
    Pure_Pursuit_With_VFH_DW.obj_a.LookaheadPoint[1] = 0.0;
    Pure_Pursuit_With_VFH_DW.obj_a.LastPose[0] = 0.0;
    Pure_Pursuit_With_VFH_DW.obj_a.LastPose[1] = 0.0;
    Pure_Pursuit_With_VFH_DW.obj_a.LastPose[2] = 0.0;
    Pure_Pursuit_With_VFH_DW.obj_a.ProjectionPoint[0] = (rtNaN);
    Pure_Pursuit_With_VFH_DW.obj_a.ProjectionPoint[1] = (rtNaN);
    Pure_Pursuit_With_VFH_DW.obj_a.ProjectionLineIndex = 0.0;

    /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
    Pure_Pursuit_With_VFH_DW.obj.NarrowOpeningThreshold = 0.8;
    Pure_Pursuit_With_VFH_DW.obj.AngularLimits[0] = -3.1415926535897931;
    Pure_Pursuit_With_VFH_DW.obj.AngularLimits[1] = 3.1415926535897931;
    Pure_Pursuit_With_VFH_DW.obj.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_c = true;
    VectorFieldHistogramBase_set_Di(&Pure_Pursuit_With_VFH_DW.obj,
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_DistanceLi);
    VectorFieldHistogramBase_set_Hi(&Pure_Pursuit_With_VFH_DW.obj,
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_HistogramT);
    Pure_Pursuit_With_VFH_DW.obj.RobotRadius =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_RobotRadiu;
    Pure_Pursuit_With_VFH_DW.obj.SafetyDistance =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_SafetyDist;
    Pure_Pursuit_With_VFH_DW.obj.MinTurningRadius =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_MinTurning;
    Pure_Pursuit_With_VFH_DW.obj.TargetDirectionWeight =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_TargetDire;
    Pure_Pursuit_With_VFH_DW.obj.CurrentDirectionWeight =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_CurrentDir;
    Pure_Pursuit_With_VFH_DW.obj.PreviousDirectionWeight =
      Pure_Pursuit_With_VFH_P.VectorFieldHistogram_PreviousDi;
    obj = &Pure_Pursuit_With_VFH_DW.obj;
    Pure_Pursuit_With_VFH_DW.obj.isInitialized = 1;
    obj->PreviousDirection = 0.0;
    angularLimits_idx_0 = obj->AngularLimits[0];
    angularLimits_idx_1 = obj->AngularLimits[1];
    Pure_Pursuit_With_VFH_linspace(angularLimits_idx_0 + 0.013089969389957471,
      angularLimits_idx_1 - 0.013089969389957471, Pure_Pursuit_With_VFH_B.dv3);
    for (i = 0; i < 240; i++) {
      obj->AngularSectorMidPoints[i] = Pure_Pursuit_With_VFH_B.dv3[i];
    }

    angularLimits_idx_0 = Pure_Pursuit_With_VFH_angdiff
      (obj->AngularSectorMidPoints[0], obj->AngularSectorMidPoints[1]);
    obj->AngularDifference = fabs(angularLimits_idx_0);
    angularLimits_idx_0 = obj->AngularDifference;
    angularLimits_idx_0 /= 2.0;
    for (i = 0; i < 240; i++) {
      obj->AngularSectorStartPoints[i] = obj->AngularSectorMidPoints[i] -
        angularLimits_idx_0;
    }

    for (i = 0; i < 240; i++) {
      obj->BinaryHistogram[i] = false;
    }

    /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */

    /* Start for Atomic SubSystem: '<S1>/Subscribe' */
    /* Start for MATLABSystem: '<S8>/SourceBlock' */
    Pure_Pursuit_With_VFH_DW.obj_p.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_a = true;
    Pure_Pursuit_With_VFH_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      Pure_Pursuit_With_VFH_B.cv1[i] = tmp_1[i];
    }

    Pure_Pursuit_With_VFH_B.cv1[18] = '\x00';
    Sub_Pure_Pursuit_With_VFH_145.createSubscriber(Pure_Pursuit_With_VFH_B.cv1,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S8>/SourceBlock' */
    /* End of Start for SubSystem: '<S1>/Subscribe' */

    /* Start for If: '<S15>/If1' */
    Pure_Pursuit_With_VFH_DW.If1_ActiveSubsystem = -1;

    /* Start for If: '<S1>/If' */
    Pure_Pursuit_With_VFH_DW.If_ActiveSubsystem = -1;

    /* Start for Enabled SubSystem: '<Root>/Outputs' */
    Pure_Pursuit_With_VFH_DW.Outputs_MODE = false;

    /* Start for Atomic SubSystem: '<S4>/Subscribe' */
    /* Start for MATLABSystem: '<S33>/SourceBlock' */
    Pure_Pursuit_With_VFH_DW.obj_e.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty = true;
    Pure_Pursuit_With_VFH_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      Pure_Pursuit_With_VFH_B.cv2[i] = tmp_0[i];
    }

    Pure_Pursuit_With_VFH_B.cv2[15] = '\x00';
    Sub_Pure_Pursuit_With_VFH_119.createSubscriber(Pure_Pursuit_With_VFH_B.cv2,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S33>/SourceBlock' */
    /* End of Start for SubSystem: '<S4>/Subscribe' */

    /* Start for Atomic SubSystem: '<S4>/Publish2' */
    /* Start for MATLABSystem: '<S32>/SinkBlock' */
    Pure_Pursuit_With_VFH_DW.obj_l.isInitialized = 0;
    Pure_Pursuit_With_VFH_DW.objisempty_p = true;
    Pure_Pursuit_With_VFH_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 30; i++) {
      Pure_Pursuit_With_VFH_B.cv0[i] = tmp[i];
    }

    Pure_Pursuit_With_VFH_B.cv0[30] = '\x00';
    Pub_Pure_Pursuit_With_VFH_81.createPublisher(Pure_Pursuit_With_VFH_B.cv0,
      Pure_Pursuit_Wi_MessageQueueLen);

    /* End of Start for MATLABSystem: '<S32>/SinkBlock' */
    /* End of Start for SubSystem: '<S4>/Publish2' */
    /* End of Start for SubSystem: '<Root>/Outputs' */
  }

  Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[0] = POS_ZCSIG;
  Pure_Pursuit_With_VFH_PrevZCX.SampleandHold_Trig_ZCE[1] = POS_ZCSIG;

  /* user code (Initialize function Body) */
  ros::NodeHandle private_nh = ros::NodeHandle("~");
  ROS_INFO_STREAM("Node's namespace: " << private_nh.getNamespace());

  // Tunable parameters of the Pure Pursuit algorithm
  double desired_lin_vel;
  if (private_nh.getParam("pure_pursuit/desired_lin_vel", desired_lin_vel))
    this->Pure_Pursuit_With_VFH_P.PurePursuit_DesiredLinearVeloci =
      desired_lin_vel;
  ROS_WARN_STREAM("Pure pursuit desired linear velocity set to " <<
                  this->Pure_Pursuit_With_VFH_P.PurePursuit_DesiredLinearVeloci <<
                  " m/s.");
  double max_ang_vel;
  if (private_nh.getParam("pure_pursuit/max_ang_vel", max_ang_vel))
    this->Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity = max_ang_vel;
  this->Pure_Pursuit_With_VFH_P.MaxAngularVelocity_Value =
    this->Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity;
  double lookahead_dist;
  if (private_nh.getParam("pure_pursuit/lookahead_dist", lookahead_dist))
    this->Pure_Pursuit_With_VFH_P.PurePursuit_LookaheadDistance = lookahead_dist;

  // Goal reach tolerance
  double goal_radius;
  if (private_nh.getParam("goal_radius", goal_radius))
    this->Pure_Pursuit_With_VFH_P.GoalRadius_Value = goal_radius;

  // Tunable parameters of the VFH algorithm
  std::vector<double> range_dist;
  if (private_nh.getParam("vfh/range_dist", range_dist)) {
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_DistanceLi[0] =
      range_dist[0];
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_DistanceLi[1] =
      range_dist[1];
  }

  std::vector<double> hist_thr;
  if (private_nh.getParam("vfh/hist_thr", hist_thr)) {
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_HistogramT[0] = hist_thr
      [0];
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_HistogramT[1] = hist_thr
      [1];
  }

  double robot_radius;
  if (private_nh.getParam("vfh/robot_radius", robot_radius))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_RobotRadiu = robot_radius;
  double safety_dist;
  if (private_nh.getParam("vfh/safety_dist", safety_dist))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_SafetyDist = safety_dist;
  double min_turning_radius;
  if (private_nh.getParam("vfh/min_turning_radius", min_turning_radius))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_MinTurning =
      min_turning_radius;
  double target_dir_w;
  if (private_nh.getParam("vfh/target_dir_w", target_dir_w))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_TargetDire = target_dir_w;
  double current_dir_w;
  if (private_nh.getParam("vfh/current_dir_w", current_dir_w))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_CurrentDir =
      current_dir_w;
  double prev_dir_w;
  if (private_nh.getParam("vfh/prev_dir_w", prev_dir_w))
    this->Pure_Pursuit_With_VFH_P.VectorFieldHistogram_PreviousDi = prev_dir_w;

  // Tunable parameters of the Linear Velocity Control
  double double_param;
  bool bool_param;
  if (private_nh.getParam("velocity_profile/a", double_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_a = double_param;
  if (private_nh.getParam("velocity_profile/c", double_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_c = double_param;
  if (private_nh.getParam("velocity_profile/k", double_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_k = double_param;
  this->Pure_Pursuit_With_VFH_P.LinVelocityControl_omega_max =
    this->Pure_Pursuit_With_VFH_P.PurePursuit_MaxAngularVelocity;
  if (private_nh.getParam("velocity_profile/v_max", double_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_v_max = double_param;
  if (private_nh.getParam("velocity_profile/v_min", double_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_v_min = double_param;
  if (private_nh.getParam("velocity_profile/enable", bool_param))
    this->Pure_Pursuit_With_VFH_P.LinVelocityControl_enable = bool_param;

  /* InitializeConditions for Memory: '<S3>/Memory' */
  Pure_Pursuit_With_VFH_DW.Memory_PreviousInput =
    Pure_Pursuit_With_VFH_P.Memory_InitialCondition;

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S24>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S28>/Out1' */
  Pure_Pursuit_With_VFH_B.In1 = Pure_Pursuit_With_VFH_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S24>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S3>/Subscribe' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subscribe1' */
  /* SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S29>/Out1' */
  Pure_Pursuit_With_VFH_B.In1_m = Pure_Pursuit_With_VFH_P.Out1_Y0_g;

  /* End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S3>/Subscribe1' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subscribe2' */
  /* SystemInitialize for Enabled SubSystem: '<S26>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S30>/Out1' */
  Pure_Pursuit_With_VFH_B.In1_p = Pure_Pursuit_With_VFH_P.Out1_Y0_m;

  /* End of SystemInitialize for SubSystem: '<S26>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S3>/Subscribe2' */

  /* InitializeConditions for UnitDelay: '<S14>/Delay Input1' */
  Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[0] =
    Pure_Pursuit_With_VFH_P.DetectChange_vinit;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  Pure_Pursuit_With_VFH_DW.obj_a.LookaheadPoint[0] *= 0.0;

  /* InitializeConditions for UnitDelay: '<S14>/Delay Input1' */
  Pure_Pursuit_With_VFH_DW.DelayInput1_DSTATE[1] =
    Pure_Pursuit_With_VFH_P.DetectChange_vinit;

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  Pure_Pursuit_With_VFH_DW.obj_a.LookaheadPoint[1] *= 0.0;
  Pure_Pursuit_With_VFH_DW.obj_a.LastPose[0] *= 0.0;
  Pure_Pursuit_With_VFH_DW.obj_a.LastPose[1] *= 0.0;
  Pure_Pursuit_With_VFH_DW.obj_a.LastPose[2] *= 0.0;
  Pure_Pursuit_With_VFH_DW.obj_a.ProjectionPoint[0] = (rtNaN);
  Pure_Pursuit_With_VFH_DW.obj_a.ProjectionPoint[1] = (rtNaN);
  Pure_Pursuit_With_VFH_DW.obj_a.ProjectionLineIndex *= 0.0;

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  memset(&Pure_Pursuit_With_VFH_DW.obj.BinaryHistogram[0], 0, 240U * sizeof
         (boolean_T));
  Pure_Pursuit_With_VFH_DW.obj.PreviousDirection *= 0.0;

  /* SystemInitialize for Atomic SubSystem: '<S1>/Subscribe' */

  /* SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem' */
  Pure_Purs_EnabledSubsystem_Init(&Pure_Pursuit_With_VFH_B.EnabledSubsystem,
    &Pure_Pursuit_With_VFH_P.EnabledSubsystem);

  /* End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem' */

  /* End of SystemInitialize for SubSystem: '<S1>/Subscribe' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/D Latch' */
  /* SystemInitialize for Outport: '<S9>/Q' */
  Pure_Pursuit_With_VFH_B.D = Pure_Pursuit_With_VFH_P.Q_Y0;

  /* End of SystemInitialize for SubSystem: '<S5>/D Latch' */

  /* SystemInitialize for Triggered SubSystem: '<S2>/Sample and Hold' */
  /* SystemInitialize for Outport: '<S16>/ ' */
  Pure_Pursuit_With_VFH_B.In = Pure_Pursuit_With_VFH_P._Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Sample and Hold' */

  /* SystemInitialize for IfAction SubSystem: '<S15>/Active' */
  /* InitializeConditions for Integrator: '<S17>/Integrator' */
  Pure_Pursuit_With_VFH_X.Integrator_CSTATE =
    Pure_Pursuit_With_VFH_P.Integrator_IC;

  /* End of SystemInitialize for SubSystem: '<S15>/Active' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/Outputs' */

  /* SystemInitialize for Atomic SubSystem: '<S4>/Subscribe' */

  /* SystemInitialize for Enabled SubSystem: '<S33>/Enabled Subsystem' */
  Pure_Purs_EnabledSubsystem_Init(&Pure_Pursuit_With_VFH_B.EnabledSubsystem_d,
    &Pure_Pursuit_With_VFH_P.EnabledSubsystem_d);

  /* End of SystemInitialize for SubSystem: '<S33>/Enabled Subsystem' */

  /* End of SystemInitialize for SubSystem: '<S4>/Subscribe' */

  /* End of SystemInitialize for SubSystem: '<Root>/Outputs' */
}

/* Model terminate function */
void Pure_Pursuit_With_VFHModelClass::terminate()
{
  /* Terminate for Atomic SubSystem: '<S3>/Subscribe' */
  /* Start for MATLABSystem: '<S24>/SourceBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_m.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_m.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S24>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S3>/Subscribe' */

  /* Terminate for Atomic SubSystem: '<S3>/Subscribe1' */
  /* Start for MATLABSystem: '<S25>/SourceBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_n.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_n.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S25>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S3>/Subscribe1' */

  /* Terminate for Atomic SubSystem: '<S3>/Subscribe2' */
  /* Start for MATLABSystem: '<S26>/SourceBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_b.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_b.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S26>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S3>/Subscribe2' */

  /* Start for MATLABSystem: '<S2>/Pure Pursuit' */
  if (Pure_Pursuit_With_VFH_DW.obj_a.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_a.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S2>/Pure Pursuit' */

  /* Start for MATLABSystem: '<S1>/Vector Field Histogram' */
  if (Pure_Pursuit_With_VFH_DW.obj.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S1>/Vector Field Histogram' */

  /* Terminate for Atomic SubSystem: '<S1>/Subscribe' */
  /* Start for MATLABSystem: '<S8>/SourceBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_p.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_p.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S8>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S1>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<Root>/Outputs' */
  /* Terminate for Atomic SubSystem: '<S4>/Subscribe' */
  /* Start for MATLABSystem: '<S33>/SourceBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_e.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_e.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S33>/SourceBlock' */
  /* End of Terminate for SubSystem: '<S4>/Subscribe' */

  /* Terminate for Atomic SubSystem: '<S4>/Publish2' */
  /* Start for MATLABSystem: '<S32>/SinkBlock' */
  if (Pure_Pursuit_With_VFH_DW.obj_l.isInitialized == 1) {
    Pure_Pursuit_With_VFH_DW.obj_l.isInitialized = 2;
  }

  /* End of Start for MATLABSystem: '<S32>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S4>/Publish2' */
  /* End of Terminate for SubSystem: '<Root>/Outputs' */
}

/* Constructor */
Pure_Pursuit_With_VFHModelClass::Pure_Pursuit_With_VFHModelClass()
{
  static const P_Pure_Pursuit_With_VFH_T Pure_Pursuit_With_VFH_P_temp = {
    /* Mask Parameter: LinVelocityControl_a
     * Referenced by:
     *   '<S17>/Constant'
     *   '<S17>/Gain1'
     */
    0.3,

    /* Mask Parameter: LinVelocityControl_c
     * Referenced by:
     *   '<S19>/Constant'
     *   '<S20>/Constant'
     */
    0.5,

    /* Mask Parameter: LinVelocityControl_k
     * Referenced by: '<S17>/Gain'
     */
    10.0,

    /* Mask Parameter: LinVelocityControl_omega_max
     * Referenced by:
     *   '<S17>/SaturationOmega'
     *   '<S21>/Constant'
     *   '<S22>/Constant'
     */
    1.0,

    /* Mask Parameter: LinVelocityControl_v_max
     * Referenced by:
     *   '<S17>/Constant1'
     *   '<S17>/Saturation'
     */
    0.9,

    /* Mask Parameter: LinVelocityControl_v_min
     * Referenced by: '<S17>/Constant2'
     */
    0.2,

    /* Mask Parameter: LinVelocityControl_enable
     * Referenced by: '<S15>/Constant1'
     */
    1,

    /* Mask Parameter: DetectChange_vinit
     * Referenced by: '<S14>/Delay Input1'
     */
    1,

    /* Computed Parameter: Out1_Y0
     * Referenced by: '<S28>/Out1'
     */
    {
      0.0F,                            /* AngleMin */
      0.0F,                            /* AngleMax */
      0.0F,                            /* AngleIncrement */
      0.0F,                            /* TimeIncrement */
      0.0F,                            /* ScanTime */
      0.0F,                            /* RangeMin */
      0.0F,                            /* RangeMax */

      {
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F }
      ,                                /* Ranges */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* Ranges_SL_Info */

      {
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F }
      ,                                /* Intensities */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* Intensities_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
        }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      }                                /* Header */
    },

    /* Computed Parameter: Constant_Value
     * Referenced by: '<S24>/Constant'
     */
    {
      0.0F,                            /* AngleMin */
      0.0F,                            /* AngleMax */
      0.0F,                            /* AngleIncrement */
      0.0F,                            /* TimeIncrement */
      0.0F,                            /* ScanTime */
      0.0F,                            /* RangeMin */
      0.0F,                            /* RangeMax */

      {
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F }
      ,                                /* Ranges */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* Ranges_SL_Info */

      {
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F }
      ,                                /* Intensities */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* Intensities_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
        }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      }                                /* Header */
    },

    /* Computed Parameter: Out1_Y0_g
     * Referenced by: '<S29>/Out1'
     */
    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* ChildFrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* ChildFrameId_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
        }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      },                               /* Header */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Position */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0,                       /* Z */
            0.0                        /* W */
          }                            /* Orientation */
        }                              /* Pose */
      },                               /* Pose */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Linear */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          }                            /* Angular */
        }                              /* Twist */
      }                                /* Twist */
    },

    /* Computed Parameter: Constant_Value_m
     * Referenced by: '<S25>/Constant'
     */
    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* ChildFrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* ChildFrameId_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
        }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      },                               /* Header */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Position */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0,                       /* Z */
            0.0                        /* W */
          }                            /* Orientation */
        }                              /* Pose */
      },                               /* Pose */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Linear */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          }                            /* Angular */
        }                              /* Twist */
      }                                /* Twist */
    },

    /* Computed Parameter: Constant_Value_e
     * Referenced by: '<S31>/Constant'
     */
    {
      {
        0.0,                           /* X */
        0.0,                           /* Y */
        0.0                            /* Z */
      },                               /* Linear */

      {
        0.0,                           /* X */
        0.0,                           /* Y */
        0.0                            /* Z */
      }                                /* Angular */
    },

    /* Computed Parameter: Out1_Y0_m
     * Referenced by: '<S30>/Out1'
     */
    {
      0.0,                             /* X */
      0.0,                             /* Y */
      0.0                              /* Z */
    },

    /* Computed Parameter: Constant_Value_l
     * Referenced by: '<S26>/Constant'
     */
    {
      0.0,                             /* X */
      0.0,                             /* Y */
      0.0                              /* Z */
    },

    /* Expression: -1
     * Referenced by: '<S1>/Constant1'
     */
    -1.0,

    /* Expression: 1
     * Referenced by: '<S1>/Constant'
     */
    1.0,

    /* Expression: [ 0.05, 2.0 ]
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    { 0.05, 2.0 },

    /* Expression: [ 3, 6 ]
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    { 3.0, 6.0 },

    /* Expression: 0.3
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    0.3,

    /* Expression: 0.1
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    0.1,

    /* Expression: 0.3
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    0.3,

    /* Expression: 5
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    5.0,

    /* Expression: 2
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    2.0,

    /* Expression: 2
     * Referenced by: '<S1>/Vector Field Histogram'
     */
    2.0,

    /* Expression: -1
     * Referenced by: '<S7>/Gain'
     */
    -1.0,

    /* Expression: 0.2
     * Referenced by: '<S7>/OmegaRec'
     */
    0.2,

    /* Expression: 0
     * Referenced by: '<S6>/Constant'
     */
    0.0,

    /* Expression: 1
     * Referenced by: '<S6>/MaxAngularVelocity'
     */
    1.0,

    /* Expression: 0.3
     * Referenced by: '<S2>/Pure Pursuit'
     */
    0.3,

    /* Expression: 1.0
     * Referenced by: '<S2>/Pure Pursuit'
     */
    1.0,

    /* Expression: 1.0
     * Referenced by: '<S2>/Pure Pursuit'
     */
    1.0,

    /* Expression: 0
     * Referenced by: '<S17>/Integrator'
     */
    0.0,

    /* Expression: initCond
     * Referenced by: '<S16>/ '
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S13>/Constant'
     */
    0.0,

    /* Expression: 0.1
     * Referenced by: '<S2>/GoalRadius'
     */
    0.1,

    /* Computed Parameter: Constant_Value_pg
     * Referenced by: '<S8>/Constant'
     */
    {
      false                            /* Data */
    },

    /* Computed Parameter: Constant_Value_oo
     * Referenced by: '<S33>/Constant'
     */
    {
      false                            /* Data */
    },

    /* Computed Parameter: Q_Y0
     * Referenced by: '<S9>/Q'
     */
    0,

    /* Computed Parameter: Q_Y0_j
     * Referenced by: '<S9>/!Q'
     */
    1,

    /* Computed Parameter: Memory_InitialCondition
     * Referenced by: '<S3>/Memory'
     */
    0,

    /* Start of '<S33>/Enabled Subsystem' */
    {
      /* Computed Parameter: Out1_Y0
       * Referenced by: '<S34>/Out1'
       */
      {
        false                          /* Data */
      }
    }
    ,

    /* End of '<S33>/Enabled Subsystem' */

    /* Start of '<S8>/Enabled Subsystem' */
    {
      /* Computed Parameter: Out1_Y0
       * Referenced by: '<S11>/Out1'
       */
      {
        false                          /* Data */
      }
    }
    /* End of '<S8>/Enabled Subsystem' */
  };                                   /* Modifiable parameters */

  /* Initialize tunable parameters */
  Pure_Pursuit_With_VFH_P = Pure_Pursuit_With_VFH_P_temp;
}

/* Destructor */
Pure_Pursuit_With_VFHModelClass::~Pure_Pursuit_With_VFHModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_Pure_Pursuit_With_VF_T * Pure_Pursuit_With_VFHModelClass::getRTM()
{
  return (&Pure_Pursuit_With_VFH_M);
}

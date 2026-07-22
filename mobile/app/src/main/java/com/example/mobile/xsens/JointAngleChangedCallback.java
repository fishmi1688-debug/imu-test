package com.example.mobile.xsens;

public interface JointAngleChangedCallback {
  void onJointAngleChanged(String side, double[] ang_hip, double[] ang_knee, double[] ang_ankle);
}

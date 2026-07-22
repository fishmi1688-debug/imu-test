package com.example.mobile.xsens;

import com.xsens.dot.android.sdk.events.XsensDotData;

import org.apache.commons.numbers.quaternion.Quaternion;

import java.util.HashMap;
import java.util.Map;

public class JointAngleCalculator {
  public static final int Side_Left = 1;
  public static final int Side_Right = 2;
  public static int DELAY = 50;
  public static String SIDE_LEFT = "left";
  public static String SIDE_RIGHT = "right";
  private final IMU[] imus;
  private final Map<String, IMU> map;
  final String side;
  double[] ang_hip;
  double[] ang_knee;
  double[] ang_ankle;
  private long updatedAt = 0;
  final JointAngleChangedCallback callback;

  public JointAngleCalculator(String side, String address1, String address2, String address3, JointAngleChangedCallback callback) {
    imus = new IMU[3];
    map = new HashMap<>();
    if (address1 != null) {
      imus[0] = IMU.of(address1, IMU.Thigh);
      map.put(address1, imus[0]);
    }
    if (address2 != null) {
      imus[1] = IMU.of(address2, IMU.LowerLeg);
      map.put(address2, imus[1]);
    }
    if (address3 != null) {
      imus[2] = IMU.of(address3, IMU.Foot);
      map.put(address3, imus[2]);
    }
    this.side = side;
    this.callback = callback;
  }

  public IMU getIMU(String address) {
    return map.get(address);
  }

  public IMU getIMU(int part) {
    switch (part) {
      case IMU.Thigh:
        return imus[0];
      case IMU.LowerLeg:
        return imus[1];
      case IMU.Foot:
        return imus[2];
    }
    return null;
  }

  private boolean update(String address, XsensDotData result) {
    if (imus[0] != null && imus[0].address.equals(address)) {
      imus[0].data = result;
      return true;
    } else if (imus[1] != null && imus[1].address.equals(address)) {
      imus[1].data = result;
      return true;
    } else if (imus[2] != null && imus[2].address.equals(address)) {
      imus[2].data = result;
      return true;
    }
    return false;
  }

  private double[] getAngles(String address) {
    if (imus[0] != null && imus[0].address.equals(address)) {
      return calculateHip(imus[0].data);
    } else if (imus[1] != null && imus[1].address.equals(address)) {
      if (imus[0] != null && imus[0].data != null) {
        return calculateKnee(imus[0].data, imus[1].data);
      } else {
        return new double[]{0, 0, 0};
      }
    } else if (imus[2] != null && imus[2].address.equals(address)) {
      return calculateAnkle(imus[2].data);
    }
    return null;
  }

  private boolean update() {
    long now = System.currentTimeMillis();
    if (now < updatedAt + DELAY) {
      return false;
    }
    if (imus[0] != null && imus[0].data != null) {
      ang_hip = calculateHip(imus[0].data);
    }
    if (imus[0] != null && imus[0].data != null && imus[1] != null && imus[1].data != null) {
      ang_knee = calculateKnee(imus[0].data, imus[1].data);
    }
    if (imus[2] != null && imus[2].data != null) {
      ang_ankle = calculateAnkle(imus[2].data);
    }
    updatedAt = now;
    callback.onJointAngleChanged(side, ang_hip, ang_knee, ang_ankle);
    return true;
  }

  public static double[] calculateHip(XsensDotData xsensDotData) {
    if (xsensDotData == null) {
      return new double[]{0 ,0, 0};
    }
    return RotationTool.quaternion2Euler(xsensDotData.getQuat());
  }

  public static double[] calculateAnkle(XsensDotData xsensDotData) {
    if (xsensDotData == null) {
      return new double[]{0 ,0, 0};
    }
    return RotationTool.quaternion2Euler(xsensDotData.getQuat());
  }

  public static double[] calculateKnee(XsensDotData xsensDot1, XsensDotData xsensDot2) {
    if (xsensDot1 == null || xsensDot2 == null) {
      return new double[]{0 ,0, 0};
    }
    float[] quat1 = xsensDot1.getQuat();
    float[] quat2 = xsensDot2.getQuat();
    Quaternion q1 = Quaternion.of(quat1[0], quat1[1], quat1[2], quat1[3]);
    Quaternion q2 = Quaternion.of(quat2[0], quat2[1], quat2[2], quat2[3]);
    Quaternion q1_inverse = q1.inverse();
    Quaternion q_result = Quaternion.multiply(q1_inverse, q2);
    return RotationTool.quaternion2Euler(q_result);
  }
}


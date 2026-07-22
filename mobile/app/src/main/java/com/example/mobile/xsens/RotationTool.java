package com.example.mobile.xsens;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.numbers.quaternion.Quaternion;

public class RotationTool {
  public static double[] quaternion2Euler(Quaternion q) {
    return quaternion2Euler(q.getW(), q.getX(), q.getY(), q.getZ());
  }

  public static double[] quaternion2Euler(double[] q) {
    return quaternion2Euler(q[0], q[1], q[2], q[3]);
  }

  public static double[] quaternion2Euler(float[] q) {
    return quaternion2Euler(q[0], q[1], q[2], q[3]);
  }

  public static double[] quaternion2Euler(double qw, double qx, double qy, double qz) {
    RotationOrder sequence = RotationOrder.ZXZ;
    Rotation rotation = new Rotation(qw, qx, qy, qz, true);
    double[] angles = rotation.getAngles(sequence, RotationConvention.FRAME_TRANSFORM);
    double[] degrees = new double[angles.length];
    for (int i = 0; i < angles.length; i++) {
      degrees[i] = FastMath.toDegrees(angles[i]);
    }
    return degrees;
  }
}

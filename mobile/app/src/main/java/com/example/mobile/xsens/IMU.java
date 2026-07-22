package com.example.mobile.xsens;

import com.xsens.dot.android.sdk.events.XsensDotData;

public class IMU {
  public static final int Thigh = 0;
  public static final int LowerLeg = 1;
  public static final int Foot = 2;

  public XsensDotData data;
  String address;
  public int part = -1;
  long updatedAt;

  public IMU(String address, int part, XsensDotData data) {
    this.address = address;
    this.data = data;
    this.part = part;
    this.updatedAt = 0;
  }
  public static IMU of(String address, int _part) {
    return new IMU(address, _part, null);
  }

  public boolean update(XsensDotData data) {
    this.data = data;
    return true;
  }
}

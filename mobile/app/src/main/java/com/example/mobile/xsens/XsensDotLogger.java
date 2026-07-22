package com.example.mobile.xsens;
import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;
import android.util.SparseArray;
import androidx.annotation.RestrictTo;
import androidx.annotation.RestrictTo.Scope;
import com.xsens.dot.android.sdk.events.XsensDotData;
import com.xsens.dot.android.sdk.models.XsensDotPayload;
import com.xsens.dot.android.sdk.utils.XsensDotParser;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class XsensDotLogger {
  public static boolean DEBUG = false;
  public static final int FLAG_DEFAULT = 0;
  public static final int FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER = 6;
  public static final int FLAG_ACC_GYR_ONLY = 7;
  private static int flag = FLAG_DEFAULT;

  private static final String SET_COMMA_SEPARATOR = "sep=,\n";
  private static final String TITLE_TAG_NAME = "DeviceTag:,";
  private static final String TITLE_FIRMWARE_VERSION = "FirmwareVersion:,";
  private static final String TITLE_APP_VERSION = "AppVersion:,";
  private static final String TITLE_SYNC_STATUS = "SyncStatus:,";
  private static final String TITLE_OUTPUT_RATE = "OutputRate:,";
  private static final String TITLE_FILTER_PROFILE = "FilterProfile:,";
  private static final String TITLE_MEASUREMENT_MODE = "Measurement Mode:,";
  private static final String TITLE_START_TIME = "StartTime:,";
  private static final String EMPTY_COLUMN_DATA = ",,,,,,,,,,,,,,,,,\n";
  private static final String TITLE_COLUMNS_DEFAULT = "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Cal_FreeAcc_X,Cal_FreeAcc_Y,Cal_FreeAcc_Z,FreeAcc_Compared_Result,Status\n";
  private static final String TITLE_COLUMNS_COMPLETE_QUATERNION = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n";
  private static final String TITLE_COLUMNS_COMPLETE_EULER = "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n";
  private static final String TITLE_COLUMNS_EXTENDED_QUATERNION = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Status\n";
  private static final String TITLE_COLUMNS_ORIENTATION_EULER = "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z\n";
  private static final String TITLE_COLUMNS_ORIENTATION_QUATERNION = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z\n";
  private static final String TITLE_COLUMNS_FREE_ACCELERATION = "PacketCounter,SampleTimeFine,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n";
  private static final String TITLE_COLUMNS_EXTENDED_EULER = "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Status\n";
  private static final String TITLE_COLUMNS_HIGH_FIDELITY_WITH_MAG = "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Status\n";
  private static final String TITLE_COLUMNS_HIGH_FIDELITY_NO_MAG = "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Status\n";
  private static final String TITLE_COLUMNS_DELTA_QUANTITIES_WITH_MAG = "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z\n";
  private static final String TITLE_COLUMNS_DELTA_QUANTITIES_NO_MAG = "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3]\n";
  private static final String TITLE_COLUMNS_RATE_QUANTITIES_WITH_MAG = "PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z,Mag_X,Mag_Y,Mag_Z\n";
  private static final String TITLE_COLUMNS_RATE_QUANTITIES_NO_MAG = "PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_1 = "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Gyr_X,Gyr_Y,Gyr_Z\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_2 = "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Mag_X,Mag_Y,Mag_Z\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_3 = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,Gyr_X,Gyr_Y,Gyr_Z\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_4 = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Status\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_5 = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_27_DEFAULT = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z,Mag_X,Mag_Y,Mag_Z,Flag\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_27_NO_EULER = "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z,Mag_X,Mag_Y,Mag_Z,Flag\n";
  private static final String TITLE_COLUMNS_CUSTOM_MODE_27_ACC_GYR_ONLY = "PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z,Flag\n";
  private static final SparseArray<String> TITLE_COLUMNS_MAP = new SparseArray<String>() {
    {
      this.put(0, "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Cal_FreeAcc_X,Cal_FreeAcc_Y,Cal_FreeAcc_Z,FreeAcc_Compared_Result,Status\n");
      this.put(3, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n");
      this.put(16, "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n");
      this.put(2, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Status\n");
      this.put(4, "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z\n");
      this.put(5, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z\n");
      this.put(6, "PacketCounter,SampleTimeFine,FreeAcc_X,FreeAcc_Y,FreeAcc_Z\n");
      this.put(7, "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Status\n");
      this.put(1, "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Status\n");
      this.put(17, "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Status\n");
      this.put(18, "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z\n");
      this.put(19, "PacketCounter,SampleTimeFine,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3]\n");
      this.put(20, "PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z,Mag_X,Mag_Y,Mag_Z\n");
      this.put(21, "PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z\n");
      this.put(22, "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Gyr_X,Gyr_Y,Gyr_Z\n");
      this.put(23, "PacketCounter,SampleTimeFine,Euler_X,Euler_Y,Euler_Z,FreeAcc_X,FreeAcc_Y,FreeAcc_Z,Mag_X,Mag_Y,Mag_Z\n");
      this.put(24, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,Gyr_X,Gyr_Y,Gyr_Z\n");
      this.put(25, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,dq_W,dq_X,dq_Y,dq_Z,dv[1],dv[2],dv[3],Mag_X,Mag_Y,Mag_Z,Status\n");
      this.put(26, "PacketCounter,SampleTimeFine,Quat_W,Quat_X,Quat_Y,Quat_Z,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z\n");
      this.put(27, TITLE_COLUMNS_CUSTOM_MODE_27_DEFAULT);
    }
  };
  private static final SparseArray<String> RECORDING_DATA_TITLE_MAPPER = new SparseArray<String>() {
    {
      this.put(0, "SampleTimeFine,");
      this.put(1, "Quat_W,Quat_X,Quat_Y,Quat_Z,");
      this.put(2, "iq_X,iq_Y,iq_Z,");
      this.put(3, "iv_X,iv_Y,iv_Z,");
      this.put(4, "Euler_X,Euler_Y,Euler_Z,");
      this.put(5, "dq_W,dq_X,dq_Y,dq_Z,");
      this.put(6, "dv[1],dv[2],dv[3],");
      this.put(7, "Acc_X,Acc_Y,Acc_Z,");
      this.put(8, "Gyr_X,Gyr_Y,Gyr_Z,");
      this.put(9, "Mag_X,Mag_Y,Mag_Z,");
      this.put(10, "Status,");
      this.put(11, "ClipCountAcc,");
      this.put(12, "ClipCountGyr,");
    }
  };
  private Context mContext;
  private int mMode;
  public static final int TYPE_CSV = 1;
  @RestrictTo({Scope.LIBRARY})
  private static final int TYPE_MTB = 2;
  private static final String MFM_DIR = "mfm";
  private String mFilename;
  private String mTagName;
  private String mFirmwareVersion;
  private String mAppVersion;
  private long mStartTime;
  private OutputStreamWriter mOut = null;
  private FileOutputStream mFos = null;
  private boolean isWriting = false;
  private byte[] mExportIds = null;
  private boolean mIsContainHighFidelityIds = false;
  private boolean mIncludeLabelColumn = false;

  public static void setFlag(int _flag) {
    flag = _flag;
  }
  @RestrictTo({Scope.LIBRARY})
  public static XsensDotLogger createMtbLogger(Context context, String address, String tag) {
    File appDir = context.getApplicationContext().getExternalFilesDir((String)null);
    if (appDir != null) {
      File mfmDir = new File(appDir + File.separator + "mfm");
      if (!mfmDir.exists()) {
        mfmDir.mkdirs();
      }

      String currentDate = (new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.getDefault())).format(new Date());
      String path = mfmDir + File.separator + currentDate + File.separator + (tag != null && !tag.equals("") ? tag.concat("_") : address.replace(":", "").concat("_")) + currentDate.concat(".mtb");
      return new XsensDotLogger(context, 2, 15, path, (String)null, (String)null, false, 1, (String)null, (String)null, 0L);
    } else {
      return new XsensDotLogger(context, 2, 15, "", (String)null, (String)null, false, 1, (String)null, (String)null, 0L);
    }
  }

  public static XsensDotLogger createRecordingsLogger(Context context, byte[] ids, String filename, String tagName, String firmwareVersion, String appVersion, long startTimeMillis) {
    XsensDotLogger logger = new XsensDotLogger(context, 1, 666, filename, tagName, firmwareVersion, false, 1, (String)null, appVersion, startTimeMillis);
    logger.setExportIds(ids);
    return logger;
  }

  public XsensDotLogger(Context context, int type, int mode, String filename, String tagName, String firmwareVersion, boolean isSynced, int outputRate, String filterProfileName, String appVersion, long startTimeMillis) {
    this(context, type, mode, filename, tagName, firmwareVersion, isSynced, outputRate, filterProfileName, appVersion, startTimeMillis, false);
  }

  public XsensDotLogger(Context context, int type, int mode, String filename, String tagName, String firmwareVersion, boolean isSynced, int outputRate, String filterProfileName, String appVersion, long startTimeMillis, boolean includeLabelColumn) {
    if (XsensDotLogger.DEBUG) {
      Log.i("XsensDotSdk", "XsensDotLogger() - fileName = " + filename);
    }

    if (filename != null && filename.length() != 0) {
      if (mode != 0 && mode != 3 && mode != 16 && mode != 2 && mode != 4 && mode != 5 && mode != 6 && mode != 7 && mode != 15 && mode != 1 && mode != 17 && mode != 18 && mode != 19 && mode != 20 && mode != 21 && mode != 666 && mode != 22 && mode != 23 && mode != 24 && mode != 25 && mode != 26 && mode != 27) {
        Log.e("XsensDotSdk", "Invalid mode value " + mode);
      } else if (outputRate != 1 && outputRate != 4 && outputRate != 10 && outputRate != 12 && outputRate != 15 && outputRate != 20 && outputRate != 30 && outputRate != 60 && outputRate != 120) {
        Log.e("XsensDotSdk", "Invalid output rate " + outputRate);
      } else {
        this.mContext = context;
        this.mMode = mode;
        this.mFilename = filename;
        this.mTagName = tagName;
        this.mFirmwareVersion = firmwareVersion;
        this.mAppVersion = appVersion;
        this.mStartTime = startTimeMillis == 0L ? System.currentTimeMillis() : startTimeMillis;
        this.mIncludeLabelColumn = includeLabelColumn;
        DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS Z", Locale.getDefault());
        DateFormat dateYearFormat = new SimpleDateFormat("yyyy", Locale.getDefault());
        if (type == 1) {
          try {
            this.mOut = new OutputStreamWriter(new FileOutputStream(filename));
            this.update("sep=,\n");
            this.update("DeviceTag:," + this.mTagName + ",,,,,,,,,,,,,,,,,\n");
            this.update("FirmwareVersion:," + this.mFirmwareVersion + ",,,,,,,,,,,,,,,,,\n");
            this.update("AppVersion:," + this.mAppVersion + ",,,,,,,,,,,,,,,,,\n");
            if (filterProfileName != null && !filterProfileName.isEmpty()) {
              this.update("SyncStatus:," + (isSynced ? "Synced" : "Un-synced") + ",,,,,,,,,,,,,,,,,\n");
              this.update("OutputRate:," + outputRate + "Hz" + ",,,,,,,,,,,,,,,,,\n");
              this.update("FilterProfile:," + filterProfileName + ",,,,,,,,,,,,,,,,,\n");
            }

            if (mode != 666) {
              this.update("Measurement Mode:," + XsensDotPayload.getPayloadTitle(this.mMode == 27 ? 26 : this.mMode) + ",,,,,,,,,,,,,,,,,\n");
            }

            this.update("StartTime:," + dateFormat.format(this.mStartTime) + ",,,,,,,,,,,,,,,,,\n");
            this.update("Movella Technologies B. V. 2005-" + dateYearFormat.format(this.mStartTime) + ",," + ",,,,,,,,,,,,,,,,,\n");
            if (mode != 666) {
              this.update("\n" + this.withLabelTitle(this.resolveTitleColumnsForMode()));
            }
          } catch (IOException e) {
            e.printStackTrace();
          }
        } else if (type == 2) {
          AssetManager assetManager = context.getAssets();
          InputStream mtbHeaderStream = null;

          try {
            mtbHeaderStream = assetManager.open("file_header.mtb");
            byte[] header = new byte[mtbHeaderStream.available()];
            mtbHeaderStream.read(header);
            File file = new File(filename);
            File dir = file.getParentFile();
            if (dir != null && !dir.exists()) {
              dir.mkdir();
            }

            this.mFos = new FileOutputStream(filename);
            this.update(header);
          } catch (IOException e) {
            e.printStackTrace();
          } finally {
            if (mtbHeaderStream != null) {
              try {
                mtbHeaderStream.close();
              } catch (IOException e) {
                e.printStackTrace();
              }
            }

          }
        }

      }
    } else {
      Log.e("XsensDotSdk", "Filename is empty");
    }
  }

  @RestrictTo({Scope.LIBRARY})
  public int getMode() {
    return this.mMode;
  }

  public String getFilename() {
    return this.mFilename;
  }

  @RestrictTo({Scope.LIBRARY})
  public String getTagName() {
    return this.mTagName;
  }

  @RestrictTo({Scope.LIBRARY})
  public String getFirmwareVersion() {
    return this.mFirmwareVersion;
  }

  @RestrictTo({Scope.LIBRARY})
  public String getAppVersion() {
    return this.mAppVersion;
  }

  public long getStartTime() {
    return this.mStartTime;
  }

  public void update(String str) {
    if (str != null && str.length() != 0) {
      synchronized(this) {
        try {
          this.isWriting = true;
          if (this.mOut != null) {
            this.mOut.write(str);
          }
        } catch (IOException ioe) {
          ioe.printStackTrace();
        } finally {
          this.isWriting = false;
        }

      }
    } else {
      if (XsensDotLogger.DEBUG) {
        Log.i("XsensDotSdk", "update() - Str is empty");
      }

    }
  }

  @RestrictTo({Scope.LIBRARY})
  public void update(byte[] bytes) {
    if (bytes == null) {
      if (XsensDotLogger.DEBUG) {
        Log.i("XsensDotSdk", "update() - bytes is null");
      }

    } else {
      synchronized(this) {
        try {
          this.isWriting = true;
          if (this.mFos != null) {
            this.mFos.write(bytes);
          }
        } catch (IOException ioe) {
          ioe.printStackTrace();
        } finally {
          this.isWriting = false;
        }

      }
    }
  }

  public void update(XsensDotData xsData) {
    this.update(xsData, (String)null);
  }

  public void update(XsensDotData xsData, String label) {
    if (xsData == null) {
      if (XsensDotLogger.DEBUG) {
        Log.i("XsensDotSdk", "update() - XsensDotData is null");
      }

    } else {
      int packetCounter = xsData.getPacketCounter();
      long sampleTimeFine = xsData.getSampleTimeFine();
      double[] acc = xsData.getAcc();
      double[] gyr = xsData.getGyr();
      double[] mag = xsData.getMag();
      float[] quat = xsData.getQuat();
      double[] euler = XsensDotParser.quaternion2Euler(quat);
      float[] freeAcc = readCalFreeAcc(xsData, xsData.getFreeAcc());
      int[] iQ = xsData.getIq();
      int[] iV = xsData.getIv();
      int status = xsData.getStatus();
      int clipCountAcc = xsData.getClipCountAcc();
      int clipCountGyr = xsData.getClipCountGyr();
      double[] dV = xsData.getDv();
      double[] dQ = xsData.getDq();
      if (acc != null && gyr != null && mag != null && quat != null && euler != null && freeAcc != null && iQ != null && iV != null) {
        if (acc.length == 3 && gyr.length == 3 && mag.length == 3 && euler.length == 3 && freeAcc.length == 3 && iQ.length == 3 && iV.length == 3 && dV.length == 3) {
          if (quat.length == 4 && dQ.length == 4) {
            String output = "";
            switch (this.mMode) {
              case 0:
                float[] calFreeAcc = readCalFreeAcc(xsData, freeAcc);
                output = packetCounter + ", " + sampleTimeFine + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + calFreeAcc[0] + ", " + calFreeAcc[1] + ", " + calFreeAcc[2] + ", " + getCalFreeAccComparedResult(freeAcc, calFreeAcc) + ", " + status + "\n";
                break;
              case 1:
                output = packetCounter + ", " + sampleTimeFine + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + ", " + status + "\n";
                break;
              case 2:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + status + "\n";
                break;
              case 3:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + "\n";
                break;
              case 4:
                output = packetCounter + ", " + sampleTimeFine + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + "\n";
                break;
              case 5:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + "\n";
                break;
              case 6:
                output = packetCounter + ", " + sampleTimeFine + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + "\n";
                break;
              case 7:
                output = packetCounter + ", " + sampleTimeFine + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + status + "\n";
                break;
              case 16:
                output = packetCounter + ", " + sampleTimeFine + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + "\n";
                break;
              case 17:
                output = packetCounter + ", " + sampleTimeFine + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + ", " + status + "\n";
                break;
              case 18:
                output = packetCounter + ", " + sampleTimeFine + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + "\n";
                break;
              case 19:
                output = packetCounter + ", " + sampleTimeFine + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + "\n";
                break;
              case 20:
                output = packetCounter + ", " + sampleTimeFine + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + "\n";
                break;
              case 21:
                output = packetCounter + ", " + sampleTimeFine + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + "\n";
                break;
              case 22:
                output = packetCounter + ", " + sampleTimeFine + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + "\n";
                break;
              case 23:
                output = packetCounter + ", " + sampleTimeFine + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + "\n";
                break;
              case 24:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + "\n";
                break;
              case 25:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + dQ[0] + ", " + dQ[1] + ", " + dQ[2] + ", " + dQ[3] + ", " + dV[0] + ", " + dV[1] + ", " + dV[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + ", " + status + "\n";
                break;
              case 26:
                output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + "\n";
                break;
              case 27:
                if (flag == FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER) {
                  output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + ", " + flag + "\n";
                } else if (flag == FLAG_ACC_GYR_ONLY) {
                  output = packetCounter + ", " + sampleTimeFine + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + ", " + flag + "\n";
                } else {
                  output = packetCounter + ", " + sampleTimeFine + ", " + quat[0] + ", " + quat[1] + ", " + quat[2] + ", " + quat[3] + ", " + euler[0] + ", " + euler[1] + ", " + euler[2] + ", " + freeAcc[0] + ", " + freeAcc[1] + ", " + freeAcc[2] + ", " + acc[0] + ", " + acc[1] + ", " + acc[2] + ", " + gyr[0] + ", " + gyr[1] + ", " + gyr[2] + ", " + mag[0] + ", " + mag[1] + ", " + mag[2] + ", " + flag + "\n";
                }
                break;
              case 666:
                if (this.mExportIds == null || this.mExportIds.length == 0) {
                  return;
                }

                int i = 0;
                StringBuilder sb = new StringBuilder();
                sb.append(packetCounter).append(", ");

                for(; i < this.mExportIds.length; ++i) {
                  byte dataId = this.mExportIds[i];
                  switch (dataId) {
                    case 0:
                      sb.append(sampleTimeFine).append(", ");
                      break;
                    case 1:
                      sb.append(quat[0]).append(", ").append(quat[1]).append(", ").append(quat[2]).append(", ").append(quat[3]).append(", ");
                      break;
                    case 2:
                      sb.append(iQ[0]).append(", ").append(iQ[1]).append(", ").append(iQ[2]).append(", ");
                      break;
                    case 3:
                      sb.append(iV[0]).append(", ").append(iV[1]).append(", ").append(iV[2]).append(", ");
                      break;
                    case 4:
                      sb.append(euler[0]).append(", ").append(euler[1]).append(", ").append(euler[2]).append(", ");
                      break;
                    case 5:
                      sb.append(dQ[0]).append(", ").append(dQ[1]).append(", ").append(dQ[2]).append(", ").append(dQ[3]).append(", ");
                      break;
                    case 6:
                      sb.append(dV[0]).append(", ").append(dV[1]).append(", ").append(dV[2]).append(", ");
                      break;
                    case 7:
                      sb.append(acc[0]).append(", ").append(acc[1]).append(", ").append(acc[2]).append(", ");
                      break;
                    case 8:
                      sb.append(gyr[0]).append(", ").append(gyr[1]).append(", ").append(gyr[2]).append(", ");
                      break;
                    case 9:
                      sb.append(mag[0]).append(", ").append(mag[1]).append(", ").append(mag[2]).append(", ");
                      break;
                    case 10:
                      sb.append(status).append(", ");
                      break;
                    case 11:
                      sb.append(clipCountAcc).append(", ");
                      break;
                    case 12:
                      sb.append(clipCountGyr).append(", ");
                  }
                }

                sb.append("\n");
                output = sb.toString();
            }

            output = this.prependLabelIfNeeded(output, label);
            synchronized(this) {
              try {
                this.isWriting = true;
                if (this.mOut != null) {
                  this.mOut.write(output);
                }
              } catch (IOException ioe) {
                ioe.printStackTrace();
              } finally {
                this.isWriting = false;
              }

            }
          } else {
            if (XsensDotLogger.DEBUG) {
              Log.i("XsensDotSdk", "update() - The length of array should be 4");
            }

          }
        } else {
          if (XsensDotLogger.DEBUG) {
            Log.i("XsensDotSdk", "update() - The length of array should be 3");
          }

        }
      } else {
        if (XsensDotLogger.DEBUG) {
          Log.i("XsensDotSdk", "update() - Array is null");
        }

      }
    }
  }

  public void stop() {
    synchronized(this) {
      if (this.isWriting) {
        this.stop();
      }

      if (null != this.mOut) {
        try {
          this.mOut.flush();
          this.mOut.close();
        } catch (IOException ioe) {
          ioe.printStackTrace();
        }
      }

      if (null != this.mFos) {
        try {
          this.mFos.flush();
          this.mFos.close();
        } catch (IOException ioe) {
          ioe.printStackTrace();
        }
      }

    }
  }

  private void setExportIds(byte[] ids) {
    this.mExportIds = ids;
    if (this.mExportIds != null) {
      StringBuilder titleRow = new StringBuilder();
      int i = 0;
      titleRow.append("PacketCounter,");

      while(i < this.mExportIds.length) {
        byte dataId = this.mExportIds[i];
        titleRow.append((String)RECORDING_DATA_TITLE_MAPPER.get(dataId));
        ++i;
      }

      this.mIsContainHighFidelityIds = false;
      this.update("\n" + this.withLabelTitle(titleRow.toString() + "\n"));
    }

  }

  private String resolveTitleColumnsForMode() {
    if (this.mMode == 27 && flag == FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER) {
      return TITLE_COLUMNS_CUSTOM_MODE_27_NO_EULER;
    }
    if (this.mMode == 27 && flag == FLAG_ACC_GYR_ONLY) {
      return TITLE_COLUMNS_CUSTOM_MODE_27_ACC_GYR_ONLY;
    }
    return (String)TITLE_COLUMNS_MAP.get(this.mMode);
  }

  private String withLabelTitle(String titleRow) {
    if (titleRow == null || titleRow.length() == 0) {
      return titleRow;
    }
    if (!this.mIncludeLabelColumn) {
      return titleRow;
    }
    return "Label," + titleRow;
  }

  private String prependLabelIfNeeded(String row, String label) {
    if (row == null || row.length() == 0 || !this.mIncludeLabelColumn) {
      return row;
    }
    return escapeCsvField(label) + "," + row;
  }

  private static String escapeCsvField(String value) {
    if (value == null || value.isEmpty()) {
      return "";
    }
    boolean needsQuote = value.contains(",") || value.contains("\"") || value.contains("\n") || value.contains("\r");
    if (!needsQuote) {
      return value;
    }
    return "\"" + value.replace("\"", "\"\"") + "\"";
  }

  private static float[] readCalFreeAcc(XsensDotData xsData, float[] fallback) {
    try {
      Object result = xsData.getClass().getMethod("getCalFreeAcc").invoke(xsData);
      if (result instanceof float[]) {
        float[] values = (float[])result;
        if (values.length == 3) {
          return values;
        }
      }
    } catch (Exception ignored) {
    }

    if (fallback != null && fallback.length == 3) {
      return fallback;
    }
    return new float[]{0.0F, 0.0F, 0.0F};
  }

  private static boolean getCalFreeAccComparedResult(float[] freeAcc, float[] freeAccExpected) {
    float xDiff = Math.abs(freeAcc[0] - freeAccExpected[0]);
    float yDiff = Math.abs(freeAcc[1] - freeAccExpected[1]);
    float zDiff = Math.abs(freeAcc[2] - freeAccExpected[2]);
    if ((double)xDiff < 1.0E-6 && (double)yDiff < 1.0E-6 && (double)zDiff < 1.0E-6) {
      if (XsensDotLogger.DEBUG) {
        Log.d("XsensDotSdk", "xDiff " + xDiff + ", yDiff " + yDiff + ", zDiff" + zDiff + ", true");
      }

      return true;
    } else {
      return false;
    }
  }
}

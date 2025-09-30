package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IMUImpl implements Gyro {
  // The gyro sensor
  private AHRS m_gyro = null;

  private final ShuffleboardTab imuTab = Shuffleboard.getTab("IMU");

  // Column/group layouts
  private final ShuffleboardLayout orientationCol =
      imuTab.getLayout("Orientation", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 6);
  private final ShuffleboardLayout headingsCol =
      imuTab.getLayout("Headings", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 6);
  private final ShuffleboardLayout quaternionCol =
      imuTab.getLayout("Quaternion", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 6);
  private final ShuffleboardLayout motionCol =
      imuTab.getLayout("Motion", BuiltInLayouts.kList).withPosition(3, 0).withSize(1, 6);
  private final ShuffleboardLayout rawGyroCol =
      imuTab.getLayout("Raw Gyro", BuiltInLayouts.kList).withPosition(4, 0).withSize(1, 5);
  private final ShuffleboardLayout rawAccelCol =
      imuTab.getLayout("Raw Accel", BuiltInLayouts.kList).withPosition(5, 0).withSize(1, 5);
  private final ShuffleboardLayout rawMagCol =
      imuTab.getLayout("Raw Mag", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 5);
  private final ShuffleboardLayout statusCol =
      imuTab.getLayout("Status", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 6);

  // Entries grouped under layouts
  private final GenericEntry imuYawEntry = orientationCol.add("IMU Yaw", 0.0).getEntry();
  private final GenericEntry imuPitchEntry = orientationCol.add("IMU Pitch", 0.0).getEntry();
  private final GenericEntry imuRollEntry = orientationCol.add("IMU Roll", 0.0).getEntry();

  private final GenericEntry imuCompassHeadingEntry = headingsCol.add("Compass Heading", 0.0).getEntry();
  private final GenericEntry imuFusedHeadingEntry = headingsCol.add("Fused Heading", 0.0).getEntry();
  private final GenericEntry imuTotalYawEntry = headingsCol.add("Total Yaw", 0.0).getEntry();
  private final GenericEntry imuYawRateEntry = headingsCol.add("Yaw Rate DPS", 0.0).getEntry();

  private final GenericEntry quaternionWEntry = quaternionCol.add("Quat W", 0.0).getEntry();
  private final GenericEntry quaternionXEntry = quaternionCol.add("Quat X", 0.0).getEntry();
  private final GenericEntry quaternionYEntry = quaternionCol.add("Quat Y", 0.0).getEntry();
  private final GenericEntry quaternionZEntry = quaternionCol.add("Quat Z", 0.0).getEntry();

  private final GenericEntry velocityXEntry = motionCol.add("Velocity X", 0.0).getEntry();
  private final GenericEntry velocityYEntry = motionCol.add("Velocity Y", 0.0).getEntry();
  private final GenericEntry imuAccelXEntry = motionCol.add("Linear Accel X", 0.0).getEntry();
  private final GenericEntry imuAccelYEntry = motionCol.add("Linear Accel Y", 0.0).getEntry();
  private final GenericEntry displacementXEntry = motionCol.add("Displacement X", 0.0).getEntry();
  private final GenericEntry displacementYEntry = motionCol.add("Displacement Y", 0.0).getEntry();

  private final GenericEntry rawGyroXEntry = rawGyroCol.add("X", 0.0).getEntry();
  private final GenericEntry rawGyroYEntry = rawGyroCol.add("Y", 0.0).getEntry();
  private final GenericEntry rawGyroZEntry = rawGyroCol.add("Z", 0.0).getEntry();

  private final GenericEntry rawAccelXEntry = rawAccelCol.add("X", 0.0).getEntry();
  private final GenericEntry rawAccelYEntry = rawAccelCol.add("Y", 0.0).getEntry();
  private final GenericEntry rawAccelZEntry = rawAccelCol.add("Z", 0.0).getEntry();

  private final GenericEntry rawMagXEntry = rawMagCol.add("X", 0.0).getEntry();
  private final GenericEntry rawMagYEntry = rawMagCol.add("Y", 0.0).getEntry();
  private final GenericEntry rawMagZEntry = rawMagCol.add("Z", 0.0).getEntry();

  private final GenericEntry imuConnectedEntry = statusCol.add("Connected", false).getEntry();
  private final GenericEntry imuIsCalibratingEntry = statusCol.add("Is Calibrating", false).getEntry();
  private final GenericEntry imuIsMovingEntry = statusCol.add("Is Moving", false).getEntry();
  private final GenericEntry imuIsRotatingEntry = statusCol.add("Is Rotating", false).getEntry();
  private final GenericEntry imuTempEntry = statusCol.add("Temp C", 0.0).getEntry();
  private final GenericEntry imuTimestampEntry = statusCol.add("Timestamp", 0.0).getEntry();
  private final GenericEntry yawAxisDirectionEntry = statusCol.add("Yaw Axis Dir", "N/A").getEntry();
  private final GenericEntry yawAxisEntry = statusCol.add("Yaw Axis", 0.0).getEntry();
  private final GenericEntry firmwareVersionEntry = statusCol.add("Firmware", "N/A").getEntry();
  private final GenericEntry imuByteCountEntry = statusCol.add("Byte Count", 0.0).getEntry();
  private final GenericEntry imuUpdateCountEntry = statusCol.add("Update Count", 0.0).getEntry();
  private final GenericEntry imuPresentEntry = statusCol.add("Present", false).getEntry();

  public IMUImpl() {
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  }

  public void DisplayIMUData() {
    if (m_gyro != null) {
      imuPresentEntry.setBoolean(true);
      imuConnectedEntry.setBoolean(m_gyro.isConnected());
      imuIsCalibratingEntry.setBoolean(m_gyro.isCalibrating());
      imuYawEntry.setDouble(m_gyro.getYaw());
      imuPitchEntry.setDouble(m_gyro.getPitch());
      imuRollEntry.setDouble(m_gyro.getRoll());
      imuCompassHeadingEntry.setDouble(m_gyro.getCompassHeading());
      imuFusedHeadingEntry.setDouble(m_gyro.getFusedHeading());
      imuTotalYawEntry.setDouble(m_gyro.getAngle());
      imuYawRateEntry.setDouble(m_gyro.getRate());
      imuAccelXEntry.setDouble(m_gyro.getWorldLinearAccelX());
      imuAccelYEntry.setDouble(m_gyro.getWorldLinearAccelY());
      imuIsMovingEntry.setBoolean(m_gyro.isMoving());
      imuIsRotatingEntry.setBoolean(m_gyro.isRotating());
      velocityXEntry.setDouble(m_gyro.getVelocityX());
      velocityYEntry.setDouble(m_gyro.getVelocityY());
      displacementXEntry.setDouble(m_gyro.getDisplacementX());
      displacementYEntry.setDouble(m_gyro.getDisplacementY());
      rawGyroXEntry.setDouble(m_gyro.getRawGyroX());
      rawGyroYEntry.setDouble(m_gyro.getRawGyroY());
      rawGyroZEntry.setDouble(m_gyro.getRawGyroZ());
      rawAccelXEntry.setDouble(m_gyro.getRawAccelX());
      rawAccelYEntry.setDouble(m_gyro.getRawAccelY());
      rawAccelZEntry.setDouble(m_gyro.getRawAccelZ());
      rawMagXEntry.setDouble(m_gyro.getRawMagX());
      rawMagYEntry.setDouble(m_gyro.getRawMagY());
      rawMagZEntry.setDouble(m_gyro.getRawMagZ());
      imuTempEntry.setDouble(m_gyro.getTempC());
      imuTimestampEntry.setDouble(m_gyro.getLastSensorTimestamp());

      AHRS.BoardYawAxis yawAxis = m_gyro.getBoardYawAxis();
      yawAxisDirectionEntry.setString(yawAxis.up ? "Up" : "Down");
      yawAxisEntry.setDouble(yawAxis.board_axis.getValue());
      firmwareVersionEntry.setString(m_gyro.getFirmwareVersion());

      quaternionWEntry.setDouble(m_gyro.getQuaternionW());
      quaternionXEntry.setDouble(m_gyro.getQuaternionX());
      quaternionYEntry.setDouble(m_gyro.getQuaternionY());
      quaternionZEntry.setDouble(m_gyro.getQuaternionZ());

      imuByteCountEntry.setDouble(m_gyro.getByteCount());
      imuUpdateCountEntry.setDouble(m_gyro.getUpdateCount());
    } else {
      publishUnavailableState();
    }
  }

  private void publishUnavailableState() {
    imuPresentEntry.setBoolean(false);
    imuConnectedEntry.setBoolean(false);
    imuIsCalibratingEntry.setBoolean(false);
    imuIsMovingEntry.setBoolean(false);
    imuIsRotatingEntry.setBoolean(false);
    imuYawEntry.setDouble(0.0);
    imuPitchEntry.setDouble(0.0);
    imuRollEntry.setDouble(0.0);
    imuCompassHeadingEntry.setDouble(0.0);
    imuFusedHeadingEntry.setDouble(0.0);
    imuTotalYawEntry.setDouble(0.0);
    imuYawRateEntry.setDouble(0.0);
    imuAccelXEntry.setDouble(0.0);
    imuAccelYEntry.setDouble(0.0);
    velocityXEntry.setDouble(0.0);
    velocityYEntry.setDouble(0.0);
    displacementXEntry.setDouble(0.0);
    displacementYEntry.setDouble(0.0);
    rawGyroXEntry.setDouble(0.0);
    rawGyroYEntry.setDouble(0.0);
    rawGyroZEntry.setDouble(0.0);
    rawAccelXEntry.setDouble(0.0);
    rawAccelYEntry.setDouble(0.0);
    rawAccelZEntry.setDouble(0.0);
    rawMagXEntry.setDouble(0.0);
    rawMagYEntry.setDouble(0.0);
    rawMagZEntry.setDouble(0.0);
    imuTempEntry.setDouble(0.0);
    imuTimestampEntry.setDouble(0.0);
    yawAxisDirectionEntry.setString("Unavailable");
    yawAxisEntry.setDouble(0.0);
    firmwareVersionEntry.setString("Unavailable");
    quaternionWEntry.setDouble(0.0);
    quaternionXEntry.setDouble(0.0);
    quaternionYEntry.setDouble(0.0);
    quaternionZEntry.setDouble(0.0);
    imuByteCountEntry.setDouble(0.0);
    imuUpdateCountEntry.setDouble(0.0);
  }

  public Rotation2d getRotation2d() {
    if (m_gyro != null) {
      return m_gyro.getRotation2d();
    }
    return new Rotation2d();
  }

  public void reset() {
    if (m_gyro != null) {
      m_gyro.reset();
    }
  }

  public double getRate() {
    double rate = 0;
    if (m_gyro != null) {
      rate = m_gyro.getRate();
    }
    return rate;
  }
}



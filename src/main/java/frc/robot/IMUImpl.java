package frc.robot;

// import frc.robot.Gyro;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMUImpl implements Gyro {
  // The gyro sensor
  private AHRS m_gyro = null;

  public IMUImpl(){
      m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  }

  public void DisplayIMUData() {
    if (m_gyro != null){
      /* Display 6-axis Processed Angle Data */
      SmartDashboard.putBoolean("IMU_Connected", m_gyro.isConnected());
      SmartDashboard.putBoolean("IMU_IsCalibrating", m_gyro.isCalibrating());
      SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
      SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());

      /* Display tilt-corrected, Magnetometer-based heading (requires */
      /* magnetometer calibration to be useful) */

      SmartDashboard.putNumber("IMU_CompassHeading", m_gyro.getCompassHeading());

      /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
      SmartDashboard.putNumber("IMU_FusedHeading", m_gyro.getFusedHeading());

      /* These functions are compatible w/the WPI Gyro Class, providing a simple */
      /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

      SmartDashboard.putNumber("IMU_TotalYaw", m_gyro.getAngle());
      SmartDashboard.putNumber("IMU_YawRateDPS", m_gyro.getRate());

      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

      SmartDashboard.putNumber("IMU_Accel_X", m_gyro.getWorldLinearAccelX());
      SmartDashboard.putNumber("IMU_Accel_Y", m_gyro.getWorldLinearAccelY());
      SmartDashboard.putBoolean("IMU_IsMoving", m_gyro.isMoving());
      SmartDashboard.putBoolean("IMU_IsRotating", m_gyro.isRotating());

      /* Display estimates of velocity/displacement. Note that these values are */
      /* not expected to be accurate enough for estimating robot position on a */
      /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
      /* of these errors due to single (velocity) integration and especially */
      /* double (displacement) integration. */

      SmartDashboard.putNumber("Velocity_X", m_gyro.getVelocityX());
      SmartDashboard.putNumber("Velocity_Y", m_gyro.getVelocityY());
      SmartDashboard.putNumber("Displacement_X", m_gyro.getDisplacementX());
      SmartDashboard.putNumber("Displacement_Y", m_gyro.getDisplacementY());

      /* Display Raw Gyro/Accelerometer/Magnetometer Values */
      /* NOTE: These values are not normally necessary, but are made available */
      /* for advanced users. Before using this data, please consider whether */
      /* the processed data (see above) will suit your needs. */

      SmartDashboard.putNumber("RawGyro_X", m_gyro.getRawGyroX());
      SmartDashboard.putNumber("RawGyro_Y", m_gyro.getRawGyroY());
      SmartDashboard.putNumber("RawGyro_Z", m_gyro.getRawGyroZ());
      SmartDashboard.putNumber("RawAccel_X", m_gyro.getRawAccelX());
      SmartDashboard.putNumber("RawAccel_Y", m_gyro.getRawAccelY());
      SmartDashboard.putNumber("RawAccel_Z", m_gyro.getRawAccelZ());
      SmartDashboard.putNumber("RawMag_X", m_gyro.getRawMagX());
      SmartDashboard.putNumber("RawMag_Y", m_gyro.getRawMagY());
      SmartDashboard.putNumber("RawMag_Z", m_gyro.getRawMagZ());
      SmartDashboard.putNumber("IMU_Temp_C", m_gyro.getTempC());
      SmartDashboard.putNumber("IMU_Timestamp", m_gyro.getLastSensorTimestamp());

      /* Omnimount Yaw Axis Information */
      /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
      AHRS.BoardYawAxis yaw_axis = m_gyro.getBoardYawAxis();
      SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
      SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

      /* Sensor Board Information */
      SmartDashboard.putString("FirmwareVersion", m_gyro.getFirmwareVersion());

      /* Quaternion Data */
      /* Quaternions are fascinating, and are the most compact representation of */
      /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
      /* from the Quaternions. If interested in motion processing, knowledge of */
      /* Quaternions is highly recommended. */
      SmartDashboard.putNumber("QuaternionW", m_gyro.getQuaternionW());
      SmartDashboard.putNumber("QuaternionX", m_gyro.getQuaternionX());
      SmartDashboard.putNumber("QuaternionY", m_gyro.getQuaternionY());
      SmartDashboard.putNumber("QuaternionZ", m_gyro.getQuaternionZ());

      /* Connectivity Debugging Support */
      SmartDashboard.putNumber("IMU_Byte_Count", m_gyro.getByteCount());
      SmartDashboard.putNumber("IMU_Update_Count", m_gyro.getUpdateCount());
    }
    else {
      SmartDashboard.putBoolean("IMU DNE", true);
    }
  }

  public Rotation2d getRotation2d(){
    if (m_gyro != null) {
      return m_gyro.getRotation2d();
    }
    else {
      return new Rotation2d();
    }
  };

  public void reset(){
    if (m_gyro != null) {
       m_gyro.reset();
    }
  }

  public double getRate(){
    double rate = 0;
    if (m_gyro != null) {
      rate = m_gyro.getRate();
    }
    return rate;
  };
}

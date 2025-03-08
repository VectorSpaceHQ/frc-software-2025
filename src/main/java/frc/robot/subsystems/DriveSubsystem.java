// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.studica.frc.AHRS;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private VisionSubsystem m_robotVision;

  private final TalonFX m_frontLeft = new TalonFX(CANIDs.kDriveSubsystemFrontLeft);
  private final TalonFX m_rearLeft = new TalonFX(CANIDs.kDriveSubsystemRearLeft);
  private final TalonFX m_frontRight = new TalonFX(CANIDs.kDriveSubsystemFrontRight);
  private final TalonFX m_rearRight = new TalonFX(CANIDs.kDriveSubsystemRearRight);

  private double m_frontLeftEncoder = 0;
  private double m_rearLeftEncoder = 0;
  private double m_frontRightEncoder = 0;
  private double m_rearRightEncoder = 0;
  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set,
      m_rearRight::set);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  /**
   * Sets the vision subsystem for pose estimation
   * 
   * @param vision The vision subsystem to use
   */
  public void setVisionSubsystem(VisionSubsystem vision) {
    this.m_robotVision = vision;
  }

  // using default frontR rearR inverted right now
  private final TalonFXConfigurator frontRightConfigurator = m_frontRight.getConfigurator();
  private final TalonFXConfigurator rearRightConfigurator = m_rearRight.getConfigurator();
  private final TalonFXConfigurator frontLeftConfigurator = m_frontLeft.getConfigurator();
  private final TalonFXConfigurator rearLeftConfigurator = m_rearLeft.getConfigurator();

  private final MotorOutputConfigs frontRightMotorConfigs = new MotorOutputConfigs();
  private final MotorOutputConfigs rearRightMotorConfigs = new MotorOutputConfigs();
  private final MotorOutputConfigs frontLeftMotorConfigs = new MotorOutputConfigs();
  private final MotorOutputConfigs rearLeftMotorConfigs = new MotorOutputConfigs();

  private final CurrentLimitsConfigs frontRightCurrentConfigs = new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs rearRightCurrentConfigs = new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs frontLeftCurrentConfigs = new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs rearLeftCurrentConfigs = new CurrentLimitsConfigs();

  // Odometry class for tracking robot pose
  MecanumDrivePoseEstimator m_poseEstimator = new MecanumDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new MecanumDriveWheelPositions(), // Where would this be?
      new Pose2d(), // What Pose2d?
      VecBuilder.fill(0.1, 0.1, 0.1), // Standard deviations for state measurements?
      VecBuilder.fill(0.45, 0.45, 0.45)); // Standard deviations for vision measurements?

  // PID Controllers
  private final PIDController kPXController = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController kPYController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController headingController = new ProfiledPIDController(AutoConstants.kPThetaController, 0,
      0, AutoConstants.kThetaControllerConstraints);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Sets the distance per pulse for the encoders (most likely won't be used but
    // was in original template)
    // m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // Inversion of two motors
    frontRightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rearRightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;

    // Current Limits
    frontRightCurrentConfigs.withSupplyCurrentLimit(40);
    frontRightCurrentConfigs.withStatorCurrentLimit(60);

    rearRightCurrentConfigs.withSupplyCurrentLimit(40);
    rearRightCurrentConfigs.withStatorCurrentLimit(60);

    frontLeftCurrentConfigs.withSupplyCurrentLimit(40);
    frontLeftCurrentConfigs.withStatorCurrentLimit(60);

    rearLeftCurrentConfigs.withSupplyCurrentLimit(40);
    rearLeftCurrentConfigs.withStatorCurrentLimit(60);

    frontRightConfigurator.apply(frontRightMotorConfigs);
    rearRightConfigurator.apply(rearRightMotorConfigs);
    frontLeftConfigurator.apply(frontLeftMotorConfigs);
    rearLeftConfigurator.apply(rearLeftMotorConfigs);

    frontRightConfigurator.apply(frontRightCurrentConfigs);
    rearRightConfigurator.apply(rearRightCurrentConfigs);
    frontLeftConfigurator.apply(frontLeftCurrentConfigs);
    rearLeftConfigurator.apply(rearLeftCurrentConfigs);

    // m_drive.setMaxOutput(0.25); // Conservative for now
  }

  /**
   * Sets the wheel speeds for the mecanum drive.
   *
   * @param wheelSpeeds The desired wheel speeds.
   */
  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    m_drive.driveCartesian(wheelSpeeds.frontLeftMetersPerSecond, wheelSpeeds.frontRightMetersPerSecond,
        wheelSpeeds.rearLeftMetersPerSecond, m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_frontLeftEncoder = m_frontLeft.getPosition().getValue().magnitude();
    m_rearLeftEncoder = m_rearLeft.getPosition().getValue().magnitude();
    m_frontRightEncoder = m_frontRight.getPosition().getValue().magnitude();
    m_rearRightEncoder = m_rearRight.getPosition().getValue().magnitude();
    m_poseEstimator.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
    if (m_robotVision != null) {
      m_robotVision.getRobotPose()
          .ifPresent(pose -> m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()));
    }
  }

  // Runs motor fault checks for logging purposes
  private void faultChecks() {
    // Supply Limit Checks
    if (m_frontLeft.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Front Left Supply Current Limit Hit");
    }

    if (m_rearLeft.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Rear Left Supply Current Limit Hit");
    }

    if (m_frontRight.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Front Right Supply Current Limit Hit");
    }

    if (m_rearRight.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Rear Right Supply Current Limit Hit");
    }

    // Stator Limit Checks
    if (m_frontLeft.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Front Left Stator Current Limit Hit");
    }

    if (m_rearLeft.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Rear Left Stator Current Limit Hit");
    }

    if (m_frontRight.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Front Right Stator Current Limit Hit");
    }

    if (m_rearRight.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Rear Right Stator Current Limit Hit");
    }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public double gyroAngle() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
        sample.vx + kPXController.calculate(pose.getX(), sample.x),
        sample.vy + kPYController.calculate(pose.getY(), sample.y),
        sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));
    MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);
    setWheelSpeeds(wheelSpeeds);

  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  public void strafe(double ySpeed) {
    m_drive.driveCartesian(0, ySpeed, 0, m_gyro.getRotation2d());
  }

  public void turn(double rot) {
    m_drive.driveCartesian(0, 0, rot, m_gyro.getRotation2d());
  }

  public double getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public double getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public double getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public double getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  public void setDriveMotorControllersVolts(
      double frontLeftVoltage,
      double frontRightVoltage,
      double rearLeftVoltage,
      double rearRightVoltage) {
    m_frontLeft.setVoltage(frontLeftVoltage);
    m_rearLeft.setVoltage(rearLeftVoltage);
    m_frontRight.setVoltage(frontRightVoltage);
    m_rearRight.setVoltage(rearRightVoltage);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeft.getVelocity().getValue().magnitude(),
        m_rearLeft.getVelocity().getValue().magnitude(),
        m_frontRight.getVelocity().getValue().magnitude(),
        m_rearRight.getVelocity().getValue().magnitude());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a
   *         MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder,
        m_rearLeftEncoder,
        m_frontRightEncoder,
        m_rearRightEncoder);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void operatorControl() {
    while (DriverStation.isTeleop() && DriverStation.isEnabled()) {

      Timer.delay(0.020); /* wait for one motor update time period (50Hz) */

      boolean zero_yaw_pressed = false; // stick.getTrigger();
      if (zero_yaw_pressed) {
        m_gyro.zeroYaw();
      }

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
  }
}

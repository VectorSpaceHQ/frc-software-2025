// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveSubsystem extends SubsystemBase {

  
  private final TalonFX m_frontLeft = new TalonFX(CANIDs.kDriveSubsystemFrontLeft);
  private final TalonFX m_rearLeft = new TalonFX(CANIDs.kDriveSubsystemRearLeft);
  private final TalonFX m_frontRight = new TalonFX(CANIDs.kDriveSubsystemFrontRight);
  private final TalonFX m_rearRight = new TalonFX(CANIDs.kDriveSubsystemRearRight);


  private double m_frontLeftEncoder = 0;
  private double m_rearLeftEncoder = 0;
  private double m_frontRightEncoder = 0;
  private double m_rearRightEncoder = 0;
  private final MecanumDrive m_drive =
     new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

   //using default frontR rearR inverted right now
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
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  MecanumDrivePoseEstimator m_mecanumDrivePoseEstimator =
      new MecanumDrivePoseEstimator(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), getCurrentWheelDistances(), getPose());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Sets the distance per pulse for the encoders (most likely won't be used but was in original template)
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
    frontRightCurrentConfigs.withStatorCurrentLimit(100);
    

    rearRightCurrentConfigs.withSupplyCurrentLimit(40);
    rearRightCurrentConfigs.withStatorCurrentLimit(100);

    frontLeftCurrentConfigs.withSupplyCurrentLimit(40);
    frontLeftCurrentConfigs.withStatorCurrentLimit(100);

    rearLeftCurrentConfigs.withSupplyCurrentLimit(40);
    rearLeftCurrentConfigs.withStatorCurrentLimit(100);

    frontRightConfigurator.apply(frontRightMotorConfigs);
    rearRightConfigurator.apply(rearRightMotorConfigs);
    frontLeftConfigurator.apply(frontLeftMotorConfigs);
    rearLeftConfigurator.apply(rearLeftMotorConfigs);

    frontRightConfigurator.apply(frontRightCurrentConfigs);
    rearRightConfigurator.apply(rearRightCurrentConfigs);
    frontLeftConfigurator.apply(frontLeftCurrentConfigs);
    rearLeftConfigurator.apply(rearLeftCurrentConfigs);

    // m_drive.setMaxOutput(0.3);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_frontLeftEncoder = m_frontLeft.getPosition().getValue().magnitude();
    m_rearLeftEncoder = m_rearLeft.getPosition().getValue().magnitude();
    m_frontRightEncoder = m_frontRight.getPosition().getValue().magnitude();
    m_rearRightEncoder = m_rearRight.getPosition().getValue().magnitude();
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
    faultChecks();
  }

  // Runs motor fault checks for logging purposes
  private void faultChecks() {
    // Supply Limit Checks
    if(m_frontLeft.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Front Left Supply Current Limit Hit");
    }

    if(m_rearLeft.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Rear Left Supply Current Limit Hit");
    }

    if(m_frontRight.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Front Right Supply Current Limit Hit");
    }

    if(m_rearRight.getFault_SupplyCurrLimit().getValue()) {
      DataLogManager.log("Rear Right Supply Current Limit Hit");
    }

    // Stator Limit Checks
    if(m_frontLeft.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Front Left Stator Current Limit Hit");
    }

    if(m_rearLeft.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Rear Left Stator Current Limit Hit");
    }

    if(m_frontRight.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Front Right Stator Current Limit Hit");
    }

    if(m_rearRight.getFault_StatorCurrLimit().getValue()) {
      DataLogManager.log("Rear Right Stator Current Limit Hit");
    }

    SmartDashboard.putNumber("front left drive current", m_frontLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("front right drive current", m_frontRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("rear left drive current", m_rearLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("rear right drive current", m_rearRight.getStatorCurrent().getValueAsDouble());
  
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("x speed", xSpeed);
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
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
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder,
        m_rearLeftEncoder,
        m_frontRightEncoder,
        m_rearRightEncoder);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
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
}

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Temporary: for vision measurements
// import frc.robot.subsystems.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_frontLeft = new TalonFX(CANIDs.kDriveSubsystemFrontLeft);
  private final TalonFX m_rearLeft = new TalonFX(CANIDs.kDriveSubsystemRearLeft);
  private final TalonFX m_frontRight = new TalonFX(CANIDs.kDriveSubsystemFrontRight);
  private final TalonFX m_rearRight = new TalonFX(CANIDs.kDriveSubsystemRearRight);

  private double m_frontLeftEncoder = 0;
  private double m_rearLeftEncoder = 0;
  private double m_frontRightEncoder = 0;
  private double m_rearRightEncoder = 0;
  

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

  private Gyro m_gyro = null;
  private RobotPoseEstimatorSubsystem m_poseEstimator = null;
  private MecanumDriveWheelSpeeds m_WheelSpeeds = new MecanumDriveWheelSpeeds();
  private ChassisSpeeds m_ChassisSpeeds = new ChassisSpeeds();
  private MecanumDriveKinematics m_Kinematics = new MecanumDriveKinematics(new Translation2d(.314, 0.292), new Translation2d(.314, -0.292), new Translation2d(-.314, 0.292), new Translation2d(-.314, -0.292));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {


    // Sets the distance per pulse for the encoders (most likely won't be used but was in original template)
    // m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    // Inversion of two motors
    // frontRightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    // rearRightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    frontLeftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rearLeftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    
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
    
    initConfig();
  }

  public void setGyro(Gyro gyro) {
    this.m_gyro = gyro;
  }

  /**
   * Gets the gyro used by this drive subsystem
   * @return The gyro
   */
  public Gyro getGyro() {
    return m_gyro;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_frontLeftEncoder = m_frontLeft.getPosition().getValue().magnitude();
    m_rearLeftEncoder = m_rearLeft.getPosition().getValue().magnitude();
    m_frontRightEncoder = m_frontRight.getPosition().getValue().magnitude();
    m_rearRightEncoder = m_rearRight.getPosition().getValue().magnitude();
    
  
    
    // Only display IMU data if gyro is initialized
    if (m_gyro != null) {
      m_gyro.DisplayIMUData();
    }

    faultChecks();
    driveLogging();
  }

  private void initConfig(){
    

    // Configure AutoBuilder last
    if (DriveConstants.config != null){
        // taken from PathPlanner Examples will edit based on needs
          AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry,
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> driveRobotChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0, 0.0, 0.0) // Rotation PID constants
            ),
            DriveConstants.config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
          );
    }
  }

  // Runs motor fault checks for logging purposes
  private void faultChecks() {
    // Supply Limit Checks
    // if(m_frontLeft.getFault_SupplyCurrLimit().getValue()) {
    //   DataLogManager.log("Front Left Supply Current Limit Hit");
    // }

    // if(m_rearLeft.getFault_SupplyCurrLimit().getValue()) {
    //   DataLogManager.log("Rear Left Supply Current Limit Hit");
    // }

    // if(m_frontRight.getFault_SupplyCurrLimit().getValue()) {
    //   DataLogManager.log("Front Right Supply Current Limit Hit");
    // }

    // if(m_rearRight.getFault_SupplyCurrLimit().getValue()) {
    //   DataLogManager.log("Rear Right Supply Current Limit Hit");
    // }

    // // Stator Limit Checks
    // if(m_frontLeft.getFault_StatorCurrLimit().getValue()) {
    //   DataLogManager.log("Front Left Stator Current Limit Hit");
    // }

    // if(m_rearLeft.getFault_StatorCurrLimit().getValue()) {
    //   DataLogManager.log("Rear Left Stator Current Limit Hit");
    // }

    // if(m_frontRight.getFault_StatorCurrLimit().getValue()) {
    //   DataLogManager.log("Front Right Stator Current Limit Hit");
    // }

    // if(m_rearRight.getFault_StatorCurrLimit().getValue()) {
    //   DataLogManager.log("Rear Right Stator Current Limit Hit");
    // }
  }

  private void driveLogging() {
    SmartDashboard.putNumber("front left drive current", m_frontLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("front right drive current", m_frontRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("rear left drive current", m_rearLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("rear right drive current", m_rearRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Chassis Forward (m/s)", m_Kinematics.toChassisSpeeds(m_WheelSpeeds).vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis Strafe (m/s)", m_Kinematics.toChassisSpeeds(m_WheelSpeeds).vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis Rotation (rad/s)", m_Kinematics.toChassisSpeeds(m_WheelSpeeds).omegaRadiansPerSecond);
  }

  private Pose2d getInitialPose() {
    return new Pose2d();
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    if (m_poseEstimator != null) {
      return m_poseEstimator.getPose();
    }
    return getInitialPose(); // Return default pose if estimator is not initialized
  }

 
  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) {
    if (m_poseEstimator != null) {
      m_poseEstimator.resetPose(pose);
    }
  }

  public void resetOdometry() {
    if (m_poseEstimator != null && m_gyro != null) {
      m_poseEstimator.resetPose(new Pose2d());
    }
  }


  public void driveRobotChassisSpeeds(ChassisSpeeds speeds) {
    m_WheelSpeeds = m_Kinematics.toWheelSpeeds(speeds);
    driveMecanumWheelSpeeds(m_WheelSpeeds);
  }

  public void driveFieldChassisSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds m_robotspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_poseEstimator.getPose().getRotation());
    m_WheelSpeeds = m_Kinematics.toWheelSpeeds(m_robotspeeds);
    driveMecanumWheelSpeeds(m_WheelSpeeds);
  }

  public void driveMecanumWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
    driveWheelSpeed(speeds.frontLeftMetersPerSecond, m_frontLeft);
    driveWheelSpeed(speeds.frontRightMetersPerSecond, m_frontRight);
    driveWheelSpeed(speeds.rearLeftMetersPerSecond, m_rearLeft);
    driveWheelSpeed(speeds.rearRightMetersPerSecond, m_rearRight);
    SmartDashboard.putNumber("Front Left WheelSpeed Output", speeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Front Right WheelSpeed Output", speeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("Rear Left WheelSpeed Output", speeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Rear Right WheelSpeed Output", speeds.rearRightMetersPerSecond);
  }

  private void driveWheelSpeed(double WheelSpeed, TalonFX MotorController){
    double MotorSpeed = WheelSpeed / DriveConstants.kMetersPerMotorRotation; // RPS
    double MotorVoltage = MotorSpeed * DriveConstants.kKrakenVoltsPerRPS;
    MotorController.setVoltage(MotorVoltage);
    SmartDashboard.putNumber(MotorController.toString(), MotorSpeed);
  }

  public ChassisSpeeds PWMInputToChassisSpeeds(double PWMInputForward, double PWMInputStrafe, double PWMInputRotational){
    return new ChassisSpeeds(PWMInputForward * DriveConstants.kForwardDriverVelocityScalar, 
    PWMInputStrafe * DriveConstants.kStrafeDriverVelocityScalar, 
    PWMInputRotational * DriveConstants.kRotationalDriverVelocityScalar);
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
    SmartDashboard.putNumber("front Left wheel calculated speed",  m_frontLeft.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation);
    SmartDashboard.putNumber("rear Left wheel calculated speed",  m_rearLeft.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation);
    SmartDashboard.putNumber("front Right wheel calculated speed",  m_frontRight.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation);
    SmartDashboard.putNumber("rear Right wheel calculated speed",  m_rearRight.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation);
    
    return new MecanumDriveWheelSpeeds(
        m_frontLeft.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation,
        m_rearLeft.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation,
        m_frontRight.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation,
        m_rearRight.getVelocity().getValue().magnitude() * DriveConstants.kMetersPerMotorRotation);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_Kinematics.toChassisSpeeds(getCurrentWheelSpeeds());
  }

  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder * DriveConstants.kMetersPerMotorRotation,
        m_rearLeftEncoder * DriveConstants.kMetersPerMotorRotation,
        m_frontRightEncoder * DriveConstants.kMetersPerMotorRotation,
        m_rearRightEncoder * DriveConstants.kMetersPerMotorRotation);
  }

  public MecanumDriveKinematics getMecanumDriveKinematics() {
    return m_Kinematics;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    if (m_gyro != null) {
      m_gyro.reset();
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    if (m_gyro != null) {
      return m_gyro.getRotation2d().getDegrees();
    }
    return 0.0; // Default heading if gyro is not initialized
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (m_gyro != null) {
      return -m_gyro.getRate();
    }
    return 0.0; // Default turn rate if gyro is not initialized
  }

  public void setPoseEstimator(RobotPoseEstimatorSubsystem estimator){
    m_poseEstimator = estimator;
  }
}
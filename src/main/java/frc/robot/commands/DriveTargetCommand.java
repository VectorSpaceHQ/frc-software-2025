package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotPoseEstimatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;

public class DriveTargetCommand extends Command {

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;
  private RobotPoseEstimatorSubsystem poseEstimator;


  private double speedscalar = 1;
  private double turnPWM = 0;

  private double forwardPWM;
  private double strafePWM;
  // private DynamicSlewRateLimiter x_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  // private DynamicSlewRateLimiter y_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  // Sets the drivetarget constructor
  public DriveTargetCommand(
      DriveSubsystem driveSubsystem,
      CommandXboxController driverController,
      ElevatorSubsystem elevatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    this.elevatorSubsystem = elevatorSubsystem;
    table.put(elevatorSubsystem.getMinHeight(), DriveConstants.kMaxAcceleration);
    table.put(elevatorSubsystem.getMaxHeight(), DriveConstants.kMinAcceleration);
    addRequirements(driveSubsystem);
  }

  public void setPoseEstimator(RobotPoseEstimatorSubsystem estimator) {
    this.poseEstimator = estimator;
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    // linearAccelerationLimit = table.get(elevatorSubsystem.getElevatorHeight());
    
    forwardPWM = driverController.getLeftY() * speedscalar;
    strafePWM = -driverController.getLeftX() * speedscalar;
    turnPWM = (-driverController.getRightX());
    var outputChassisSpeeds = driveSubsystem.PWMInputToChassisSpeeds(forwardPWM, strafePWM, turnPWM);
    SmartDashboard.putNumber("Chassis Forward (m/s)", outputChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis Strafe (m/s)", outputChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis Rotation (rad/s)", outputChassisSpeeds.omegaRadiansPerSecond);

    driveSubsystem.driveRobotChassisSpeeds(outputChassisSpeeds);
    // Can add a field relative using ChassisSpeeds.fromFieldRelativeSpeeds()
    DriveTargetCommandLogger();
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveRobotChassisSpeeds(new ChassisSpeeds());
    SmartDashboard.putString("Aiming Status", "Command Ended");
  }

  public void setSpeedScalar(double val){
    speedscalar = val;
  }

  private void DriveTargetCommandLogger() {
    SmartDashboard.putNumber("Forward PWM", forwardPWM);
    SmartDashboard.putNumber("Strafe PWM", strafePWM);
    SmartDashboard.putNumber("Turn PWM", turnPWM);
  }
}
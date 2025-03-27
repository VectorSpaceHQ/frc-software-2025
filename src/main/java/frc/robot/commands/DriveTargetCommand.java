package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.DynamicSlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;

public class DriveTargetCommand extends Command {
  private final double visionThingy = 1.25;

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;

  private double speedscalar = 1;
  private double targetID = 0;
  private double forward =  0;
  private double strafe =  0;
  private double turn = 0;
  private boolean autoDriveFeatureToggle = false;

  private double forwardPWM;
  private double strafePWM;
  private double forwardVoltage;
  private double strafeVoltage;
  private double forwardLinearSpeed;
  private double strafeLinearSpeed;
  private double forwardAdjustedLinearSpeed;
  private double strafeAdjustedLinearSpeed;
  private double forwardAdjustedVoltage;
  private double strafeAdjustedVoltage;
  private double forwardAdjustedPWM;
  private double strafeAdjustedPWM;
  private double linearAccelerationLimit;
  private DynamicSlewRateLimiter x_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private DynamicSlewRateLimiter y_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  // Sets the drivetarget constructor
  public DriveTargetCommand(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      CommandXboxController driverController,
      ElevatorSubsystem elevatorSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    this.elevatorSubsystem = elevatorSubsystem;
    table.put(elevatorSubsystem.getMinHeight(), DriveConstants.kMaxAcceleration);
    table.put(elevatorSubsystem.getMaxHeight(), DriveConstants.kMinAcceleration);
    addRequirements(visionSubsystem, driveSubsystem);
    setTargetID(AprilTags.None);
  }

  public void setTargetID(AprilTags tagId) {
    targetID = tagId.getId();
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    if (visionSubsystem.getRobotPose().isPresent()) {
    driveSubsystem.addVisionUpdate(visionSubsystem.getRobotPose().get(), Timer.getFPGATimestamp());
    }
    linearAccelerationLimit = table.get(elevatorSubsystem.getElevatorHeight());

    forwardPWM = driverController.getLeftY() * speedscalar;
    forwardVoltage = forwardPWM * DriveConstants.kDefaultBusVoltage;
    forwardLinearSpeed = forwardVoltage / DriveConstants.kForwardVoltsPerMeterPerSecond;
    forwardAdjustedLinearSpeed = x_rate.calculate(forwardLinearSpeed, linearAccelerationLimit);
    forwardAdjustedVoltage = forwardAdjustedLinearSpeed * DriveConstants.kForwardVoltsPerMeterPerSecond;
    forwardAdjustedPWM = forwardAdjustedVoltage / DriveConstants.kDefaultBusVoltage;

    strafePWM = -driverController.getLeftX() * speedscalar;
    strafeVoltage = strafePWM * DriveConstants.kDefaultBusVoltage;
    strafeLinearSpeed = strafeVoltage / DriveConstants.kStrafeVoltsPerMeterPerSecond;
    strafeAdjustedLinearSpeed = y_rate.calculate(strafeLinearSpeed, linearAccelerationLimit);
    strafeAdjustedVoltage = strafeAdjustedLinearSpeed * DriveConstants.kStrafeVoltsPerMeterPerSecond;
    strafeAdjustedPWM = strafeAdjustedVoltage / DriveConstants.kDefaultBusVoltage;
    
    forward = forwardAdjustedPWM;
    strafe = strafeAdjustedPWM;
    turn = (-0.3 * driverController.getRightX());

    DriveTargetCommandLogger();
    driveSubsystem.drive(forward, strafe, turn, true);
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
    SmartDashboard.putString("Aiming Status", "Command Ended");
  }

  public void setSpeedScalar(double val){
    speedscalar = val;
  }

  private void DriveTargetCommandLogger() {
    SmartDashboard.putNumber("Linear Acceleration Limit", linearAccelerationLimit);
    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Forward PWM", forwardPWM);
    SmartDashboard.putNumber("Forward Voltage", forwardVoltage);
    SmartDashboard.putNumber("Forward Linear Speed", forwardLinearSpeed);
    SmartDashboard.putNumber("Forward Adjusted Linear Speed", forwardAdjustedLinearSpeed);
    SmartDashboard.putNumber("Forward Adjusted Voltage", forwardAdjustedVoltage);
    SmartDashboard.putNumber("Forward Adjusted PWM", forwardAdjustedPWM);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Strafe PWM", strafePWM);
    SmartDashboard.putNumber("Forward Voltage", strafeVoltage);
    SmartDashboard.putNumber("strafe Linear Speed", strafeLinearSpeed);
    SmartDashboard.putNumber("strafe Adjusted Linear Speed", strafeAdjustedLinearSpeed);
    SmartDashboard.putNumber("strafe Adjusted Voltage", strafeAdjustedVoltage);
    SmartDashboard.putNumber("strafe Adjusted PWM", strafeAdjustedPWM);
  }
}

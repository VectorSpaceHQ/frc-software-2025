package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Gyro;

public class RobotPoseEstimatorSubsystem extends SubsystemBase {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final Gyro gyro;

  // The main pose estimator
  private final MecanumDrivePoseEstimator poseEstimator;

  private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.02, 0.02, 0.01); // For odometry
  private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.45, 0.45, 0.45); // For vision

  // Track if we've received a vision update this cycle
  private boolean visionUpdateThisCycle = false;

  public RobotPoseEstimatorSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Gyro gyro) {
    // Validate required components
    if (driveSubsystem == null) {
      throw new IllegalArgumentException("DriveSubsystem cannot be null");
    }
    if (visionSubsystem == null) {
      throw new IllegalArgumentException("VisionSubsystem cannot be null");
    }
    if (gyro == null) {
      throw new IllegalArgumentException("Gyro cannot be null");
    }

    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.gyro = gyro;

    // Initialize pose estimator with safe starting values
    try {
      poseEstimator = new MecanumDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances(),
          new Pose2d(),
          stateStdDevs,
          visionStdDevs);
      SmartDashboard.putBoolean("Pose Estimator Constructor Success", true);
    } catch (Exception e) {
      // Critical initialization failure - rethrow the exception
      SmartDashboard.putBoolean("Pose Estimator Constructor Success", false);
      SmartDashboard.putString("Pose Estimator Init Error", e.getMessage());
      throw e;
    }
  }

  @Override
  public void periodic() {
    try {
      // Always update with odometry data
      poseEstimator.update(
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances());

      // Reset vision update
      visionUpdateThisCycle = false;

      // Process vision data if available and fresh
      if (visionSubsystem.isCameraConnected() && visionSubsystem.isFreshPose()) {
        Optional<Pose2d> visionPose = visionSubsystem.getRobotPose();
        double timestamp = visionSubsystem.getTimestamp();

        if (visionPose.isPresent() && timestamp > 0) {
          try {
            // Add the vision measurement
            poseEstimator.addVisionMeasurement(visionPose.get(), timestamp);
            visionUpdateThisCycle = true;
            SmartDashboard.putNumber("Last Vision Update", Timer.getFPGATimestamp());
          } catch (Exception e) {
            // Log vision integration errors but continue with odometry
            SmartDashboard.putString("Vision Integration Error", e.getMessage());
          }
        }
      }

      if (!visionUpdateThisCycle) {
        SmartDashboard.putString("Pose Source", "Odometry Only");
      }

      // Publish estimated pose
      Pose2d currentPose = getPose();
      SmartDashboard.putNumber("Robot Pose X", currentPose.getX());
      SmartDashboard.putNumber("Robot Pose Y", currentPose.getY());
      SmartDashboard.putNumber("Robot Pose Heading", currentPose.getRotation().getDegrees());

      // Calculate time since last vision update
      double timeSinceVisionUpdate = Timer.getFPGATimestamp() - visionSubsystem.getTimestamp();
      SmartDashboard.putNumber("Time Since Vision Update", timeSinceVisionUpdate);
    } catch (Exception e) {
      // Log any errors during periodic updates
      SmartDashboard.putString("Pose Estimator Periodic Error", e.getMessage());
    }
  }

  public Pose2d getPose() {
    try {
      return poseEstimator.getEstimatedPosition();
    } catch (Exception e) {
      SmartDashboard.putString("Get Pose Error", e.getMessage());
      return new Pose2d(); // Return origin if error occurs
    }
  }

  // Resets pose
  public void resetPose(Pose2d pose) {
    try {
      poseEstimator.resetPosition(
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances(),
          pose);
      SmartDashboard.putString("Pose Reset Status", "Success");
    } catch (Exception e) {
      SmartDashboard.putString("Pose Reset Error", e.getMessage());
    }
  }

  public boolean hasRecentVisionUpdate() {
    return visionUpdateThisCycle;
  }
}
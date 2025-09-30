package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gyro;

public class RobotPoseEstimatorSubsystem extends SubsystemBase {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final Gyro gyro;

  private final MecanumDrivePoseEstimator poseEstimator;

  private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.02, 0.02, 0.01);
  private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.45, 0.45, 0.45);

  private boolean visionUpdateThisCycle = false;

  private final ShuffleboardTab localizationTab = Shuffleboard.getTab("Localization");
  private final ShuffleboardLayout initCol = localizationTab.getLayout("Init", BuiltInLayouts.kList).withPosition(0,0).withSize(1,4);
  private final ShuffleboardLayout visionCol = localizationTab.getLayout("Vision", BuiltInLayouts.kList).withPosition(1,0).withSize(1,6);
  private final ShuffleboardLayout poseCol = localizationTab.getLayout("Pose", BuiltInLayouts.kList).withPosition(2,0).withSize(1,5);
  private final ShuffleboardLayout errorsCol = localizationTab.getLayout("Errors", BuiltInLayouts.kList).withPosition(3,0).withSize(1,6);

  private final GenericEntry constructorSuccessEntry = initCol.add("Init Success", false).getEntry();
  private final GenericEntry constructorErrorEntry = initCol.add("Init Error", "").getEntry();
  private final GenericEntry poseResetStatusEntry = initCol.add("Pose Reset Status", "").getEntry();
  private final GenericEntry poseResetErrorEntry = initCol.add("Pose Reset Error", "").getEntry();

  private final GenericEntry lastVisionUpdateEntry = visionCol.add("Last Vision Update", 0.0).getEntry();
  private final GenericEntry visionUpdateFlagEntry = visionCol.add("Vision Update This Cycle", false).getEntry();
  private final GenericEntry poseSourceEntry = visionCol.add("Pose Source", "Odometry Only").getEntry();
  private final GenericEntry timeSinceVisionUpdateEntry = visionCol.add("Time Since Vision Update", 0.0).getEntry();

  private final GenericEntry robotPoseXEntry = poseCol.add("Robot Pose X", 0.0).getEntry();
  private final GenericEntry robotPoseYEntry = poseCol.add("Robot Pose Y", 0.0).getEntry();
  private final GenericEntry robotPoseHeadingEntry = poseCol.add("Robot Pose Heading", 0.0).getEntry();

  private final GenericEntry periodicErrorEntry = errorsCol.add("Periodic Error", "").getEntry();
  private final GenericEntry visionIntegrationErrorEntry = errorsCol.add("Vision Integration Error", "").getEntry();
  private final GenericEntry poseErrorEntry = errorsCol.add("Get Pose Error", "").getEntry();

  public RobotPoseEstimatorSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Gyro gyro) {

    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.gyro = gyro;

    try {
      poseEstimator = new MecanumDrivePoseEstimator(
          driveSubsystem.getMecanumDriveKinematics(),
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances(),
          new Pose2d(),
          stateStdDevs,
          visionStdDevs);
      constructorSuccessEntry.setBoolean(true);
      constructorErrorEntry.setString("");
    } catch (Exception e) {

      constructorSuccessEntry.setBoolean(false);
      constructorErrorEntry.setString(e.getMessage());
      throw e;
    }
  }

  @Override
  public void periodic() {

    try {

      poseEstimator.update(
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances());

      visionUpdateThisCycle = false;
      visionUpdateFlagEntry.setBoolean(false);
      visionIntegrationErrorEntry.setString("");

      if (visionSubsystem.isCameraConnected() && visionSubsystem.isFreshPose()) {
        Optional<Pose2d> visionPose = visionSubsystem.getRobotPose();
        double timestamp = visionSubsystem.getTimestamp();

        if (visionPose.isPresent() && timestamp > 0) {

          try {
            poseEstimator.addVisionMeasurement(visionPose.get(), timestamp);
            visionUpdateThisCycle = true;
            visionUpdateFlagEntry.setBoolean(true);
            lastVisionUpdateEntry.setDouble(Timer.getFPGATimestamp());
          } catch (Exception e) {
            visionIntegrationErrorEntry.setString(e.getMessage());
          }
        }
      }

      if (visionUpdateThisCycle) {
        poseSourceEntry.setString("Vision + Odometry");
      } else {
        poseSourceEntry.setString("Odometry Only");
      }

      Pose2d currentPose = getPose();
      robotPoseXEntry.setDouble(currentPose.getX());
      robotPoseYEntry.setDouble(currentPose.getY());
      robotPoseHeadingEntry.setDouble(currentPose.getRotation().getDegrees());

      double timeSinceVisionUpdate = Timer.getFPGATimestamp() - visionSubsystem.getTimestamp();
      timeSinceVisionUpdateEntry.setDouble(timeSinceVisionUpdate);
      periodicErrorEntry.setString("");

    } catch (Exception e) {
      periodicErrorEntry.setString(e.getMessage());
    }
  }

  public Pose2d getPose() {

    try {
      poseErrorEntry.setString("");
      return poseEstimator.getEstimatedPosition();

    } catch (Exception e) {
      poseErrorEntry.setString(e.getMessage());
      return new Pose2d();
    }
  }

  public void resetPose(Pose2d pose) {

    try {
      poseEstimator.resetPosition(
          gyro.getRotation2d(),
          driveSubsystem.getCurrentWheelDistances(),
          pose);
      poseResetStatusEntry.setString("Success");
      poseResetErrorEntry.setString("");

    } catch (Exception e) {
      poseResetStatusEntry.setString("Failed");
      poseResetErrorEntry.setString(e.getMessage());
    }
  }

  public boolean hasRecentVisionUpdate() {
    return visionUpdateThisCycle;
  }
}

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;

public class VisionSubsystem extends SubsystemBase {
  // Strategy for processing multiple AprilTags on the coprocessor using multiple
  // teams
  public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_PROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  // Constants for the camera name and field layout path
  private static final String camera_name = "Front_Camera_Robot2";
  private String field_layout_path = new File(Filesystem.getDeployDirectory(), "2025-reefscape.json")
      .getAbsolutePath();

  // Photon camera and april tag components
  private final PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private PhotonPoseEstimator poseEstimator;

  // Constants for the maximum pose age and ambiguity
  private final double maxPoseAge = 0.5;
  private final double maxAmbiguity = 0.2;

  // Transformation3d objects for the camera and robot

  // Distance from the camera to the robot
  private static final Transform3d cameraToRobot = new Transform3d(
      new Translation3d(0.0, 0.0, 0.0),
      new Rotation3d(0.0, Math.toRadians(0), 0.0));

  // Distance from the robot to the camera
  private Transform3d robotToCamera = cameraToRobot.inverse();

  // Status of the camera
  private boolean cameraConnected;

  // Stored estimated pose
  private Optional<EstimatedRobotPose> storedEstimatedPose = Optional.empty();

  // Getting All Unread Results
  private List<PhotonPipelineResult> allUnreadResults = new ArrayList<>();

  // Shuffleboard entries
  private ShuffleboardTab visionTab;
  private GenericEntry yawEntry;
  private GenericEntry pitchEntry;
  private GenericEntry skewEntry;
  private GenericEntry idEntry;
  private GenericEntry areaEntry;
  private GenericEntry ambiguityEntry;
  private GenericEntry xEntry;
  private GenericEntry yEntry;
  private GenericEntry zEntry;

  // Vision Subsystem constructor
  public VisionSubsystem() {
    // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
    camera = new PhotonCamera(camera_name);

    initializeSubsystem();
    initializeShuffleboard();

    try {
      initializeAprilTagFieldLayout();
    } catch (IOException e) {
      System.err.println("Error initializing AprilTag field layout: " + e.getMessage());
      cameraConnected = false;
    }
  }

  private void initializeSubsystem() {
    try {
      cameraConnected = camera.isConnected(); // True if camera is connected

      if (!cameraConnected) {
        System.err.println("Warning: Camera not connected.");
        return;
      }
      System.out.println("Camera connected. Vision Subsystem initialized.");

    } catch (Exception e) {
      System.err.println("Error initializing camera: " + e.getMessage());
      cameraConnected = false;
    }
  }

  // Method to check if the camera is connected
  public boolean isCameraConnected() {
    return cameraConnected;
  }

  // Initializes the AprilTag field layout from the JSON file containing the 2025
  // layout
  private void initializeAprilTagFieldLayout() throws IOException {
    try { // Try-catch block to catch any exceptions that may occur (such as a missing
          // JSON file)
      Path path = Paths.get(field_layout_path);
      layout = new AprilTagFieldLayout(path);

      // Origin Point
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

      // Initialize pose estimator
      poseEstimator = new PhotonPoseEstimator(layout, MULTI_TAG_PNP_ON_PROCESSOR, robotToCamera);
    } catch (IOException e) {
      System.err.println("Error initializing AprilTag field layout: " + e.getMessage());
      cameraConnected = false;
    }

  }

  // Initializing the shuffleboard entries
  private void initializeShuffleboard() {
    visionTab = Shuffleboard.getTab("Vision Results");
    yawEntry = visionTab.add("Yaw", 0.0).getEntry();
    pitchEntry = visionTab.add("Pitch", 0.0).getEntry();
    areaEntry = visionTab.add("Area", 0.0).getEntry();
    skewEntry = visionTab.add("Skew", 0.0).getEntry();
    idEntry = visionTab.add("ID", 0.0).getEntry();
    ambiguityEntry = visionTab.add("Ambiguity", 0.0).getEntry();
    xEntry = visionTab.add("X", 0.0).getEntry();
    yEntry = visionTab.add("Y", 0.0).getEntry();
    zEntry = visionTab.add("Z", 0.0).getEntry();
  }

  // Gets the target yaw
  public double getTargetYaw(int id) {
    double yawValue = Double.NaN; // Default to NaN

    var tagPose = layout.getTagPose(id);
    if (tagPose.isPresent()) {
      Pose2d tagPose2d = tagPose.get().toPose2d();
      Optional<Pose2d> robotPose = getRobotPose();

      if (robotPose.isPresent()) {
        Rotation2d returnYaw = PhotonUtils.getYawToPose(robotPose.get(), tagPose2d);
        yawValue = returnYaw.getDegrees();
        SmartDashboard.putNumber("Raw Target Yaw", yawValue);
      }
    }

    return yawValue;
  }

  // for the DriveTargetCommand but still kinda useless
  public boolean isTargetVisible(int id) {
    boolean targetIsVisible = false;

    if (cameraConnected && !allUnreadResults.isEmpty()) {
      var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

      if (latestResult.hasTargets()) {
        for (var target : latestResult.getTargets()) {
          if (target.getFiducialId() == id) {
            targetIsVisible = true;
            break;
          }
        }
      }
    }

    return targetIsVisible;
  }

  // Gets the target range
  public double getTargetRange(int id) {

    double rangeValue = Double.NaN; // Default to NaN
    var tagPose = layout.getTagPose(id);

    if (tagPose.isPresent()) {
      Optional<Pose2d> robotPose = getRobotPose();

      if (robotPose.isPresent()) {
        rangeValue = PhotonUtils.getDistanceToPose(
            robotPose.get(),
            tagPose.get().toPose2d());
        SmartDashboard.putNumber("Raw Target Range", rangeValue);
      }
    }
    return rangeValue;
  }

  // Updates the robot pose using the pose estimator (periodic)
  private void updateRobotPoseEstimate() {

    // Start from most recent result
    for (int resultsIndex = allUnreadResults.size() - 1; resultsIndex >= 0; resultsIndex--) {
      var result = allUnreadResults.get(resultsIndex);

      if (result.hasTargets()) {
        boolean hasValidTags = false;
        for (var target : result.getTargets()) {

          // Target has to have a certain ambiguity and a valid tag pose
          if (layout.getTagPose(target.getFiducialId()).isPresent() &&
              target.getPoseAmbiguity() < maxAmbiguity) {
            hasValidTags = true;
            break;
          }
        }

        if (hasValidTags) {
          Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);

          if (estimatedRobotPose.isPresent()) {
            // Stores pose and logs
            storedEstimatedPose = estimatedRobotPose;
            Pose2d pose = estimatedRobotPose.get().estimatedPose.toPose2d();
            SmartDashboard.putNumber("Robot Pose X", pose.getX());
            SmartDashboard.putNumber("Robot Pose Y", pose.getY());
            SmartDashboard.putNumber("Robot Pose Heading", pose.getRotation().getDegrees());
          }
          break;
        }
      }
    }
  }

  // Used to ensure that the pose is not stale
  public boolean isFreshPose() {
    if (!storedEstimatedPose.isPresent()) {
      return false;
    }

    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double poseTime = storedEstimatedPose.get().timestampSeconds;
    return (currentTime - poseTime) < maxPoseAge;
  }

  // Converts 3d pose to 2d pose and gets it
  public Optional<Pose2d> getRobotPose() {
    if (storedEstimatedPose.isPresent()) {
      return Optional.of(storedEstimatedPose.get().estimatedPose.toPose2d());
    }

    return Optional.empty();
  }

  // Method to get the timestamp of the latest pose
  public double getTimestamp() {

    double timestamp = -1;

    if (storedEstimatedPose.isPresent()) {
      EstimatedRobotPose estimatedRobotPose = storedEstimatedPose.get();
      timestamp = estimatedRobotPose.timestampSeconds;
    }

    return timestamp;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if (cameraConnected) {
      allUnreadResults = camera.getAllUnreadResults();
      // Update the robot pose estimate using the estimator
      updateRobotPoseEstimate();

      if (!allUnreadResults.isEmpty() && allUnreadResults.get(allUnreadResults.size() - 1).hasTargets()) {
        var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        for (PhotonTrackedTarget target : latestResult.getTargets()) {

          double yaw = target.getYaw();
          double pitch = target.getPitch();
          double area = target.getArea();
          double skew = target.getSkew();
          double id = target.getFiducialId();
          double ambiguity = target.getPoseAmbiguity();
          Transform3d transform = target.getBestCameraToTarget();
          double x = transform.getTranslation().getX();
          double y = transform.getTranslation().getY();
          double z = transform.getTranslation().getZ();

          yawEntry.setDouble(yaw);
          pitchEntry.setDouble(pitch);
          areaEntry.setDouble(area);
          skewEntry.setDouble(skew);
          idEntry.setDouble(id);
          ambiguityEntry.setDouble(ambiguity);
          xEntry.setDouble(x);
          yEntry.setDouble(y);
          zEntry.setDouble(z);

          // To lazy to make this shufflboard entries
          SmartDashboard.putBoolean("Has Valid Pose", storedEstimatedPose.isPresent());
          SmartDashboard.putNumber("Pose Timestamp", getTimestamp());

          // Updates using the fresh pose in SmartDashboard when present
          if (isFreshPose()) {
            Pose2d pose = getRobotPose().orElse(new Pose2d());
            SmartDashboard.putNumber("Fresh Pose X", pose.getX());
            SmartDashboard.putNumber("Fresh Pose Y", pose.getY());
            SmartDashboard.putNumber("Fresh Pose Heading", pose.getRotation().getDegrees());
          } else {
            SmartDashboard.putString("Pose Status", "Stale Pose");
          }

        }
      }
    }
  }
}

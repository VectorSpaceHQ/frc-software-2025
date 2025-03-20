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

  // Command to get the target yaw
  public double getTargetYaw(int id) {

    Rotation2d returnYaw = new Rotation2d();
    double returnYawDouble = 0.0;
    var tagPose = layout.getTagPose(id);
    Pose2d tagPose2d = tagPose.get().toPose2d();
    
    Optional<Pose2d> robotPose = getRobotPose();
    if (robotPose.isPresent()) {
        returnYaw = PhotonUtils.getYawToPose(robotPose.get(), tagPose2d);
    }
    returnYawDouble = returnYaw.getDegrees();
            
    return returnYawDouble;
  
  }

  // Basically useless (repetitive)
  public boolean isTargetVisible(double id) {
    boolean result = false;

    if (!allUnreadResults.isEmpty()) {
      var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

      if (latestResult.hasTargets()) {
        for (var target : latestResult.getTargets()) {
          double tagID = target.getFiducialId();
          if (tagID == id) {
            result = true;
            break;
          }
        }
      }
    }
    return result;
  }

  // Command to get the target range
  public double getTargetRange(double id) {

    double returnRange = -1.0;
    if (!allUnreadResults.isEmpty() && isTargetVisible(id)) {
      var latestResult = allUnreadResults.get(allUnreadResults.size() - 1);

        for (var target : latestResult.getTargets()) {
          double tagID = target.getFiducialId();

          if (id == tagID) {
            var tagPose = layout.getTagPose(target.getFiducialId());
            Pose2d tagPose2d = tagPose.get().toPose2d();
            if (tagPose.isPresent()) {
              Optional<Pose2d> robotPose = getRobotPose();
              if (robotPose.isPresent()) {
                returnRange = PhotonUtils.getDistanceToPose(robotPose.get(), tagPose2d);
              }
              // PhotonUtils.calculateDistanceToTargetMeters(0.228, // Measured with a tape measure or in
              //                                                                  // CAD.
              //     tagPose.get().getTranslation().getZ(),
              //     Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
              //     Units.degreesToRadians(target.getPitch()));

              break;
            }
          }
        }
      }
    return returnRange;
  }

  private void updateRobotPoseEstimate() {
    // Start from most recent result
    for (int resultsIndex = allUnreadResults.size() - 1; resultsIndex >= 0; resultsIndex--) {
      var result = allUnreadResults.get(resultsIndex);

      if (result.hasTargets()) {
        boolean hasValidTags = false;
        for (var target : result.getTargets()) {

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
          }
          break;
        }
      }
    }
  }


  // Converts 3d pose to 2d pose and gets it
  public Optional<Pose2d> getRobotPose() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
        var target = result.getBestTarget();
        var tagPose = layout.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {
            Optional<EstimatedRobotPose> optionalPose = poseEstimator.update(result);
            if (optionalPose.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = optionalPose.get();
                Pose3d estimatedRobotPose3d = estimatedRobotPose.estimatedPose;
                Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();
                return Optional.of(estimatedRobotPose2d);
            }
        }
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

        }
      }
    }
  }
}

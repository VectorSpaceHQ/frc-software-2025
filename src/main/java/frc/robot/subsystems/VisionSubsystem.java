package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;

public class VisionSubsystem extends SubsystemBase {
  // Strategy for processing multiple AprilTags on the coprocessor using multiple
  // teams
  public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_PROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  // Constants for the camera name and field layout path
  private static final String camera_name = "Arducam_OV9281_USB_Camera";
  private String field_layout_path = new File(Filesystem.getDeployDirectory(), "2025-reefscape.json")
      .getAbsolutePath();

  // Photon camera and april tag components
  private final PhotonCamera camera;
  private AprilTagFieldLayout layout;
  private PhotonPoseEstimator poseEstimator;

  // Transformation3d objects for the camera and robot
  private static final Transform3d cameraToRobot = new Transform3d(
      new Translation3d(0.0, 0.0, 0.0), // Translation from camera to robot center
      new Rotation3d(0.0, 0.0, 0.0)); // Rotation from camera to robot center

  // Inverse transform of cameraToRobot
  private Transform3d robotToCamera = cameraToRobot.inverse();

  // Status of the camera
  private boolean cameraConnected = false;

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
  private GenericEntry poseEntry;
  private GenericEntry pose3dEntry;
  private GenericEntry pose2dEntry;

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
    poseEntry = visionTab.add("Pose", 0.0).getEntry();
  }

  // Command to get the target yaw
  public double getTargetYaw(int id) {
    var results = camera.getAllUnreadResults();
    double returnYaw = 0.0;

    if (!results.isEmpty()) {
      var latestResult = results.get(results.size() - 1);

      if (latestResult.hasTargets()) {
        for (var target : latestResult.getTargets()) {
          if (target.getFiducialId() == id) {
            returnYaw = target.getYaw();
            break;
          }
        }
      }
    }
    return returnYaw;
  }

  // Command to get the target range
  public double getTargetRange(int id) {
    var results = camera.getAllUnreadResults();
    double returnRange = 0.0;

    if (!results.isEmpty()) {
      var latestResult = results.get(results.size() - 1);

      if (latestResult.hasTargets()) {
        for (var target : latestResult.getTargets()) {
          if (target.getFiducialId() == id) {
            var tagPose = layout.getTagPose(target.getFiducialId());
            returnRange = PhotonUtils.calculateDistanceToTargetMeters(0.5, // Measured with a tape measure or in CAD.
                tagPose.get().getTranslation().getZ(),
                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));

            break;
          }
        }
      }
    }
    return returnRange;
  }

  // Converts 3d pose to 2d pose
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
          // Use this 2d pose as the added vision measurement for the pose estimator

        }

      }
    }

    return Optional.empty();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (cameraConnected) {
      var results = camera.getLatestResult();
      List<PhotonTrackedTarget> targets = results.getTargets();

      if (results.hasTargets()) {
        for (PhotonTrackedTarget target : targets) {

          if (results.hasTargets()) {
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
            Optional<Pose3d> tagPose = layout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
              Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(results);

              if (estimatedRobotPose.isPresent()) {

                Pose3d estimatedRobotPose3d = estimatedRobotPose.get().estimatedPose;
                Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();

                yawEntry.setDouble(yaw);
                pitchEntry.setDouble(pitch);
                areaEntry.setDouble(area);
                skewEntry.setDouble(skew);
                idEntry.setDouble(id);
                ambiguityEntry.setDouble(ambiguity);
                xEntry.setDouble(x);
                yEntry.setDouble(y);
                zEntry.setDouble(z);
                poseEntry.setDouble(estimatedRobotPose.get().timestampSeconds);
                pose3dEntry.setValue(estimatedRobotPose3d);
                pose2dEntry.setValue(estimatedRobotPose2d);

              }

            }

          }
        }
      }
    }
  }
}

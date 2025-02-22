// These comments are for ME to understand

// Import statements for Java file operations, PhotonVision libraries, and WPILib classes
package frc.robot.subsystems;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;

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
    private static final String camera_name = "Front_Camera_Robot";
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
    // private GenericEntry pose3dEntry;
    private GenericEntry pose2dEntry;

    // Xbox controller for driver input
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Vision Subsystem constructor
    public VisionSubsystem() {
        // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
        camera = new PhotonCamera(camera_name);
    }

    // Try-catch block to catch any exceptions that may occur (such as a
    // disconnected camera)
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

    private void initializeShuffleBoard() {
        // Create a new Shuffleboard tab for the vision subsystem

        var results = camera.getAllUnreadResults();

        var frame = results.get(0);

        List<PhotonTrackedTarget> targets = frame.getTargets();

        if (results.size() > 0) {
            for (PhotonTrackedTarget target : targets) {
                String prefix = "Tag " + target.getFiducialId() + " ";
                double yaw = target.getYaw(); // Horizontal angle
                double pitch = target.getPitch(); // Vertical angle
                double area = target.getArea(); // Size in image (percentage)
                double skew = target.getSkew(); // Rotation
                double id = target.getFiducialId(); // AprilTag ID
                double ambiguity = target.getPoseAmbiguity(); // Ambiguity of the pose
                Transform3d transform = target.getBestCameraToTarget(); // Maps camera space to target (april tag) space
                // double x = transform.getTranslation().getX(); // X position translation
                // double y = transform.getTranslation().getY(); // Y position translation
                // double z = transform.getTranslation().getZ(); // Z position translation

                // displayed on SmartDashboard
                ShuffleboardTab visionTab = Shuffleboard.getTab(prefix);

                yawEntry = visionTab.add("Yaw", yaw).getEntry();
                pitchEntry = visionTab.add("Pitch", pitch).getEntry();
                skewEntry = visionTab.add("Skew", skew).getEntry();
                areaEntry = visionTab.add("Area", area).getEntry();
                idEntry = visionTab.add("ID", id).getEntry();
                ambiguityEntry = visionTab.add("Ambiguity", ambiguity).getEntry();
                xEntry = visionTab.add("X", transform.getTranslation().getX()).getEntry();
                yEntry = visionTab.add("Y", transform.getTranslation().getY()).getEntry();
                zEntry = visionTab.add("Z", transform.getTranslation().getZ()).getEntry();
                // Gets tag pose and measurements
                Optional<Pose3d> tagPose = layout.getTagPose(target.getFiducialId());

                // Calculates robot pose if tag position is known

            }
        }

    }

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

    public double getTargetRange(int id) {
        var results = camera.getAllUnreadResults();
        double returnRange = 0.0;
        
        
        if (!results.isEmpty()) {
            var latestResult = results.get(results.size() - 1);
            if (latestResult.hasTargets()) {
            
                for (var target : latestResult.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        returnRange = PhotonUtils.calculateDistanceToTargetMeters(0.5, // Measured with a tape measure, or in CAD.
                        1.435, // From 2024 game manual for ID 7
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

                double timestamp = estimatedRobotPose.timestampSeconds;
                   
                    Pose3d estimatedRobotPose3d = estimatedRobotPose.estimatedPose;
                    Pose2d estimatedRobotPose2d = new Pose2d(estimatedRobotPose3d.getTranslation().toTranslation2d(),
                            estimatedRobotPose3d.getRotation().toRotation2d());
                    // pose3dEntry = visionTab.add("EstimatedRobotPose",
                    // estimatedRobotPose3d).getEntry();

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

            if (results.hasTargets()) {

                var target = results.getBestTarget();
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
                        Pose2d estimatedRobotPose2d = new Pose2d(
                                estimatedRobotPose3d.getTranslation().toTranslation2d(),
                                estimatedRobotPose3d.getRotation().toRotation2d());
                        // pose3dEntry = visionTab.add("EstimatedRobotPose",
                        // estimatedRobotPose3d).getEntry();

                        pitchEntry.setDouble(pitch);
                        areaEntry.setDouble(area);
                        skewEntry.setDouble(skew);
                        idEntry.setDouble(id);
                        ambiguityEntry.setDouble(ambiguity);
                        xEntry.setDouble(x);
                        yEntry.setDouble(y);
                        zEntry.setDouble(z);
                        pose2dEntry.setDouble(estimatedRobotPose2d.getRotation().getDegrees());

                    }

                }

            }
        }
    }
}

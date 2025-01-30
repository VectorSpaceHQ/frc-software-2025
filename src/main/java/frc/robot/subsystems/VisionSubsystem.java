// These comments are for ME to understand

// Import statements for Java file operations, PhotonVision libraries, and WPILib classes
package frc.robot.subsystems;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


 // Trash Subsystem for Vision (Not done yet, might need to add more code idk)
public class VisionSubsystem extends SubsystemBase {
    // Strategy for processing multiple AprilTags on the coprocessor using multiple teams
    public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_PROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    // Constants for the camera name and field layout path
    private static final String camera_name = "Front_Camera_Robot";
    private static final String field_layout_path = "src\\main\\resources\\2025-reefscape.json";

    // Photon camera and april tag components
    private final PhotonCamera camera;
    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator poseEstimator;

    // Transformation3d objects for the camera and robot
    private Transform3d cameraToRobot; // Physical offset from camera to robot center (whatever that may be)
    private Transform3d robotToCamera; // Inverse transform of cameraToRobot

    // Status of the camera
    private boolean cameraConnected;

     // Vision Subsystem constructor
    public VisionSubsystem() {
        // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
        camera = new PhotonCamera(camera_name);
        // Try-catch block to catch any exceptions that may occur (such as a disconnected camera)
        try { 
            cameraConnected = camera.isConnected();
            if (!cameraConnected) {
                System.err.println("Warning: Camera not connected, idiot.");
                return;
            }

            // Initialize transformation3d objects for the camera and robot
            cameraToRobot = new Transform3d(); // Default transform (probably 0,0,0)
            robotToCamera = cameraToRobot.inverse();

            // AprilTag Field Layout Setup (initialize the field layout)
            initializeAprilTagFieldLayout();
        } catch (Exception e) {
            System.err.println("Error initializing camera: " + e.getMessage());
            cameraConnected = false;
        }
    }


    // Initializes the AprilTag field layout from the JSON file containing the 2025 layout
    private void initializeAprilTagFieldLayout() {
        try { // Try-catch block to catch any exceptions that may occur (such as a missing JSON file)
            Path path = Paths.get(getClass().getResource(field_layout_path).toURI());
            layout = new AprilTagFieldLayout(path);

            // Origin Point
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            // Initialize pose estimator
            poseEstimator = new PhotonPoseEstimator(layout, MULTI_TAG_PNP_ON_PROCESSOR, robotToCamera);
        } catch (IOException | URISyntaxException e) {
            e.printStackTrace();
        }
    }

    // Periodic method runs repeatedly, processes vision telemetric data, and updates SmartDashboard
    @Override
    public void periodic() {
        try {
            var results = camera.getAllUnreadResults();
            if (results.size() > 0) {
                var frame = results.get(0);
                List<PhotonTrackedTarget> targets = frame.getTargets();
                boolean hasTargets = frame.hasTargets();

                if (hasTargets) {
                    for (PhotonTrackedTarget target : targets) {
                        String prefix = "Tag " + target.getFiducialId() + " "; // So that we can have multiple unique tags displayed on SmartDashboard

                        // Gets tag pose and measurements
                        Optional<Pose3d> tagPose = layout.getTagPose(target.getFiducialId());
                        double yaw = target.getYaw(); // Horizontal angle
                        double pitch = target.getPitch(); // Vertical angle
                        double area = target.getArea(); // Size in image (percentage)
                        double skew = target.getSkew(); // Rotation
                        double id = target.getFiducialId(); // AprilTag ID
                        Transform3d transform = target.getBestCameraToTarget(); // Maps camera space to target (april tag) space

                        // Calculates robot pose if tag position is known
                        if (layout.getTagPose(target.getFiducialId()).isPresent()) {
                            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                    target.getBestCameraToTarget(),
                                    layout.getTagPose(target.getFiducialId()).get(),
                                    cameraToRobot); // Not sure if it is cameraToRobot or robotToCamera
                            SmartDashboard.putData("RobotPose", (Sendable) robotPose); // Sends the robot pose to SmartDashboard
                        }

                        // Additional Random Measurements
                        double ambiguity = target.getPoseAmbiguity(); // Ambiguity of the pose
                        double x = transform.getTranslation().getX(); // X position translation
                        double y = transform.getTranslation().getY(); // Y position translation
                        double z = transform.getTranslation().getZ(); // Z position translation

                        // Sends all data to SmartDashboard with unique prefixes
                        tagPose.ifPresent(
                                tagPoseValue -> SmartDashboard.putData("TagPoseValue", (Sendable) tagPoseValue)); // Tag pose value
                        SmartDashboard.putNumber(prefix + "Yaw", yaw);
                        SmartDashboard.putNumber(prefix + "Pitch", pitch);
                        SmartDashboard.putNumber(prefix + "Area", area);
                        SmartDashboard.putNumber(prefix + "Skew", skew);
                        SmartDashboard.putData(prefix + "Transform3d", (Sendable) transform);
                        SmartDashboard.putNumber(prefix + "Ambiguity", ambiguity);
                        SmartDashboard.putNumber(prefix + "ID", id);
                        SmartDashboard.putNumber(prefix + "X", x);
                        SmartDashboard.putNumber(prefix + "Y", y);
                        SmartDashboard.putNumber(prefix + "Z", z);
                        SmartDashboard.putBoolean(prefix + "Camera Connected", cameraConnected);

                        // Updates pose estimator with new data (periodic)
                        poseEstimator.update(frame);
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("Error in periodic vision update: " + e.getMessage());
            cameraConnected = false;
        }
    }

     // Optional returns a value if present
    public Optional<Pose3d> getRobotPose() {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        var result = results.get(0);
        return poseEstimator.update(result).map(pose -> pose.estimatedPose);
    }

     // Returns true if camera is connected, false if not
    public boolean isCameraConnected() {
        return cameraConnected;
    }
}
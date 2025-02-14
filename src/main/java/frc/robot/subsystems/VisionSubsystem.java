// These comments are for ME to understand

// Import statements for Java file operations, PhotonVision libraries, and WPILib classes
package frc.robot.subsystems;


import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.XboxController;

import java.io.File;
import java.io.IOException;

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

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;


// Trash Subsystem for Vision (Not done yet, might need to add more code idk)
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
    private Transform3d cameraToRobot; // Physical offset from camera to robot center (whatever that may be)
    private Transform3d robotToCamera; // Inverse transform of cameraToRobot

    // Status of the camera
    private boolean cameraConnected;

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

    // Xbox controller for driver input
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Vision Subsystem constructor
    public VisionSubsystem() {
        // Initialize camera with name matching PhotonVision GUI (HAS TO MATCH)
        camera = new PhotonCamera(camera_name);
        // Try-catch block to catch any exceptions that may occur (such as a
        // disconnected camera)
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

    // Initializes the AprilTag field layout from the JSON file containing the 2025
    // layout
    private void initializeAprilTagFieldLayout() {
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
                if (layout.getTagPose(target.getFiducialId()).isPresent()) {

                    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            target.getBestCameraToTarget(),
                            layout.getTagPose(target.getFiducialId()).get(),
                            cameraToRobot);
                    poseEntry = visionTab.add("RobotPose", robotPose).getEntry();
                }

                // Not sure if it is cameraToRobot or robotToCamera
                // Shuffleboard.getTab(prefix).add("RobotPose", robotPose); // Sends the robot
                // pose to SmartDashboard
            }
        }

    }

    public void teleopPeriodic() {
        double forward = -m_driverController.getLeftY() * AutoConstants.kMaxSpeedMetersPerSecond;
        double strafe = -m_driverController.getRightX() * AutoConstants.kMaxSpeedMetersPerSecond;
        double turn = -m_driverController.getLeftX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond;

        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (results.size() > 0) {
            var result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        targetYaw = target.getYaw();
                    }
                }

            }

        }
        if (m_driverController.getAButton() && targetVisible) {
            // If the A button is pressed and a target is visible, drive towards the target
            turn = -1.0 * targetYaw * 0.1 * AutoConstants.kMaxAngularSpeedRadiansPerSecond; // Adjust the turn speed as
                                                                                            // needed
        }
        

        SmartDashboard.putBoolean("Target Visible", targetVisible); // SmartDashboard Target Visibility
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
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                tagPose.get(),
                cameraToRobot);
                poseEntry.setDouble(robotPose.getTranslation().getX()); // Example: setting X
                }
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

        // Sends all data to SmartDashboard with unique prefixes

        // Shuffleboard.getTab(prefix).add("Z", z);
        // Shuffleboard.getTab(prefix).add("Camera Connected", cameraConnected);
        // SmartDashboard.putBoolean(prefix + "Camera Connected", cameraConnected); //
        // Sends camera connection status to SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag ID", id); // Sends tag ID to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Area", area); // Sends tag area to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Yaw", yaw); // Sends tag yaw to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Pitch", pitch); // Sends tag pitch to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Skew", skew); // Sends tag skew to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Ambiguity", ambiguity); // Sends tag
        // ambiguity to SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag X", x); // Sends tag X position to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Y", y); // Sends tag Y position to
        // SmartDashboard
        // SmartDashboard.putNumber(prefix + "Tag Z", z); // Sends tag Z position to
        // SmartDashboard

        // cameraConnected = false;
    }

}

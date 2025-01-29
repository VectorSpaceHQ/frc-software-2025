package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera;
    public static final PhotonPoseEstimator.PoseStrategy MULTI_TAG_PNP_ON_PROCESSOR = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    
    public VisionSubsystem() {
        camera = new PhotonCamera("Front_Camera_Robot");
        }

    @Override
    public void periodic() {

        var result = camera.getAllUnreadResults();
        
        if(result.size() > 0) {

            var frame = result.get(0);
            List<PhotonTrackedTarget> targets = frame.getTargets();
            boolean hasTargets = frame.hasTargets();

            if (hasTargets) {

                for (PhotonTrackedTarget target : targets) {
                
                    double yaw = target.getYaw();
                    double pitch = target.getPitch();
                    double area = target.getArea();
                    double skew = target.getSkew();
                    double id = target.getFiducialId();
                    Transform3d transform = target.getBestCameraToTarget();
                    double x = transform.getTranslation().getX();
                    double y = transform.getTranslation().getY();
                    double z = transform.getTranslation().getZ();

                    SmartDashboard.putNumber("Yaw", yaw);
                    SmartDashboard.putNumber("Pitch", pitch);
                    SmartDashboard.putNumber("Area", area);
                    SmartDashboard.putNumber("Skew", skew);
                    SmartDashboard.putNumber("ID", id);
                    SmartDashboard.putNumber("X", x);
                    SmartDashboard.putNumber("Y", y);
                    SmartDashboard.putNumber("Z", z);

                }
            }
        }

    }
}

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera("Front_Camera_Robot");

    public VisionSubsystem() {

        }

    @Override
    public void periodic() {
        var result = camera.getAllUnreadResults();
        var frame = result.get(0);
        List<PhotonTrackedTarget> targets = frame.getTargets();

        }

@Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

}

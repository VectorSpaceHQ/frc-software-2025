package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTags;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class GetAlgaeCommand extends Command{
    private DriveTargetCommand Target = null;
    private AlgaeSubsystem Algae = null;
    private ElevatorSubsystem Elevator = null;
    private String kReefID;

    private Map<String , ElevatorSubsystem.Level> algaeMap = new HashMap<>();
    private Map<String, AprilTags> targetMap;

    public GetAlgaeCommand(DriveTargetCommand aimTarget, AlgaeSubsystem m_robotAlgae, ElevatorSubsystem m_robotElevator, String reefID, Map<String, AprilTags> fieldMap) {
        Target = aimTarget;
        Algae = m_robotAlgae;
        Elevator = m_robotElevator;
        targetMap = fieldMap;
        kReefID = reefID;
        addRequirements(Algae, Elevator);

        algaeMap.put("reef1", Level.L3);
        algaeMap.put("reef2", Level.L2);
        algaeMap.put("reef3", Level.L3);
        algaeMap.put("reef4", Level.L2);
        algaeMap.put("reef5", Level.L3);
        algaeMap.put("reef6", Level.L2);
    }

    @Override
    public void initialize() {
        Target.setTargetID(targetMap.get(kReefID));
        Elevator.GoTo(algaeMap.get(kReefID));
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

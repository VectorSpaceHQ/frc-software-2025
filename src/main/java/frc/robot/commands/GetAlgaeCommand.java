package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AprilTags;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class GetAlgaeCommand extends Command{
    DriveTargetCommand Target = null;

    AlgaeSubsystem Algae = null;
    ElevatorSubsystem Elevator = null;

    public GetAlgaeCommand(DriveTargetCommand aimTarget, AlgaeSubsystem m_robotAlgae, ElevatorSubsystem m_robotElevator) {
        Target = aimTarget;
        Algae = m_robotAlgae;
        Elevator = m_robotElevator;
        addRequirements(Algae, Elevator);
    }

    @Override
    public void execute() {

    }

    public Command getAlgae(Level Height, AprilTags tagID){
        return new SequentialCommandGroup(Target.withTimeout(2), Elevator.GoTo(Height).withTimeout(2), Algae.runClaws(.2).withTimeout(2));
    }
}

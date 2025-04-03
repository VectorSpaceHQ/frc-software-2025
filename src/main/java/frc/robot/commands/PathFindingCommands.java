package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivePoses;
import frc.robot.Constants.PathLimitations;
import frc.robot.subsystems.DriveSubsystem;

public class PathFindingCommands {
    private DriveSubsystem m_DriveSubsystem = null;

    public PathFindingCommands(DriveSubsystem m_robotDrive){
        m_DriveSubsystem = m_robotDrive;
    }

    public Command GoToBlueBarge() {
        var result = AutoBuilder.pathfindToPose(DrivePoses.BlueBargePose, PathLimitations.constraints, 0);
        result.addRequirements(m_DriveSubsystem);
        return result;
    }

}

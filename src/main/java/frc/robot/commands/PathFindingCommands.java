package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivePoses;
import frc.robot.Constants.PathLimitations;

public class PathFindingCommands {

    public Command GoToBlueBarge() {
        return AutoBuilder.pathfindToPose(DrivePoses.BlueBargePose, PathLimitations.constraints, 0);
    }
    
}

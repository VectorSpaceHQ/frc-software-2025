package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        return result;
    }
    public Command forwardTest() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        return AutoBuilder.followPath(path).beforeStarting(() -> m_DriveSubsystem.resetOdometry(path.getStartingDifferentialPose()));
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    }
}
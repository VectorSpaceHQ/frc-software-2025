package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MecanumControllerCommandGenerator {
    
    DriveSubsystem mDriveSubsystem = null;
    MecanumDriveKinematics mDriveKinematics = null;
    Trajectory mTrajectory = null;


    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private final ProfiledPIDController headingController = new ProfiledPIDController(1, 0.0, 0.0, new Constraints(360, 360));
    private final double maxWheelVelocityMetersPerSecond = 4.6;

    public MecanumControllerCommandGenerator(Trajectory traj, DriveSubsystem m_RobotDrive) {
        mDriveSubsystem = m_RobotDrive;
        mTrajectory = traj;
        mDriveKinematics = mDriveSubsystem.getMecanumDriveKinematics();
        headingController.enableContinuousInput(0, 360);
    }

    public MecanumControllerCommand getCommand() {
        return new MecanumControllerCommand(mTrajectory, () -> mDriveSubsystem.getPose(), mDriveKinematics, xController, yController, headingController, null, maxWheelVelocityMetersPerSecond, mDriveSubsystem::driveMecanumWheelSpeeds, mDriveSubsystem);
    }


    

}

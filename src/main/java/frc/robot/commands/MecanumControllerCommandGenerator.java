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


    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private final ProfiledPIDController headingController = new ProfiledPIDController(2, 0.0, 0.0, new Constraints(30, 4));
    private final double maxWheelVelocityMetersPerSecond = 4.6;

    public MecanumControllerCommandGenerator(Trajectory traj, DriveSubsystem m_RobotDrive) {
        mDriveSubsystem = m_RobotDrive;

        headingController.enableContinuousInput(0, 360);
    }

    public MecanumControllerCommand getCommand() {
        return null;
    }

    

}

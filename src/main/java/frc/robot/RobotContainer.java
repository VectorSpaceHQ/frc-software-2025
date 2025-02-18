// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_robotCoral = new CoralSubsystem();
  private final VisionSubsystem m_robotVision = new VisionSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final AlgaeSubsystem m_robotAlgae = new AlgaeSubsystem();
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_driverController.getLeftY(),
                    -m_driverController.getRightX(),
                    -m_driverController.getLeftX(),
                    false),
            m_robotDrive));
    m_robotAlgae.setDefaultCommand(
        // Left + Right Full Pressed = 0
        // Left Closes Right Opens (this result can be scaled down by a constant multiple if needed)
    
                m_robotAlgae.runClaws(
                    m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Spin Coral Discharge on Hold / Stop on release
    m_driverController
        .a()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .whileTrue(new InstantCommand(() -> m_robotCoral.releaseCoral()))
        .onFalse(new InstantCommand(() -> m_robotCoral.suspendCoral()));

    // Elevator to L2 - Add CMD in Feature Branch
    m_driverController
        .b()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L2));
    
    // Elevator to L3 - Add CMD in Feature Branch
    m_driverController
        .x()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L3));

    // Elevator to L4 - Add CMD in Feature Branch
    m_driverController
        .y()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L4));

    // Manually Raise Elevator - Add Function in Feature Branch
    m_driverController
        .back()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .whileTrue(m_robotElevator.ElevatorRaiseCommand());
    
    // Manually Lower Elevator - Add Function in Feature Branch
    m_driverController
        .start()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .whileTrue(m_robotElevator.ElevatorLowerCommand());

    // Go To Dispenser 1 (Left) - Add CMD in Feature Branch
    m_driverController
        .leftStick()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> System.out.println("ls")));
    
    // Go To Dispenser 2 (Right) - Add CMD in Feature Branch
    m_driverController 
        .rightStick()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> System.out.println("right")));

    // Go To Reef 1
    m_driverController
        .leftBumper()
        .and(m_driverController.a())
        .onTrue(new InstantCommand(() -> System.out.println("lb + a")));
    
    // Go To Reef 2
    m_driverController
        .leftBumper()
        .and(m_driverController.b())
        .onTrue(new InstantCommand(() -> System.out.println("lb + b")));
    
    // Go To Reef 3
    m_driverController
        .leftBumper()
        .and(m_driverController.x())
        .onTrue(new InstantCommand(() -> System.out.println("lb + x")));

    // Go To Reef 4
    m_driverController
        .leftBumper()
        .and(m_driverController.y())
        .onTrue(new InstantCommand(() -> System.out.println("lb + y")));
    
    // Go To Reef 5
    m_driverController
        .leftBumper()
        .and(m_driverController.back())
        .onTrue(new InstantCommand(() -> System.out.println("lb + back")));
    
    // Go To Reef 6
    m_driverController
    .leftBumper()
    .and(m_driverController.start())
    .onTrue(new InstantCommand(() -> System.out.println("lb + start")));

    // Go To Reef 7
    m_driverController
        .rightBumper()
        .and(m_driverController.a())
        .onTrue(new InstantCommand(() -> System.out.println("rb + a")));
    
    // Go To Reef 8
    m_driverController
        .rightBumper()
        .and(m_driverController.b())
        .onTrue(new InstantCommand(() -> System.out.println("rb + b")));
    
    // Go To Reef 9
    m_driverController
        .rightBumper()
        .and(m_driverController.x())
        .onTrue(new InstantCommand(() -> System.out.println("rb + x")));

    // Go To Reef 10
    m_driverController
        .rightBumper()
        .and(m_driverController.y())
        .onTrue(new InstantCommand(() -> System.out.println("rb + y")));
    
    // Go To Reef 11
    m_driverController
        .rightBumper()
        .and(m_driverController.back())
        .onTrue(new InstantCommand(() -> System.out.println("rb + back")));
    
    // Go To Reef 12
    m_driverController
    .rightBumper()
    .and(m_driverController.start())
    .onTrue(new InstantCommand(() -> System.out.println("rb + start")));

    // Homing Routines
    m_driverController
    .rightBumper()
    .and(m_driverController.leftBumper())
    .and(m_driverController.a())
    .onTrue(m_robotElevator.Homing())
    .onTrue(new InstantCommand(() -> System.out.println("Homing Process B")));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Pose2d.kZero,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.kZero),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_robotDrive::getCurrentWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_robotDrive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        mecanumControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  }
}
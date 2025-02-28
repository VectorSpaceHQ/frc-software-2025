// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import choreo.auto.AutoFactory;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Optional<Trajectory<SwerveSample>> trajectory1;
  private Optional<Trajectory<SwerveSample>> trajectory2;
  private Optional<Trajectory<SwerveSample>> trajectory3;
  private Optional<Trajectory<SwerveSample>> trajectory4;
  private final Timer timer = new Timer();
  private AutoFactory autoFactory;
  private final RobotContainer m_robotContainer;
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    trajectory1 = Choreo.loadTrajectory("2-Piece Coral Auto Part 1");
    trajectory2 = Choreo.loadTrajectory("2-Piece Coral Auto Part 2");
    trajectory3 = Choreo.loadTrajectory("2-Piece Coral Auto Part 3");
    trajectory4 = Choreo.loadTrajectory("2-Piece Coral Auto Part 4");
    autoFactory = new AutoFactory(
        m_driveSubsystem::getPose, // A function that returns the current robot pose
        m_driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
        m_driveSubsystem::followTrajectory, // The drive subsystem trajectory follower
        true, // Alliance Switch

        m_driveSubsystem // The drive subsystem
    );
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = coralAutoCommand();
    if (trajectory1.isPresent()) {
      Optional<Pose2d> initialPose = trajectory1.get().getInitialPose(isRedAlliance());
      if (initialPose.isPresent()) {
        m_driveSubsystem.resetOdometry(initialPose.get());
      }
    }

    timer.restart();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (trajectory1.isPresent()) {
      Optional<SwerveSample> sample = trajectory1.get().sampleAt(timer.get(), isRedAlliance()); //Assuming if we are on the red side
      if (sample.isPresent()) {
        m_driveSubsystem.followTrajectory(sample.get());
      }
    }
    CommandScheduler.getInstance().run();
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public Command coralAutoCommand() {
    return Commands.sequence(
        autoFactory.trajectoryCmd("2-Piece Coral Auto Part 1"),
        m_elevatorSubsystem.GoTo(ElevatorSubsystem.Level.Bottom).withTimeout(1),
        m_coralSubsystem.runCoralDispenser().withTimeout(1),
        autoFactory.trajectoryCmd("2-Piece Coral Auto Part 2"),
        m_elevatorSubsystem.GoTo(ElevatorSubsystem.Level.L3).withTimeout(1),
        m_coralSubsystem.runCoralDispenser().withTimeout(1),
        autoFactory.trajectoryCmd("2-Piece Coral Auto Part 3"),
        m_elevatorSubsystem.GoTo(ElevatorSubsystem.Level.Bottom).withTimeout(1),
        m_coralSubsystem.runCoralDispenser().withTimeout(1),
        autoFactory.trajectoryCmd("2-Piece Coral Auto Part 4"),
        m_elevatorSubsystem.GoTo(ElevatorSubsystem.Level.L3).withTimeout(1));
}
}
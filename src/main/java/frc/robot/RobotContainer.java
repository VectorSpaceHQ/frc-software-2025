// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ComplexAuto;
import frc.robot.commands.DriveTargetCommand;
import frc.robot.commands.GetAlgaeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final CoralSubsystem m_robotCoral = new CoralSubsystem();
  private final VisionSubsystem m_robotVision = new VisionSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final AlgaeSubsystem m_robotAlgae = new AlgaeSubsystem();
  private final FieldTagMap fieldTagMap = new FieldTagMap();
  
  Map<String, AprilTags> fieldMap = fieldTagMap.getRedMap();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final DriveTargetCommand aimTarget = new DriveTargetCommand(m_robotDrive, m_robotVision, m_driverController ,m_robotElevator);
//   private final GetAlgaeCommand getAlgae = new GetAlgaeCommand(aimTarget, m_robotAlgae, m_robotElevator);
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(aimTarget);
    m_robotElevator.setDefaultCommand(m_robotElevator.ElevatorRaiseCommand(m_operatorController));
    m_robotAlgae.setDefaultCommand(m_robotAlgae.runClaws(m_operatorController));

    m_chooser.setDefaultOption("Simple Auto", getSimpleAutonomousCommand());
    m_chooser.addOption("Reef5", getReef5Command());
    m_chooser.addOption("Complex Auto", getComplexReef5Command());
    SmartDashboard.putData(m_chooser);

    // Default to red
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            fieldMap = fieldTagMap.getRedMap();
        }
        if (ally.get() == Alliance.Blue) {
            fieldMap = fieldTagMap.getBlueMap();
        }
    }
    
 }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController
        .leftBumper()
        .onTrue(new InstantCommand(() -> aimTarget.setSpeedScalar(0.3)))
        .onFalse(new InstantCommand(() -> aimTarget.setSpeedScalar(1)));

    // m_operatorController
    //     .b()
    //     .onTrue(m_robotElevator.GoTo(Level.L2));
    // Spin Coral Discharge on Hold / Stop on release
    // m_operatorController
    //     .a()
    //     .whileTrue(m_robotCoral.runCoralDispenser());

    // Elevator to L2 - Add CMD in Feature Branch
    m_operatorController
        .b()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L2));

    
    // Elevator to L3 - Add CMD in Feature Branch
    m_operatorController
        .x()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L3));

    // Elevator to L4 - Add CMD in Feature Branch
    // m_operatorController
    //     .y()
    //     .and(m_driverController.leftBumper().negate())
    //     .and(m_driverController.rightBumper().negate())
    //     .onTrue(m_robotElevator.GoTo(Level.L4));

    // Manually Raise Elevator - Add Function in Feature Branch
    m_operatorController
        .back()
        .and(m_operatorController.leftBumper().negate())
        .and(m_operatorController.rightBumper().negate());
        // .whileTrue(m_robotElevator.ElevatorLowerCommand());

    // Interrupts GoTo by Rescheduling Triggers
    m_operatorController
        .rightTrigger(0.1)
        .or(m_operatorController.leftTrigger(0.1))
        .onTrue(m_robotElevator.ElevatorRaiseCommand(m_operatorController));
    
    // Manually Lower Elevator - Add Function in Feature Branch
    m_operatorController
        .start()
        .and(m_operatorController.leftBumper().negate())
        .and(m_operatorController.rightBumper().negate());
        // .whileTrue(m_robotElevator.ElevatorRaiseCommand());

    // Go To Dispenser 1 (Left) - Add CMD in Feature Branch
    m_driverController
        .leftStick()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("coral1"))));
    
    // Go To Dispenser 2 (Right) - Add CMD in Feature Branch
    m_driverController 
        .rightStick()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("coral2"))));

    // Go To Reef 1
    m_driverController
        .leftBumper()
        .and(m_driverController.a())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef1"))));
    
    // Go To Reef 2
    m_driverController
        .leftBumper()
        .and(m_driverController.b())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef2"))));
    
    // Go To Reef 3
    m_driverController
        .leftBumper()
        .and(m_driverController.x())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef3"))));

    // Go To Reef 4
    m_driverController
        .leftBumper()
        .and(m_driverController.y())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef4"))));
    
    // Go To Reef 5
    m_driverController
        .leftBumper()
        .and(m_driverController.back())
        .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef5"))));
    
    // Go To Reef 6
    m_driverController
    .leftBumper()
    .and(m_driverController.start())
    .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef6"))));

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
    // .onTrue(m_robotElevator.Homing())
    .onTrue(m_robotAlgae.homeClaws());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getSimpleAutonomousCommand() {
    return m_robotDrive.run(() -> m_robotDrive.drive(-0.2, 0, 0 ,false)).withTimeout(2);
  }

  public Command getComplexReef5Command() {
    return new ComplexAuto(m_robotAlgae, m_robotElevator, aimTarget, fieldMap.get("reef5"));
  }
  
  public Command getReef5Command() {
    // aimTarget.setTargetID(fieldMap.get("reef5"));
    return aimTarget;
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

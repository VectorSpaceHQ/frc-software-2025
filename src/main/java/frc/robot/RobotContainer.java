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
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.RobotPoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem m_robotDrive = null;
  private CoralSubsystem m_robotCoral = null;
  private VisionSubsystem m_robotVision = null;
  private ElevatorSubsystem m_robotElevator = null;
  private AlgaeSubsystem m_robotAlgae = null;
  private FieldTagMap fieldTagMap = null;
  private IMUImpl m_IMU = null;
  private RuntimeParameters m_Parameters;
  private Map<String, AprilTags> fieldMap = null;
  private RobotPoseEstimatorSubsystem m_poseEstimator = null;

  private PathPlannerPath examplePath = null;
  
  // The driver's controller
  CommandXboxController m_driverController = null;
  CommandXboxController m_operatorController = null;

  private DriveTargetCommand aimTarget;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final ShuffleboardTab statusTab = Shuffleboard.getTab("Robot Status");
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
  private final ShuffleboardLayout sensorsCol = statusTab.getLayout("Sensors", BuiltInLayouts.kList).withPosition(0,0).withSize(1,4);
  private final ShuffleboardLayout poseCol = statusTab.getLayout("Pose Estimator", BuiltInLayouts.kList).withPosition(1,0).withSize(1,4);
  private final ShuffleboardLayout modeCol = statusTab.getLayout("Mode", BuiltInLayouts.kList).withPosition(2,0).withSize(1,3);
  private final GenericEntry gyroConnectedEntry = sensorsCol.add("Gyro Connected", false).getEntry();
  private final GenericEntry poseEstimatorActiveEntry = poseCol.add("Active", false).getEntry();
  private final GenericEntry poseEstimatorErrorEntry = poseCol.add("Error", "idk").getEntry();
  private final GenericEntry targetingModeEntry = modeCol.add("Targeting Mode", "Direct Vision Only").getEntry();
  private final GenericEntry allianceIsRedEntry = modeCol.add("Is Red", false).getEntry();
  private boolean autoChooserPublished = false;

  public RobotContainer() {
    // Initialize IMU because errors occur otherwise for some reason
    m_IMU = new IMUImpl();
    
    // Create controllers 
    m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    
    // Initialize subsystems (using feature toggles)
    if (Constants.FeatureToggles.enableMecanum) {
      m_robotDrive = new DriveSubsystem();

      // Set the gyro on the drive subsystem if available
      if (m_IMU != null) {
        m_robotDrive.setGyro(m_IMU);
                gyroConnectedEntry.setBoolean(true);
      } else {
                gyroConnectedEntry.setBoolean(false);
      }
    }

    if (Constants.FeatureToggles.enableCoral) {
      m_robotCoral = new CoralSubsystem();
    }

    if (Constants.FeatureToggles.enableVision) {
      m_robotVision = new VisionSubsystem();
    }

    if (Constants.FeatureToggles.enableScissorLift) {
      m_robotElevator = new ElevatorSubsystem();
    }

    if (Constants.FeatureToggles.enableAlgae) {
      m_robotAlgae = new AlgaeSubsystem();
    }

    if (Constants.FeatureToggles.enableRuntimeParams) {
      m_Parameters = new RuntimeParameters();
    }

    if (Constants.FeatureToggles.enablePoseEstimator) {
      try {
        m_poseEstimator = new RobotPoseEstimatorSubsystem(m_robotDrive, m_robotVision, m_IMU);
        poseEstimatorActiveEntry.setBoolean(true);
        poseEstimatorErrorEntry.setString("");

      } catch (Exception e) {
        System.err.println("Error initializing pose estimator: " + e.getMessage());

        poseEstimatorActiveEntry.setBoolean(false);
        poseEstimatorErrorEntry.setString(e.getMessage());
      }

    } else {
      poseEstimatorActiveEntry.setBoolean(false);
      poseEstimatorErrorEntry.setString("");
    }

    // Initialize FieldTagMap
    fieldTagMap = new FieldTagMap();
    
    // Determine alliance and set appropriate field map
    determineAlliance();
    
    // Configure controller bindings
    configureButtonBindings();

    // Setup default commands for subsystems
    setupDefaultCommands();

    // Setup autonomous command chooser
    setupAutonomousCommands();

    loadPaths();
}

  // Default commands
  private void setupDefaultCommands() {
    // Setup DriveTargetCommand as default (will change to a binding later)
    if (m_robotDrive != null && m_robotVision != null && m_driverController != null) {
      aimTarget = new DriveTargetCommand(m_robotDrive, m_driverController, m_robotElevator);

      // Connect pose estimator
      if (m_poseEstimator != null) {
        aimTarget.setPoseEstimator(m_poseEstimator);
                targetingModeEntry.setString("Using Pose Estimator");
      } else {
                targetingModeEntry.setString("Direct Vision Only");
      }

      m_robotDrive.setDefaultCommand(aimTarget);
    }

    // Setup default commands for other subsystems
    if (m_robotElevator != null) {
      m_robotElevator.setDefaultCommand(m_robotElevator.ElevatorRaiseCommand(m_operatorController));
    }

    if (m_robotAlgae != null) {
      m_robotAlgae.setDefaultCommand(m_robotAlgae.runClaws(m_operatorController));
    }
  }
  private void determineAlliance() {
    // Default to red alliance
    fieldMap = fieldTagMap.getRedMap();
  }

  private void setupAutonomousCommands() {
    if (m_robotDrive != null) {
      m_chooser.setDefaultOption("Simple Auto", getSimpleAutonomousCommand());
      m_chooser.addOption("Reef5", getReef5Command());
      m_chooser.addOption("Complex Auto", getComplexReef5Command());
    } else {
      // Add a dummy command when drive isn't available
      m_chooser.setDefaultOption("No Drive Available", new InstantCommand());
    }

    publishAutonomousChooser();
  }

  private void publishAutonomousChooser() {
    if (!autoChooserPublished) {
      autonomousTab.add("Autonomous Mode", m_chooser).withSize(2,1);
      autoChooserPublished = true;
    }
  }

  // Button bindings for driver and operator controllers
  private void configureButtonBindings() {
    m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    // Driver controller bindings
    
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
        .a()
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
    m_operatorController
        .y()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate())
        .onTrue(m_robotElevator.GoTo(Level.L4));

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

    // // Go To Reef 1
    // m_driverController
    //     .leftBumper()
    //     .and(m_driverController.a())
    //     .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef1"))));
    
    // // Go To Reef 2
    // m_driverController
    //     .leftBumper()
    //     .and(m_driverController.b())
    //     .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("reef2"))));
    
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

    // Manually control elevator
    m_operatorController
        .back()
        .and(m_operatorController.leftBumper().negate())
        .and(m_operatorController.rightBumper().negate());

    // Interrupts GoTo by Rescheduling Triggers
    m_operatorController
        .rightTrigger(0.1)
        .or(m_operatorController.leftTrigger(0.1))
        .onTrue(m_robotElevator.ElevatorRaiseCommand(m_operatorController));

    // Manually Lower Elevator
    m_operatorController
        .start()
        .and(m_operatorController.leftBumper().negate())
        .and(m_operatorController.rightBumper().negate());
  }
 
  // Autonomous commands
  public Command getSimpleAutonomousCommand() {
    return m_robotDrive.run(() -> m_robotDrive.drive(-0.2, 0, 0, false)).withTimeout(2);
  }

  public Command getComplexReef5Command() {
    return new ComplexAuto(m_robotAlgae, m_robotElevator, aimTarget, fieldMap.get("reef5"));
  }

  public Command getReef5Command() {
    return aimTarget;
  }

  public Command getAutonomousCommand() {
    if (m_robotDrive != null) {
      return m_chooser.getSelected();
    } else {
      return new InstantCommand(); // Return a default command if m_robotDrive is null
    }
  }

  private void loadPaths() {
    try {
    examplePath = PathPlannerPath.fromPathFile("Example Path");
    }
    catch(Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }
}


















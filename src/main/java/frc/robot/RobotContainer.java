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
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  // The driver's controller
  CommandXboxController m_driverController = null;
  CommandXboxController m_operatorController = null;

  private DriveTargetCommand aimTarget;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

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
        SmartDashboard.putBoolean("Gyro Connected to Drive", true);
      } else {
        SmartDashboard.putBoolean("Gyro Connected to Drive", false);
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
        SmartDashboard.putBoolean("Pose Estimator Active", true);

      } catch (Exception e) {
        System.err.println("Error initializing pose estimator: " + e.getMessage());

        SmartDashboard.putBoolean("Pose Estimator Active", false);
        SmartDashboard.putString("Pose Estimator Error", e.getMessage());
      }

    } else {
      SmartDashboard.putBoolean("Pose Estimator Active", false);
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
}

  // Default commands
  private void setupDefaultCommands() {
    // Setup DriveTargetCommand as default (will change to a binding later)
    if (m_robotDrive != null && m_robotVision != null && m_driverController != null) {
      aimTarget = new DriveTargetCommand(m_robotDrive, m_robotVision, m_driverController, m_robotElevator);

      // Connect pose estimator
      if (m_poseEstimator != null) {
        aimTarget.setPoseEstimator(m_poseEstimator);
        SmartDashboard.putString("Targeting Mode", "Using Pose Estimator");
      } else {
        SmartDashboard.putString("Targeting Mode", "Direct Vision Only");
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

  private void setupAutonomousCommands() {

    if (m_robotDrive != null) {
        m_chooser.setDefaultOption("Simple Auto", getSimpleAutonomousCommand());
        m_chooser.addOption("Reef5", getReef5Command());
        m_chooser.addOption("Complex Auto", getComplexReef5Command());
        SmartDashboard.putData(m_chooser);
    } else {
        // Add a dummy command when drive isn't available
        m_chooser.setDefaultOption("No Drive Available", new InstantCommand());
        SmartDashboard.putData(m_chooser);
    }
}
  // Determine the appropriate alliance
  private void determineAlliance() {
    // Default to red alliance
    fieldMap = fieldTagMap.getRedMap();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        fieldMap = fieldTagMap.getRedMap();
        SmartDashboard.putString("Alliance", "Red");
      }
      if (ally.get() == Alliance.Blue) {
        fieldMap = fieldTagMap.getBlueMap();
        SmartDashboard.putString("Alliance", "Blue");
      }
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

    // Configure april tag targeting buttons
    if (aimTarget != null) {
      // Go To Dispenser 1 (Left)
      m_driverController
          .leftStick()
          .and(m_driverController.leftBumper().negate())
          .and(m_driverController.rightBumper().negate())
          .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("coral1"))));

      // Go To Dispenser 2 (Right)
      m_driverController
          .rightStick()
          .and(m_driverController.leftBumper().negate())
          .and(m_driverController.rightBumper().negate())
          .onTrue(new InstantCommand(() -> aimTarget.setTargetID(fieldMap.get("coral2"))));

      // Configure reef targeting buttons
      configureReefTargetingButtons();
    }

    // Operator controller bindings for elevator
    if (m_robotElevator != null) {
      configureElevatorButtons();
    }

    // Homing routines
    if (m_robotAlgae != null) {
      m_driverController
          .rightBumper()
          .and(m_driverController.leftBumper())
          .and(m_driverController.a())
          .onTrue(m_robotAlgae.homeClaws());
    }
  }

  // Remaining methods unchanged...

  private void configureReefTargetingButtons() {

    if (m_poseEstimator != null) {

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
    }
  }

  // Elevator buttons
  private void configureElevatorButtons() {
    // Elevator to L2
    m_operatorController
        .b()
        .onTrue(m_robotElevator.GoTo(Level.L2));

    // Elevator to L3
    m_operatorController
        .x()
        .onTrue(m_robotElevator.GoTo(Level.L3));

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
}
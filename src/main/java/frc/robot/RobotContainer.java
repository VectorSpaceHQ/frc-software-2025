package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
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
import frc.robot.commands.PathFindingCommands;

import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private PathFindingCommands m_pathFindingCommands = null;
  private Field2d field = null;
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
      m_pathFindingCommands = new PathFindingCommands(m_robotDrive);
      // Set the gyro on the drive subsystem if available
      if (m_IMU != null) {
        m_robotDrive.setGyro(m_IMU);
        SmartDashboard.putBoolean("Gyro Connected to Drive", true);
      } else {
        SmartDashboard.putBoolean("Gyro Connected to Drive", false);
      }
      // Puts field to Smart Dashboard
      enableFieldLogging();
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
            SmartDashboard.putBoolean("IsRed", true);
        }
        if (ally.get() == Alliance.Blue) {
            fieldMap = fieldTagMap.getBlueMap();
            SmartDashboard.putBoolean("IsRed", false);
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

    m_driverController
        .a()
        .onTrue(m_pathFindingCommands.GoToBlueBarge());

    m_driverController
      .b()
      .onTrue(m_pathFindingCommands.forwardTest());
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
        .and(m_driverController.rightBumper().negate());
    
    // Go To Dispenser 2 (Right) - Add CMD in Feature Branch
    m_driverController 
        .rightStick()
        .and(m_driverController.leftBumper().negate())
        .and(m_driverController.rightBumper().negate());

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
        .and(m_driverController.x());

    // Go To Reef 4
    m_driverController
        .leftBumper()
        .and(m_driverController.y());
    
    // Go To Reef 5
    m_driverController
        .leftBumper()
        .and(m_driverController.back());
    
    // Go To Reef 6
    m_driverController
    .leftBumper()
    .and(m_driverController.start());

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
    return m_robotDrive.run(() -> m_robotDrive.driveRobotChassisSpeeds(new ChassisSpeeds(.75, 0, 0))).withTimeout(2);
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
    // try {
    // examplePath = PathPlannerPath.fromPathFile("Example Path");
    // }
    // catch(Exception e) {
    //   DriverStation.reportError(e.getMessage(), e.getStackTrace());
    // }
  }

  private void enableFieldLogging() {
    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
  }
}
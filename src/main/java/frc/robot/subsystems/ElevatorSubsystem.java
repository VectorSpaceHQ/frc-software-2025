package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DigitalInputPorts;
import frc.robot.Constants.ElevatorSpecifics;
import frc.robot.Constants.PIDTunings;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.lang.Math;



public class ElevatorSubsystem extends SubsystemBase{

  public enum Level {
    Bottom(ElevatorSpecifics.kInitialHeight),
    L2(31.875),
    L3(47.65),
    L4(72.), 
    Inspection(36)
    ;

    private double value;

    private Level(double height) {
      this.value = height;
    }

    public double getLevel() {
      return value;
    }
  };

  private final SparkMax motor1 = new SparkMax(CANIDs.kElevatorSubsystemMain, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(CANIDs.kElevatorSubsystemSecondary, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkMaxConfig config2 = new SparkMaxConfig();
  RelativeEncoder encoder = motor1.getEncoder();
  RelativeEncoder encoder2 = motor2.getEncoder();
  private SparkClosedLoopController controller1 = motor1.getClosedLoopController();
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(500,20);
  ProfiledPIDController pid = new ProfiledPIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD, constraints);
  private DigitalInput l_top = new DigitalInput(DigitalInputPorts.kElevatorSubsystemUp);
  private DigitalInput l_bottom = new DigitalInput(DigitalInputPorts.kElevatorSubsystemDown);
  private DigitalInput l_slow = new DigitalInput(DigitalInputPorts.kElevatorSubsystemSlow);

  // True when pressed
  private boolean limitTop = l_top.get();
  private boolean limitBottom = l_bottom.get();
  private boolean limitSlow = l_slow.get();

  private double y_currentHeight = 0;
  private double y_targetHeight = 21.875;
  private double r_currentRotations = 0;
  private double r_targetRotations = 0;
  private double v_feedforward;
  private double PIDFeedback = 0;
  private double scissor_speed = 0;
  private double y_stageHeight;
  private double r_topSwitchRotations;
  private double r_slowSwitchRotations;
  private boolean limitTopOnceTrue = false;
  private boolean limitBottomOnceTrue = false;
  private boolean limitSlowOnceTrue = false;

  private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
  private final ShuffleboardLayout controlCol = elevatorTab.getLayout("Control", BuiltInLayouts.kList).withPosition(0,0).withSize(1,3);
  private final ShuffleboardLayout limitsCol = elevatorTab.getLayout("Limits", BuiltInLayouts.kList).withPosition(1,0).withSize(1,4);
  private final ShuffleboardLayout currentsCol = elevatorTab.getLayout("Currents", BuiltInLayouts.kList).withPosition(2,0).withSize(1,3);
  private final ShuffleboardLayout stateCol = elevatorTab.getLayout("State", BuiltInLayouts.kList).withPosition(3,0).withSize(1,5);
  private final ShuffleboardLayout controllerCol = elevatorTab.getLayout("Controller", BuiltInLayouts.kList).withPosition(4,0).withSize(1,5);

  private final GenericEntry scissorSpeedEntry = controlCol.add("Lift Speed", 0.0).getEntry();
  private final GenericEntry limitBottomEntry = limitsCol.add("Bottom", false).getEntry();
  private final GenericEntry limitBottomInitializedEntry = limitsCol.add("Bottom Init", false).getEntry();
  private final GenericEntry limitTopEntry = limitsCol.add("Top", false).getEntry();
  private final GenericEntry limitSlowEntry = limitsCol.add("Slow", false).getEntry();
  private final GenericEntry motor1CurrentEntry = currentsCol.add("Motor1 Current", 0.0).getEntry();
  private final GenericEntry motor2CurrentEntry = currentsCol.add("Motor2 Current", 0.0).getEntry();
  private final GenericEntry velocityEntry = currentsCol.add("Velocity", 0.0).getEntry();
  private final GenericEntry targetHeightEntry = stateCol.add("Target Height", 0.0).getEntry();
  private final GenericEntry currentHeightEntry = stateCol.add("Current Height", 0.0).getEntry();
  private final GenericEntry stageHeightEntry = stateCol.add("Stage Height", 0.0).getEntry();
  private final GenericEntry currentRotationsEntry = stateCol.add("Current Rotations", 0.0).getEntry();
  private final GenericEntry targetRotationsEntry = stateCol.add("Target Rotations", 0.0).getEntry();
  private final GenericEntry appliedVoltageEntry = stateCol.add("Applied Voltage", 0.0).getEntry();
  private final GenericEntry pidFeedbackEntry = controllerCol.add("PID Feedback", 0.0).getEntry();
  private final GenericEntry feedforwardEntry = controllerCol.add("Feedforward", 0.0).getEntry();
  public ElevatorSubsystem() {
    // Invert the SparkMax
    config.inverted(true);
    config.smartCurrentLimit(95);
    config.closedLoop.p(.0001).i(0.000001).d(0.001);
    // Apply the Inversion
    motor1.configure(config, null, null);
    // Increase PID error tolerance from 0.05 to 0.2
    pid.setTolerance(0.2);
    config2.smartCurrentLimit(95);
    config2.follow(CANIDs.kElevatorSubsystemMain);
    motor2.configure(config2, null, null);
    
  }


  @Override
  public void periodic() {
    refreshLimitStates();
    //r_currentRotations = (encoder.getPosition() + encoder2.getPosition()) / 2;
    r_currentRotations = encoder.getPosition();
    calculateCurrentHeight();
    ElevatorLogger();
  }

  // Internal Function for updating constants
  private void refreshLimitStates() {
    limitTop = !l_top.get();
    limitBottom = !l_bottom.get();
    limitSlow = !l_slow.get();

    // Adam------------
    // Reset counts every time bottom limit is hit. The momentum of the scissor is causing counts to become inaccurate.
    if (limitBottom){
      encoder.setPosition(0);
      r_currentRotations = 0;
      limitSlowOnceTrue = false;
      limitTopOnceTrue = false;
    }
    //----------------

    // To do: add a way to clear the status of the limit switches once toggled
    if (limitTop){
      limitTopOnceTrue = true;
      r_topSwitchRotations = encoder.getPosition();
    }

    if (limitSlow){
      limitSlowOnceTrue = true;
      r_slowSwitchRotations = encoder.getPosition();
    }

    if (limitBottom && !limitBottomOnceTrue) {
      limitBottomOnceTrue = true;
    }
 

  }

  private void ElevatorLogger() {
    scissorSpeedEntry.setDouble(scissor_speed);
    limitBottomEntry.setBoolean(limitBottom);
    limitBottomInitializedEntry.setBoolean(limitBottomOnceTrue);
    limitTopEntry.setBoolean(limitTop);
    limitSlowEntry.setBoolean(limitSlow);
    motor1CurrentEntry.setDouble(motor1.getOutputCurrent());
    motor2CurrentEntry.setDouble(motor2.getOutputCurrent());
    targetHeightEntry.setDouble(y_targetHeight);
    currentHeightEntry.setDouble(y_currentHeight);
    currentRotationsEntry.setDouble(r_currentRotations);
    targetRotationsEntry.setDouble(r_targetRotations);
    pidFeedbackEntry.setDouble(PIDFeedback);
    feedforwardEntry.setDouble(v_feedforward);
    stageHeightEntry.setDouble(y_stageHeight);
    velocityEntry.setDouble(encoder.getVelocity());
    appliedVoltageEntry.setDouble(motor1.getBusVoltage() * motor1.getAppliedOutput());
  }

  // R is rotations
  // L is scissor length
  // X is target height above initial
  // C is fixed length between farthest linkage connection and point of lead screw rotation
  // P is screw pitch
  // N*sqrt(L^2 - (C-R*P)^2))
  private void calculateCurrentHeight() {
    y_stageHeight = Math.sqrt(
      Math.pow(ElevatorSpecifics.kScissorLength, 2) - 
      (Math.pow(ElevatorSpecifics.kC - (r_currentRotations * (1 / ElevatorSpecifics.kScrewPitch)), 2)));
    double result = ElevatorSpecifics.kLinkageCount * y_stageHeight;
    y_currentHeight = result + ElevatorSpecifics.kInitialHeight;
  }

  private void calculateTargetRotations(double targetHeight) {
    double scissorheight = targetHeight - ElevatorSpecifics.kInitialHeight;
    double term1 = Math.pow(scissorheight / ElevatorSpecifics.kLinkageCount, 2);
    double term2 = Math.sqrt(Math.pow(ElevatorSpecifics.kScissorLength, 2) - term1); 
    double term3 = ElevatorSpecifics.kC - term2;
    r_targetRotations = term3 * ElevatorSpecifics.kScrewPitch;
  }


  private void setSpeed(double speed){
    // positive speed raises scissor
    if(limitBottom)
    {
      speed = Math.max(0, speed);
    }
    if(limitTop)
    {
      speed = Math.min(speed, 0);
    }
    if(limitSlow)
    {
      // Positive (going up, but really slow)
      speed = Math.min(speed, 0.1);
    }
    // software limit
    if ((r_currentRotations < 0) && limitBottomOnceTrue) {
      speed = Math.max(0, speed);
    }
    if ((r_currentRotations > r_topSwitchRotations) && limitTopOnceTrue){
      speed = Math.min(speed, 0);
    }
    if ((r_currentRotations > r_slowSwitchRotations) && limitSlowOnceTrue){
      speed = Math.min(speed, 0.1);
    }


    // if((speed < 0) && r_currentRotations < 5)
    // {
    //   speed = 0.2 * speed;
    // }

    motor1.set(speed);

    // controller1.setReference(speed*500, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 
    // speed>0?1:0);

    // scissor_speed = speed*500;
    
    scissor_speed = speed;
  }

  private void setRPM(double speed){
    // positive speed raises scissor
    if(limitBottom)
    {
      speed = Math.max(0, speed);
    }
    if(limitTop)
    {
      speed = Math.min(speed, 0);
    }
    if(limitSlow)
    {
      speed = Math.min(speed, 60); // 60 rpm
    }
    if (r_currentRotations < 1) {
      speed = Math.max(0, speed);
    }
    
    controller1.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, speed>0?1:0);

    scissor_speed = speed;
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // lower on iteration stop on termination
  // public Command ElevatorLowerCommand() {
  //   return runEnd(
  //     () -> {
  //       refreshLimitStates();
  //       //this.manualAdjustment(-0.20);
  //       this.setSpeed(-0.25)
  //     },
  //     () -> {
  //       motor1.stopMotor();
  //     }
  //   );
  // }

  public Command ElevatorRaiseCommand(CommandXboxController m_operatorController) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        refreshLimitStates();
        // RT Up
        double Raise = m_operatorController.getRightTriggerAxis();
        // DT Down
        double Lower = 0.65 * m_operatorController.getLeftTriggerAxis();
        this.setSpeed(Raise - Lower);
      },
      interrupted -> {
        motor1.stopMotor();
    },
      () -> (false),
      this

    );
  }

  public Command GoTo(Level target) {

      return new FunctionalCommand(
      // onInit: Initialize our values
      () -> {
        y_targetHeight = target.getLevel();
        calculateTargetRotations(y_targetHeight);
        pid.setGoal(r_targetRotations);
      },
      // onExecute: Update our calculations and drive the motor
      () -> {
        refreshLimitStates();
        calculateTargetRotations(y_targetHeight);
        PIDFeedback = pid.calculate(r_currentRotations);//rps
        v_feedforward = pid.getSetpoint().velocity + 10;//rps
      
        this.setRPM(60 * (v_feedforward + PIDFeedback));
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor1.stopMotor(); 
      },
      // isFinished: End the command when the target is reached
      () ->(pid.atSetpoint()),
      // Require this subsystem
      this
    );
}

public double getElevatorHeight() {
  return y_currentHeight;
}

public double getMaxHeight() {
  return (ElevatorSpecifics.kScissorLength * ElevatorSpecifics.kLinkageCount) + ElevatorSpecifics.kInitialHeight;
}

public double getMinHeight() {
  return (ElevatorSpecifics.kLinkageCount * Math.sqrt(Math.pow(ElevatorSpecifics.kScissorLength,2) - Math.pow(ElevatorSpecifics.kC, 2))) + ElevatorSpecifics.kInitialHeight;
} 

  // public Command Homing() {
  //   return new FunctionalCommand(
  //     () -> {}, 
  //     () -> {
  //       this.setSpeed(-0.05);
  //     }, 
  //     interrupted -> {
  //       encoder.setPosition(0);
  //     }, 
  //     () -> (limitBottom), 
  //     this);
  // }
}
















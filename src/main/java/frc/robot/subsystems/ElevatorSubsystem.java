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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // L4(72.), //not possible atm because of linkage limitations
  
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
  private SparkClosedLoopController controller1 = motor1.getClosedLoopController();
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(500,20);
  ProfiledPIDController pid = new ProfiledPIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD, constraints);
  private DigitalInput l_top = new DigitalInput(DigitalInputPorts.kElevatorSubsystemUp);
  private DigitalInput l_bottom = new DigitalInput(DigitalInputPorts.kElevatorSubsystemDown);

  // True when pressed
  private boolean limitTop = l_top.get();
  private boolean prevLimitBottom;
  // True when pressed
  private boolean limitBottom = l_bottom.get();
  private double y_currentHeight = 0;
  private double y_targetHeight = 21.875;
  private double r_currentRotations = 0;
  private double r_targetRotations = 0;
  private double v_feedforward;
  private double PIDFeedback = 0;
  private double scissor_speed = 0;
  private double y_stageHeight;

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
    ElevatorLogger();
    r_currentRotations = encoder.getPosition();
    calculateCurrentHeight();
  }

  // Internal Function for updating constants
  private void update() {
    prevLimitBottom = limitBottom;
    limitTop = !l_top.get();
    limitBottom = !l_bottom.get();
    if (limitBottom && !prevLimitBottom) {
      encoder.setPosition(0);
    }

  }

  private void ElevatorLogger(){
    SmartDashboard.putNumber("Scissor Lift Speed", scissor_speed);
    SmartDashboard.putBoolean("Scissor Bottom Limit", limitBottom);
    SmartDashboard.putBoolean("Scissor Top Limit", limitTop);
    SmartDashboard.putNumber("Scissor Motor1 Current", motor1.getOutputCurrent());
    SmartDashboard.putNumber("Scissor Motor2 Current", motor2.getOutputCurrent());
    SmartDashboard.putNumber("Target Height", y_targetHeight);
    SmartDashboard.putNumber("Current Height Estimate", y_currentHeight);
    SmartDashboard.putNumber("Current Rotations", r_currentRotations);
    SmartDashboard.putNumber("Target Rotations", r_targetRotations);
    SmartDashboard.putNumber("PIDFeedback", PIDFeedback);
    SmartDashboard.putNumber("Feedforward", v_feedforward);
    SmartDashboard.putNumber("Stage Height", y_stageHeight);
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Applied Voltage", motor1.getBusVoltage() * motor1.getAppliedOutput());
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
    controller1.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, speed>0?1:0);

    scissor_speed = speed;
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // lower on iteration stop on termination
  // public Command ElevatorLowerCommand() {
  //   return runEnd(
  //     () -> {
  //       update();
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
        update();
        // RT Up
        double Raise = m_operatorController.getRightTriggerAxis();
        // DT Down
        double Lower = m_operatorController.getLeftTriggerAxis();
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
        update();
        calculateTargetRotations(y_targetHeight);
        PIDFeedback = pid.calculate(r_currentRotations);//rps
        v_feedforward = pid.getSetpoint().velocity + 10;//rps
      
        this.setRPM(60 * (v_feedforward + PIDFeedback));
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor1.stopMotor(); 
        // SmartDashboard.putBoolean("Elevator GoTo Interrupt", interrupted);
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



package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DigitalInputPorts;
import frc.robot.Constants.ElevatorSpecifics;
import frc.robot.Constants.PIDTunings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class ElevatorSubsystem extends SubsystemBase{

  public enum Level {
    Bottom(ElevatorSpecifics.kInitialHeight),
    L2(31.875),
    L3(47.65),
    L4(72.0),
    ;

    private double value;

    private Level(double height) {
      this.value = height;
    }

    public double getLevel() {
      return this.value;
    }
  };

  private final SparkMax motor1 = new SparkMax(CANIDs.kElevatorSubsystemMain, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(CANIDs.kElevatorSubsystemSecondary, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkMaxConfig config2 = new SparkMaxConfig();
  RelativeEncoder encoder = motor1.getEncoder();
  PIDController pid = new PIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD);
  private DigitalInput l_top = new DigitalInput(DigitalInputPorts.kElevatorSubsystemUp);
  private DigitalInput l_bottom = new DigitalInput(DigitalInputPorts.kElevatorSubsystemDown);

  // True when pressed
  private boolean limitTop = l_top.get();
  // True when pressed
  private boolean limitBottom = l_bottom.get();
  private double y_targetHeight = ElevatorSpecifics.kInitialHeight;
  private double r_currentRotations = 0;
  private double r_targetRotations = 0;
  private double PIDFeedback = 0;

  public ElevatorSubsystem() {
    // Invert the SparkMax
    config.inverted(true);
    config.smartCurrentLimit(100);
    // Apply the Inversion
    motor1.configure(config, null, null);
    // Reduce PID error tolerance from 0.05 to 0.02
    pid.setTolerance(0.02);
    config2.smartCurrentLimit(100);
    config2.follow(CANIDs.kElevatorSubsystemMain);
    motor2.configure(config2, null, null);
  }


  @Override
  public void periodic() {
    
  }

  // Internal Function for updating constants
  private void update() {
    calculateTargetRotations();
    r_currentRotations = encoder.getPosition();
    // y_currentHeight = calculateCurrentHeight();
    PIDFeedback = pid.calculate(r_currentRotations, r_targetRotations);
    // DataLogManager.log("PID Feedback" + PIDFeedback);
    // DataLogManager.log("Target Height" + y_targetHeight);
    // DataLogManager.log("Adjusted Speed" + SigmoidAdjustment(PIDFeedback));
    // DataLogManager.log("At Setpoint " + pid.atSetpoint());
    // DataLogManager.log("Target Rotational Value: " + r_targetRotations);
    limitTop = !l_top.get();
    limitBottom = !l_bottom.get();
    if (limitBottom) {
      encoder.setPosition(0);
    }
  }

  // R = (sqrt(L^2 - X^2) - C) / P
  // R is rotations
  // L is scissor length
  // X is target height
  // C is fixed length between farthest linkage connection and point of lead screw rotation (24.38 inches or 24 inches)
  // P is screw pitch
  // https://erobtic.wixsite.com/erobtic/post/scissor-lifting-elevator-mechanism
  // https://vectorspace.slack.com/archives/C07H5JJBLCX/p1739492047940889
  private void calculateTargetRotations() {
    double LSquared = Math.pow(ElevatorSpecifics.kScissorLength,2);
    double XSquared = Math.pow(y_targetHeight / ElevatorSpecifics.kLinkageCount,2);
    double LS_XS = LSquared - XSquared;
    double sqrt = Math.sqrt(LS_XS);
    double num = sqrt - ElevatorSpecifics.kScissorLength;
    double denom = ElevatorSpecifics.kScrewPitch;
    r_targetRotations = num / denom;
  }

  // Scaling PIDFeedback -0.3 to 0.3
  private double SigmoidAdjustment(double value) {
    double num = -value * 0.1; // Adjust this second value to change maximum / minimum output of this function (-0.3 = {-0.3 to 0.3})
    double denom = 1 + Math.abs(value);
    return num / denom;
  }

  // for manually raising / lowering elevator -> contains logic for limit switches
  private void manualAdjustment(double speed){
    // If speed is within -1 to 1 and neither limit switch is triggered
    if (!limitBottom && !limitTop){
      motor1.set(speed);
      System.out.println("First Logic");
    }
    // If speed is positive and bottom limit switch is triggered
    else if (limitBottom && !limitTop && (speed > 0)){
      motor1.set(speed);
      System.out.println("Second Logic");
    }
    // If speed is negative and top limit switch is triggered
    else if (limitTop && !limitBottom && (speed < 0)){
      motor1.set(speed);
      System.out.println("Third Logic");
    }
    // 
    else{
      System.out.println("Stop Logic");
      motor1.stopMotor();
    }
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // lower on iteration stop on termination
  public Command ElevatorLowerCommand() {
    return runEnd(
      () -> {
        update();
        this.manualAdjustment(-0.20);
      },
      () -> {
        motor1.stopMotor();
      }
    );
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // raise on iteration stop on termination
  public Command ElevatorRaiseCommand() {
    return runEnd(
      () -> {
        update();
        this.manualAdjustment(0.5);
      },
      () -> {
        motor1.stopMotor();
      }
    );
  }

  public Command GoTo(Level target) {
    y_targetHeight = target.getLevel();
    if (r_targetRotations >= r_currentRotations) {
      return new FunctionalCommand(
      // onInit: Initialize our values
      () -> {},
      // onExecute: Update our calculations and drive the motor
      () -> {
        update();
        double x = SigmoidAdjustment(PIDFeedback);
        if (!limitTop) {motor1.set(x);}
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor1.stopMotor(); 
      },
      // isFinished: End the command when the target is reached or we hit our limit switch
      () -> ( limitTop || r_currentRotations >= r_targetRotations) || pid.atSetpoint(),
      // Require this subsystem
      this
    );
    }
    else{
    return new FunctionalCommand(
      // onInit: Initialize our values
      () -> {},
      // onExecute: Update our calculations and drive the motor
      () -> {
        update();
        double x = SigmoidAdjustment(PIDFeedback);
        if (!limitBottom) {motor1.set(x);}
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor1.stopMotor(); 
      },
      // isFinished: End the command when the target is reached or we hit our limit switch
      () -> ( limitBottom || r_currentRotations <= r_targetRotations || pid.atSetpoint()),
      // Require this subsystem
      this
    );
  }
}
  public Command Homing() {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        manualAdjustment(-0.05);
      }, 
      interrupted -> {
        motor1.stopMotor();
        encoder.setPosition(0);
      }, 
      () -> (limitBottom), 
      this);
  }
}



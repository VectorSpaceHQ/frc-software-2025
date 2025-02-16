package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIDs;
import frc.robot.Constants.ElevatorSpecifics;
import frc.robot.Constants.PIDTunings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private final SparkMax motor = new SparkMax(CANIDs.kElevatorSubsystemMain, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid = new PIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD);
  private DigitalInput l_top = new DigitalInput(0);
  private DigitalInput l_bottom = new DigitalInput(1);

  // True when pressed
  private boolean limitTop = l_top.get();
  // True when pressed
  private boolean limitBottom = l_bottom.get();
  private double y_currentHeight = ElevatorSpecifics.kInitialHeight;
  private double y_targetHeight = calculateCurrentHeight();
  private double PIDFeedback = 0;

  public ElevatorSubsystem() {
    // Registers the subsystem with the command scheduler
    register();
    // Invert the SparkMax
    config.inverted(true);
    // Apply the Inversion
    motor.configure(config, null, null);
  }


  @Override
  public void periodic() {
    // update();
  }

  // Internal Function for updating constants
  private void update() {
    y_currentHeight = calculateCurrentHeight();
    PIDFeedback = pid.calculate(y_currentHeight, y_targetHeight);
    // DataLogManager.log("PID Feeback" + PIDFeedback);
    // DataLogManager.log("Current Height" + y_currentHeight);
    // DataLogManager.log("Target Height" + y_targetHeight);
    // DataLogManager.log("Adjusted Speed" + SigmoidAdjustment(PIDFeedback));
    // DataLogManager.log("At Setpoint " + pid.atSetpoint());
    limitTop = l_top.get();
    limitBottom = l_bottom.get();
    if (limitBottom) {
      encoder.setPosition(0);
    }
  }

  // y=SquareRootOf(z^2-(z-pr)^2 ) -- per scissor 
  // z is scissor length
  // p is pitch
  // r is rotations
  // total h = (n * y) + y0
  private double calculateCurrentHeight() {
    // The following can be consolidated, but I left open for testing later
    double ScissorLengthSquared = ElevatorSpecifics.kScissorLength * ElevatorSpecifics.kScissorLength;
    double PitchTimesRotations = encoder.getPosition() * ElevatorSpecifics.kScrewPitch;
    double ScissorLengthMinusPitchTimesRotations = ElevatorSpecifics.kScissorLength - PitchTimesRotations;
    double ScissorLengthMinusPitchTimesRotationsSquared = ScissorLengthMinusPitchTimesRotations * ScissorLengthMinusPitchTimesRotations;
    double HeightSquared = ScissorLengthSquared - ScissorLengthMinusPitchTimesRotationsSquared;
    double result = Math.sqrt(HeightSquared) * ElevatorSpecifics.kLinkageCount;
    return result + ElevatorSpecifics.kInitialHeight;
  }

  // (-sqrt(z^2 - (yf - y0)^2) + z) / p = R
  // R is the rotations to go from y0 to yf
  // z is scissor length
  // yf is target height
  // y0 is current height
  // p is screw pitch
  private double calculateRotationsToY(double targetHeight, double currentHeight) {
    double Zsquared = Math.pow(ElevatorSpecifics.kScissorLength, 2);
    double deltaY = targetHeight - currentHeight;
    double deltaYsquared = Math.pow(deltaY, 2);
    double ZsquaredMinusDY2 = Zsquared - deltaYsquared;
    double num = ElevatorSpecifics.kScissorLength - Math.sqrt(ZsquaredMinusDY2);
    double denom = ElevatorSpecifics.kScrewPitch * ElevatorSpecifics.kLinkageCount;
    return num / denom;
  }

  // Scaling PIDFeedback -0.3 to 0.3
  private double SigmoidAdjustment(double value) {
    double num = -value * 0.1; // Adjust this second value to change maximum / minimum output of this function (-0.3 = {-0.3 to 0.3})
    double denom = 1 + Math.abs(value);
    return num / denom;
  }

  // Used by commands to designate a target height of the platform
  public void setElevatorTargetHeight(double h) {
    y_targetHeight = h;
  }
  // Used by commands to designate a target height of the input
  public void setInputTargetHeight(double h) {
    y_targetHeight = h - ElevatorSpecifics.kPlatformToInputHeight;
  }

  public void resetElevator() {
    setElevatorTargetHeight(ElevatorSpecifics.kInitialHeight);
  }

  // Can most likely be removed
  public boolean getTopLimitSwitch() {
    return limitTop;
  }
  // Can most likely be removed
  public boolean getBottomLimitSwitch() {
    return limitBottom;
  }

  // Not used can most likely be removed
  public double getPlatformHeight() {
    return y_currentHeight;
  } 

  // Added for interupt behaviors on manual adjustments (might be removable I just wanted to avoid weird edge cases)
  public void stopMotor(){
    motor.stopMotor();
  }

  // Getter for debugging
  public double getInputHeight() {
    return y_currentHeight + ElevatorSpecifics.kPlatformToInputHeight;
  }

  // yet another getter
  public double getEncoderposition() {
    return encoder.getPosition();
  }

  // for manually raising / lowering elevator -> contains logic for limit switches
  private void manualAdjustment(double speed){
    // If speed is within -1 to 1 and neither limit switch is triggered
    if (!limitBottom && !limitTop){
      motor.set(speed);
      System.out.println("First Logic");
    }
    // If speed is positive and bottom limit switch is triggered
    else if (limitBottom && !limitTop && (speed < 0)){
      motor.set(speed);
      System.out.println("Second Logic");
    }
    // If speed is negative and top limit switch is triggered
    else if (limitTop && !limitBottom && (speed > 0)){
      motor.set(speed);
      System.out.println("Third Logic");
    }
    // 
    else{
      System.out.println("Stop Logic");
      motor.stopMotor();
    }
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // lower on iteration stop on termination
  public Command ElevatorLowerCommand() {
    return runEnd(
      () -> {
        update();
        this.manualAdjustment(-0.05);
      },
      () -> {
        this.stopMotor();
      }
    );
  }

  // runEnd adds a runnable on iteration and a runnable on termination
  // raise on iteration stop on termination
  public Command ElevatorRaiseCommand() {
    return runEnd(
      () -> {
        update();
        this.manualAdjustment(0.05);
      },
      () -> {
        this.stopMotor();
      }
    );
  }

  public Command GoTo(Level target) {
    y_targetHeight = target.getLevel();
    if (y_targetHeight >= y_currentHeight) {
      return new FunctionalCommand(
      // onInit: Initialize our values
      () -> {},
      // onExecute: Update our calculations and drive the motor
      () -> {
        update();
        double x = SigmoidAdjustment(PIDFeedback);
        if (!limitTop) {motor.set(x);}
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor.stopMotor(); 
      },
      // isFinished: End the command when the target is reached or we hit our limit switch
      () -> ( limitTop || y_currentHeight >= y_targetHeight),
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
        if (!limitBottom) {motor.set(x);}
      },
      // onEnd: Stop the motor
      interrupted -> {
        motor.stopMotor(); 
      },
      // isFinished: End the command when the target is reached or we hit our limit switch
      () -> ( limitBottom || y_currentHeight <= y_targetHeight),
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
        motor.stopMotor();
        encoder.setPosition(0);
        y_currentHeight = ElevatorSpecifics.kInitialHeight;
      }, 
      () -> (limitBottom), 
      this);
  }
}



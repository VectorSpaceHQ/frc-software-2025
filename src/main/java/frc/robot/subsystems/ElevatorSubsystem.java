package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.ElevatorSpecifics;
import frc.robot.Constants.PIDTunings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class ElevatorSubsystem extends SubsystemBase{

  private final SparkMax motor = new SparkMax(CANIDs.kElevatorSubsystemMain, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid = new PIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD);
  private final DigitalInput l_top = new DigitalInput(0);
  private final DigitalInput l_bottom = new DigitalInput(1);

  // True when pressed
  private boolean limitTop = !l_top.get();
  // True when pressed
  private boolean limitBottom = !l_bottom.get();
  private double r_rotations = 0;
  private double y_currentHeight = calculateCurrentHeight();
  private double y_targetHeight = calculateCurrentHeight();
  private double PIDFeedback = 0;

  public ElevatorSubsystem() {
  }


  @Override
  public void periodic() {
    update();
  }

  // Internal Function for updating constants
  private void update() {
    r_rotations = encoder.getPosition();
    y_currentHeight = calculateCurrentHeight();
    PIDFeedback = pid.calculate(y_currentHeight, y_targetHeight);
    limitTop = !l_top.get();
    limitBottom = !l_top.get();
  }

  // y=SquareRootOf(z^2-(z-pr)^2 ) -- per scissor 
  // total h = (n * y) + y0
  // Needs to be revamped when Rohan gives us the formula :/
  private double calculateCurrentHeight() {
    // The following can be consolidated, but I left open for testing later
    double z2 = ElevatorSpecifics.kScissorLength * ElevatorSpecifics.kScissorLength;
    double pr = r_rotations * ElevatorSpecifics.kScrewPitch;
    double z_pr = ElevatorSpecifics.kScissorLength - pr;
    double z_pr2 = z_pr * z_pr;
    double y2 = z2 - z_pr2;
    double result = Math.sqrt(y2) * ElevatorSpecifics.kLinkageCount;
    return result + ElevatorSpecifics.kInitialHeight;
  }

  public void adjustToSetPoint() {
    if (!limitTop && !limitBottom){
    motor.set(SigmoidAdjustment(PIDFeedback));
    }
    else if (limitTop && SigmoidAdjustment(PIDFeedback) < 0) {
      motor.set(SigmoidAdjustment(PIDFeedback));
    }
    else if (limitBottom && SigmoidAdjustment(PIDFeedback) > 0) {
      motor.set(SigmoidAdjustment(PIDFeedback));
    }
    else {
      motor.set(0);
    }
  }

  // Scaling PIDFeedback -0.3 to 0.3
  private double SigmoidAdjustment(double value) {
    // Keep this first term negative 
    double num = value * -0.3; // Adjust this second value to change maximum / minimum output of this function (-0.3 = {-0.3 to 0.3})
    double denom = 1 + Math.abs(value);
    return num / denom;
  }

  // Not used can most likely be removed
  public void resetEncoder() {
    encoder.setPosition(0);
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

  // for isfinished() command behaviors
  public boolean atSetpoint(){
    return pid.atSetpoint();
  }

  // for manually raising / lowering elevator -> contains logic for limit switches
  public void manualAdjustment(double speed){
    if (1.0 >= speed && -1.0 <= speed && !limitBottom && !limitTop){
      motor.set(speed);
    }
    else if (limitBottom && speed >= 0){
      motor.set(speed);
    }
    else if (limitTop && speed <= 1){
      motor.set(speed);
    }
    else{
      motor.stopMotor();
    }
  }
}



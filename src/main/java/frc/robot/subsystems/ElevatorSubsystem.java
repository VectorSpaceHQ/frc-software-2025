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

  private final SparkMax motor = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid = new PIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD);
  private final DigitalInput l_top = new DigitalInput(0);
  private final DigitalInput l_bottom = new DigitalInput(1);

  private boolean motortoggle = false;
  // True when pressed
  private boolean limitTop = !l_top.get();
  // True when pressed
  private boolean limitBottom = !l_bottom.get();
  private double r_rotations = 0;
  private double y_currentHeight = calculateCurrentHeight();
  private double y_targetHeight = calculateCurrentHeight();
  private double PIDFeedback;

  public ElevatorSubsystem() {
    resetEncoder();
  }


  @Override
  public void periodic() {
    update();
    adjustToSetPoint();
  }

  private void update() {
    r_rotations = encoder.getPosition();
    y_currentHeight = calculateCurrentHeight();
    PIDFeedback = pid.calculate(y_currentHeight, y_targetHeight);
    limitTop = !l_top.get();
    limitBottom = !l_top.get();
    motortoggle = limitTop && limitBottom;
  }

  // y=SquareRootOf(z^2-(z-pr)^2 ) -- per scissor 
  // total h = (n * y) + y0
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

  private void adjustToSetPoint() {
    if (!pid.atSetpoint() && motortoggle){
    motor.set(SigmoidAdjustment(PIDFeedback));
    }
    else if (!pid.atSetpoint() && limitTop && SigmoidAdjustment(PIDFeedback) < 0) {
      motor.set(SigmoidAdjustment(PIDFeedback));
    }
    else if (!pid.atSetpoint() && limitBottom && SigmoidAdjustment(PIDFeedback) > 0) {
      motor.set(SigmoidAdjustment(PIDFeedback));
    }
  }

  // Scaling PIDFeedback -1 to 1 
  private double SigmoidAdjustment(double value) {
    // Keep this first term negative 
    double num = value * -0.3; // Adjust this second value to change maximum / minimum output of this function (-0.3 = {-0.3 to 0.3})
    double denom = 1 + Math.abs(value);
    return num / denom;
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void setElevatorTargetHeight(double h) {
    y_targetHeight = h;
  }

  public void setInputTargetHeight(double h) {
    y_targetHeight = h - ElevatorSpecifics.kPlatformToInputHeight;
  }

  public void resetElevator() {
    setElevatorTargetHeight(ElevatorSpecifics.kInitialHeight);
  }

  public double getPlatformHeight() {
    return y_currentHeight;
  } 

  public double getInputHeight() {
    return y_currentHeight + ElevatorSpecifics.kPlatformToInputHeight;
  }

  public double getEncoderposition() {
    return encoder.getPosition();
  }
}



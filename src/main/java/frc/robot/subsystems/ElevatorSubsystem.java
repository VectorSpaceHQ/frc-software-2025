package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.ElevatorSpecifics;
import frc.robot.Constants.PIDTunings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class ElevatorSubsystem extends SubsystemBase{

  private final SparkMax motor = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid = new PIDController(PIDTunings.kElevatorKP, PIDTunings.kElevatorKI, PIDTunings.kElevatorKD);

  private boolean motortoggle = false;
  private double r_rotations = 0;
  private double y_currentHeight;
  private double y_targetHeight;
  private double PIDFeedback;

  public ElevatorSubsystem() {
    resetEncoder();
  }

  // Might need to move some of these value assigning lines to their own function if it gets too crowded
  @Override
  public void periodic() {
    r_rotations = encoder.getPosition();
    y_currentHeight = calculateCurrentHeight();
    PIDFeedback = pid.calculate(y_currentHeight, y_targetHeight);
    if (motortoggle) {
      adjustToSetPoint();
    }
    if (pid.atSetpoint()) {
      motortoggle = false;
    }
  }

  // y=SquareRootOf(z^2-(z-pr)^2 ) -- per scissor 
  // total h = (n * y) + y0
  private double calculateCurrentHeight() {
    double result;
    // The following can be consolidated, but I left open for testing later
    double z2 = ElevatorSpecifics.kScissorLength * ElevatorSpecifics.kScissorLength;
    double pr = r_rotations * ElevatorSpecifics.kScrewPitch;
    double z_pr = ElevatorSpecifics.kScissorLength - pr;
    double z_pr2 = z_pr * z_pr;
    double y2 = z2 - z_pr2;
    result = Math.sqrt(y2) * ElevatorSpecifics.kLinkageCount;
    return result + ElevatorSpecifics.kInitialHeight;
  }

  private void adjustToSetPoint() {
    motor.set(SigmoidAdjustment(PIDFeedback));
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

  public void runElevator() {
    motortoggle = true;
  }

  public void stopElevator() {
    motortoggle = false;
  }

  public void resetElevator() {
    // Add limit switch conditionals
    // Lower Until Limit Switch is triggered
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



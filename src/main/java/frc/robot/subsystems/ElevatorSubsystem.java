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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.lang.Math;

import org.ejml.dense.row.SpecializedOps_CDRM;

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
  private double y_currentHeight = 0;
  private double y_targetHeight = ElevatorSpecifics.kInitialHeight;
  private double r_currentRotations = 0;
  private double r_targetRotations = 0;
  private double PIDFeedback = 0;
  private double scissor_speed = 0;

  public ElevatorSubsystem() {
    // Invert the SparkMax
    config.inverted(true);
    config.smartCurrentLimit(105);
    // Apply the Inversion
    motor1.configure(config, null, null);
    // Reduce PID error tolerance from 0.05 to 0.02
    pid.setTolerance(0.02);
    config2.smartCurrentLimit(105);
    config2.follow(CANIDs.kElevatorSubsystemMain);
    motor2.configure(config2, null, null);

  }


  @Override
  public void periodic() {
    ElevatorLogger();
  }

  // Internal Function for updating constants
  private void update() {
    calculateTargetRotations();
    calculateCurrentHeight();
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
  }

  // // sqrt((RP+C)^2 - L^2) + y0 = yx
  // private void calculateCurrentHeight() {
  //   double term1 = Math.sqrt(Math.pow((r_currentRotations * ElevatorSpecifics.kScrewPitch) + ElevatorSpecifics.kScissorLength, 2) - Math.pow(ElevatorSpecifics.kC,2));
  //   double result = Math.sqrt(term1) + ElevatorSpecifics.kInitialHeight;
  //   y_currentHeight = result;
  // }

  // 4*sqrt(L^2 - ((C-R*P)^2))
  private void calculateCurrentHeight() {
    double result = ElevatorSpecifics.kLinkageCount * Math.sqrt(Math.pow(ElevatorSpecifics.kScissorLength, 2) - (Math.pow(ElevatorSpecifics.kC - (r_currentRotations * (1 / ElevatorSpecifics.kScrewPitch)), 2)));
    y_currentHeight = result + ElevatorSpecifics.kInitialHeight;
  }

  // R = (sqrt(L^2 - X^2) - C) / P
  // (RP + C)^2 = L^2 + X^2
  // sqrt((RP+C)^2 - L^2) = X
  // sqrt((RP+C)^2 - L^2) + y0 = yx
  // R is rotations
  // L is scissor length
  // X is target height above initial
  // C is fixed length between farthest linkage connection and point of lead screw rotation
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

  // runEnd adds a runnable on iteration and a runnable on termination
  // raise on iteration stop on termination
  public Command ElevatorRaiseCommand(CommandXboxController m_operatorController) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        update();
        //this.manualAdjustment(0.5);
        // RT Up
        double Raise = m_operatorController.getRightTriggerAxis();
        // DT Down
        double Lower = 0.25 * m_operatorController.getLeftTriggerAxis();
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
        setSpeed(-0.05);
      }, 
      interrupted -> {
        motor1.stopMotor();
        encoder.setPosition(0);
      }, 
      () -> (limitBottom), 
      this);
  }
}



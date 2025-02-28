package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem controlling the release of currently loaded coral
 */
public class CoralSubsystem extends SubsystemBase {

  // Creates two instances of SparkMax Motor Controlled Brushless Motors on
  // provided CANIDs
  // Main
  private final SparkMax motor_left = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
  // Follower
  private final SparkMax motor_right = new SparkMax(CANIDs.kCoralSubsystemRight, MotorType.kBrushless);
  // Config to be passed to right (inverted, follower, currentlimits)
  private final SparkMaxConfig config_right = new SparkMaxConfig();
  // Config to be passed to left (currentlimits)
  private final SparkMaxConfig config_left = new SparkMaxConfig();

  // Motors are Synchronized so setspeed is controlled via common input 0.1 = 10%
  // of max output
  private final double motorspeed = 0.2;

  public CoralSubsystem() {

    // Left config
    config_left.smartCurrentLimit(20, 20);
    config_left.idleMode(IdleMode.kBrake);
    motor_left.configure(config_left, null, null);

    // Right Config
    config_right.smartCurrentLimit(20, 20);
    config_right.idleMode(IdleMode.kBrake);
    config_right.follow(CANIDs.kCoralSubsystemLeft, true);

    motor_right.configure(config_right, null, null);
  }

  @Override
  public void periodic() {
  }

  public Command runCoralDispenser() {
    return new FunctionalCommand(() -> {
    },
        () -> {
          motor_left.set(motorspeed);
        },
        interrupted -> {
          motor_left.stopMotor();
        },
        () -> (false),
        this);
  }
}

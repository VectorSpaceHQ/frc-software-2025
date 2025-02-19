package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * The subsystem controlling the release of currently loaded coral
 */
public class CoralSubsystem extends SubsystemBase{

    // Creates two instances of SparkMax Motor Controlled Brushless Motors on provided CANIDs
    private final SparkMax motor_left = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kCoralSubsystemRight, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    // Motors are Synchronized so setspeed is controlled via common input 0.1 = 10% of max output
    private final double motorspeed = 0.1;

    public CoralSubsystem() {
      config.smartCurrentLimit(1,1);
      motor_left.configure(config, null, null);
      config.inverted(true);
      config.follow(CANIDs.kCoralSubsystemLeft);
      motor_right.configure(config, null, null);
    }

    @Override
    public void periodic() {
    }

    public Command runCoralDispenser() {
      return new FunctionalCommand
      (() -> {}, 
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


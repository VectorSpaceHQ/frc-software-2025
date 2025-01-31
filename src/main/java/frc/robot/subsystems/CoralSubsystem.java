package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** 
 * The subsystem controlling the release of currently loaded coral
 */
public class CoralSubsystem extends SubsystemBase{

    // Creates two instances of SparkMax Motor Controlled Brushless Motors on provided CANIDs
    private final SparkMax motor_left = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kCoralSubsystemRight, MotorType.kBrushless);

    // Motors are Synchronized so setspeed is controlled via common input 0.1 = 10% of max output
    private final double motorspeed = 0.1;
    // Simple On/Off Toggle for Both Motors
    private boolean motortoggle = false;

    public CoralSubsystem() {

    }

    @Override
    public void periodic() {
        if (motortoggle == true) {
          motor_left.set(motorspeed);
          motor_right.set(motorspeed);
        }
        else {
          motor_left.stopMotor();
          motor_right.stopMotor();
        }
    }

    public void releaseCoral() {
      motortoggle = true;
    }

    public void suspendCoral() {
      motortoggle = false;
    }
}


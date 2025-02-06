package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax motor_left = new SparkMax(CANIDs.kAlgaeSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kAlgaeSubsystemRight, MotorType.kBrushless);
    private final DigitalInput l_Left = new DigitalInput(0);
    private final DigitalInput l_Right = new DigitalInput(0);
    private boolean LimitSwitchLeft;
    private boolean LimitSwitchRight;
    private double speed = 0.1;
    private boolean motortoggle;

    public AlgaeSubsystem() {

    }

    @Override
    public void periodic() {
        updateSensorStatus();
        if (motortoggle){
            motor_left.set(speed);
            motor_right.set(-speed);
        }
        if (!motortoggle) {
            motor_left.stopMotor();
            motor_right.stopMotor();
        }
    }

    private void updateSensorStatus() {
        LimitSwitchLeft = !l_Left.get();
        LimitSwitchRight = !l_Right.get();
    }

    public void motorSetSpeed(double rate) {
        speed = rate;
    }

    public void motorStop() {
        motortoggle = false;
    }

    public void motorStart() {
        motortoggle = true;
    }

    private void homeLeftClaw() {
        while(!LimitSwitchLeft){
        motor_left.set(-0.05);
        }
        motor_left.stopMotor();
    }

    private void homeRightClaw() {
        while(!LimitSwitchRight){
            motor_right.set(0.05);
        }
        motor_right.stopMotor();
    }

    public void homeClaws(){
        homeLeftClaw();
        homeRightClaw();
    }
}
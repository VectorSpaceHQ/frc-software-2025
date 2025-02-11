package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DigitalInputPorts;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax motor_left = new SparkMax(CANIDs.kAlgaeSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kAlgaeSubsystemRight, MotorType.kBrushless);
    private final DigitalInput l_Left = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemLeft);
    private final DigitalInput l_Right = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemRight);
    private boolean limitSwitchLeft = !l_Left.get();
    private boolean limitSwitchRight = !l_Right.get();
    private double speed = 0.0;
    // Primarily for Debug Purposes
    private boolean motortoggle = true;

    public AlgaeSubsystem() {

    }

    @Override
    public void periodic() {
        updateSensorStatus();
        if (motortoggle){
            driveMotors(speed);
        }
        // Another option is to set rate = 0 -- For Debugging
        if (!motortoggle) {
            stopMotors();
        }
    }

    private void updateSensorStatus() {
        limitSwitchLeft = !l_Left.get();
        limitSwitchRight = !l_Right.get();
    }

    private void stopMotors() {
        motor_left.stopMotor();
        motor_right.stopMotor();
    }

    private void driveMotors(double rate) {
        driveLeftMotor(rate);
        driveRightMotor(-rate);
    }

    private void driveLeftMotor(double rate) {
        if(!limitSwitchLeft) {
            motor_left.set(rate);
        }
        if(limitSwitchLeft && rate >= 0) {
            motor_left.set(rate);
        }
    }

    private void driveRightMotor(double rate) {
        if(!limitSwitchRight) {
            motor_right.set(rate);
        }
        if(limitSwitchRight && rate <= 0) {
            motor_right.set(rate);
        }
    }

    public void motorSetSpeed(double rate) {
        if(1.0 >= rate && rate >= -1.0){
            speed = rate;
        }
    }

    public void motorStop() {
        motortoggle = false;
    }

    public void motorStart() {
        motortoggle = true;
    }

    public void homeClaws(){
        motortoggle = false;
        homeLeftClaw();
        homeRightClaw();
        motortoggle = true;
    }

    private void homeLeftClaw() {
        while(!limitSwitchLeft){
            motor_left.set(-0.05);
        }
        motor_left.stopMotor();
    }

    private void homeRightClaw() {
        while(!limitSwitchRight){
            motor_right.set(0.05);
        }
        motor_right.stopMotor();
    }
}
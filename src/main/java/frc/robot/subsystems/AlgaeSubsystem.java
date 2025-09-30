package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DigitalInputPorts;

public class AlgaeSubsystem extends SubsystemBase {
    private double speed = 0;
    private final SparkMax motor_left = new SparkMax(CANIDs.kAlgaeSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kAlgaeSubsystemRight, MotorType.kBrushless);
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();
    private final DigitalInput l_Left = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemLeft);
    private final DigitalInput l_Right = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemRight);
    private boolean limitSwitchLeft = l_Left.get();
    private boolean limitSwitchRight = l_Right.get();

    private final Timer overheatTimer = new Timer();

    private final ShuffleboardTab algaeTab = Shuffleboard.getTab("Algae");
    private final ShuffleboardLayout currentCol = algaeTab.getLayout("Currents", BuiltInLayouts.kList).withPosition(0,0).withSize(1,3);
    private final ShuffleboardLayout limitCol = algaeTab.getLayout("Limits", BuiltInLayouts.kList).withPosition(1,0).withSize(1,3);
    private final ShuffleboardLayout controlCol = algaeTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2,0).withSize(1,2);
    private final GenericEntry motorSpeedEntry = controlCol.add("Motor Speed", 0.0).getEntry();
    private final GenericEntry leftMotorCurrentEntry = currentCol.add("Left Current", 0.0).getEntry();
    private final GenericEntry rightMotorCurrentEntry = currentCol.add("Right Current", 0.0).getEntry();
    private final GenericEntry leftLimitEntry = limitCol.add("Left Limit", false).getEntry();
    private final GenericEntry rightLimitEntry = limitCol.add("Right Limit", false).getEntry();

    public AlgaeSubsystem() {
        leftConfig.smartCurrentLimit(30, 20);
        rightConfig.smartCurrentLimit(30, 20);
        rightConfig.inverted(true);

        motor_right.configure(rightConfig, null, null);
        motor_left.configure(leftConfig, null, null);
    }

    @Override
    public void periodic() {
        limitSwitchLeft = !l_Left.get();
        limitSwitchRight = !l_Right.get();
    }

    private void update(){
        // reserved for future logic
    }

    private void AlgaeLogger(){
      motorSpeedEntry.setDouble(speed);
      leftMotorCurrentEntry.setDouble(motor_left.getOutputCurrent());
      rightMotorCurrentEntry.setDouble(motor_right.getOutputCurrent());
      leftLimitEntry.setBoolean(limitSwitchLeft);
      rightLimitEntry.setBoolean(limitSwitchRight);
    }

    private void setSpeed(double speed){
        double left_speed = -speed;
        double right_speed = -speed;

        if(overheatTimer.get() >= 30.0){
            overheatTimer.reset();
        }
        if(speed == 0){
            overheatTimer.reset();
        } else if(overheatTimer.get() >= 25.0){
            speed = 0;
        }

        left_speed = Math.min(left_speed, 0.30);
        right_speed = Math.min(right_speed, 0.30);

        if(overheatTimer.get() >= 10.0){
            left_speed = Math.max(left_speed, -0.75);
            right_speed = Math.max(right_speed, -0.75);
        } else if(overheatTimer.get() >= 20.0){
            left_speed = Math.max(left_speed, -0.50);
            right_speed = Math.max(right_speed, -0.50);
        }

        if(limitSwitchLeft){
            left_speed = Math.min(0, speed);
        }

        if(limitSwitchRight){
            right_speed = Math.min(0, speed);
        }

        motor_left.set(left_speed);
        motor_right.set(right_speed);
    }

    public Command runClaws(CommandXboxController m_operatorcontroller) {
        return new FunctionalCommand(
            () -> {
                overheatTimer.start();
                },
            () -> {
                update();
                speed = m_operatorcontroller.getRightY();
                speed = 0.3 * speed;
                setSpeed(speed);
                AlgaeLogger();
            },
            interrupted -> {
                motor_left.stopMotor();
            },
            () -> false,
            this);
    }

    public Command runClaws(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                update();
                setSpeed(speed);
                AlgaeLogger();
            },
            interrupted -> {
                motor_left.stopMotor();
            },
            () -> false,
            this);
    }

    public Command homeClaws() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                update();
                if (!limitSwitchLeft) {
                    motor_left.set(0.1);
                } else {
                    motor_left.stopMotor();
                }
                if (!limitSwitchRight) {
                    motor_right.set(-0.1);
                } else {
                    motor_right.stopMotor();
                }
            },
            interrupted -> {},
            () -> (limitSwitchLeft && limitSwitchRight),
            this);
    }
}

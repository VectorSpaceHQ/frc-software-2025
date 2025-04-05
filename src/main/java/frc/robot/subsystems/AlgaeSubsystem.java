package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public AlgaeSubsystem() {
        //config.follow(CANIDs.kAlgaeSubsystemLeft,true);
        leftConfig.smartCurrentLimit(30, 20);
        rightConfig.smartCurrentLimit(30, 20);
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
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
        // True when limit switch is pressed.
        // limitSwitchLeft = !l_Left.get();
        // limitSwitchRight = !l_Right.get();
    }

    private void AlgaeLogger(){
      SmartDashboard.putNumber("Algae motor speed", speed);
      //SmartDashboard.putNumber("Algae Left Motor Current", motorspeed);
      //SmartDashboard.putNumber("Algae Right Motor Current", motorspeed);
      SmartDashboard.putNumber("claw motor_left Current", motor_left.getOutputCurrent());
      SmartDashboard.putNumber("claw motor_right Current", motor_right.getOutputCurrent());
      SmartDashboard.putBoolean("Algae Left Motor Limit", limitSwitchLeft);
      SmartDashboard.putBoolean("Algae Right Motor Limit", limitSwitchRight);
    }

    private void setSpeed(double speed){
        // negative speed opens claws.
        double left_speed = speed;
        double right_speed = speed;

        // limit the opening speeds
        left_speed = Math.max(left_speed, -0.30);
        right_speed = Math.max(right_speed, -0.30);

        if(limitSwitchLeft){
            left_speed = Math.max(0, speed);
        }

        if(limitSwitchRight)
        {
            right_speed = Math.max(0, speed);
        }

        motor_left.set(left_speed);
        motor_right.set(right_speed);
    }
    
    // Sets both motors
    // Stops on either limit switch pressed
    public Command runClaws(CommandXboxController m_operatorcontroller) {
        return new FunctionalCommand(
            () -> {                
                },
            () -> {
                update();
                speed = m_operatorcontroller.getRightY();
                speed = 0.3 * speed;
                //motor_left.set(speed);
                setSpeed(speed);
                AlgaeLogger();
            },
            interrupted -> {
                motor_left.stopMotor();
            },
            //() -> ((limitSwitchLeft || limitSwitchRight) && (speed <= 0)),
            //this);
            () -> (false),  this);
    }

    public Command runClaws(double speed) {
        return new FunctionalCommand(
            () -> {                
                },
            () -> {
                update();
                //motor_left.set(speed);
                setSpeed(speed);
                AlgaeLogger();
            },
            interrupted -> {
                motor_left.stopMotor();
            },
            //() -> ((limitSwitchLeft || limitSwitchRight) && (speed <= 0)),
            //this);
            () -> (false),  this);
    }

    // Homes claws then on finish sets right motor to inverted follower
    // Returns both motors to their respective limit switches
    public Command homeClaws() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                update();
                if (!limitSwitchLeft) {
                    motor_left.set(0.1);
                }
                else { motor_left.stopMotor();}
                if (!limitSwitchRight) {
                    motor_right.set(-0.1);
                }
                else {
                    motor_right.stopMotor();
                }
            }, 
            interrupted -> {}, 
            () -> (limitSwitchLeft && limitSwitchRight), 
            this);
    }
}
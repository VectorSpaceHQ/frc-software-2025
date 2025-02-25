package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private SparkMaxConfig defaultconfig = new SparkMaxConfig();
    private SparkMaxConfig config = new SparkMaxConfig();
    private final DigitalInput l_Left = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemLeft);
    private final DigitalInput l_Right = new DigitalInput(DigitalInputPorts.kAlgaeSubsystemRight);
    private boolean limitSwitchLeft = l_Left.get();
    private boolean limitSwitchRight = l_Right.get();

    public AlgaeSubsystem() {
        // Sets right motor to an inverted follower of the left
        defaultconfig.smartCurrentLimit(20, 20);
        config.follow(CANIDs.kAlgaeSubsystemLeft,true);
        config.smartCurrentLimit(20, 20);
    }

    @Override
    public void periodic() {
        
    }

    private void update(){
        limitSwitchLeft = !l_Left.get();
        limitSwitchRight = !l_Right.get();
    }
    
    // Sets both motors
    // Stops on either limit switch pressed
    public Command runClaws(CommandXboxController m_drivercontroller) {
        return new FunctionalCommand(
            () -> {motor_right.configure(config, null, null);},
            () -> {
                update();
                speed = m_drivercontroller.getLeftTriggerAxis() - m_drivercontroller.getRightTriggerAxis();
                speed = 0.3 * speed;
                motor_left.set(speed);
            },
            interrupted -> {
                motor_left.stopMotor();
            },
            () -> ((limitSwitchLeft || limitSwitchRight) && (speed <= 0)),
            this);
    }

    // Homes claws then on finish sets right motor to inverted follower
    // Returns both motors to their respective limit switches
    public Command homeClaws() {
        return new FunctionalCommand(
            () -> {motor_right.configure(defaultconfig, null, null);},
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
            interrupted -> {motor_right.configure(config, null, null);}, 
            () -> (limitSwitchLeft && limitSwitchRight), 
            this);
    }
}
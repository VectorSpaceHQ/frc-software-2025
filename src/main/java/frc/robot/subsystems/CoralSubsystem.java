package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

/**
 * The subsystem controlling the release of currently loaded coral
 */
public class CoralSubsystem extends SubsystemBase{

    private final SparkMax motor_left = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(CANIDs.kCoralSubsystemRight, MotorType.kBrushless);
    private final SparkMaxConfig config_right = new SparkMaxConfig();
    private final SparkMaxConfig config_left = new SparkMaxConfig();

    private final double motorspeed = 0.3;

    private final ShuffleboardTab coralTab = Shuffleboard.getTab("Coral");
    private final ShuffleboardLayout controlCol = coralTab.getLayout("Control", BuiltInLayouts.kList).withPosition(0,0).withSize(1,2);
    private final GenericEntry coralSpeedEntry = controlCol.add("Coral Speed", 0.0).getEntry();

    public CoralSubsystem() {
      config_left.smartCurrentLimit(20,20);
      config_left.idleMode(IdleMode.kBrake);
      motor_left.configure(config_left, null, null);

      config_right.smartCurrentLimit(20,20);
      config_right.idleMode(IdleMode.kBrake);
      config_right.follow(CANIDs.kCoralSubsystemLeft, true);

      motor_right.configure(config_right, null, null);
    }

    @Override
    public void periodic() {
    }

    private void CoralLogger(){
      coralSpeedEntry.setDouble(motorspeed);
    }

    public Command runCoralDispenser() {

      return new FunctionalCommand
      (() -> {},
      () -> {
            CoralLogger();
            motor_left.set(motorspeed);
      },
      interrupted -> {
        motor_left.stopMotor();
      },
      () -> false,
       this);
    }
}

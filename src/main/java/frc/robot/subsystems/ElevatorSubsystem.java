package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CANIDs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class ElevatorSubsystem extends SubsystemBase{

  private final SparkMax motor = new SparkMax(CANIDs.kCoralSubsystemLeft, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  private boolean motortoggle = false;
  private final double z_scissorLength = 0;
  private final double p_screwPitch = 0;
  private final double r_rotations = 0;
  private final double y0_initialHeight = 0;

  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
  }

  // y=SquareRootOf(z^2-(z-pr)^2 )
  private double calculateCurrentHeight() {
    
    return 0;
  }
}



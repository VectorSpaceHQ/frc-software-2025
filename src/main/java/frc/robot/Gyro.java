package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
  public Rotation2d getRotation2d();
  public double getRate();
  public void reset();
  public void DisplayIMUData();
}

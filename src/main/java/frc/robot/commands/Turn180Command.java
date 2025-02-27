//Layout
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turn180Command extends Command {
  private final DriveSubsystem driveSubsystem;
  private final double rot;
  private final double speed;
  private final Timer timer = new Timer();
  private final double duration;
  private double initialAngle;

  // Sets the Turn180Command constructor
  public Turn180Command(DriveSubsystem driveSubsystem, double speed, double rot, double duration) {
    this.driveSubsystem = driveSubsystem;
    this.rot = rot; // Turn speed I guess
    this.speed = speed;
    this.duration = duration; // Variate for now
    addRequirements(driveSubsystem);
  }

  // Initializes the Turn180Command and sets the timer for the duration
  @Override
  public void initialize() {
    timer.reset();
    initialAngle = driveSubsystem.gyroAngle(); // Get the robot's initial angle in degrees
    SmartDashboard.putString("Aiming Status", "Turn180Command Initialized. Speed = " + speed + ", Rot = "+ rot + "Duration = " + duration);
  }

  // Executes the Turn180Command
  @Override
  public void execute() {
    driveSubsystem.turn(rot); // Adjust speed for turning
    SmartDashboard.putString("Aiming Status", "Turning 180...");
  }

  // Checks if the Turn180Command is finished (can use a gyro or encoder to track rotation?)
  @Override
  public boolean isFinished() {
    double currentAngle = driveSubsystem.gyroAngle(); // Get the robot's current angle in degrees
    double targetAngle = Math.abs(currentAngle - initialAngle); // Takes the target angle should be 180 degrees 
    if (Math.abs(targetAngle) >= 180 || timer.get() >= duration) { // Check if the robot has turned 180 degrees
      SmartDashboard.putString("Aiming Status", "Turn180Command Finished");
      return true;
    }
    return false;
  }

  // Ends the Turn180Command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false); // Stop the robot after the turn
    SmartDashboard.putString("Aiming Status", "Turn180Command Ended");
  }
}

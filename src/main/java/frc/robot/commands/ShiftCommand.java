//Literally just copied the DriveTargetCommand layout
package frc.robot.commands;

// Imports for the Shift command
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final double speed;
  private final Timer timer = new Timer();
  private final double duration;
  private final double rot;

  // Sets the ShiftCommand constructor
  public ShiftCommand(DriveSubsystem driveSubsystem, double speed, double rot, double duration) {
    this.driveSubsystem = driveSubsystem;
    this.speed = speed; //Default speed is input speed (idk if it works)
    this.rot = rot;
    this.duration = duration;
    addRequirements(driveSubsystem);
  }

  // Not doing anything with the camera connected thing because that is too complicated
  // Initializes the ShiftCommand and sets the timer for the duration
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SmartDashboard.putString("Shifting Status", "ShiftCommand Initialized. Speed = " + speed + ", Rot = " + rot + " Duration = " + duration);
  }

  // Executes the ShiftCommand
  @Override
  public void execute() {
    driveSubsystem.strafe(speed);
    SmartDashboard.putString("Shifting Status", "Shifting...");
  }

  // Checks if the ShiftCommand is finished
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("Shifting Status", "ShiftCommand Finished");
    return timer.get() >= duration;
  }

  // Ends the ShiftCommand
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    SmartDashboard.putString("Shifting Status", "ShiftCommand Ended");
  }
}
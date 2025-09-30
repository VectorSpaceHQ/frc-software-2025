package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// We want this auto to run into the reef and dump a coral into the trough.
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class SimpleAuto extends Command {
    private final ElevatorSubsystem robotElevator;
    private final DriveSubsystem robotDrive;

    public SimpleAuto(DriveSubsystem m_robotDrive, ElevatorSubsystem m_robotElevator) {
        robotElevator = m_robotElevator;
        robotDrive = m_robotDrive;
        addRequirements(robotElevator, robotDrive);
    }
@Override
  public void initialize() {
    new SequentialCommandGroup(
        robotDrive.run(() -> robotDrive.drive(-0.3, 0, 0, false)).withTimeout(2.5),
        robotElevator.GoTo(Level.L2)
    ).schedule();
  }

  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

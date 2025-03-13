package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AprilTags;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class ComplexAuto extends Command {

  private final AlgaeSubsystem robotAlgae;
  private final ElevatorSubsystem robotElevator;
  private final DriveTargetCommand robotTarget;
  private final AprilTags AutotagID;

  public ComplexAuto(AlgaeSubsystem m_robotAlgae, ElevatorSubsystem m_robotElevator, DriveTargetCommand aimTarget, AprilTags tagID) {
    robotAlgae = m_robotAlgae;
    robotElevator = m_robotElevator;
    robotTarget = aimTarget;
    AutotagID = tagID;
    addRequirements(robotElevator, robotAlgae);
  }

  @Override
  public void initialize() {
    robotTarget.setTargetID(AutotagID);
    new SequentialCommandGroup(
      robotTarget.withTimeout(2), 
      robotElevator.GoTo(Level.L2).withTimeout(2),
      robotAlgae.runClaws(.2).withTimeout(2)
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
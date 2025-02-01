// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorSubsystem;

public class RobotContainer {
  XboxController m_driverController = new XboxController(0);
  private final MotorSubsystem m_MotorSubsystem = new MotorSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
     
       new JoystickButton(m_driverController, Button.kRightBumper.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.addNtoRotate(0.1)));

     new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.addNtoRotate(1.)));
    
     new JoystickButton(m_driverController, Button.kX.value)
      // .debounce(0.5)
      .onTrue(new InstantCommand(() -> m_MotorSubsystem.addNtoRotate(10.)));

     new JoystickButton(m_driverController, Button.kY.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.addNtoRotate(100.)));
     
     new JoystickButton(m_driverController, Button.kA.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.runMotor()));

     new JoystickButton(m_driverController, Button.kB.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.stopMotor()));
     new JoystickButton(m_driverController, Button.kStart.value)
    //  .debounce(0.5)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.increasekP()));
     new JoystickButton(m_driverController, Button.kBack.value)
     .onTrue(new InstantCommand(() -> m_MotorSubsystem.resetEncoder()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void dashboardupdate() {
    SmartDashboard.putNumber("Encoder Position (Rotations)", m_MotorSubsystem.getEncoderPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_MotorSubsystem.getVelocity());
    SmartDashboard.putNumber("PID Feedback", m_MotorSubsystem.getPIDFeedback());
    SmartDashboard.putNumber("PID Setpoint", m_MotorSubsystem.getSetpoint());
    SmartDashboard.putNumber("Sigmoid Return", m_MotorSubsystem.getSigmoid());
    SmartDashboard.putNumber("kP", m_MotorSubsystem.getkP());
    SmartDashboard.putNumber("kI", m_MotorSubsystem.getkI());
    SmartDashboard.putNumber("kD", m_MotorSubsystem.getkD());
  }

  public void TeleOP_init() {
    m_MotorSubsystem.resetEncoder();
  }
}

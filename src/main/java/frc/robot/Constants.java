// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *A
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 0;
    public static final int kRearLeftMotorPort = 1;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearRightMotorPort = 3;

    public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = true;

    // TO-DO Reconfigure
    public static final double kTrackWidth = 0.635;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5842;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  // Stores CANIDs of Specific Motors
  public static final class CANIDs {
    public static final int kCoralSubsystemLeft = 5;
    public static final int kCoralSubsystemRight = 4;
    public static final int kDriveSubsystemFrontRight = 7;
    public static final int kDriveSubsystemFrontLeft = 6;
    public static final int kDriveSubsystemRearRight = 8;
    public static final int kDriveSubsystemRearLeft = 9;
    public static final int kElevatorSubsystemSecondary = 10;
    public static final int kElevatorSubsystemMain = 11;
    public static final int kAlgaeSubsystemLeft = 3;
    public static final int kAlgaeSubsystemRight = 2;
  }

  public static final class PIDTunings {
    public static final double kElevatorKP = 1.2;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;
  }

  public static final class ElevatorSpecifics {
    public static final double kScissorLength = 23.75; // inches -- Length of Each Rod
    public static final double kScrewPitch = 10; // threads / inch -- of lead screw
    // public static final double kInitialHeight = 21.78; // inches -- from bottom to platform
    public static final double kInitialHeight = 11.75; // inches -- from bottom to platform ignoring scissor
    public static final double kLinkageCount = 2; // Linkage Counts per Side
    public static final double kPlatformToInputHeight = 14.26; // Distance between Platform and Coral Input
    public static final double kC = 23.375;
  }

  public static final class DigitalInputPorts {
    public static final int kAlgaeSubsystemRight = 0;
    public static final int kAlgaeSubsystemLeft = 1;
    public static final int kElevatorSubsystemUp = 2;
    public static final int kElevatorSubsystemDown = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/4;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    public static final TrapezoidProfile.Constraints kLinearControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
  }
}

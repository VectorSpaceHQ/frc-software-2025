// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final class FeatureToggles {
    public static final boolean enableScissorLift = true;
    public static final boolean enableAlgae = true;
    public static final boolean enableMecanum = true;
    public static final boolean enableVision = true;
    public static final boolean enableCoral = false;
    public static final boolean enableIMU = true;
    public static final boolean enableRuntimeParams = true;
    public static final boolean enablePoseEstimator = true;
  }

  public static final class DriveConstants {
    public static RobotConfig config = null;
    public static final int kFrontLeftMotorPort = 0;
    public static final int kRearLeftMotorPort = 1;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearRightMotorPort = 3;

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = true;

    // TO-DO Reconfigure
    public static final double kTrackWidth = 0.635;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5842;
    // Distance between centers of front and back wheels on robot

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelDistanceFromCenter = Math.sqrt((kWheelBase * kWheelBase)+(kTrackWidth * kTrackWidth))/2;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    public static final double kGearRatio = 60/9; // Drive gear ratio
    public static final double kKrakenVoltsPerRPM = 1 / 502.1;
    public static final double kKrakenVoltsPerRPS = 60 / 502.1;
    public static final double kDefaultBusVoltage = 12;
    public static final double kStrafeMultiplier = 1 / Math.sqrt(2);
    public static final double kMetersPerMotorRotation = kWheelDiameterMeters * Math.PI / kGearRatio;
    public static final double kForwardVoltsPerMeterPerSecond = (kKrakenVoltsPerRPM * 60) / (kMetersPerMotorRotation);
    public static final double kStrafeVoltsPerMeterPerSecond = kForwardVoltsPerMeterPerSecond * kStrafeMultiplier;
    public static final double kVoltsPerDegreePerSecond = (kForwardVoltsPerMeterPerSecond) * 180/(Math.sqrt(2) * Math.PI * kWheelDistanceFromCenter)/100;//todomake right
    public static final double kMaxAcceleration = 5.5; // Drive team traction limited accel should be 5.5
    public static final double kMinAcceleration = 2.5; // To prevent tipping when scissor lift extended should be 3.5
    public static final double kForwardDriverVelocityScalar = 3.6; // m/s
    public static final double kStrafeDriverVelocityScalar = 3.6; // m/s
    public static final double kRotationalDriverVelocityScalar = Math.PI; // rad/s

    public static void initConfig() {
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      }
    }
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
    public static final double kElevatorKP = 0.5;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;
  }

  public static final class ElevatorSpecifics {
    public static final double kScissorLength = 23.75; // inches -- Length of Each Rod
    public static final double kScrewPitch = 10; // threads / inch -- of lead screw
    // public static final double kInitialHeight = 21.78; // inches -- from bottom to platform
    public static final double kInitialHeight = 11.75; // inches -- from bottom to platform ignoring scissor
    public static final double kLinkageCount = 4; // Linkage Counts per Side
    public static final double kPlatformToInputHeight = 14.26; // Distance between Platform and Coral Input
    public static final double kC = 23.4; // fixed length between farthest linkage connection and point of lead screw rotation
  }

  public static final class DigitalInputPorts {
    public static final int kAlgaeSubsystemRight = 2;
    public static final int kAlgaeSubsystemLeft = 3;
    public static final int kElevatorSubsystemUp = 1;
    public static final int kElevatorSubsystemDown = 0;
    public static final int kElevatorSubsystemSlow = 5;

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

  public static class DrivePoses {
    // Just a little bit of precision
    public static final Pose2d BlueBargePose = new Pose2d(7.805382251739502, 6.705155849456787, new Rotation2d(0));
  }

  public static class PathLimitations{
    public static final PathConstraints constraints = new PathConstraints(3, 3.6, Math.PI, Math.PI / 2);
  }
}

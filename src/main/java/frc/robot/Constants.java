// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftFrontCAN = 9;
    public static final int kLeftRearCAN = 10;
    public static final int kRightFrontCAN = 7;
    public static final int kRightRearCAN = 8;

    // public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    // public static final int[] kRightEncoderPorts = new int[] {2, 3};
    // public static final boolean kLeftEncoderReversed = false;
    // public static final boolean kRightEncoderReversed = true;

    public static final double kConvertInchToMeter = 0.0254;

    public static final double kTrackwidthInches = 18.5;
    public static final double kTrackwidthMeters = kTrackwidthInches * kConvertInchToMeter;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1;
    public static final double kWheelDiameterInches = 6.0;
    public static final double kGearRatio = 8.45;
    public static final double kWheelDiameterMeters = kWheelDiameterInches * kConvertInchToMeter;
    // public static final double kEncoderDistancePerPulse =
    //     // Assumes the encoders are directly mounted on the wheel shafts
    //     (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final double kUnitsPerRotation = (kWheelDiameterMeters * Math.PI);  // !!! Use this value in SysID!
    public static final double kEncoderDistancePerRevolution = (kWheelDiameterMeters * Math.PI) / kGearRatio;

    public static final double kVelocityRatio = 1/kGearRatio;  // Motor spins kGearRatio times before output wheel turns once.

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    // public static final double ksVolts = 0.22 CHANGE THIS!;
    // public static final double kvVoltSecondsPerMeter = 1.98 CHANGE THIS!;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.2 CHANGE THIS!;

    // SysID file used: C:\Users\FabLab9\FRC2024\ThomasCanTrack\sysid_data\sysid_data20231029-134230.json
    public static final double ksVolts = 0.11161; // Don't change!
    public static final double kvVoltSecondsPerMeter = 2.2496; // Don't change!
    public static final double kaVoltSecondsSquaredPerMeter = 0.346; // Don't change!

    public static final double kPDriveVel = 0.001; // Don't change!
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    // !!!alh
    public static final int kButtonX = 3;
    public static final int kButtonY = 4;
    public static final int kButtonLeftBumper = 5;
    public static final int kButtonRightBumper = 6;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.1; //3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create CAN motor objects
  private CANSparkMax leftFrontSparkMax = new CANSparkMax(DriveConstants.kLeftFrontCAN, MotorType.kBrushless);
  private CANSparkMax leftRearSparkMax = new CANSparkMax(DriveConstants.kLeftRearCAN, MotorType.kBrushless);
  private CANSparkMax rightFrontSparkMax = new CANSparkMax(DriveConstants.kRightFrontCAN, MotorType.kBrushless);
  private CANSparkMax rightRearSparkMax = new CANSparkMax(DriveConstants.kRightRearCAN, MotorType.kBrushless);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(leftFrontSparkMax, leftRearSparkMax);
  // new MotorControllerGroup(
      //     new PWMSparkMax(DriveConstants.kLeftMotor1Port),
      //     new PWMSparkMax(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(rightFrontSparkMax, rightRearSparkMax);
  // new MotorControllerGroup(
      //     new PWMSparkMax(DriveConstants.kRightMotor1Port),
      //     new PWMSparkMax(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);
  RelativeEncoder m_leftEncoder = leftFrontSparkMax.getEncoder();
  RelativeEncoder m_rightEncoder = rightFrontSparkMax.getEncoder();

  // The gyro sensor
  // private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Default top speed
    m_drive.setMaxOutput(0.5);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_leftEncoder.setVelocityConversionFactor(1.0);
    m_rightEncoder.setVelocityConversionFactor(1.0);
    // WRONG -> SysId relies on the SparkMax controllers to be configured, so we must store the above settings in flash memory
    // leftFrontSparkMax.burnFlash();
    // leftRearSparkMax.burnFlash();
    // rightFrontSparkMax.burnFlash();
    // rightRearSparkMax.burnFlash();

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            // m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
            // m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
            m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());

    // SmartDashboard.putData(m_odometry);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        // m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        // m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    SmartDashboard.putNumber("m_leftEncoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("m_rightEncoder", getRightEncoderPosition());
    SmartDashboard.putNumber("leftEncoder(Inch)", getLeftEncoderPosition()/DriveConstants.kConvertInchToMeter);
    SmartDashboard.putNumber("rightEncoder(Inch)", getRightEncoderPosition()/DriveConstants.kConvertInchToMeter);
    SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocity());

    SmartDashboard.putNumber("Pose Rotation", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getTranslation().getY());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    return new DifferentialDriveWheelSpeeds(-getLeftEncoderVelocity(), -getRightEncoderVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        // m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
        // m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("Left Voltage", leftVolts);
    SmartDashboard.putNumber("Right Voltage", rightVolts);

    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    // return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderPosition() {
    return -m_rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity() {
    return -m_rightEncoder.getVelocity();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}

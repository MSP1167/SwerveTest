// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.util.Util;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemChassis extends SubsystemBase {

  Translation2d frontLeftLocation = new Translation2d(-0.300125, 0.2970625);
  Translation2d frontRightLocation = new Translation2d(0.300125, 0.2970625);
  Translation2d backLeftLocation = new Translation2d(-0.300125, -0.2970625);
  Translation2d backRightLocation = new Translation2d(0.300125, -0.2970625);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  //AHRS gyro = new AHRS(SerialPort.Port.kUSB);
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  // 1 m/s forward | 3 m/s left | 1.5 radians/s
  ChassisSpeeds speeds;// = new ChassisSpeeds(1, 3, 1.5);

  // Convert to module states
  SwerveModuleState[] moduleStates;// = kinematics.toSwerveModuleStates(speeds);

  SwerveDriveOdometry odometry;

  Pose2d pose = new Pose2d();

  TalonFX FrontLeftDrive;
  TalonFX FrontLeftTurn;
  TalonFX FrontRightDrive;
  TalonFX FrontRightTurn;
  TalonFX BackLeftDrive;
  TalonFX BackLeftTurn;
  TalonFX BackRightDrive;
  TalonFX BackRightTurn;

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  SwerveModule[] swerveModules = new SwerveModule[4];

  private final SlewRateLimiter xSpeedLimiter   = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter   = new SlewRateLimiter(3);
  private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);

  Joystick driver = new Joystick(0);

  AHRS ahrs;

  private double DEGREE_PER_TICK =  360 / 44032;
  private double TICKS_PER_DEGREE =  44032 / 360;

  /** Creates a new SubsystemChassis. */
  public SubsystemChassis() {
    odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), new Pose2d(5, 13.5, new Rotation2d()));
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,Rotation2d.fromDegrees(45));

    FrontLeftDrive  = new TalonFX(2); //2
    FrontLeftTurn   = new TalonFX(1); //1
    FrontRightDrive = new TalonFX(8); //4
    FrontRightTurn  = new TalonFX(7); //3
    BackLeftDrive   = new TalonFX(4); //8
    BackLeftTurn    = new TalonFX(3); //7
    BackRightDrive  = new TalonFX(6); //6
    BackRightTurn   = new TalonFX(5); //5

    frontLeftModule = new SwerveModule(FrontLeftDrive, FrontLeftTurn);
    frontRightModule = new SwerveModule(FrontRightDrive, FrontRightTurn);
    backLeftModule = new SwerveModule(BackLeftDrive, BackLeftTurn);
    backRightModule = new SwerveModule(BackRightDrive, BackRightTurn);
    
    swerveModules[0] = frontLeftModule;
    swerveModules[1] = frontRightModule;
    swerveModules[2] = backLeftModule;
    swerveModules[3] = backRightModule;

    try {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX Micro:  " + ex.getMessage(), true);
    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putData(ahrs);
    // This method will be called once per scheduler run
    //var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
    //pose = odometry.update(gyroAngle, moduleStates);
    //FrontLeftDrive.set(ControlMode.PercentOutput, driver.getX() * 0.5);
    SmartDashboard.putNumber("Motor Position", frontLeftModule.Turn.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Motor Position Absolute", frontLeftModule.Turn.getSensorCollection().getIntegratedSensorAbsolutePosition());
    SmartDashboard.putNumber("Motor Error Bands", frontLeftModule.Turn.getClosedLoopError(0));
  }

  public void drive(Joystick driver, boolean fieldRotation){
    
    SmartDashboard.putNumber("Stick X", Util.deadzone(driver.getX(), 0.1));
    SmartDashboard.putNumber("Stick Y", Util.deadzone(driver.getY(), 0.1));
    SmartDashboard.putNumber("Stick Z", Util.deadzone(-driver.getZ(), 0.2));

    final double xSpeed   = xSpeedLimiter.calculate(Util.deadzone(driver.getX(), 0.1)) * Constants.DRIVE_X_MAX_SPEED;
    final double ySpeed   = ySpeedLimiter.calculate(Util.deadzone(driver.getY(), 0.1)) * Constants.DRIVE_Y_MAX_SPEED;
    final double rot      = rotSpeedLimiter.calculate(Util.deadzone(driver.getZ(), 0.2)) * Constants.DRIVE_ROT_MAX_SPEED;
    
    var SwerveModuleStates = kinematics.toSwerveModuleStates(fieldRotation ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, Constants.DRIVE_MAX_SPEED);
    

    SmartDashboard.putNumber("Swerve Module 0 % Drive",SwerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module 0 Turn",SwerveModuleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("Swerve Module 1 % Drive",SwerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module 1 Turn",SwerveModuleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("Swerve Module 2 % Drive",SwerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module 2 Turn",SwerveModuleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("Swerve Module 3 % Drive",SwerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module 3 Turn",SwerveModuleStates[3].angle.getDegrees());

    if (xSpeed == 0 && ySpeed == 0 && rot == 0){
      frontLeftModule.Turn.set(ControlMode.PercentOutput, 0);
      frontRightModule.Turn.set(ControlMode.PercentOutput, 0);
      backLeftModule.Turn.set(ControlMode.PercentOutput, 0);
      backRightModule.Turn.set(ControlMode.PercentOutput, 0);
      frontLeftModule.Drive.set(ControlMode.PercentOutput, 0);
      frontRightModule.Drive.set(ControlMode.PercentOutput, 0);
      backLeftModule.Drive.set(ControlMode.PercentOutput, 0);
      backRightModule.Drive.set(ControlMode.PercentOutput, 0);
      return;
    }

    frontLeftModule.setDesiredState(SwerveModuleStates[0]);
    frontRightModule.setDesiredState(SwerveModuleStates[1]);
    backLeftModule.setDesiredState(SwerveModuleStates[2]);
    backRightModule.setDesiredState(SwerveModuleStates[3]);

    SmartDashboard.putNumber("Swerve Module 0 PID Setpoint",SwerveModuleStates[0].angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);
    SmartDashboard.putNumber("Swerve Module 1 PID Setpoint",SwerveModuleStates[1].angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);
    SmartDashboard.putNumber("Swerve Module 2 PID Setpoint",SwerveModuleStates[2].angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);
    SmartDashboard.putNumber("Swerve Module 3 PID Setpoint",SwerveModuleStates[3].angle.getDegrees() * Constants.TURN_RATIO * TICKS_PER_DEGREE);
    
    //FrontLeftDrive.set(ControlMode.PercentOutput, driver.getX());
    //FrontRightDrive.set(ControlMode.PercentOutput, driver.getX());
    //BackLeftDrive.set(ControlMode.PercentOutput, driver.getX());
    //BackRightDrive.set(ControlMode.PercentOutput, driver.getX());

    //Util.getAndSetDouble("Test", 5);

  }

  public SwerveModule[] getModules(){
    return swerveModules;
  }
}

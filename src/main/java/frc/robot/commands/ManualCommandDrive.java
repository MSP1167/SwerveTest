// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemChassis;

public class ManualCommandDrive extends CommandBase {

  SubsystemChassis Chassis;
  Joystick Driver;
  /** Creates a new ManualCommandDrive. */
  public ManualCommandDrive(SubsystemChassis Chassis, Joystick Driver) {
    this.Chassis = Chassis;
    this.Driver = Driver;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Chassis.drive(Driver, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

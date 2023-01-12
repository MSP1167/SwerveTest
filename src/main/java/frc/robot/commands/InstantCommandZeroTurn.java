// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.subsystems.SubsystemChassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandZeroTurn extends InstantCommand {
  SubsystemChassis Chassis;


  public InstantCommandZeroTurn(SubsystemChassis Chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Chassis = Chassis;
    addRequirements(Chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModule[] swerveModules = Chassis.getModules();
    swerveModules[0].Turn.setSelectedSensorPosition(0, 0, 30);
    swerveModules[1].Turn.setSelectedSensorPosition(0, 0, 30);
    swerveModules[2].Turn.setSelectedSensorPosition(0, 0, 30);
    swerveModules[3].Turn.setSelectedSensorPosition(0, 0, 30);
  }
}

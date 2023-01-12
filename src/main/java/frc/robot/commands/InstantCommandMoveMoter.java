// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SwerveModule;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandMoveMoter extends InstantCommand {
  SwerveModule module;
  Double position;
  public InstantCommandMoveMoter(SwerveModule module) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.module = module;
    this.position = Util.getAndSetDouble("Set Motor Position", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    module.Turn.set(TalonFXControlMode.Position,  position);// * (2048/360));
    
    
  }
}

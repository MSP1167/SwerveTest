// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.util.Util;

public class PIDLoop extends CommandBase {
  SwerveModule module;
  Double position;
  /** Creates a new PIDLoop. */
  public PIDLoop(SwerveModule module) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.module = module;
    this.position = Util.getAndSetDouble("Set Motor Position", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.position = Util.getAndSetDouble("Set Motor Position", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.Turn.set(TalonFXControlMode.MotionMagic,  position);// * (2048/360));
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

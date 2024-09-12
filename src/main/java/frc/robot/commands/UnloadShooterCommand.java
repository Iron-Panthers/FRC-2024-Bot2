// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class UnloadShooterCommand extends Command {
  /** Creates a new UnloadShooterCommand. */
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  boolean pass;
  public UnloadShooterCommand(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pass = shooterSubsystem.isSerializerBeamBreakSensorTriggered();
    shooterSubsystem.setShooterMode(ShooterMode.SHOOTER_UNLOAD);
    pivotSubsystem.setTargetDegrees(20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.isSerializerBeamBreakSensorTriggered()&&pass==false){
      pass=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterMode(ShooterMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooterSubsystem.isSerializerBeamBreakSensorTriggered()&&pass==true;
  }
}

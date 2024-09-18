// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class LoadShooterCommand extends Command {
  /** Creates a new LoadShooterCommand. */
  ShooterSubsystem shooterSubsystem;

  ElevatorSubsystem elevatorSubsystem;
  PivotSubsystem pivotSubsystem;

  public LoadShooterCommand(
      ShooterSubsystem shooterSubsystem,
      PivotSubsystem pivotSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(elevatorSubsystem, shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setShooterMode(ShooterMode.LOAD_SHOOTER);
    elevatorSubsystem.setTargetHeight(0);
    pivotSubsystem.setTargetDegrees(20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterMode(ShooterMode.RAMP_SPEAKER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.isShooterBeamBreakSensorTriggered();
  }
}

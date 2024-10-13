// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class SerializerBackupCommand extends Command {
  /** Creates a new SerializerBackupCommand. */
  ShooterSubsystem shooterSubsystem;
  double serializerPos;
  public SerializerBackupCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setShooterMode(ShooterMode.SERIALIZER_BACKUP);
    serializerPos = shooterSubsystem.getSerializerPositon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterMode(ShooterMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return serializerPos - 0.5 > shooterSubsystem.getSerializerPositon();
  }
}
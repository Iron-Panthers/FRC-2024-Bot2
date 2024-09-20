// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeMode;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class IntakeCommand extends Command {

  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  ElevatorSubsystem elevatorSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      PivotSubsystem pivotSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem, pivotSubsystem, elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(intakeSubsystem, shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setTargetHeight(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSubsystem.atTargetHeight()){
      intakeSubsystem.setIntakeMode(IntakeMode.INTAKE);
      shooterSubsystem.setShooterMode(ShooterMode.INTAKE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMode(IntakeSubsystem.IntakeMode.HOLD);
    shooterSubsystem.setShooterMode(ShooterMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.isSerializerBeamBreakSensorTriggered();
  }
}

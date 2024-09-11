package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class TransferNoteCommand extends Command {

  ShooterSubsystem shooterSubsystem;

  public TransferNoteCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void execute() {

  }

  @Override
  public void initialize() {
    if (shooterSubsystem.isSerializerBeamBreakSensorTriggered()){
      shooterSubsystem.setShooterMode(ShooterMode.LOAD_SHOOTER);
    }
    else{
      shooterSubsystem.setShooterMode(ShooterMode.SHOOTER_UNLOAD);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isReadyToShoot();
  }
}

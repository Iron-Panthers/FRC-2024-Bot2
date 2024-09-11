package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class TransferNoteCommand extends Command {

  ShooterSubsystem shooterSubsystem;
  DrivebaseSubsystem drivebaseSubsystem;
  DoubleSupplier translationXSupplier;
  DoubleSupplier translationYSupplier;

  public TransferNoteCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

  }

  @Override
  public void execute() {

  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterMode(ShooterMode.LOAD_SHOOTER);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isReadyToShoot();
  }
}

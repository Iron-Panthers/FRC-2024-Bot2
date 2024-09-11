package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TransferNoteCommand extends SequentialCommandGroup {

  public TransferNoteCommand(
    ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem) {
    if (shooterSubsystem.isSerializerBeamBreakSensorTriggered()){
      addCommands(new LoadShooterCommand(shooterSubsystem));
    }
    else if (shooterSubsystem.isShooterBeamBreakSensorTriggered()){
      addCommands(new UnloadShooterCommand(shooterSubsystem, pivotSubsystem).andThen(new IntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem)));
    }
  }
}

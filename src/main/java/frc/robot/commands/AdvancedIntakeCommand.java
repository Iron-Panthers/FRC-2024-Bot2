package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AdvancedIntakeCommand extends SequentialCommandGroup {

  public AdvancedIntakeCommand(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      PivotSubsystem pivotSubsystem) {
    addCommands(
        new IntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem),
        new ParallelCommandGroup(
            new OuttakeCommand(intakeSubsystem)
                .withTimeout(3)
                .andThen(new StopIntakeCommand(intakeSubsystem)),
            new ShooterRampUpCommand(shooterSubsystem, ShooterMode.RAMP_SPEAKER)));
  }
}

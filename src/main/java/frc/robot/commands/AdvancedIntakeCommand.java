package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AdvancedIntakeCommand extends SequentialCommandGroup {

  public AdvancedIntakeCommand(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      PivotSubsystem pivotSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    addCommands(
        new IntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem, elevatorSubsystem),
        new ParallelCommandGroup(
            new SerializerBackupCommand(shooterSubsystem)
                .withTimeout(0.04)
                .andThen(new StopShooterCommand(shooterSubsystem))));
  }
}

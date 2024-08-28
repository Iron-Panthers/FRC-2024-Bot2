package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class VariableShooterCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private DoubleSupplier joystickSupplier;

  public VariableShooterCommand(
      ShooterSubsystem shooterSubsystem, DoubleSupplier joystickSupplier) {
    this.shooterSubsystem = shooterSubsystem;
    this.joystickSupplier = joystickSupplier;

    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.setVariableVelocity(
        MathUtil.clamp(
            shooterSubsystem.variableVelocity + joystickSupplier.getAsDouble() * 0.75, 10, 70));
  }

  @Override
  public boolean isFinished() {
    return false; // bad?
  }
}

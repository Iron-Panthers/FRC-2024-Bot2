// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  private IntakeMode intakeMode;

  public enum IntakeMode {
    INTAKE(Intake.Modes.INTAKE),
    HOLD(Intake.Modes.HOLD),
    REVERSE(Intake.Modes.REVERSE);

    public final IntakePowers modePowers; // rename to modePower

    private IntakeMode(IntakePowers modePowers) {
      this.modePowers = modePowers;
    }
  }

  public record IntakePowers(double intakeSpeed) {
    public IntakePowers(double intakeSpeed) {
      this.intakeSpeed = intakeSpeed;
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR_PORT);
    intakeMotor.clearStickyFaults();

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setInverted(false);

    intakeMode = IntakeMode.HOLD;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("intake voltage", () -> intakeMotor.getMotorVoltage().getValueAsDouble());
      tab.addString("Current Mode", () -> intakeMode.toString());
    }
  }

  public void setIntakeMode(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
  }

  private IntakeMode getIntakeMode() {
    return intakeMode;
  }

  @Override
  public void periodic() {
    intakeMotor.set(intakeMode.modePowers.intakeSpeed);
  }
}

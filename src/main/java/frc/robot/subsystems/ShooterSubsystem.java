// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX pivotShooterTopMotor; // pivotShooterTop
  private final TalonFX pivotShooterBottomMotor; // pivotShooterBottom
  private final TalonFX pivotAcceleratorMotor; // pivotAccelerator
  private final TalonFX serializerMotor; // ampShooter

  private DigitalInput shooterSensor;
  private DigitalInput serializerSensor;

  private ShooterMode shooterMode;

  // DEBUG
  private double d_topToBottomRatio = .068d;
  private GenericEntry ampRollerRatioEntry;

  private double d_ShooterSpeed = .5d;
  private GenericEntry shooterSpeedEntry;
  private GenericEntry useDebugControls;

  public double variableVelocity = Shooter.Modes.VARIABLE_VELOCITY.roller();

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withSlot(0);

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    SERIALIZER_BACKUP(Shooter.Modes.SERIALIZER_BACKUP),
    IDLE(Shooter.Modes.IDLE),
    RAMP_SPEAKER(Shooter.Modes.RAMP_SPEAKER),
    SHOOT_SPEAKER(Shooter.Modes.SHOOT_SPEAKER),
    SHOOT_AMP(Shooter.Modes.SHOOT_AMP),
    SHUTTLE(Shooter.Modes.RAMP_SHUTTLE),
    SHOOT_SHUTTLE(Shooter.Modes.SHOOT_SHUTTLE),
    ACCEL_SECURE(Shooter.Modes.ACCEL_SECURE),
    VARIABLE_VELOCITY(Shooter.Modes.VARIABLE_VELOCITY),
    SHOOT_VAR(Shooter.Modes.SHOOT_VAR),
    LOAD_SHOOTER(Shooter.Modes.LOAD_SHOOTER),
    SHOOTER_UNLOAD(Shooter.Modes.UNLOAD_SHOOTER),
    RAMP_AMP(Shooter.Modes.RAMP_AMP);

    public final ShooterPowers shooterPowers;

    private ShooterMode(ShooterPowers shooterPowers) {
      this.shooterPowers = shooterPowers;
    }
  }

  public record ShooterPowers(
      double roller, double topToBottomRatio, double accelerator, double serializerSpeed) {
    public ShooterPowers(
        double roller, double topToBottomRatio, double accelerator, double serializerSpeed) {
      this.roller = roller;
      this.topToBottomRatio = topToBottomRatio;
      this.accelerator = accelerator;
      this.serializerSpeed = serializerSpeed;
    }
  }

  public ShooterSubsystem() {
    pivotShooterBottomMotor = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    pivotShooterTopMotor = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    pivotAcceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    serializerMotor = new TalonFX(Intake.Ports.SERIALIZER_MOTOR_PORT);

    shooterSensor = new DigitalInput(Shooter.Ports.SHOOTER_BEAM_BREAK_SENSOR_PORT);
    serializerSensor = new DigitalInput(Shooter.Ports.SERIALIZER_BEAM_BREAK_SENSOR_PORT);

    pivotShooterBottomMotor.clearStickyFaults();
    pivotAcceleratorMotor.clearStickyFaults();
    pivotShooterTopMotor.clearStickyFaults();
    serializerMotor.clearStickyFaults();

    pivotShooterTopMotor.setControl(new Follower(pivotShooterBottomMotor.getDeviceID(), false));

    pivotAcceleratorMotor.setInverted(true);
    pivotShooterTopMotor.setInverted(true);
    pivotShooterBottomMotor.setInverted(true);
    serializerMotor.setInverted(true);

    serializerMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotAcceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotShooterBottomMotor.setNeutralMode(NeutralModeValue.Coast);
    pivotShooterTopMotor.setNeutralMode(NeutralModeValue.Coast);

    shooterMode = ShooterMode.IDLE;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      shooterTab.addBoolean("Shooter Sensor Input", this::isShooterBeamBreakSensorTriggered);
      shooterTab.addBoolean("Serializer Sensor Input", this::isSerializerBeamBreakSensorTriggered);
      shooterTab
          .addDouble(
              "Top Roller Velocity (RPS)",
              () -> pivotShooterBottomMotor.getVelocity().getValueAsDouble())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(2, 1);
      shooterTab.addDouble(
          "Serializer motor voltage", () -> serializerMotor.getMotorVoltage().getValueAsDouble());
      shooterTab
          .addDouble(
              "Bottom Roller Velocity (RPS)",
              () -> pivotShooterTopMotor.getVelocity().getValueAsDouble())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(2, 1);
      shooterTab.addDouble(
          "Top roller amps", () -> pivotShooterBottomMotor.getSupplyCurrent().getValueAsDouble());
      shooterTab.addDouble(
          "Bottom roller amps", () -> pivotShooterTopMotor.getSupplyCurrent().getValueAsDouble());
      shooterTab.addString("mode", () -> shooterMode.toString());

      ampRollerRatioEntry =
          shooterTab
              .add("DEBUG Amp Top to Bottom Roller Ratio", 1)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 1))
              .withSize(3, 1)
              .getEntry();
      shooterSpeedEntry =
          shooterTab
              .add("DEBUG Shooter Velocity", .5)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 90))
              .withSize(3, 1)
              .getEntry();
      useDebugControls =
          shooterTab
              .add("Use Debug Controls", false)
              .withWidget(BuiltInWidgets.kToggleSwitch)
              .withSize(2, 1)
              .getEntry();
    }
  }

  public ShooterMode getMode() {
    return shooterMode;
  }

  public double getSerializerPositon(){
    return serializerMotor.getRotorPosition().getValueAsDouble();
  }

  public boolean isShooterUpToSpeed() {
    return pivotShooterTopMotor.getVelocity().getValueAsDouble()
            >= Shooter.SHOOTER_VELOCITY_THRESHOLD
        && pivotShooterBottomMotor.getVelocity().getValueAsDouble()
            >= Shooter.SHOOTER_VELOCITY_THRESHOLD;
  }

  public boolean isShooterBeamBreakSensorTriggered() {
    return !shooterSensor.get();
  }

  public boolean isSerializerBeamBreakSensorTriggered() {
    // if is triggered return true
    return !serializerSensor.get();
  }

  public boolean isReadyToShootSpeaker() {
    return isShooterBeamBreakSensorTriggered() && isShooterUpToSpeed();
  }

  public void setShooterMode(ShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }

  public void setVariableVelocity(double velocity) {
    if (shooterMode != ShooterMode.VARIABLE_VELOCITY) shooterMode = ShooterMode.VARIABLE_VELOCITY;
    this.variableVelocity = velocity;
  }

  public void advanceToShootMode() {
    switch (shooterMode) {
      case RAMP_AMP:
        shooterMode = ShooterMode.SHOOT_AMP;
      case SHUTTLE:
        shooterMode = ShooterMode.SHOOT_SHUTTLE;
        break;
      case VARIABLE_VELOCITY:
        shooterMode = ShooterMode.SHOOT_VAR;
        break;
      case RAMP_SPEAKER:
      default:
        shooterMode = ShooterMode.SHOOT_SPEAKER;
    }
  }

  @Override
  public void periodic() {
    if (useDebugControls.getBoolean(false)) {
      // In debug use all of the debug values for our mode
      d_topToBottomRatio = ampRollerRatioEntry.getDouble(1);
      d_ShooterSpeed = shooterSpeedEntry.getDouble(0);
      pivotShooterTopMotor.setControl(velocityVoltageRequest.withVelocity(d_ShooterSpeed));
      pivotShooterBottomMotor.setControl(
          velocityVoltageRequest.withVelocity(d_ShooterSpeed * d_topToBottomRatio));
    } else if (shooterMode == ShooterMode.VARIABLE_VELOCITY
        || shooterMode == ShooterMode.SHOOT_VAR) {
      pivotShooterTopMotor.setControl(velocityVoltageRequest.withVelocity(variableVelocity));
      pivotShooterBottomMotor.setControl(velocityVoltageRequest.withVelocity(variableVelocity));
    } else {
      // Otherwise just use our current mode for the values
      pivotShooterTopMotor.setControl(
          velocityVoltageRequest.withVelocity(shooterMode.shooterPowers.roller()));
      pivotShooterBottomMotor.setControl(
          velocityVoltageRequest.withVelocity(
              shooterMode.shooterPowers.roller() * shooterMode.shooterPowers.topToBottomRatio()));
    }
    pivotAcceleratorMotor.set(shooterMode.shooterPowers.accelerator());
    serializerMotor.set(shooterMode.shooterPowers.serializerSpeed());
  }
}

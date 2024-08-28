// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Pivot;
import frc.robot.Constants.Pivot.Setpoints;
import frc.util.Util;
import java.util.Map;
import java.util.Optional;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor;

  private final CANcoder pivotCANcoder;

  private PIDController pidController;

  private double targetDegrees;
  private double pidVoltageOutput;
  private double calculatedTargetDegrees;

  private boolean inRange;
  private Pose2d pose;
  private double distance;
  private GenericEntry debugTarget;

  private double pastDebugTarget = 0;
  private boolean listenToDebug = true;

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    pivotMotor = new TalonFX(Pivot.Ports.PIVOT_MOTOR_PORT);

    pivotCANcoder = new CANcoder(Pivot.Ports.CANCODER_PORT);
    pivotCANcoder.getConfigurator().apply(Pivot.MotorConfigs.CANCODER_CONFIG);

    pivotMotor.getConfigurator().apply(Pivot.MotorConfigs.PIVOT_CONFIG);
    pivotMotor.setInverted(true);
    pivotMotor.clearStickyFaults();
    pivotMotor.set(0);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    pidController = new PIDController(0.3, 0, 0);

    targetDegrees = 0;
    pidVoltageOutput = 0;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      pivotTab.addNumber(
          "Current Motor Position", () -> pivotMotor.getPosition().getValueAsDouble());
      pivotTab.addNumber("Current Pivot Angle", this::getCurrentAngle);
      pivotTab.addBoolean("Is at target", this::isAtTargetDegrees);
      pivotTab.addNumber("Motor Error", this::getCurrentError);
      pivotTab.addNumber("PID Error", pidController::getPositionError);
      pivotTab.addNumber("Target Degrees", this::getTargetDegrees);
      pivotTab.addNumber("Applied Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());
      pivotTab.addDouble("PID Voltage Output", () -> pidVoltageOutput);
      pivotTab.addDouble("Calculated Target Angle", () -> calculatedTargetDegrees);
      pivotTab.addDouble("Distance", () -> distance);
      pivotTab.add(pidController);
      debugTarget =
          pivotTab
              .add("Debug target degrees", 23.5)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 15, "max", 90))
              .getEntry();
    }
  }

  private double getFeedForward() {
    return Math.cos(Math.toRadians(getCurrentAngle())) * Pivot.GRAVITY_VOLTAGE;
  }

  public double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  public double getCurrentAngle() {
    return rotationsToDegrees(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Util.epsilonEquals(getCurrentAngle(), targetDegrees, Pivot.EPSILON);
  }

  public boolean isReadyToShoot() {
    return isAtTargetDegrees() && inRange;
  }

  public void setTargetDegrees(double degrees) {
    this.targetDegrees = MathUtil.clamp(degrees, Setpoints.MINIMUM_ANGLE, Setpoints.MAXIMUM_ANGLE);
    listenToDebug = false;
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  public void calculatePivotTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double x = pose.getX();
    double y = pose.getY();
    double speakerX;
    double speakerY;
    Optional<Alliance> color = DriverStation.getAlliance();

    if (color.isPresent() && color.get() == Alliance.Red) {
      speakerX = Pivot.RED_SPEAKER_POSE.getX();
      speakerY = Pivot.RED_SPEAKER_POSE.getY();
    } else {
      speakerX = Pivot.BLUE_SPEAKER_POSE.getX();
      speakerY = Pivot.BLUE_SPEAKER_POSE.getY();
    }
    distance =
        (Math.sqrt(Math.pow((x - speakerX), 2) + Math.pow((y - speakerY), 2)))
            - Pivot.CENTER_OF_ROBOT_TO_BUMPER;
    targetDegrees =
        -0.006073 * Math.pow(distance, 6)
            + 0.167509 * Math.pow(distance, 5)
            - 1.803043 * Math.pow(distance, 4)
            + 9.309355 * Math.pow(distance, 3)
            - 21.517994 * Math.pow(distance, 2)
            + 6.8762 * distance
            + 64.78029
            - 2.7;
  }

  public double getAngularError() {
    return targetDegrees - getCurrentAngle();
  }

  @Override
  public void periodic() {

    double currentTarget = debugTarget.getDouble(23.5);

    if (currentTarget != pastDebugTarget) {
      pastDebugTarget = currentTarget;
      listenToDebug = true;
    }

    double pidOutput =
        pidController.calculate(
            getCurrentAngle(),
            MathUtil.clamp(targetDegrees, Setpoints.MINIMUM_ANGLE, Setpoints.MAXIMUM_ANGLE));

    pidVoltageOutput = MathUtil.clamp(pidOutput + getFeedForward(), -10, 10);

    pivotMotor.setVoltage(pidVoltageOutput);
  }
}

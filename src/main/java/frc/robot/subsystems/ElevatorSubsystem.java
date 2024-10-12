package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

// private PIDController controller;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  private TalonFX elevatorMotor;
  private PIDController elevatorController;
  private PIDController armController;
  private double targetHeight;
  private double targetAngle;
  private double elevatorMotorPower;
  private double armMotorPower;
  private double currentHeight;
  private double currentAngle;

  private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotor = new TalonFX(Elevator.Ports.ELEVATOR_MOTOR_PORT);
    armMotor = new TalonFX(Elevator.Ports.ARM_MOTOR_PORT);

    elevatorMotor.clearStickyFaults();
    armMotor.clearStickyFaults();
    elevatorMotor.setPosition(0);
    armMotor.setPosition(0);

    // FIXME placeholder values
    elevatorController = new PIDController(0.4, 0, 0.0125);
    armController = new PIDController(0.2, 0, 0);

    ElevatorTab.addNumber("Target Height", () -> this.targetHeight);
    ElevatorTab.addNumber("Target Angle", () -> this.targetAngle);
    ElevatorTab.addNumber("Current Height", () -> this.currentHeight);
    ElevatorTab.add(elevatorController);
    ElevatorTab.add(armController);

    targetHeight = 0;
    currentHeight = 0;
    targetAngle = 0;
    currentAngle = 0;
  }

  public double getArmPosition() {
    return armInchesToRotations(armMotor.getPosition().getValueAsDouble());
  }

  public double getElevatorPosition() {
    return inchesToRotations(elevatorMotor.getPosition().getValueAsDouble());
  }

  public static double inchesToRotations(double height) {
    return height / Elevator.ELEVATOR_GEAR_RATIO;
  }

  public static double rotationsToInches(double rotations) {
    return rotations * Elevator.ELEVATOR_GEAR_RATIO;
  }

  public static double armInchesToRotations(double degrees) {
    return degrees / Elevator.Arm.ARM_GEAR_RATIO;
  }

  public static double rotationsToArmInches(double rotations) {
    return rotations * Elevator.Arm.ARM_GEAR_RATIO;
  }

  public void setTargetHeight(double targetHeight) {
    this.targetHeight = MathUtil.clamp(targetHeight, Elevator.MAX_HEIGHT, 0);
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = MathUtil.clamp(targetAngle, Elevator.Arm.MAX_ANGLE, 0);
  }

  public boolean atTargetAngle() {
    // replace with epsilon equals
    if (targetAngle - 0.5 <= getArmPosition() && getArmPosition() <= targetAngle + 0.5) {
      return true;
    }
    return false;
  }

  public boolean atTargetHeight() {
    // if (targetHeight - 0.5 <= getElevatorPosition()
    //     && getElevatorPosition() <= targetHeight + 0.5) {
    //   return true;
    // }
    // return false;
    return true;
  }

  public double calculateArmFeedforward() {
    return 0; // FIXME
    // using a constant force spring
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorMotorPower = elevatorController.calculate(getElevatorPosition(), targetHeight);
    armMotorPower = armController.calculate(getArmPosition(), targetAngle);
    elevatorMotor.setVoltage(
        MathUtil.clamp(elevatorMotorPower + Elevator.ELEVATOR_FEEDFORWARD, -10, 10));
    armMotor.setVoltage(MathUtil.clamp(armMotorPower + calculateArmFeedforward(), -10, 10));
  }
}

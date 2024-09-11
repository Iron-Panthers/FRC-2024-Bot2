package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

// private PIDController controller;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX motor;
    private PIDController controller;
    private double targetHeight;
    private double targetAngle;
    private double motorPower;
    private double currentHeight;
    private double currentAngle;
    private final ShuffleboardTab ElevatorTab = Shuffleboard.getTab("Elevator");

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {

        motor = new TalonFX(15);

        motor.clearStickyFaults();

        controller = new PIDController(0.4, 0, 0.0125);

        ElevatorTab.addNumber("Current Motor Power", () -> this.motorPower);
        ElevatorTab.addNumber("Target Height", () -> this.targetHeight);
        ElevatorTab.addNumber("Target Angle", () -> this.targetAngle);
        // ElevatorTab.addNumber("PID output", () -> this.controller);
        ElevatorTab.addNumber("Current Height", () -> this.currentHeight);
        ElevatorTab.add(controller);

        targetHeight = 0;
        currentHeight = 0;
        targetAngle = 0;
        currentAngle = 0;
    }

    public static double inchesToRotations(double height) {
        return height / Elevator.GEAR_RATIO;
    }

    public static double rotationsToInches(double rotations) {
        return rotations * Elevator.GEAR_RATIO;
    }

    public static double degreesToRotations(double degrees) {
        return degrees / Elevator.GEAR_RATIO;
    }

    public static double rotationsToDegrees(double rotations) {
        return rotations * Elevator.GEAR_RATIO;
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        motorPower = controller.calculate(inchesToRotations(targetHeight), targetHeight);
        motorPower = controller.calculate(degreesToRotations(targetAngle), targetAngle);
    }

    public boolean nearTargetHeight() {
        if (targetHeight - 0.5 <= currentHeight && currentHeight <= targetHeight + 0.5) {
            return true;
        }
        return false;
    }

    public boolean nearTargetAngle() {
        if (targetAngle - 1 <= currentAngle && currentAngle <= targetAngle + 1) {
            return true;
        }
        return false;
    }
}

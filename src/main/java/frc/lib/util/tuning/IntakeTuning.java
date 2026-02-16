package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Intake;

public class IntakeTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Intake);
	private static Trigger runIntake = baseTrigger.and(() -> tester.getAButton());
	private static Trigger pidPosition = baseTrigger.and(() -> tester.getBButton());
	private static Trigger pidMotionPosition = baseTrigger.and(() -> tester.getYButton());
	private static Trigger setPosition = baseTrigger.and(() -> tester.getXButton());

	private static double targetVoltage = 0.0;
	private static Angle targetAngle = Degrees.of(0);

	public static void init(Intake intake) {
		DogLog.tunable("Intake/voltage", 0.0, target -> targetVoltage = target);
		DogLog.tunable("Intake/Deploy PID/angle", 0.0, target -> targetAngle = Degrees.of(target));
		intake.logTuning();

		runIntake.whileTrue(
				Commands.run(() -> intake.setIntakeVoltage(targetVoltage), intake)
						.handleInterrupt(() -> intake.disable()));
		pidPosition.whileTrue(
				Commands.run(() -> intake.setIntakeAngle(targetAngle))
						.handleInterrupt(() -> intake.disable()));
		pidMotionPosition.whileTrue(
				Commands.run(() -> intake.setIntakeMotionAngle(targetAngle))
						.handleInterrupt(() -> intake.disable()));

		setPosition.onTrue(Commands.runOnce(() -> intake.resetPosition(targetAngle)));
	}

	public static void logPID(String key, TalonFX motor, TalonFXConfiguration config) {
		DogLog.tunable(key + "/kP", config.Slot0.kP,
				k -> motor.getConfigurator().apply(config.Slot0.withKP(k)));
		DogLog.tunable(key + "/kI", config.Slot0.kI,
				k -> motor.getConfigurator().apply(config.Slot0.withKI(k)));
		DogLog.tunable(key + "/kD", config.Slot0.kD,
				k -> motor.getConfigurator().apply(config.Slot0.withKD(k)));
		DogLog.tunable(key + "/kS", config.Slot0.kS,
				k -> motor.getConfigurator().apply(config.Slot0.withKS(k)));
		DogLog.tunable(key + "/kG", config.Slot0.kG,
				k -> motor.getConfigurator().apply(config.Slot0.withKG(k)));
		DogLog.tunable(key + "/vel",
				RotationsPerSecond.of(config.MotionMagic.MotionMagicCruiseVelocity)
						.in(DegreesPerSecond),
				k -> motor.getConfigurator()
						.apply(config.MotionMagic.withMotionMagicCruiseVelocity(
								DegreesPerSecond.of(k))));
		DogLog.tunable(key + "/acc",
				RotationsPerSecondPerSecond.of(config.MotionMagic.MotionMagicAcceleration)
						.in(DegreesPerSecondPerSecond),
				k -> motor.getConfigurator()
						.apply(config.MotionMagic.withMotionMagicAcceleration(
								DegreesPerSecondPerSecond.of(k))));
	}
}

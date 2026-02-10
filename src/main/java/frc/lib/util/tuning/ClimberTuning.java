package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Climber;

public class ClimberTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Climber);

	private static Trigger runClimber = baseTrigger.and(() -> tester.getYButton());
	private static Trigger unwind = baseTrigger.and(() -> tester.getXButton());
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getAButton());
	private static Trigger resetPosition = baseTrigger.and(() -> tester.getBButton());

	private static Distance targetPosition = Meters.zero();
	private static double voltage = 2.0;

	public static void init(Climber climber) {
		DogLog.tunable("Climber/target", 0.0, target -> targetPosition = Meters.of(target));
		DogLog.tunable("Climber/voltage", voltage, target -> voltage = target);
		climber.logPID();

		runVoltage.whileTrue(Commands.run(() -> climber.setVoltage(voltage)).handleInterrupt(() -> climber.disable()));
		unwind.whileTrue(Commands.run(() -> climber.setVoltage(-voltage)).handleInterrupt(() -> {
			climber.disable();
		}));

		runClimber.whileTrue(
				Commands.run(() -> climber.setClimberPosition(targetPosition), climber)
						.handleInterrupt(() -> climber.disable()));
		resetPosition.onTrue(Commands.runOnce(() -> climber.resetClimberPosition()));
	}

	public static void createPID(String key, TalonFX motor, TalonFXConfiguration config) {
		DogLog.tunable(key + "/kP", config.Slot0.kP,
				newP -> motor.getConfigurator().apply(config.Slot0.withKP(newP)));
		DogLog.tunable(key + "/kI", config.Slot0.kI,
				newI -> motor.getConfigurator().apply(config.Slot0.withKI(newI)));
		DogLog.tunable(key + "/kD", config.Slot0.kD,
				newD -> motor.getConfigurator().apply(config.Slot0.withKD(newD)));
		DogLog.tunable(key + "/kS", config.Slot0.kS,
				newS -> motor.getConfigurator().apply(config.Slot0.withKS(newS)));
		DogLog.tunable(key + "/kV", config.Slot0.kV,
				newV -> motor.getConfigurator().apply(config.Slot0.withKV(newV)));
		DogLog.tunable(key + "/kG", config.Slot0.kG,
				newG -> motor.getConfigurator().apply(config.Slot0.withKG(newG)));
	}
}


package frc.lib.util.tuning;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Indexer;

public class IndexerTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Indexer);
	private static Trigger runVelocity = baseTrigger.and(() -> tester.getXButton());
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getAButton());

	private static double targetRPS = 0;
	private static double voltage = 0;

	public static void init(Indexer indexer) {
		DogLog.tunable("Indexer/target rps", 0.0, target -> targetRPS = target);
		DogLog.tunable("Indexer/voltage", 0.0, target -> voltage = target);
		indexer.logPID();

		runVoltage.whileTrue(Commands.run(() -> {
			indexer.runVoltage(voltage);
		}).handleInterrupt(() -> indexer.disable()));

		runVelocity.whileTrue(Commands.run(() -> {
			indexer.runVelocity(targetRPS);
		}).handleInterrupt(() -> {
			indexer.disable();
		}));
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

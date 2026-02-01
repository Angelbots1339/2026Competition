package frc.lib.util.tuning;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Shooter);
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());

	private static double shooterTargetRPS = ShooterConstants.shootRPS;
	private static double spinnerTargetRPS = ShooterConstants.shootRPS;

	public static void init(Shooter shooter) {
		DogLog.tunable(TuningConstants.ShooterTuningConstants.ShooterTargetNTName, ShooterConstants.shootRPS,
				target -> shooterTargetRPS = target);
		DogLog.tunable(TuningConstants.ShooterTuningConstants.SpinnerTargetNTName, ShooterConstants.shootRPS,
				target -> spinnerTargetRPS = target);

		shooter.logPIDTuning();

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setRPS(shooterTargetRPS, spinnerTargetRPS);
		}).handleInterrupt(() -> shooter.disable()));
	}

	public static void createMotorPID(String key, TalonFX motor, TalonFXConfiguration config) {
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

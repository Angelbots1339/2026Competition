package frc.lib.util;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());

	private static double targetRPS = ShooterConstants.shootRPS;

	public static void init(Shooter shooter) {
		DogLog.tunable("/target", ShooterConstants.shootRPS, target -> targetRPS = target);
		createPID(shooter.leader, ShooterConstants.config);

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setVelocityFOC(targetRPS);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));
	}

	public static void createPID(TalonFX motor, TalonFXConfiguration config) {
		DogLog.tunable("/kP", config.Slot0.kP,
				newP -> motor.getConfigurator().apply(config.Slot0.withKP(newP)));
		DogLog.tunable("/kI", config.Slot0.kI,
				newI -> motor.getConfigurator().apply(config.Slot0.withKI(newI)));
		DogLog.tunable("/kD", config.Slot0.kD,
				newD -> motor.getConfigurator().apply(config.Slot0.withKD(newD)));
		DogLog.tunable("/kS", config.Slot0.kS,
				newS -> motor.getConfigurator().apply(config.Slot0.withKS(newS)));
		DogLog.tunable("/kV", config.Slot0.kV,
				newV -> motor.getConfigurator().apply(config.Slot0.withKV(newV)));
		DogLog.tunable("/kG", config.Slot0.kG,
				newG -> motor.getConfigurator().apply(config.Slot0.withKG(newG)));
	}
}

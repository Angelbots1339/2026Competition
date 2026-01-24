package frc.lib.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getYButton());
	private static Trigger pidtune = baseTrigger.and(() -> tester.getAButton());

	private static Shooter shooter;

	public static void init(Shooter shooter) {
		ShooterTuning.shooter = shooter;
		shooter.logTuning();
		runVoltage.whileTrue(Commands.run(() -> {
			shooter.setVoltage();
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));
		pidtune.whileTrue(Commands.run(() -> {
			shooter.leader.getConfigurator().apply(shooter.getPID());
			shooter.setVelocity(
					RotationsPerSecond.of(SmartDashboard.getNumber(TuningConstants.Shooter.velocityNTName, 10)));
		}));
	}
}

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Shooter);
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getYButton());
	private static Trigger pidtune = baseTrigger.and(() -> tester.getAButton());
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());

	private static double targetAngle = 0;

	public static void init(Shooter shooter) {
		shooter.logTuning();

		runVoltage.whileTrue(Commands.run(() -> {
			shooter.setVoltage(Volts.of(SmartDashboard.getNumber(TuningConstants.Shooter.voltageNTName, 0)));
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));

		pidtune.or(pidtuneFOC).onTrue(Commands.runOnce(() -> {
			shooter.leader.getConfigurator().apply(shooter.getPID());
			targetAngle = SmartDashboard.getNumber(TuningConstants.Shooter.velocityNTName, 0);
		}));

		pidtune.whileTrue(Commands.run(() -> {
			shooter.setVelocity(targetAngle);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setVelocityFOC(targetAngle);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));
	}
}

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.regression.ShooterRegression;
import frc.robot.subsystems.Shooter;

public class ShooterTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Shooter);
	private static Trigger runShooter = baseTrigger.and(() -> tester.getXButton());
	private static Trigger runVoltage = baseTrigger.and(() -> tester.getAButton());
	private static Trigger shootReg = baseTrigger.and(() -> tester.getYButton());
	private static Trigger addData = baseTrigger.and(() -> tester.getBButton());
	private static Trigger clearData = baseTrigger.and(() -> tester.getStartButton());

	private static List<double[]> regressionData = new ArrayList<double[]>();

	private static double shooterTargetRPS = ShooterConstants.shootRPS;
	private static double spinnerTargetRPS = ShooterConstants.shootRPS;
	private static double kickerRPS = ShooterConstants.KickerRPS;
	private static double voltage = 0;
	private static Distance distance = Meters.zero();

	public static void init(Shooter shooter) {
		DogLog.tunable("Shooter/Spinner target", ShooterConstants.shootRPS,
				target -> spinnerTargetRPS = target);
		DogLog.tunable("Shooter/Shooter target", ShooterConstants.shootRPS,
				target -> shooterTargetRPS = target);
		DogLog.tunable("Shooter/voltage", 0.0, target -> voltage = target);
		DogLog.tunable("Shooter/kicker velocity", kickerRPS, target -> kickerRPS = target);
		DogLog.tunable("Shooter/distance", 0.0, target -> distance = Meters.of(target));
		shooter.logPID();

		runVoltage.whileTrue(Commands.run(() -> {
			shooter.setVoltage(Volts.of(voltage));
			shooter.setKickerVelocity(kickerRPS);
		}).handleInterrupt(() -> shooter.disable()));

		runShooter.whileTrue(Commands.run(() -> {
			shooter.setRPS(shooterTargetRPS, spinnerTargetRPS);
			if (shooter.atSetpoint()) {
				shooter.setKickerVelocity(kickerRPS);
			}
		}).handleInterrupt(() -> {
			shooter.disable();
		}));

		shootReg.whileTrue(Commands.run(() -> {
			double[] speed = ShooterRegression.getRegressionRPS(distance.in(Meters));
			shooter.setRPS(speed[0], speed[1]);
			if (shooter.atSetpoint()) {
				shooter.setKickerVelocity(kickerRPS);
			}
		}).handleInterrupt(() -> {
			shooter.disable();
		}));

		clearData.onTrue(Commands.runOnce(() -> {
			ShooterRegression.shotRPSMap.clear();
			regressionData.clear();
		}));

		addData.onTrue(Commands.runOnce(() -> {
			double[] data = { distance.in(Meters), shooterTargetRPS, spinnerTargetRPS };
			regressionData.add(data);
			ShooterRegression.shotRPSMap.put(distance.in(Meters),
					new double[] { shooterTargetRPS, spinnerTargetRPS });
			DogLog.log("regression data/" + regressionData.size(), data);
		}));
	}
}

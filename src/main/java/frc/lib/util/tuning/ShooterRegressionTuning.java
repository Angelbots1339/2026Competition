package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ShooterRegressionTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);
	private static Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-tester.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private static Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-tester.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.ShooterRegression);
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());

	private static double shooterTargetRPS = ShooterConstants.shootRPS;
	private static double spinnerTargetRPS = ShooterConstants.shootRPS;

	public static void init(Swerve swerve, Shooter shooter) {
		DogLog.tunable(TuningConstants.ShooterTuningConstants.ShooterTargetNTName, ShooterConstants.shootRPS,
				target -> shooterTargetRPS = target);
		DogLog.tunable(TuningConstants.ShooterTuningConstants.SpinnerTargetNTName, ShooterConstants.shootRPS,
				target -> spinnerTargetRPS = target);

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setRPS(shooterTargetRPS, spinnerTargetRPS);
		}).handleInterrupt(() -> shooter.setVoltage(Volts.of(0))));

		baseTrigger.whileTrue(swerve.pointDriveCommand(leftY, leftX, () -> FieldUtil.getHubCenter(), () -> true));
	}
}

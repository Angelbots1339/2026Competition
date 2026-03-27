// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.commands.RegressionShoot;
import frc.robot.commands.Shoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class RegressionTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Regression);
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getRightTriggerAxis() > 0.2);
	private static Trigger regression = baseTrigger.and(() -> tester.getRightBumperButton());
	private static Trigger runIntake = baseTrigger.and(() -> tester.getLeftBumperButton());

	private static Trigger addData = baseTrigger.and(() -> tester.getXButton());
	private static Trigger clearData = baseTrigger.and(() -> tester.getStartButton());
	private static Trigger drive = baseTrigger.and(() -> tester.getLeftTriggerAxis() > 0.2);
	private static Trigger resetGyro = baseTrigger.and(() -> tester.getBButton());

	private static Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-tester.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private static Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-tester.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private static Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-tester.getRightX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);

	private static double shooterRPS = 0.0;
	private static double spinnerRPS = 0.0;
	private static List<double[]> regressionData = new ArrayList<double[]>();

	public static void init(Swerve swerve, Shooter shooter, Indexer indexer, Intake intake) {
		DogLog.tunable("Regression/Shooter RPS Target", 0.0, target -> shooterRPS = target);
		DogLog.tunable("Regression/Spinner RPS Target", 0.0, target -> spinnerRPS = target);

		resetGyro.onTrue(swerve.run(swerve::resetGyro));
		pidtuneFOC.whileTrue(Commands.parallel(
				swerve.run(() -> swerve.angularDriveRequest(leftY, leftX, () -> {

					return Rotation2d.fromRadians(
							FieldUtil.getHubCenter().getTranslation().minus(swerve.getPose().getTranslation())
									.getAngle().plus(Rotation2d.k180deg).getRadians());
				}, () -> true)),
				new Shoot(shooter, indexer, intake, () -> shooterRPS, () -> spinnerRPS, swerve::atRotation)));

		regression.whileTrue(new RegressionShoot(swerve, shooter, indexer, intake, leftY, leftX));
		drive.whileTrue(swerve.driveCommand(leftY, leftX, rightX, () -> true));
		runIntake.whileTrue(intake.runIntake());

		clearData.onTrue(Commands.runOnce(() -> {
			ShooterRegression.shotRPSMap.clear();
			regressionData.clear();
		}));

		addData.onTrue(Commands.runOnce(() -> {
			double[] data = { swerve.getDistanceToHub(), shooterRPS, spinnerRPS };
			regressionData.add(data);
			ShooterRegression.shotRPSMap.put(swerve.getDistanceToHub(),
					new double[] { shooterRPS, spinnerRPS });
			DogLog.log("regression data/" + regressionData.size(), data);
		}));
	}
}

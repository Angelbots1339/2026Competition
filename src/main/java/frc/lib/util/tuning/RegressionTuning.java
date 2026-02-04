// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
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
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.commands.Shoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class RegressionTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Regression);
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());
	private static Trigger drive = baseTrigger.and(() -> tester.getAButton());
	private static Trigger regression = baseTrigger.and(() -> tester.getYButton());

	private static Trigger addData = baseTrigger.and(() -> tester.getBButton());
	private static Trigger clearData = baseTrigger.and(() -> tester.getStartButton());

	private static Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-tester.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private static Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-tester.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);

	private static double targetRPS = ShooterConstants.shootRPS;
	private static List<double[]> regressionData = new ArrayList<double[]>();

	public static void init(Swerve swerve, Shooter shooter) {
		DogLog.tunable("Regression/target", ShooterConstants.shootRPS, target -> targetRPS = target);

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setRPS(targetRPS);
			shooter.runIndex(2);
		}).handleInterrupt(() -> {
			shooter.disable();
		}));

		regression.whileTrue(new Shoot(swerve, shooter, leftY, leftX, () -> true));

		drive.whileTrue(swerve.pointDriveCommand(leftY, leftX, () -> FieldUtil.getHubCenter(), () -> true));

		clearData.onTrue(Commands.runOnce(() -> ShooterRegression.shotRPSMap.clear()));
		addData.onTrue(Commands.runOnce(() -> {
			double[] data = { swerve.getDistanceToHub(), targetRPS, targetRPS };
			regressionData.add(data);
			ShooterRegression.shotRPSMap.put(swerve.getDistanceToHub(),
					new double[] { targetRPS, targetRPS });
			DogLog.log("regression data/" + regressionData.size(), data);
		}));
	}
}

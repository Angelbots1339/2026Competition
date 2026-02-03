// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.tuning;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants.TuningConstants.TuningMode;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class RegressionTuning {
	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(
			() -> DriverStation.isTestEnabled() && TuningManager.tuningMode == TuningMode.Shooter);
	private static Trigger pidtune = baseTrigger.and(() -> tester.getAButton());
	private static Trigger pidtuneFOC = baseTrigger.and(() -> tester.getXButton());
	private static Trigger drive = baseTrigger.and(() -> tester.getAButton());
	private static Trigger regression = baseTrigger.and(() -> tester.getYButton());

	private static Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-tester.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private static Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-tester.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);

	private static double targetRPS = 0;

	public static void init(Swerve swerve, Shooter shooter) {
		shooter.logTuning();
		pidtune.or(pidtuneFOC).onTrue(Commands.runOnce(() -> {
			shooter.leader.getConfigurator().apply(shooter.getPID());
			targetRPS = SmartDashboard.getNumber(TuningConstants.Shooter.velocityNTName, 0);
		}));

		pidtuneFOC.whileTrue(Commands.run(() -> {
			shooter.setVelocityFOC(targetRPS);
			shooter.runIndex(2);
		}).handleInterrupt(() -> {
			shooter.setVoltage(Volts.of(0));
			shooter.disableIndex();
		}));

		regression.whileTrue(new Shoot(swerve, shooter, leftY, leftX));

		drive.whileTrue(swerve.pointDriveCommand(leftY, leftX, () -> FieldUtil.getHubCenter(), () -> true));
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AlignUtil;
import frc.lib.util.FieldUtil;
import frc.lib.util.tuning.TuningManager;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
	@Logged(name = "Driver Controller")
	private XboxController driver = new XboxController(DriverConstants.DriverPort);

	private Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-driver.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-driver.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-driver.getRightX(), true)
			* RobotConstants.maxRot.in(RadiansPerSecond);

	@Logged(importance = Importance.CRITICAL)
	private Swerve swerve = TunerConstants.swerve;
	// private Intake intake = new Intake();
	// private Shooter shooter = new Shooter();
	private Climber climber = new Climber();

	@Logged(name = "Reset Gyro")
	private Trigger resetGyro = new Trigger(() -> driver.getStartButton());

	private Trigger pidtoPose = new Trigger(() -> driver.getBButton());
	@Logged(name = "Point Drive")
	private Trigger pointDrive = new Trigger(() -> driver.getXButton());

	@Logged(name = "Bump Drive")
	private Trigger bumpDrive = new Trigger(() -> driver.getYButton());

	@Logged(name = "Snake Drive")
	private Trigger snakeDrive = new Trigger(() -> driver.getAButton());

	// priate Trigger deployIntake = new Trigger(() ->
	// driver.getLeftBumperButton());

	@Logged(name = "Current Auto")
	private AutoChooser autoChooser = new AutoChooser();
	private Autos autos = new Autos(swerve);

	public RobotContainer() {
		configureBindings();
		configureControllerAlerts();
		setDefaultCommands();

		autoChooser.addCmd("Hub Depot Outpost Tower",
				autos::hubDepotOutpostTowerAuto);
		autoChooser.addCmd("Hub Depot Tower", autos::hubDepotTowerAuto);
		autoChooser.addCmd("bump test", autos::bumpTest);
		autoChooser.addCmd("left neutral", autos::leftNeutralAuto);
		autoChooser.addCmd("right neutral", autos::rightNeutralAuto);
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configureBindings() {
		swerve.setDefaultCommand(swerve.driveCommand(leftY, leftX, rightX, () -> true));

		resetGyro.onTrue(Commands.runOnce(() -> swerve.resetGyro(), swerve));
		pidtoPose.whileTrue(swerve.defer(() -> AlignUtil.driveToClimbPosition(swerve)));
		pointDrive.whileTrue(swerve.pointDriveCommand(leftY, leftX, () -> FieldUtil.getHubCenter(), () -> true));
		bumpDrive.whileTrue(
				Commands.run(() -> swerve.angularDriveRequest(leftY, leftX, () -> swerve.getClosest15(), () -> true),
						swerve));

		snakeDrive.whileTrue(Commands.run(() -> swerve.angularDriveRequest(leftY,
				leftX, () -> {
					ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
							swerve.getYaw());
					// prevent turning when at very low speeds
					if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1) {
						return swerve.getYaw();
					}
					return Rotation2d.fromRadians(Math.atan2(speeds.vyMetersPerSecond,
							speeds.vxMetersPerSecond));
				}, () -> true), swerve));
		// deployIntake.onTrue(Commands.run(() -> {
		// intake.setWristAngle(IntakeConstants.deployedAngle);
		// intake.setIntakeVelocity(IntakeConstants.intakeVelocity);
		// }, intake))
		// .onFalse(Commands.run(() -> {
		// intake.setWristAngle(IntakeConstants.retractedAngle);
		// intake.setIntakeVelocity(0);
		// }, intake));
	}

	public void configureControllerAlerts() {
		// 10/20sec endgame alert
		new Trigger(() -> {
			return DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
					&& ((DriverStation.getMatchTime() <= Math.round(20)
							&& DriverStation.getMatchTime() > Math.round(13))
							|| DriverStation.getMatchTime() <= Math.round(10));
		})
				.onTrue(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 1.0);
				}).withTimeout(1.5).andThen(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 0.0);
				}).withTimeout(1.0)));

		// 5 sec before hub switch alert
		new Trigger(() -> {
			return DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
					&& FieldUtil.getShiftTimeLeft() <= Math.round(5);
		})
				.onTrue(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 1.0);
				}).withTimeout(1.5).andThen(Commands.run(() -> {
					driver.setRumble(RumbleType.kBothRumble, 0.0);
				}).withTimeout(1.0)));
	}

	public void setDefaultCommands() {
	}

	@Logged(name = "Current auto")
	public Command getAutonomousCommand() {
		return autoChooser.selectedCommand();
	}

	public void testingInit() {
		TuningManager.init(swerve, null, null, climber);
	}

	@Logged(importance = Importance.CRITICAL, name = "Is Hub Active")
	public boolean isHubActive() {
		return FieldUtil.isHubActive();
	}
}

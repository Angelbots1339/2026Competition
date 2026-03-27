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
import frc.lib.util.FieldUtil;
import frc.lib.util.tuning.TuningManager;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.RegressionShoot;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
	@Logged(name = "Driver Controller")
	private XboxController driver = new XboxController(DriverConstants.DriverPort);
	// private XboxController operater = new
	// XboxController(DriverConstants.OperatorPort);

	@Logged(importance = Importance.CRITICAL)
	private Swerve swerve = TunerConstants.swerve;
	private Indexer indexer = new Indexer();
	private Intake intake = new Intake();
	private Shooter shooter = new Shooter();

	@Logged(name = "Reset Gyro")
	private Trigger resetGyro = new Trigger(() -> driver.getStartButton());

	private Trigger pass = new Trigger(() -> driver.getBButton());

	@Logged(name = "Bump Drive")
	private Trigger bumpDrive = new Trigger(() -> driver.getYButton());

	@Logged(name = "Snake Drive")
	private Trigger snakeDrive = new Trigger(() -> driver.getAButton());

	@Logged(name = "Shoot")
	private Trigger shoot = new Trigger(() -> driver.getRightTriggerAxis() > 0.2);

	private Trigger trenchShot = new Trigger(() -> driver.getXButton());

	@Logged(name = "Spin Up")
	private Trigger shooterSpinup = new Trigger(() -> driver.getRightBumperButton());

	@Logged(name = "Run Intake")
	private Trigger runIntake = new Trigger(() -> driver.getLeftTriggerAxis() > 0.2);

	private Trigger toggleIntakeDeploy = new Trigger(() -> driver.getLeftBumperButton());

	private Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-driver.getLeftY(), true)
			* (runIntake.getAsBoolean() ? RobotConstants.maxSpeed.in(MetersPerSecond) * 0.6
					: RobotConstants.maxSpeed.in(MetersPerSecond));
	private Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-driver.getLeftX(), true)
			* (runIntake.getAsBoolean() ? RobotConstants.maxSpeed.in(MetersPerSecond) * 0.6
					: RobotConstants.maxSpeed.in(MetersPerSecond));
	private Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-driver.getRightX(), true)
			* (runIntake.getAsBoolean() ? RobotConstants.maxSpeed.in(MetersPerSecond) * 0.8
					: RobotConstants.maxRot.in(RadiansPerSecond));

	// private Trigger reverse = new Trigger(() -> operater.getXButton());

	@Logged(name = "Current Auto")
	private AutoChooser autoChooser = new AutoChooser();
	private Autos autos = new Autos(swerve, shooter, intake, indexer);

	public RobotContainer() {
		configureBindings();
		configureControllerAlerts();
		setDefaultCommands();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.addCmd("Hub Depot", autos::hubDepotAuto);
		autoChooser.addCmd("Hub Depot Neutral", autos::hubDepotNeutralAuto);
		autoChooser.addRoutine("Right 2x Neutral", autos::rightNeutral);
		autoChooser.addRoutine("Left 2x Neutral", autos::leftNeutral);
		autoChooser.addRoutine("Right Neutral Sweep", autos::rightNeutralSweep);
		autoChooser.addRoutine("Left Neutral Sweep", autos::leftNeutralSweep);
		autoChooser.addRoutine("Custom Auto", autos::customAuto);
		autoChooser.select("Nothing");
	}

	private void configureBindings() {
		swerve.setDefaultCommand(swerve.driveCommand(leftY, leftX, rightX, () -> true));
		resetGyro.onTrue(Commands.runOnce(() -> swerve.resetGyro(), swerve));
		pass.whileTrue(Commands.parallel(
				swerve.run(() -> swerve.angularDriveRequest(leftY, leftX,
						() -> FieldUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero, () -> true)),
				new Shoot(shooter, indexer, intake, () -> 45.0, () -> 45.0, swerve::atRotation)));
		bumpDrive.whileTrue(
				Commands.run(() -> swerve.angularDriveRequest(leftY, leftX, () -> swerve.getClosestBumpAngle(),
						() -> true),
						swerve));
		snakeDrive.whileTrue(Commands.run(() -> swerve.angularDriveRequest(leftY,
				leftX, () -> {
					ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
							swerve.getRobotRelativeSpeeds(),
							swerve.getYaw());

					// prevent turning when at very low speeds
					if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1) {
						return swerve.getYaw();
					}
					return Rotation2d.fromRadians(Math.atan2(speeds.vyMetersPerSecond,
							speeds.vxMetersPerSecond));
				}, () -> true), swerve));

		shoot.whileTrue(new RegressionShoot(swerve, shooter, indexer, intake, leftY, leftX));
		shooterSpinup.whileTrue(shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));

		runIntake.whileTrue(intake.runIntake()
				.alongWith(indexer.run(() -> indexer.runVoltage(IndexerConstants.IntakeIndexerVoltage))));
		toggleIntakeDeploy.toggleOnTrue(intake.retract());
		trenchShot.whileTrue(new Shoot(shooter, indexer, intake, () -> 45.0, () -> 12.6, () -> true));

		// reverse.whileTrue(Commands.parallel(
		// shooter.run(() -> shooter.setKickerVelocity(-ShooterConstants.KickerRPS)),
		// indexer.run(() -> indexer.runVoltage(-IndexerConstants.IndexerVolts)),
		// intake.run(() -> intake.setIntakeVoltage(-IntakeConstants.IntakeVoltage))));
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
		intake.setDefaultCommand(intake.runOnce(intake::disable).andThen(intake.deploy()));
		indexer.setDefaultCommand(indexer.run(indexer::disable));
		shooter.setDefaultCommand(shooter.run(shooter::disable));
	}

	@Logged(name = "Current auto")
	public Command getAutonomousCommand() {
		return autoChooser.selectedCommand();
	}

	public void testingInit() {
		if (Constants.useTesting)
			TuningManager.init(swerve, shooter, intake, indexer);
	}

	@Logged(importance = Importance.CRITICAL, name = "Is Hub Active")
	public String isHubActive() {
		return FieldUtil.getHubState();
	}

	@Logged(importance = Importance.CRITICAL, name = "Shift Time Left")
	public int shifttimeleft() {
		FieldUtil.getShiftOrder();
		return FieldUtil.getShiftTimeLeft();
	}

	@Logged(importance = Importance.CRITICAL, name = "Shift Next")
	public String shiftNext() {
		return FieldUtil.getHubState(FieldUtil.shift + 1);
	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isTransitionPeriod() {
		return FieldUtil.isTransitionPeriod();

	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isShift1() {
		return FieldUtil.isShift1();

	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isShift2() {
		return FieldUtil.isShift2();
	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isShift3() {
		return FieldUtil.isShift3();

	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isShift4() {
		return FieldUtil.isShift4();
	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isEndGame() {
		return FieldUtil.isEndGame();
	}
}

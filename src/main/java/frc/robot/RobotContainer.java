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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
	@Logged(name = "Driver Controller")
	private XboxController driver = new XboxController(DriverConstants.DriverPort);
	private XboxController operater = new XboxController(DriverConstants.OperatorPort);

	private Supplier<Double> leftY = () -> DriverConstants.joystickDeadband(-driver.getLeftY(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private Supplier<Double> leftX = () -> DriverConstants.joystickDeadband(-driver.getLeftX(), true)
			* RobotConstants.maxSpeed.in(MetersPerSecond);
	private Supplier<Double> rightX = () -> DriverConstants.joystickDeadband(-driver.getRightX(), true)
			* RobotConstants.maxRot.in(RadiansPerSecond);

	@Logged(importance = Importance.CRITICAL)
	private Swerve swerve = TunerConstants.swerve;
	private Intake intake = new Intake();
	private Shooter shooter = new Shooter();
	// private Climber climber = new Climber();

	@Logged(name = "Reset Gyro")
	private Trigger resetGyro = new Trigger(() -> driver.getStartButton());

	private Trigger pass = new Trigger(() -> driver.getBButton());

	@Logged(name = "Bump Drive")
	private Trigger bumpDrive = new Trigger(() -> driver.getYButton());

	@Logged(name = "Snake Drive")
	private Trigger snakeDrive = new Trigger(() -> driver.getAButton());

	@Logged(name = "Shoot")
	private Trigger shoot = new Trigger(() -> driver.getRightTriggerAxis() > 0.2);

	@Logged(name = "Spin Up")
	private Trigger shooterSpinup = new Trigger(() -> driver.getRightBumperButton());

	@Logged(name = "Run Intake")
	private Trigger runIntake = new Trigger(() -> driver.getLeftTriggerAxis() > 0.2);

	private Trigger toggleIntakeDeploy = new Trigger(() -> driver.getLeftBumperButton());

	private Trigger reverse = new Trigger(() -> operater.getXButton());

	@Logged(name = "Current Auto")
	private AutoChooser autoChooser = new AutoChooser();
	private Autos autos = new Autos(swerve, shooter, intake);

	public RobotContainer() {
		configureBindings();
		configureControllerAlerts();
		setDefaultCommands();
		autoChooser.addCmd("Hub Depot Outpost Tower", autos::hubDepotOutpostTowerAuto);
		autoChooser.addCmd("right outpost neutral", autos::rightOutpostNeutral);
		autoChooser.addCmd("left depot neutral", autos::leftDepotNeutral);
		autoChooser.addCmd("right neutral depot", autos::rightNeutralDepot);
		SmartDashboard.putData("Auto Chooser", autoChooser);

		// TODO: remove this during comp
		new Trigger(() -> !DriverStation.isTeleopEnabled())
				.onTrue(Commands.run(() -> FieldUtil.allianceWithActiveHubStart = null).ignoringDisable(true));
	}

	private void configureBindings() {
		swerve.setDefaultCommand(swerve.driveCommand(leftY, leftX, rightX, () -> true));
		resetGyro.onTrue(Commands.runOnce(() -> swerve.resetGyro(), swerve));
		pass.whileTrue(Commands.parallel(
				swerve.run(() -> swerve.angularDriveRequest(leftY, leftX,
						() -> FieldUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg, () -> true)),
				shooter.run(() -> {
					shooter.setRPS(40, 40);
					shooter.setKickerVelocity(ShooterConstants.KickerRPS);
				})));
		shoot.whileTrue(new Shoot(swerve, shooter, leftY, leftX, () -> true));
		bumpDrive.whileTrue(
				Commands.run(() -> swerve.angularDriveRequest(leftY, leftX, () -> swerve.getClosest15(),
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

		shoot.whileTrue(new Shoot(swerve, shooter, leftY, leftX, () -> true));
		shooterSpinup.whileTrue(shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));

		runIntake.whileTrue(intake.runIntake())
				.onFalse(intake.stopIntake());
		toggleIntakeDeploy.toggleOnTrue(intake.retract());

		// TODO: also reverse the indexer as well
		reverse.whileTrue(Commands.parallel(
				shooter.run(() -> shooter.setKickerVelocity(-ShooterConstants.KickerRPS)),
				intake.run(() -> intake.setIntakeVoltage(-IntakeConstants.IntakeVoltage))));
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
		shooter.setDefaultCommand(shooter.run(shooter::disable));
		intake.setDefaultCommand(intake.deploy());
	}

	@Logged(name = "Current auto")
	public Command getAutonomousCommand() {
		return autoChooser.selectedCommand();
	}

	public void testingInit() {
		TuningManager.init(swerve, shooter, intake, null);
	}

	@Logged(importance = Importance.CRITICAL, name = "Is Hub Active")
	public String isHubActive() {
		return FieldUtil.isHubActive();
	}

	@Logged(importance = Importance.CRITICAL, name = "Shift Time Left")
	public int shifttimeleft() {
		FieldUtil.getShiftOrder();
		return FieldUtil.getShiftTimeLeft();
	}

	@Logged(importance = Importance.CRITICAL, name = "Shift Next")
	public String shiftNext() {
		return FieldUtil.isHubActive(FieldUtil.shift + 1);
	}

	@Logged(importance = Importance.CRITICAL, name = "Shift Next Next")
	public String shiftNextNext() {
		return FieldUtil.isHubActive(FieldUtil.shift + 2);
	}

	@Logged(importance = Importance.CRITICAL)
	public boolean isAuto() {
		return FieldUtil.isAuto();

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

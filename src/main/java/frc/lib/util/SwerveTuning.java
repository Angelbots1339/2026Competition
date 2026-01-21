package frc.lib.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class SwerveTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private static Trigger characterizeSwerveRadius = baseTrigger.and(() -> tester.getAButton());
	private static Trigger testRotation = baseTrigger.and(() -> tester.getBButton());

	private static Swerve swerve;

	public static void init(Swerve swerve) {
		SwerveTuning.swerve = swerve;
		swerve.logTuning();

		characterizeSwerveRadius.whileTrue(characterizeWheelRadius());

		testRotation.whileTrue(Commands.run(() -> swerve.angularDriveRequest(() -> 0.0, () -> 0.0,
				() -> Rotation2d.fromDegrees(
						SmartDashboard.getNumber(TuningConstants.Swerve.angularPIDNTName + "/setpoint", 0))),
				swerve));

	}

	/* yoinked from mechanical advantage */
	public static Command characterizeWheelRadius() {
		SlewRateLimiter limiter = new SlewRateLimiter(Math.PI / 4);
		WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
		double driveBaseRadius = Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
		return Commands.parallel(
				Commands.sequence(
						Commands.runOnce(() -> limiter.reset(0)),
						Commands.run(() -> {
							// double speed = limiter.calculate(Math.PI / 16);
							swerve.driveRobotRelative(new ChassisSpeeds(0, 0, Math.PI / 4));
						}, swerve)),
				Commands.sequence(
						// wait for module reorient
						Commands.waitSeconds(1.0),
						Commands.runOnce(() -> {
							for (int i = 0; i < swerve.getModules().length; i++) {
								state.positions[i] = swerve.getModule(i).getPosition(true).distanceMeters;
							}
							state.lastAngle = swerve.getYaw();
							state.gyroDelta = 0;
						}),
						Commands.run(() -> {
							var rotation = swerve.getYaw();
							state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
							state.lastAngle = rotation;
						}).finallyDo(() -> {
							double[] positions = new double[4];
							for (int i = 0; i < swerve.getModules().length; i++) {
								positions[i] = swerve.getModule(i).getPosition(true).distanceMeters;
							}
							double wheelDelta = 0.0;
							for (int i = 0; i < 4; i++) {
								wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
							}
							double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta
									* Units.metersToInches(TunerConstants.FrontLeft.WheelRadius);
							SmartDashboard.putNumber("wheel radius", wheelRadius);
						})));
	}

	private static class WheelRadiusCharacterizationState {
		double[] positions = new double[4];
		Rotation2d lastAngle = Rotation2d.kZero;
		double gyroDelta = 0.0;
	}
}

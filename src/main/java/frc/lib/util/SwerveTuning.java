package frc.lib.util;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class SwerveTuning {

	private static XboxController tester = new XboxController(DriverConstants.TesterPort);

	private static Trigger baseTrigger = new Trigger(() -> DriverStation.isTestEnabled());
	private static Trigger characterizeSwerveRadius = baseTrigger.and(() -> tester.getAButton());
	private static Trigger testRotation = baseTrigger.and(() -> tester.getBButton());
	private static Trigger sysIdRotation = baseTrigger.and(() -> tester.getYButton());

	private static Swerve swerve;

	private static final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
	private static SysIdRoutine m_sysIdRoutineRotation = null;

	public static void init(Swerve swerve) {
		SwerveTuning.swerve = swerve;
		setupSysId();
		swerve.logTuning();

		characterizeSwerveRadius.whileTrue(characterizeWheelRadius());

		testRotation.whileTrue(Commands.run(() -> swerve.angularDriveRequest(() -> 0.0, () -> 0.0,
				() -> Rotation2d.fromRadians(
						SmartDashboard.getNumber(TuningConstants.Swerve.angularPIDNTName + "/goal", 0))),
				swerve));

		sysIdRotation.whileTrue(Commands.sequence(
				Commands.runOnce(SignalLogger::start),
				sysIdQuasistatic(m_sysIdRoutineRotation, Direction.kForward),
				Commands.waitSeconds(2),
				sysIdQuasistatic(m_sysIdRoutineRotation, Direction.kReverse),
				Commands.waitSeconds(2),
				sysIdDynamic(m_sysIdRoutineRotation, Direction.kForward),
				Commands.waitSeconds(2),
				sysIdDynamic(m_sysIdRoutineRotation, Direction.kReverse),
				Commands.runOnce(SignalLogger::stop)));
	}

	public static void setupSysId() {
		m_sysIdRoutineRotation = new SysIdRoutine(
				new SysIdRoutine.Config(
						/* This is in radians per second², but SysId only supports "volts per second" */
						Volts.of(Math.PI / 6).per(Second),
						/* This is in radians per second, but SysId only supports "volts" */
						Volts.of(Math.PI),
						null, // Use default timeout (10 s)
						// Log state with SignalLogger class
						state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
				new SysIdRoutine.Mechanism(
						output -> {
							/* output is actually radians per second, but SysId only supports "volts" */
							swerve.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
							/* also log the requested output for SysId */
							SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
						},
						null,
						swerve));
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public static Command sysIdQuasistatic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public static Command sysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
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

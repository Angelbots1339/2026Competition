package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.RobotConstants;

@Logged
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	@NotLogged
	private static final double kSimLoopPeriod = 0.004; // 4 ms
	@NotLogged
	private Notifier m_simNotifier = null;
	@NotLogged
	private double m_lastSimTime;

	private PIDController angularDrivePID = new PIDController(RobotConstants.angularDriveKP,
			RobotConstants.angularDriveKI, RobotConstants.angularDriveKD);

	private PIDController pidToPoseXController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);
	private PIDController pidToPoseYController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);

	public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);

		angularDrivePID.setTolerance(RobotConstants.angularDriveTolerance);
		angularDrivePID.enableContinuousInput(0, 360);
		pidToPoseXController.setTolerance(RobotConstants.pidToPoseTolerance);
		pidToPoseYController.setTolerance(RobotConstants.pidToPoseTolerance);

		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, Supplier<Boolean> fieldCentric) {
		return run(() -> {
			ChassisSpeeds speeds = new ChassisSpeeds(x.get(), y.get(), rot.get());
			SwerveRequest req;

			if (fieldCentric.get()) {
				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());
			}

			req = new SwerveRequest.RobotCentric()
					.withVelocityX(speeds.vxMetersPerSecond)
					.withVelocityY(speeds.vyMetersPerSecond)
					.withRotationalRate(speeds.omegaRadiansPerSecond);

			this.setControl(req);
		});
	}

	public Command pidtoPose(Supplier<Pose2d> target) {
		return run(() -> {
			double x = MathUtil.clamp(
					pidToPoseXController.calculate(getPose().getX(),
							target.get().getX())
							+ Math.signum(pidToPoseXController.getError()) * Math.abs(RobotConstants.pidToPoseKS),
					-RobotConstants.maxSpeed, RobotConstants.maxSpeed);
			double y = MathUtil.clamp(
					pidToPoseYController.calculate(getPose().getY(),
							target.get().getY())
							+ Math.signum(pidToPoseYController.getError()) * Math.abs(RobotConstants.pidToPoseKS),
					-RobotConstants.maxSpeed, RobotConstants.maxSpeed);

			// convert from blue origin coordinates to field oriented (alliance origin) coordinates
			if (FieldUtil.isRedAlliance()) {
				angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : -x,
						() -> pidToPoseYController.atSetpoint() ? 0 : -y, () -> target.get().getRotation().rotateBy(Rotation2d.k180deg));
			} else {
				angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : x,
						() -> pidToPoseYController.atSetpoint() ? 0 : y, () -> target.get().getRotation());
			}
		});
	}

	public void angularDriveRequest(Supplier<Double> translationX, Supplier<Double> translationY,
			Supplier<Rotation2d> desiredRotation) {

		ChassisSpeeds speeds = angularPIDCalc(translationX, translationY, desiredRotation);

		SwerveRequest req;

		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());

		req = new SwerveRequest.RobotCentric()
				.withDriveRequestType(DriveRequestType.Velocity)
				.withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
				.withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
				.withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond);

		this.setControl(req);
	}

	private ChassisSpeeds angularPIDCalc(Supplier<Double> translationX, Supplier<Double> translationY,
			Supplier<Rotation2d> desiredRotation) {
		double pid = angularDrivePID.calculate(getRelativeYaw().getDegrees(), desiredRotation.get().getDegrees());

		ChassisSpeeds speeds = new ChassisSpeeds(translationX.get(), translationY.get(),
				MathUtil.clamp(
						angularDrivePID.atSetpoint() ? 0 : pid + (RobotConstants.angularDriveKS * Math.signum(pid)),
						-RobotConstants.maxRot, RobotConstants.maxRot));

		return speeds;
	}

	public Pose2d getPose() {
		return this.getState().Pose;
	}

	public Rotation2d getYaw() {
		return this.getPigeon2().getRotation2d();
	}

	public void setYaw(Rotation2d yaw) {
		this.getPigeon2().setYaw(yaw.getDegrees());
	}

	public void resetGyro() {
		if (FieldUtil.isRedAlliance()) {
			setYaw(Rotation2d.k180deg);
		} else {
			setYaw(Rotation2d.kZero);
		}
	}

	public Rotation2d getRelativeYaw() {
		return FieldUtil.isRedAlliance() ? getYaw().rotateBy(Rotation2d.k180deg) : getYaw();
	}

	@Override
	public void periodic() {
	}

	public void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* Use the measured time delta, get battery voltage from WPILib */
			this.updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}
}

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;

@Logged
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	@NotLogged
	private static final double kSimLoopPeriod = 0.004; // 4 ms
	@NotLogged
	private Notifier m_simNotifier = null;
	@NotLogged
	private double m_lastSimTime;

	public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);

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

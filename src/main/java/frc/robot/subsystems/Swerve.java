package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.google.errorprone.annotations.concurrent.LockMethod;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants.VisionConstants;

@Logged
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	private static final double kSimLoopPeriod = 0.004; // 4 ms
	private Notifier m_simNotifier = null;

	@Logged(name = "Last Sim Time")
	private double m_lastSimTime;
	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */

	@Logged(name = "Has Applied Operator Perspective")
	private boolean m_hasAppliedOperatorPerspective = false;

	private ProfiledPIDController angularDrivePID = new ProfiledPIDController(RobotConstants.angularDriveKP,
			RobotConstants.angularDriveKI, RobotConstants.angularDriveKD, RobotConstants.angularDriveConstraints);

	private PIDController pidToPoseXController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);
	private PIDController pidToPoseYController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);

	public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);

		angularDrivePID.setTolerance(RobotConstants.angularDriveTolerance.in(Radians));
		angularDrivePID.enableContinuousInput(0, 2 * Math.PI);
		pidToPoseXController.setTolerance(RobotConstants.pidToPoseTolerance.in(Meters));
		pidToPoseYController.setTolerance(RobotConstants.pidToPoseTolerance.in(Meters));

		resetPose(Pose2d.kZero);
		resetGyro();

		configPathPlanner();

		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public Command driveCommand(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
			Supplier<Boolean> isFieldRelative) {
		return run(() -> driveRequest(x, y, rot, isFieldRelative));
	}

	// drive either field centric or robot centric while facing a blue origin pose
	public Command pointDriveCommand(Supplier<Double> x, Supplier<Double> y,
			Supplier<Pose2d> pose,
			Supplier<Boolean> fieldCentric) {
		return run(() -> {
			double xdiff = pose.get().getX() - getPose().getX();
			double ydiff = pose.get().getY() - getPose().getY();
			Rotation2d angle = Rotation2d.fromRadians(Math.atan2(ydiff, xdiff));

			angularDriveRequest(x, y, () -> angle, fieldCentric);
		});
	}

	public void driveRequest(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
			Supplier<Boolean> isFieldRelative) {
		ChassisSpeeds speeds = new ChassisSpeeds(x.get(), y.get(), rot.get());
		if (isFieldRelative.get())
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());
		driveRobotRelative(speeds);
	}

	// applies either robot relative or field relative translation with blue origin
	// rotation target
	public void angularDriveRequest(Supplier<Double> x, Supplier<Double> y, Supplier<Rotation2d> rot,
			Supplier<Boolean> isFieldRelative) {
		double rotation = angularDrivePID.calculate(getYaw().getRadians(), rot.get().getRadians());
		ChassisSpeeds speeds = new ChassisSpeeds(x.get(), y.get(), rotation);

		if (isFieldRelative.get())
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw());

		driveRobotRelative(speeds);
	}

	public Command pidtoPose(Supplier<Pose2d> target) {
		return run(() -> {
			double x = pidToPoseXController.calculate(getPose().getX(), target.get().getX());
			double y = pidToPoseYController.calculate(getPose().getY(), target.get().getY());
			double rotation = angularDrivePID.calculate(getYaw().getRadians(), target.get().getRotation().getRadians());

			x = MathUtil.clamp(x, -RobotConstants.maxSpeed.in(MetersPerSecond),
					RobotConstants.maxSpeed.in(MetersPerSecond));
			y = MathUtil.clamp(y, -RobotConstants.maxSpeed.in(MetersPerSecond),
					RobotConstants.maxSpeed.in(MetersPerSecond));

			ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw());
			driveRobotRelative(speeds);
		});
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
				.withDriveRequestType(DriveRequestType.Velocity));
	}

	public void logTuning() {
		SmartDashboard.putData(TuningConstants.Swerve.angularPIDNTName, angularDrivePID);
	}

	@Logged(name = "Closest 15")
	public Rotation2d getClosest15() {
		Rotation2d closest = Rotation2d.fromDegrees(15);
		for (var angle : Arrays.asList(15, 75, 105, 165, 195, 255, 285, 345)) {
			if (Math.abs(Rotation2d.fromDegrees(angle).minus(getYaw()).getDegrees()) < Math
					.abs(closest.minus(getYaw()).getDegrees())) {
				closest = Rotation2d.fromDegrees(angle);
			}
		}

		return closest;
	}

	public void setYaw(Rotation2d yaw) {
		getPigeon2().setYaw(yaw.getDegrees());
		resetRotation(yaw);
	}

	public void resetGyro() {
		if (FieldUtil.isRedAlliance())
			setYaw(Rotation2d.k180deg);
		else
			setYaw(Rotation2d.kZero);
	}

	@Logged(name = "yaw")
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(getPigeon2().getYaw().getValueAsDouble());
	}

	@Logged(name = "Relative yaw")
	public Rotation2d getRelativeYaw() {
		// double rawYaw = getPigeon2().getYaw().getValue().in(Degrees) +
		// (FieldUtil.isRedAlliance() ? 180 : 0);
		// double yawWithRollover = rawYaw > 0 ? rawYaw % 360 : 360 - Math.abs(rawYaw %
		// 360);

		// return Rotation2d.fromDegrees(yawWithRollover);
		if (FieldUtil.isRedAlliance())
			return getYaw().plus(Rotation2d.k180deg);

		return getYaw();
	}

	@Logged(importance = Importance.CRITICAL)
	public Pose2d getPose() {
		return this.getState().Pose;
	}

	public void resetPose(Pose2d pose) {
		super.resetPose(pose);
		setYaw(pose.getRotation());
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return getState().Speeds;
	}

	@SuppressWarnings("resource")
	public Consumer<SwerveSample> followChoreoPath() {
		final PIDController xController = new PIDController(10.0, 0.0, 0.0);
		final PIDController yController = new PIDController(10.0, 0.0, 0.0);
		final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
		headingController.enableContinuousInput(-Math.PI, Math.PI);
		return (sample) -> {
			Pose2d pose = getPose();

			// Generate the next speeds for the robot
			ChassisSpeeds speeds = new ChassisSpeeds(
					sample.vx + xController.calculate(pose.getX(), sample.x),
					sample.vy + yController.calculate(pose.getY(), sample.y),
					sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

			// Apply the generated speeds
			this.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(speeds));
		};
	}

	public void configPathPlanner() {
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		// Configure AutoBuilder last
		AutoBuilder.configure(
				this::getPose,
				this::resetPose,
				this::getRobotRelativeSpeeds,
				(speeds, feedforwards) -> driveRobotRelative(speeds),
				new PPHolonomicDriveController(
						new PIDConstants(5.0, 0.0, 0.0),
						new PIDConstants(5.0, 0.0, 0.0)),
				config, // The robot configuration
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	public void updateVision() {
		LimelightHelpers.SetRobotOrientation("limelight",
				getYaw().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
		if (mt2 == null)
			return;
		if (mt2.tagCount < 1)
			return;

		if (mt2.avgTagDist > 3.4) {
			return;
		}

		double xyStdDev2 = VisionConstants.calcStdDev(mt2.avgTagDist);

		setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev2, xyStdDev2, 9999999));
		addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
	}

	@Override
	public void periodic() {
		updateVision();

		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
						allianceColor == Alliance.Red
								? kRedAlliancePerspectiveRotation
								: kBlueAlliancePerspectiveRotation);
				m_hasAppliedOperatorPerspective = true;
			});
		}
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
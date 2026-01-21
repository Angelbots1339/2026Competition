package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;

public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	private static final double kSimLoopPeriod = 0.004; // 4 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;
	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean m_hasAppliedOperatorPerspective = false;

	public PIDController angularDrivePID = new PIDController(RobotConstants.angularDriveKP,
			RobotConstants.angularDriveKI, RobotConstants.angularDriveKD);

	private PIDController pidToPoseXController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);
	private PIDController pidToPoseYController = new PIDController(RobotConstants.pidToPoseKP, 0,
			RobotConstants.pidToPoseKD);
	private final Field2d m_field = new Field2d();

	public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);

		angularDrivePID.setTolerance(RobotConstants.angularDriveTolerance.in(Degrees));
		angularDrivePID.enableContinuousInput(-180, 180);
		pidToPoseXController.setTolerance(RobotConstants.pidToPoseTolerance.in(Meters));
		pidToPoseYController.setTolerance(RobotConstants.pidToPoseTolerance.in(Meters));

		SendableRegistry.setName(angularDrivePID, "rotation PID");

		SmartDashboard.putData("Field", m_field);
		resetPose(Pose2d.kZero);
		resetGyro();

		configPathPlanner();

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

	public Command pointDrive(Supplier<Double> x, Supplier<Double> y, Supplier<Pose2d> pose,
			Supplier<Boolean> fieldCentric) {
		return run(() -> {
			double xdiff = pose.get().getX() - getPose().getX();
			double ydiff = pose.get().getY() - getPose().getY();
			Rotation2d angle = Rotation2d.fromRadians(Math.atan2(ydiff, xdiff));

			angularDriveRequest(x, y, () -> fieldtoRobotRotation(angle));
		});
	}

	public Rotation2d fieldtoRobotRotation(Rotation2d rot) {
		return FieldUtil.isRedAlliance() ? rot.plus(Rotation2d.k180deg) : rot;
	}

	public Command pidtoPose(Supplier<Pose2d> target) {
		return run(() -> {
			double x = MathUtil.clamp(
					pidToPoseXController.calculate(getPose().getX(),
							target.get().getX())
							+ Math.signum(pidToPoseXController.getError()) * Math.abs(RobotConstants.pidToPoseKS),
					-RobotConstants.maxSpeed.in(MetersPerSecond), RobotConstants.maxSpeed.in(MetersPerSecond));
			double y = MathUtil.clamp(
					pidToPoseYController.calculate(getPose().getY(),
							target.get().getY())
							+ Math.signum(pidToPoseYController.getError()) * Math.abs(RobotConstants.pidToPoseKS),
					-RobotConstants.maxSpeed.in(MetersPerSecond), RobotConstants.maxSpeed.in(MetersPerSecond));

			// convert from blue origin coordinates to field oriented (alliance origin)
			// coordinates
			if (FieldUtil.isRedAlliance()) {
				angularDriveRequest(() -> pidToPoseXController.atSetpoint() ? 0 : -x,
						() -> pidToPoseYController.atSetpoint() ? 0 : -y,
						() -> target.get().getRotation().rotateBy(Rotation2d.k180deg));
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
						-RobotConstants.maxRot.in(RadiansPerSecond), RobotConstants.maxRot.in(RadiansPerSecond)));

		return speeds;
	}

	public Pose2d getPose() {
		return this.getState().Pose;
	}

	public Rotation2d getClosest45() {
		Rotation2d closest = Rotation2d.fromDegrees(45);
		for (var angle : Arrays.asList(45, 135, 225, 315)) {
			if (Math.abs(Rotation2d.fromDegrees(angle).minus(getRelativeYaw()).getDegrees()) < Math
					.abs(closest.minus(getRelativeYaw()).getDegrees())) {
				closest = Rotation2d.fromDegrees(angle);
			}
		}

		return closest;
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
		double rawYaw = getPigeon2().getYaw().getValue().in(Degrees) + (FieldUtil.isRedAlliance() ? 180 : 0);
		double yawWithRollover = rawYaw > 0 ? rawYaw % 360 : 360 - Math.abs(rawYaw % 360);

		return Rotation2d.fromDegrees(yawWithRollover);
		// return FieldUtil.isRedAlliance() ? getYaw().rotateBy(Rotation2d.k180deg) :
		// getYaw();
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return this.getState().Speeds;
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
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
			driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRelativeYaw()));
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
				this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
																		// RELATIVE ChassisSpeeds. Also optionally
																		// outputs individual module feedforwards
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
												// holonomic drive trains
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
				),
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
		SmartDashboard.putNumber("limelight rot", LimelightHelpers.getTX("limelight"));
		if (mt2.tagCount < 1)
			return;

		if (mt2.avgTagDist > 3.4) {
			return;
		}

		double xyStdDev2 = VisionConstants.calcStdDev(mt2.avgTagDist);

		setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev2, xyStdDev2, 9999999));
		addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
		SmartDashboard.putNumber("tag dist", mt2.avgTagDist);
	}

	@Override
	public void periodic() {
		updateVision();
		m_field.setRobotPose(this.getPose());
		SmartDashboard.putNumber("target", angularDrivePID.getSetpoint());
		SmartDashboard.putNumber("cur", getRelativeYaw().getDegrees());

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
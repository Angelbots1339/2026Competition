package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
	public static final boolean useTesting = !DriverStation.isFMSAttached();

	public class DriverConstants {
		public static final int DriverPort = 0;
		public static final int OperatorPort = 1;
		public static final int TesterPort = 2;

		public static final double joystickDeadband = 0.1;

		public static double joystickDeadband(double val, boolean curveInputs) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? 2 : 1),
					joystickDeadband) * Math.signum(val);
		}
	}

	public class ShootingConstants {
		public static final Time IntakeRetractTime = Seconds.of(1);
		public static final Time IntakeRetractOffsetTime = Seconds.of(1);
	}

	public class RobotConstants {
		public static final LinearVelocity maxSpeed = MetersPerSecond.of(5);
		public static final AngularVelocity maxRot = RadiansPerSecond.of(Math.PI * 2);
		public static final Distance length = Inches.of(32.25);
		public static final Distance width = Inches.of(32.25);
	}

	public class AlignConstants {

		// has least amount of error, overshooting seems to be result of wheel slip on
		// concrete
		public static final double angularDriveKP = 5;
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0;
		public static final double angularDriveKS = 0.17;
		public static final double angularDriveKV = 0;
		public static final SimpleMotorFeedforward angularDriveFeedforward = new SimpleMotorFeedforward(
				angularDriveKS,
				angularDriveKV);
		public static final TrapezoidProfile.Constraints angularDriveConstraints = new TrapezoidProfile.Constraints(
				10,
				20);
		public static final Angle angularDriveTolerance = Degrees.of(3); // Degrees

		public static final double pidToPoseKP = 2.5;
		public static final double pidToPoseKD = 0;
		public static final double pidToPoseKS = 0.15;
		public static final Distance pidToPoseTolerance = Meters.of(0.03); // Meters
		public static final LinearVelocity pidToPoseMaxSpeed = MetersPerSecond.of(1); // Meters per second
		public static final PathConstraints ppConstraints = new PathConstraints(
				3.0, 4.0,
				Units.degreesToRadians(540), Units.degreesToRadians(720));
	}

	public class VisionConstants {
		public static final String Limelight3Name = "limelight-front";
		public static final String Limelight4Name = "limelight-back";
		public static final double maxUsableLL3Range = 4.3;
		public static final double maxUsableLL4Range = 4.6; // TODO: find a value for this

		// limelight 4 (back limelight) location
		// forward = -0.081m
		// left = 0.000m
		// top = 0.684m
		// pitch = 15deg
		// roll = 180deg
		// yaw = 180deg

		/*
		 * LL3 (front) location
		 * forward = 0.033m
		 * right = 0.015m
		 * top = 0.673m
		 * pitch = 15 deg
		 * roll = 180deg
		 * yaw = 0 deg
		 */

		public static double calcStdDev(double metersFromTarget) {
			return 0.08 * Math.pow(metersFromTarget, 2);
		}
	}

	public class TuningConstants {
		public static enum TuningMode {
			Shooter,
			Swerve,
			Regression,
			Intake,
			Indexer,
		};
	}

	public class ShooterConstants {
		public static final int Shooter1Port = 30;
		public static final int Shooter2Port = 32;
		public static final int Shooter3Port = 38;
		public static final int SpinnerPort = 34;
		public static final int KickerPort = 36;

		public static final double shootRPS = 41.5;
		public static final double KickerRPS = 20.0;
		public static final double rpsTolerence = 1;

		public static TalonFXConfiguration ShooterConfig = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(100))
						.withSupplyCurrentLimitEnable(false)
						.withStatorCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
						.withNeutralMode(NeutralModeValue.Coast)
						.withInverted(InvertedValue.CounterClockwise_Positive))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(24.0 / 18.0))
				.withSlot0(new Slot0Configs()
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
						.withKP(18)
						.withKI(0)
						.withKV(0.049)
						.withKS(6.3));

		public static TalonFXConfiguration SpinnerConfig = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(50))
						.withSupplyCurrentLimit(Amps.of(30))
						.withSupplyCurrentLimitEnable(false)
						.withStatorCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
						.withNeutralMode(NeutralModeValue.Coast)
						.withInverted(InvertedValue.Clockwise_Positive))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(24.0 / 12.0))
				.withSlot0(new Slot0Configs()
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
						.withKP(6)
						.withKI(4.5)
						.withKV(0)
						.withKS(5.5));

		public static TalonFXConfiguration KickerConfig = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(Amps.of(100))
						.withStatorCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
						.withNeutralMode(NeutralModeValue.Coast)
						.withInverted(InvertedValue.CounterClockwise_Positive))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(29.0 / 12.0))
				.withSlot0(new Slot0Configs()
						.withKP(16)
						.withKI(2)
						.withKD(0)
						.withKS(18)
						.withKV(0));

		public static VelocityTorqueCurrentFOC velocityTorqueControl = new VelocityTorqueCurrentFOC(0)
				.withUpdateFreqHz(Hertz.of(1000));
	}

	public class IntakeConstants {
		public static final int intakeMotorId = 22;
		public static final int deployMotorId = 24;

		public static final Angle DeployedAngle = Degrees.of(12.5);
		public static final Angle RetractedAngle = Degrees.of(116);
		public static final double IntakeVoltage = 7;

		public static final double deployIntakeGearRatio = 32.0 / 16.0 * 9;

		// angle of COM in CAD
		// public static final Angle MinAngle = Degrees.of(6.76);
		// public static final Angle MaxAngle = Degrees.of(144.975);

		// resting is 13.97
		public static final Angle MinAngle = Degrees.of(12.5);
		public static final Angle MaxAngle = Degrees.of(116.977);
		public static final Angle IntakeAngleTolerence = Degrees.of(10);

		// Recalc:
		// https://www.reca.lc/arm?armMass=%7B%22s%22%3A5.134%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A10.203%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A70%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A115.558%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%22%7D&ratio=%7B%22magnitude%22%3A18%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A15.5614%2C%22u%22%3A%22deg%22%7D
		public static final Slot0Configs deploySlot0 = new Slot0Configs()
				.withKP(70)
				.withKI(0)
				.withKD(0)
				.withKV(0)
				.withKA(0)
				.withKS(0.3)
				.withKG(0)
				.withGravityType(GravityTypeValue.Arm_Cosine)
				.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

		public static final TalonFXConfiguration deployConfigs = new TalonFXConfiguration()
				.withMotorOutput(
						new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Brake)
								.withInverted(InvertedValue.CounterClockwise_Positive))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(deployIntakeGearRatio))
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimit(60)
								.withStatorCurrentLimitEnable(true))
				.withSoftwareLimitSwitch(
						new SoftwareLimitSwitchConfigs()
								.withForwardSoftLimitEnable(false)
								.withReverseSoftLimitEnable(false)
								.withForwardSoftLimitThreshold(MaxAngle)
								.withReverseSoftLimitThreshold(MinAngle))
				.withSlot0(deploySlot0)
				.withMotionMagic(new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(0.5)
						.withMotionMagicAcceleration(4));

		public static final Slot0Configs intakeSlot0 = new Slot0Configs()
				.withKP(0)
				.withKI(0)
				.withKD(0)
				.withKV(0)
				.withKA(0)
				.withKS(0)
				.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

		public static final double intakeWheelGearRatio = 18.0 / 12.0;

		public static final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration()
				.withMotorOutput(
						new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Coast)
								.withInverted(InvertedValue.Clockwise_Positive))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(intakeWheelGearRatio))
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimit(50)
								.withStatorCurrentLimitEnable(true))
				.withSlot0(deploySlot0);

	}

	public class IndexerConstants {
		public static final int IndexerMotorPort = 26;

		public static final double IndexerVolts = 3.5;
		public static final double IntakeIndexerVoltage = 2;

		public static final TalonFXConfiguration IndexerMotorConfig = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(80)
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(false))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(16.0 / 16.0))
				.withMotorOutput(new MotorOutputConfigs()
						.withInverted(InvertedValue.CounterClockwise_Positive)
						.withNeutralMode(NeutralModeValue.Coast))
				.withSlot0(new Slot0Configs()
						.withKP(0)
						.withKI(0)
						.withKD(0)
						.withKS(0)
						.withKV(0)
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign));
	}
}

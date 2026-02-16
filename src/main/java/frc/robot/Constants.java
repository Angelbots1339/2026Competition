package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
	public class DriverConstants {
		public static final int DriverPort = 0;
		public static final int TesterPort = 2;

		public static final double joystickDeadband = 0.1;

		public static double joystickDeadband(double val, boolean curveInputs) {
			return MathUtil.applyDeadband(Math.pow(Math.abs(val), curveInputs ? 2 : 1),
					joystickDeadband) * Math.signum(val);
		}
	}

	public class RobotConstants {
		public static final LinearVelocity maxSpeed = MetersPerSecond.of(5);
		public static final AngularVelocity maxRot = RadiansPerSecond.of(Math.PI * 2);
		public static final Distance length = Inches.of(32.25);
		public static final Distance width = Inches.of(32.25);

		public static final Translation2d climberOffset = new Translation2d(Inches.of(-12.046), Meters.zero());

	}

	public class AlignConstants {

		// has least amount of error, overshooting seems to be result of wheel slip on
		// concrete
		public static final double angularDriveKP = 5.3;
		public static final double angularDriveKI = 0;
		public static final double angularDriveKD = 0;
		public static final double angularDriveKS = 0.12;
		public static final double angularDriveKV = 0;
		public static final SimpleMotorFeedforward angularDriveFeedforward = new SimpleMotorFeedforward(
				angularDriveKS,
				angularDriveKV);
		public static final TrapezoidProfile.Constraints angularDriveConstraints = new TrapezoidProfile.Constraints(
				10,
				20);
		public static final Angle angularDriveTolerance = Degrees.of(0.5); // Degrees

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
		public static final String LimelightName = "limelight";
		public static final double maxUsableRange = 4.0;

		public static double calcStdDev(double metersFromTarget) {
			return 0.08 * Math.pow(metersFromTarget, 2);
		}
	}

	public class TuningConstants {
		public static final String tuningNTPrefix = "Tuning/";
		public static final String tuningModeNTName = tuningNTPrefix + "Mode";

		public static enum TuningMode {
			Shooter,
			Swerve,
			Regression,
			Intake,
			Climber,
		};

		public class Swerve {
			public static final String angularPIDNTName = tuningNTPrefix + "angular PID";
		}

		public class Shooter {
			public static final String voltageNTName = tuningNTPrefix + "voltage";
			public static final String velocityNTName = tuningNTPrefix + "velocity";
			public static final String PNTName = tuningNTPrefix + "P";
			public static final String INTName = tuningNTPrefix + "I";
			public static final String DNTName = tuningNTPrefix + "D";
			public static final String SNTName = tuningNTPrefix + "S";
			public static final String VNTName = tuningNTPrefix + "V";
		}
	}

	public class ShooterConstants {
		public static final int LeaderPort = 30;
		public static final int FollowerPort = 32;
		public static final int SpinnerPort = 36;
		public static final int IndexPort = 34;

		public static final double shootRPS = 41.5;
		public static final double rpsTolerence = 1;

		public static TalonFXConfiguration base = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withSupplyCurrentLimit(Amps.of(70))
						.withStatorCurrentLimit(Amps.of(120))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(true));

		public static TalonFXConfiguration config = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withSupplyCurrentLimit(Amps.of(70))
						.withStatorCurrentLimit(Amps.of(120))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
						.withNeutralMode(NeutralModeValue.Coast)
						.withInverted(InvertedValue.CounterClockwise_Positive))
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(18.0 / 44.0))
				.withSlot0(new Slot0Configs()
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
						.withKP(10)
						.withKI(6)
						.withKV(0)
						.withKS(10));

		public static TalonFXConfiguration spinnerConfig = base.clone()
				.withMotorOutput(new MotorOutputConfigs()
						.withNeutralMode(NeutralModeValue.Coast)
						.withInverted(InvertedValue.Clockwise_Positive)) // for some reason, if
													// we extend
													// config, this
													// doesn't get
													// overridden
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(18.0 / 36.0))
				.withSlot0(new Slot0Configs()
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
						// TODO: actual find values
						.withKP(9)
						.withKI(6)
						.withKV(0)
						.withKS(6));

		public static TalonFXConfiguration indexConfig = base.clone().withMotorOutput(new MotorOutputConfigs()
				.withNeutralMode(NeutralModeValue.Coast)
				.withInverted(InvertedValue.CounterClockwise_Positive))
				.withSlot0(new Slot0Configs().withKP(0.35).withKI(0).withKD(0).withKS(0.5)
						.withKV(0.065));

		public static VelocityTorqueCurrentFOC velocityTorqueControl = new VelocityTorqueCurrentFOC(0)
				.withUpdateFreqHz(Hertz.of(100));
	}

	public class IntakeConstants {
		public static final int intakeMotorId = 8;
		public static final int deployMotorId = 9;

		public static final double deployIntakeGearRatio = 32.0 / 16.0;

		public static final Slot0Configs deploySlot0 = new Slot0Configs()
				.withKP(0)
				.withKI(0)
				.withKD(0)
				.withKV(0)
				.withKA(0)
				.withKS(0)
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
								.withStatorCurrentLimit(35)
								.withSupplyCurrentLimit(70)
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimitEnable(true))
				.withSoftwareLimitSwitch(
						new SoftwareLimitSwitchConfigs()
								// TODO: figure out the software limits
								.withForwardSoftLimitEnable(false)
								.withReverseSoftLimitEnable(false)
								.withForwardSoftLimitThreshold(1)
								// this may need to change as well
								.withReverseSoftLimitThreshold(0))
				.withSlot0(deploySlot0);

		public static final Slot0Configs intakeSlot0 = new Slot0Configs()
				.withKP(0)
				.withKI(0)
				.withKD(0)
				.withKV(0)
				.withKA(0)
				.withKS(0)
				.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

		public static final double intakeWheelGearRatio = 1 * 1;

		public static final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration()
				.withMotorOutput(
						new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Coast)
								.withInverted(InvertedValue.CounterClockwise_Positive))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(intakeWheelGearRatio))
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimit(35)
								.withSupplyCurrentLimit(70)
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimitEnable(true))
				.withSlot0(deploySlot0);

		public static final double deployedAngle = 0.0;
		public static final double retractedAngle = 0.0;
		public static final double intakeVelocity = 0.0;
	}

	public class ClimberConstants {
		public static final int ClimberMotorPort = 28;
		public static final Distance PitchDiameter = Inches.of(0.5);
		public static final Distance MaxDistance = Centimeters.of(17);

		public static final Distance ClimbPosition = Centimeters.of(0);
		public static final Distance HomePosition = Inches.of(10);

		public static final TalonFXConfiguration ClimberMotorConfig = new TalonFXConfiguration()
				.withSoftwareLimitSwitch(
						new SoftwareLimitSwitchConfigs()
								.withForwardSoftLimitThreshold(MaxDistance.in(Meters))
								.withReverseSoftLimitThreshold(0)
								.withForwardSoftLimitEnable(true)
								.withReverseSoftLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
						// Driving the hook down = positive
						.withInverted(InvertedValue.CounterClockwise_Positive)
						.withNeutralMode(NeutralModeValue.Brake))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(3 * 5 * 20.0)) // 1/20 is a tested roough
												// diameter of the
												// climber
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withSupplyCurrentLimit(Amps.of(70))
						.withStatorCurrentLimit(Amps.of(120))
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(true))
				.withSlot0(new Slot0Configs()
						// TODO do motion magic later
						.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
						.withKP(120)
						.withKI(0)
						.withKD(0)
						.withKS(0)
						.withKG(10)
						.withGravityType(GravityTypeValue.Elevator_Static));
	}

	public class IndexerConstants {
		public static final int IndexerMotorPort = 26;
		public static final TalonFXConfiguration IndexerMotorConfig = new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(80)
						.withSupplyCurrentLowerTime(50)
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimitEnable(true))
				.withFeedback(new FeedbackConfigs()
						.withSensorToMechanismRatio(24.0 / 24.0))
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

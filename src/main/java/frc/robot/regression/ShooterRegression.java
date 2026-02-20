package frc.robot.regression;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.FieldUtil;
import frc.robot.subsystems.Swerve;

public class ShooterRegression {
	public static final double[][] shotRPSData = {
			// distance (m), shooter rps, spinner rps
			// -0.038 is the offset of the shooter front from center of robot
			// original distance is the distance from front of shooter to hub wall
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(49)).in(Meters), 41, 23 },
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(60)).in(Meters), 41, 25 },
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(85)).in(Meters), 37, 40 },
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(104)).in(Meters), 40, 39 },
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(136)).in(Meters), 43, 43 },
			{ -0.038 + FieldUtil.hubWidth.div(2).plus(Inches.of(170)).in(Meters), 45, 45 },
			{ 1.48, 41, 16 },
	};

	public static final double[][] tofData = {
			// distance, tof from ball leaving the shooter to the hub
			{ 1, 1 },
			{ 2, 1.2 },
			{ 5, 1.3 },
	};

	public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

	public static final InterpolatingTreeMap<Double, double[]> shotRPSMap = new InterpolatingTreeMap<Double, double[]>(
			InverseInterpolator.forDouble(), new Interpolator<double[]>() {
				public double[] interpolate(double[] startValue, double[] endValue, double t) {
					double[] new_vals = { MathUtil.interpolate(startValue[0], endValue[0], t),
							MathUtil.interpolate(startValue[1], endValue[1], t), };
					return new_vals;
				};
			});

	static {
		for (double[] data : shotRPSData) {
			shotRPSMap.put(data[0], new double[] { data[1], data[2] });
		}
		for (double[] data : tofData) {
			timeOfFlightMap.put(data[0], data[1]);
		}
	}

	public record ShooterParams(Rotation2d angle, double shooterRPS, double spinnerRPS, Angle maxAngleError) {
	};

	public static ShooterParams getShotParams(Swerve swerve) {
		Translation2d target = FieldUtil.getHubCenter().getTranslation();
		Translation2d pose = swerve.getPose().getTranslation();
		double distance = target.getDistance(pose);

		ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getRobotRelativeSpeeds(),
				swerve.getYaw());
		double vx = fieldSpeeds.vxMetersPerSecond;
		double vy = fieldSpeeds.vyMetersPerSecond;

		// When shooting, we are assuming the TOF of the ball being the same as when
		// leaving the current pose.
		// however, as we are shooting farther/closer depending on our
		// velocity/lookahead pose, the actual TOF is shorter/longer
		// iterate and use the TOF from the lookaheadpose to calculate a better
		// lookahead pose
		Translation2d lookaheadPose = pose;
		double lookaheadDistance = distance;
		double tof = 0;
		for (int i = 0; i < 20; i++) {
			tof = timeOfFlightMap.get(lookaheadDistance);
			lookaheadPose = pose.plus(new Translation2d(vx * tof, vy * tof));
			lookaheadDistance = target.getDistance(lookaheadPose);
		}
		DogLog.log("Regression/Lookahead Pose", new Pose2d(lookaheadPose, swerve.getYaw()));

		double[] rps = shotRPSMap.get(lookaheadDistance);

		Rotation2d angle = Rotation2d.fromRadians(
				target.minus(lookaheadPose).getAngle().getRadians());

		Angle maxAngleError = Radians
				.of(Math.abs(Math.atan2(lookaheadDistance, FieldUtil.hubRadius.in(Meters))));

		return new ShooterParams(angle, rps[0], rps[1], maxAngleError);
	}

	public static double[] getRegressionRPS(double meters) {
		return shotRPSMap.get(meters);
	}

}

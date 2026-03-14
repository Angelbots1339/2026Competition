package frc.robot.regression;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
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
			{ 1.828, 41, 3 },
			{ 2.315, 43, 7 },
			{ 2.614, 43, 10 },
			{ 2.953, 45, 10 },
			{ 3.317, 45.5, 12 },
			{ 3.550, 46, 13 },
			{ 3.90, 47, 15 },
	};

	public static final double[][] tofData = {
			// distance, tof from ball leaving the shooter to the hub
			{ 1.816, 24.0 / 29.949 },
			{ 2.820, 29.0 / 29.949 },
			{ 3.90, 35.0 / 29.949 },
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
		for (int i = 0; i < 3; i++) {
			tof = timeOfFlightMap.get(lookaheadDistance);
			lookaheadPose = pose.plus(new Translation2d(vx * tof, vy * tof));
			lookaheadDistance = target.getDistance(lookaheadPose);
		}

		double[] rps = shotRPSMap.get(lookaheadDistance);

		Rotation2d angle = Rotation2d.fromRadians(
				target.minus(lookaheadPose).getAngle().getRadians());

		Angle maxAngleError = Radians
				.of(Math.abs(Math.atan2(FieldUtil.hubRadius.in(Meters), distance)));

		DogLog.log("Regression/Lookahead Pose", new Pose2d(lookaheadPose, angle));
		DogLog.log("Regression/Max Angle Error", maxAngleError);
		DogLog.log("Regression/Angle", angle);
		DogLog.log("Regression/Shooter RPS", rps[0]);
		DogLog.log("Regression/Spinner RPS", rps[1]);

		return new ShooterParams(angle, rps[0], rps[1], maxAngleError);
	}

	public static double[] getRegressionRPS(double meters) {
		return shotRPSMap.get(meters);
	}

}

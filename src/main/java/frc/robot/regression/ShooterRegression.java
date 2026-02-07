package frc.robot.regression;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.FieldUtil;
import frc.robot.subsystems.Swerve;

public class ShooterRegression {
	// TODO: actually fill data
	public static final double[][] shotRPSData = {
			// distance (m), shooter rps, spinner rps, TOF
			{ Inches.of(130).in(Meters), 48.5, 43 },
			{ Inches.of(100).in(Meters), 42, 38 }, // top spinner slightly slower makes it better
			{ Inches.of(80).in(Meters), 39, 37 }, // top spinner slightly slower makes it better
			{ Inches.of(45).in(Meters), 36, 32 }, // top spinner slightly slower makes it better
			// { Inches.of(96).in(Meters), 41.5, 41.5 },
			// { Inches.of(83).in(Meters), 38.5, 38.5 },
			// { Inches.of(65).in(Meters), 36.5, 36.5 },
			{ Inches.of(35).in(Meters), 35, 35 },
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
	}

	public record ShooterParams(Rotation2d angle, double shooterRPS, double spinnerRPS, Angle maxAngleError) {

	};

	public static ShooterParams getShotParams(Swerve swerve) {
		Translation2d target = FieldUtil.getHubCenter().getTranslation();
		Pose2d estimatedPose = swerve.getPose();
		double distance = target.getDistance(estimatedPose.getTranslation());
		double[] rps = getRegressionRPS(distance);

		Rotation2d angle = Rotation2d.fromRadians(
				target.minus(estimatedPose.getTranslation()).getAngle().getRadians());

		Angle maxAngleError = Radians
				.of(Math.abs(Math.atan2(distance, FieldUtil.hubRadius.in(Meters))));

		return new ShooterParams(angle, rps[0], rps[1], maxAngleError);
	}

	public static double[] getRegressionRPS(double distance) {
		return shotRPSMap.get(distance);
	}
}

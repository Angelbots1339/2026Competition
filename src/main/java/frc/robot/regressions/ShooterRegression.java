package frc.robot.regressions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterRegression {
	public static final double[][] shotRPSData = {
			// distance (m), shooter rps, spinner rps
			// TODO: actually fill data
			{ 1.0, 60.0, 60.0 },
	};

	public static final InterpolatingTreeMap<Double, double[]> shotRPSMap = new InterpolatingTreeMap<Double, double[]>(
			InverseInterpolator.forDouble(), new Interpolator<double[]>() {
				public double[] interpolate(double[] startValue, double[] endValue, double t) {
					double[] new_vals = {
							MathUtil.interpolate(startValue[0], endValue[0], t),
							MathUtil.interpolate(startValue[1], endValue[1], t),
					};
					return new_vals;
				};
			});

	static {
		for (double[] data : shotRPSData) {
			shotRPSMap.put(data[0], new double[] { data[1], data[2] });
		}
	}

	public static double[] getRegressionRPS(double distance) {
		return shotRPSMap.get(distance);
	}
}

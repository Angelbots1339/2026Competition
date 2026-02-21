package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	private Swerve swerve;
	private Shooter shooter;

	private Supplier<Double> x;
	private Supplier<Double> y;
	private Supplier<Boolean> runKicker;

	public Shoot(Swerve swerve, Shooter shooter, Supplier<Double> x, Supplier<Double> y,
			Supplier<Boolean> runKicker) {
		this.swerve = swerve;
		this.shooter = shooter;

		this.x = x;
		this.y = y;
		this.runKicker = runKicker;
		addRequirements(shooter, swerve);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);

		shooter.setRPS(params.shooterRPS(), params.spinnerRPS());

		if (shooter.atSetpoint() && runKicker.get()) {
			shooter.setKickerVelocity(ShooterConstants.KickerRPS);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.disableKicker();
		shooter.disableShooter();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

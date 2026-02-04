package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	private Swerve swerve;
	private Shooter shooter;

	private Supplier<Double> x;
	private Supplier<Double> y;

	public Shoot(Swerve swerve, Shooter shooter, Supplier<Double> x, Supplier<Double> y) {
		this.swerve = swerve;
		this.shooter = shooter;

		this.x = x;
		this.y = y;
		addRequirements(shooter, swerve);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		ShooterParams params = ShooterRegression.getShotParams(swerve);
		swerve.angularDriveRequest(x, y, () -> params.angle(), () -> true);
		shooter.setRPS(params.shooterRPS());
		shooter.runIndex(2);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.disableIndex();
		shooter.disableShooter();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

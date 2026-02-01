package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
	private Shooter shooter;
	private Indexer indexer;
	private Swerve swerve;

	private Supplier<Double> x;
	private Supplier<Double> y;

	private Supplier<Boolean> runIndex;

	public Shoot(Shooter shooter, Indexer indexer, Swerve swerve, Supplier<Double> x, Supplier<Double> y,
			Supplier<Boolean> runIndex) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.swerve = swerve;

		this.x = x;
		this.y = y;

		this.runIndex = runIndex;
		addRequirements(shooter, indexer, swerve);
	}

	@Override
	public void initialize() {
		// TODO: replace with regression
		shooter.setRPS(ShooterConstants.shootRPS);

	}

	@Override
	public void execute() {
		Pose2d target = FieldUtil.getHubCenter();

		Supplier<Rotation2d> angle = () -> {
			double xdiff = target.getX() - swerve.getPose().getX();
			double ydiff = target.getY() - swerve.getPose().getY();
			return Rotation2d.fromRadians(Math.atan2(ydiff, xdiff));
		};

		swerve.angularDriveRequest(x, y, angle, () -> true);

		if (shooter.isAtSetpoint() && runIndex.get()) {
			indexer.setVoltage(Volts.of(2));
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.disable();
		indexer.disable();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

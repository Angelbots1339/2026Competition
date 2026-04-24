package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.choreo.ChoreoTraj;
import frc.lib.util.FieldUtil;
import frc.robot.commands.AutoShoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// this is where we call all and create our autos
// we are using choreo to create time-optimized trajectories and calling 
// commands at certain points in the path
public class Autos {
	private AutoFactory factory;

	Supplier<Command> shoot = null;

	public Autos(Swerve swerve, Shooter shooter, Intake intake, Indexer indexer) {
		factory = new AutoFactory(
				swerve::getPose, // A function that returns the current field-relative Pose2d of the robot
				swerve::resetPose, // A function that receives a field-relative Pose2d to reset the robot's
									// odometry to.
				swerve.followChoreoPath(), // A function that receives the current SampleType and controls the
											// robot. More info below
				true, // If this is true, when on the red alliance, the path will be mirrored to the
						// opposite side, while keeping the same coordinate system origin.
				swerve); // The drive Subsystem to require for AutoTrajectory Commands.

		shoot = () -> new AutoShoot(swerve, shooter, indexer, intake, () -> 0.0, () -> 0.0);

		// this binds a command to run when the timer in a trajectory overlaps an event
		// this is global and can not be overwritten
		factory.bind("RevUpShooter", shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));
		factory.bind("IntakeStart", intake.runIntake());
		factory.bind("IntakeStop", intake.stopIntake());
		// when java runs classes, it has to initially load in classes, which could take a lot of time, especially
		// at startup (auto start)
		// as such, try to run through the autofactory classes to cache these
		// and save time on startup
		CommandScheduler.getInstance().schedule(factory.trajectoryCmd("").ignoringDisable(true));
	}

	public AutoRoutine rightNeutralNeutral() {
		final var routine = factory.newRoutine("Right Neutral Neutral");
		// Choreo can automatically create ChoreoTraj and ChoreoVars which we 
		// can use to ensure we have the right path names
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.5);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine leftNeutralNeutral() {
		final var routine = factory.newRoutine("Left Neutral Neutral");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name());
		final var leftNeutral2 = routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.5);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine rightNeutralSweep() {
		final var routine = factory.newRoutine("Right Neutral Sweep");
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.1);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine leftNeutralSweep() {
		final var routine = factory.newRoutine("Left Neutral Sweep");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name());
		final var leftNeutral2 = routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.1);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine rightNeutralSendSweep() {
		final var routine = factory.newRoutine("Right Send Sweep");
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_NeutralSend.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.1);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine leftNeutralSendSweep() {
		final var routine = factory.newRoutine("Left Send Sweep");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.Bump_To_NeutralSend.name());
		final var leftNeutral2 = routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(2.8);
		final var shoot2 = shoot.get().withTimeout(3.1);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	// thin function flips all trajectories across the horizontol center line
	// its done by flipping the y value across the horizontal center line,
	// flipping rotation, omega, and velocity y, and module direction y components
	private Trajectory<SwerveSample> flipTrajectoryX(Trajectory<SwerveSample> traj) {
		SwerveSample[] new_samples = new SwerveSample[traj.samples().size()];
		int i = 0;
		for (SwerveSample sample : traj.samples()) {
			new_samples[i++] = new SwerveSample(sample.t, sample.x, FieldUtil.FieldHeight.in(Meters) - sample.y,
					-sample.heading,
					sample.vx, -sample.vy,
					-sample.omega, sample.ax, -sample.ay, -sample.alpha, sample.moduleForcesX(), new double[] {
							-sample.moduleForcesY()[1],
							-sample.moduleForcesY()[0],
							-sample.moduleForcesY()[3],
							-sample.moduleForcesY()[2]
					});
		}

		return new Trajectory<SwerveSample>(traj.name(), List.of(new_samples), traj.splits(), traj.events());
	}
}

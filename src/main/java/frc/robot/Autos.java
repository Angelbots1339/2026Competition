package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.choreo.ChoreoTraj;
import frc.lib.util.FieldUtil;
import frc.robot.commands.RegressionShoot;
import frc.robot.regression.ShooterRegression;
import frc.robot.regression.ShooterRegression.ShooterParams;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Autos {
	private AutoFactory factory;

	SendableChooser<String> firstPathChooser = new SendableChooser<String>();
	SendableChooser<String> secondPathChooser = new SendableChooser<String>();
	SendableChooser<String> thirdPathChooser = new SendableChooser<String>();
	SendableChooser<String> sideChooser = new SendableChooser<String>();

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

		shoot = () -> new RegressionShoot(swerve, shooter, indexer, intake, () -> 0.0, () -> 0.0);

		factory.bind("RevUpShooter", shooter.run(() -> {
			ShooterParams params = ShooterRegression.getShotParams(swerve);
			shooter.setRPS(params.shooterRPS(), params.spinnerRPS());
		}));
		factory.bind("IntakeStart", intake.runIntake());
		factory.bind("IntakeStop", intake.stopIntake());
		// CommandScheduler.getInstance().schedule(factory.trajectoryCmd("").ignoringDisable(true));

		publishAutoPaths();
	}

	public void publishAutoPaths() {
		String[] startPaths = ChoreoTraj.ALL_TRAJECTORIES.keySet().stream()
				.filter(traj -> traj.startsWith("Bump") || traj.startsWith("Hub")).toArray(String[]::new);

		String[] secondPaths = ChoreoTraj.ALL_TRAJECTORIES.keySet().stream()
				.filter(traj -> !(traj.startsWith("Bump") || traj.startsWith("Hub"))).toArray(String[]::new);

		for (String path : startPaths) {
			firstPathChooser.addOption(path, path);
		}

		for (String path : secondPaths) {
			secondPathChooser.addOption(path, path);
		}

		thirdPathChooser.addOption(ChoreoTraj.NeutralShoot_SendToNeutral.name(),
				ChoreoTraj.NeutralShoot_SendToNeutral.name());

		sideChooser.setDefaultOption("Left", "Left");
		sideChooser.addOption("Right", "Right");

		firstPathChooser.setDefaultOption("None", "");
		secondPathChooser.setDefaultOption("None", "");
		thirdPathChooser.setDefaultOption("None", "");

		SmartDashboard.putData("Auto/First Path", firstPathChooser);
		SmartDashboard.putData("Auto/Second Path", secondPathChooser);
		SmartDashboard.putData("Auto/Third Path", thirdPathChooser);
		SmartDashboard.putData("Auto/Side", sideChooser);
	}

	// TODO: figure out if calling resetOdometry() in the command is necessary

	public AutoRoutine customAuto() {
		final var routine = factory.newRoutine("Custom Auto");
		AutoTrajectory startTraj = routine.trajectory("");
		AutoTrajectory secondTraj = routine.trajectory("");
		AutoTrajectory thirdTraj = routine.trajectory("");

		boolean needsFlip = sideChooser.getSelected() == "Right";
		String firstPath = firstPathChooser.getSelected();
		String secondPath = secondPathChooser.getSelected();
		String thirdPath = thirdPathChooser.getSelected();

		if (needsFlip) {
			startTraj = routine.trajectory(flipTrajectoryX(routine.trajectory(firstPath).getRawTrajectory()));
			secondTraj = routine.trajectory(flipTrajectoryX(routine.trajectory(secondPath).getRawTrajectory()));
			thirdTraj = routine.trajectory(flipTrajectoryX(routine.trajectory(thirdPath).getRawTrajectory()));
		} else {
			startTraj = routine.trajectory(firstPath);
			secondTraj = routine.trajectory(secondPath);
			thirdTraj = routine.trajectory(thirdPath);
		}

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		CommandScheduler.getInstance().schedule(
				startTraj.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));

		routine.active().onTrue(startTraj.cmd());
		startTraj.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(secondTraj.cmd());
		secondTraj.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(thirdTraj.cmd());

		return routine;
	}

	public Command hubDepotAuto() {
		final var routine = factory.newRoutine("Hub Depot");
		final var hubToDepot = routine.trajectory(ChoreoTraj.Hub_To_Depot.name());

		final var shoot1 = shoot.get().withTimeout(5);

		CommandScheduler.getInstance().schedule(
				hubToDepot.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));
		routine.active().onTrue(hubToDepot.cmd());
		hubToDepot.done().onTrue(shoot1);

		return routine.cmd();
	}

	public Command hubDepotNeutralAuto() {
		final var routine = factory.newRoutine("Hub Depot Neutral");
		final var hubToDepot = routine.trajectory(ChoreoTraj.Hub_To_Depot.name());
		final var NeutralShootNeutral2 = routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(4.5);
		final var shoot2 = shoot.get().withTimeout(5);

		CommandScheduler.getInstance().schedule(
				hubToDepot.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));
		routine.active().onTrue(hubToDepot.cmd());
		hubToDepot.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(NeutralShootNeutral2.cmd());
		NeutralShootNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine.cmd();
	}

	public AutoRoutine rightNeutral() {
		final var routine = factory.newRoutine("Right Neutral");
		final var bumpToNeutral = routine
				.trajectory(
						flipTrajectoryX(routine.trajectory(ChoreoTraj.Bump_To_Neutral.name()).getRawTrajectory()));
		final var leftNeutral2 = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name()).getRawTrajectory()));
		final var NeutralSend = routine.trajectory(
				flipTrajectoryX(routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name()).getRawTrajectory()));

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		CommandScheduler.getInstance().schedule(
				bumpToNeutral.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));

		routine.active().onTrue(bumpToNeutral.cmd());
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

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		routine.active().onTrue(bumpToNeutral.resetOdometry().andThen(bumpToNeutral.cmd()));
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(leftNeutral2.cmd());
		leftNeutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine leftNeutral() {
		final var routine = factory.newRoutine("Left Neutral");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.Bump_To_Neutral.name());
		final var neutral2 = routine.trajectory(ChoreoTraj.Shoot_To_Neutral.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		CommandScheduler.getInstance().schedule(
				bumpToNeutral.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));
		routine.active().onTrue(bumpToNeutral.cmd());
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(neutral2.cmd());
		neutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

	public AutoRoutine leftNeutralSweep() {
		final var routine = factory.newRoutine("Left Neutral Sweep");
		final var bumpToNeutral = routine.trajectory(ChoreoTraj.Bump_To_NeutralSweep.name());
		final var neutral2 = routine.trajectory(ChoreoTraj.Shoot_To_HubSweep.name());
		final var NeutralSend = routine.trajectory(ChoreoTraj.NeutralShoot_SendToNeutral.name());

		final var shoot1 = shoot.get().withTimeout(3.5);
		final var shoot2 = shoot.get().withTimeout(3.5);

		CommandScheduler.getInstance().schedule(
				bumpToNeutral.resetOdometry().repeatedly().ignoringDisable(true).onlyWhile(DriverStation::isDisabled));
		routine.active().onTrue(bumpToNeutral.cmd());
		bumpToNeutral.done().onTrue(shoot1);
		routine.observe(shoot1::isFinished).onTrue(neutral2.cmd());
		neutral2.done().onTrue(shoot2);
		routine.observe(shoot2::isFinished).onTrue(NeutralSend.cmd());

		return routine;
	}

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

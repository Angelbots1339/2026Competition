package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Autos {
	private AutoFactory factory;

	public Autos(Swerve swerve) {
		factory = new AutoFactory(
				swerve::getPose, // A function that returns the current field-relative Pose2d of the robot
				swerve::resetPose, // A function that receives a field-relative Pose2d to reset the robot's
									// odometry to.
				swerve.followChoreoPath(), // A function that receives the current SampleType and controls the
											// robot. More info below
				true, // If this is true, when on the red alliance, the path will be mirrored to the
						// opposite side, while keeping the same coordinate system origin.
				swerve); // The drive Subsystem to require for AutoTrajectory Commands.
	}

	public Command hubDepotTowerAuto() {
		final var routine = factory.newRoutine("Hub Depot Tower");
		final var traj = routine.trajectory("HubDepotTower");
		routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
		return routine.cmd();
	}

	public Command leftPassAuto() {
		final var routine = factory.newRoutine("Left Pass");
		final var traj = routine.trajectory("PassLeft");
		routine.active().whileTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
		return routine.cmd();
	}
}

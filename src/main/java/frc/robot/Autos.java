package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import java.util.HashMap;
import java.util.List;

public final class Autos {
	// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
	// return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	// }

	public static Command simpleWall(Swerve swerve) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("simple wall",
			new PathConstraints(SwerveConstants.maxVelocity_mps, 3.0));

		if (pathGroup == null)
			return new PrintCommand("no path group");

		HashMap<String, Command> eventMap = new HashMap<>();
		eventMap.put("start", new PrintCommand("put block down"));

		return new FollowPathWithEvents(
			new PathPlannerCommand(pathGroup.get(0), swerve, true),
			pathGroup.get(0).getMarkers(),
			eventMap);
	}
}

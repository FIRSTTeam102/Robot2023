package frc.robot;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.scoring.SetElevatorArmPosition;
import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

	// used by pathplanner
	public static final PIDController ppXController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppYController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppRotationController = new PIDController(autoAngleKp, autoAngleKi, autoAngleKd); // rad
	static {
		ppRotationController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public static Command runAutoPath(String path, RobotContainer robo) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path,
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		if (pathGroup == null)
			return new PrintCommand("no path group");

		HashMap<String, Command> eventMap = new HashMap<>();
		eventMap.put("allIn",
			new SetElevatorArmPosition(robo.elevator, robo.arm,
				ElevatorConstants.inHeight_m, ArmConstants.inExtension_m, 0.1));
		eventMap.put("scoreMidCone", Commands.sequence(
			new SetElevatorArmPosition(robo.elevator, robo.arm,
				ElevatorConstants.midConeHeight_m, ArmConstants.midConeExtension_m, 0.1),
			new MoveElevatorBy(robo.elevator, ElevatorConstants.coneMoveDownHeight_m),
			new ReleaseGrabber(robo.grabber)));
		eventMap.put("intakeGround", Commands.sequence(
			// both sets are async
			new SetArmPosition(robo.arm, ArmConstants.groundExtension_m),
			new SetElevatorPosition(robo.elevator, ElevatorConstants.groundHeight_m),
			new GrabGrabberUntilGrabbed(robo.grabber)));
		eventMap.put("scoreHighCube", Commands.sequence(
			new SetElevatorArmPosition(robo.elevator, robo.arm,
				ElevatorConstants.highCubeHeight_m, ArmConstants.highCubeExtension_m, 0.1),
			new ReleaseGrabber(robo.grabber)));

		var sequence = new SequentialCommandGroup();

		for (int i = 0; i < pathGroup.size(); i++)
			sequence.addCommands(new FollowPathWithEvents(
				new PathPlannerCommand(pathGroup.get(i), robo.swerve, true, true),
				pathGroup.get(0).getMarkers(),
				eventMap));

		return sequence;
	}
}

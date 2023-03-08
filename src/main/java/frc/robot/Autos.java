package frc.robot;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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

	public static Command deadlineSeconds(double time_s, Command... commands) {
		return Commands.deadline(Commands.waitSeconds(time_s), commands);
	}

	public static Command allIn(RobotContainer robo) {
		return new SetScoringPosition(robo.elevator, robo.arm, ScoringPosition.AllIn, AutoConstants.tolerance_m);
	}

	public static Command intakeGround(RobotContainer robo) {
		return Commands.sequence(
			// both sets are async
			new SetArmPosition(robo.arm, ScoringPosition.Ground.armExtension_m),
			new SetElevatorPosition(robo.elevator, ScoringPosition.Ground.elevatorHeight_m),
			new GrabGrabberUntilGrabbed(robo.grabber));
	}

	public static Command score(RobotContainer robo, ScoringPosition pos) {
		var sequence = new SequentialCommandGroup(
			new SetScoringPosition(robo.elevator, robo.arm, pos, 0.1));
		if (pos.moveDown)
			sequence.addCommands(new MoveElevatorBy(robo.elevator, ElevatorConstants.coneMoveDownHeight_m));
		sequence.addCommands(new ReleaseGrabber(robo.grabber));
		return sequence;
	}

	public static Command runAutoPath(PathPlannerTrajectory path, Swerve swerve) {
		return runAutoPath(path, swerve, false);
	}

	public static Command runAutoPath(PathPlannerTrajectory path, Swerve swerve, boolean firstPathEver) {
		return new PathPlannerCommand(path, swerve, true, firstPathEver);
	}

	/** 2 pice auto */
	public static Command two(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("2 piece",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			score(robo, ScoringPosition.MidCone),
			allIn(robo),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGround(robo),
			allIn(robo),
			runAutoPath(path.get(1), robo.swerve),
			score(robo, ScoringPosition.HighCube),
			allIn(robo));
	}
}

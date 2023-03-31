package frc.robot.commands;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.PathPlannerCommand;
import frc.robot.commands.swerve.XStance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

	public static Command allIn(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllIn, 0.2, 0.2);
	}

	public static Command goForward(Swerve swerve, double speed_mps) {
		return Commands.startEnd(
			() -> swerve.drive(new Translation2d(speed_mps, 0), 0, false),
			() -> swerve.stop(), swerve);
	}

	public static Command intakeGround(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		return deadlineSeconds(2, Commands.sequence(
			new SetScoringPosition(elevator, arm, ScoringPosition.Ground, elevatorTolerance_m, 0.2),
			Commands.deadline(new GrabGrabberUntilGrabbed(grabber, GrabberConstants.cubeGrabSpeed), goForward(swerve, 1.3))));
	}

	public static Command intakeGroundUntimed(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		return Commands.sequence(
			new SetScoringPosition(elevator, arm, ScoringPosition.Ground, elevatorTolerance_m, 0.2),
			Commands.deadline(new GrabGrabberUntilGrabbed(grabber, GrabberConstants.cubeGrabSpeed), goForward(swerve, 1.3)),
			new SetScoringPosition(elevator, arm, ScoringPosition.AllIn));
	}

	public static Command score(Elevator elevator, Arm arm, Grabber grabber, ScoringPosition pos) {
		var sequence = new SequentialCommandGroup(
			Commands.print("setting scoring position"),
			new SetScoringPosition(elevator, arm, pos, elevatorTolerance_m, armTolerance_m),
			// fixme: why does this get stuck here
			Commands.print("scoring position set"));
		if (pos.moveDown)
			sequence.addCommands(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m),
				Commands.print("moved down"));
		sequence.addCommands(Commands.print("releasing"), new ReleaseGrabber(grabber), Commands.waitSeconds(0.3));
		return sequence;
	}

	public static Command grabTimed(Grabber grabber) {
		return deadlineSeconds(0.1, new GrabGrabber(grabber, GrabberConstants.coneGrabSpeed));
	}

	/** runs balance until balanced or auto is about to end, then locks the wheels */
	public static Command balance(Swerve swerve) {
		return Commands.sequence(
			Commands.race(Commands.waitUntil(() -> DriverStation.getMatchTime() < 0.5),
				new ChargeStationBalance(swerve)),
			new XStance(swerve));
	}

	// angle = ppState.holonomicRotation
	public static Command swerveAnglesTo(Swerve swerve, Rotation2d angle) {
		var states = swerve.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0), swerve.getCenterRotation());
		for (var state : states)
			state.angle = angle;
		return new InstantCommand(() -> {
			swerve.setModuleStates(states, true, true);
		});
	}

	public static Command swerveAnglesTo0(Swerve swerve) {
		return swerveAnglesTo(swerve, new Rotation2d(0));
	}

	public static Command autoPath(Swerve swerve, PathPlannerTrajectory path) {
		return autoPath(swerve, path, false);
	}

	public static Command autoPath(Swerve swerve, PathPlannerTrajectory path, boolean firstPathEver) {
		return new PathPlannerCommand(path, swerve, true, firstPathEver);
	}

	/*
	 * routines
	 * for naming: start location (lz, fw, coop) then one-word description for each part (cube, cone, balance, ...)
	 */

	public static Command initAndScore(RobotContainer robo, ScoringPosition position) {
		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, position),
			allIn(robo.elevator, robo.arm));
	}

	public static Command coopCubeBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coopCubeBalance",
			new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			balance(robo.swerve));
	}

	public static Command coopCubeMobilityBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coopCubeMobilityBalance",
			new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2),
			new PathConstraints(balanceMaxVelocity_mps + 0.4, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			Commands.waitSeconds(0.5), // wait for charge station to stabilize first
			autoPath(robo.swerve, path.get(1)),
			balance(robo.swerve));
	}

	public static Command lzCube(RobotContainer robo, boolean intakeAfter) {
		var path = PathPlanner.loadPathGroup("lzCube",
			new PathConstraints(slowerMaxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true));
		// intakeAfter ? intakeGroundClose(robo.swerve, robo.elevator, robo.arm, robo.grabber) : Commands.none());
	}

	public static Command fwCube(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fwCube",
			new PathConstraints(slowerMaxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true));
	}

	public static Command fwCubeBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fwCubeBalance",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2),
			new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			// initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			autoPath(robo.swerve, path.get(1)),
			balance(robo.swerve));
	}

	/** 2 pice auto by field wall */
	@Deprecated
	public static Command fw2Cube(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fw2Cube",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			intakeGround(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(1)),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
			allIn(robo.elevator, robo.arm));
	}

	/** 2 piece auto w/ charge station by field wall */
	@Deprecated
	public static Command fw2CubeBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fw2CubeBalance",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			intakeGround(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(1)),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(2)),
			balance(robo.swerve));
	}

	/** two piece by the loading zone wall */
	@Deprecated
	public static Command lz2Cube(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lz2Cube",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			intakeGround(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(1)),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
			allIn(robo.elevator, robo.arm));
	}

	@Deprecated
	public static Command lz2CubeBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lz2CubeBalance",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

		return new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			autoPath(robo.swerve, path.get(0), true),
			intakeGround(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(1)),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
			allIn(robo.elevator, robo.arm),
			autoPath(robo.swerve, path.get(2)),
			balance(robo.swerve));
	}
}

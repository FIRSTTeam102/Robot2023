package frc.robot;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

	public static Command allIn(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllIn, 0.2, 0.2);
	}

	public static Command goForward(Swerve swerve) {
		return Commands.startEnd(
			() -> swerve.drive(new Translation2d(SwerveConstants.maxVelocity_mps * 0.2, 0), 0, true),
			() -> swerve.stop(), swerve);
	}

	public static Command intakeGroundClose(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		return deadlineSeconds(3, Commands.sequence(
			// both sets are async
			new SetArmPosition(arm, ScoringPosition.Ground.armExtension_m),
			new SetElevatorPosition(elevator, ScoringPosition.Ground.elevatorHeight_m),
			Commands.deadline(new GrabGrabberUntilGrabbed(grabber), goForward(swerve))));
	}

	public static Command intakeGroundFar(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		return deadlineSeconds(3, Commands.sequence(
			// both sets are async
			new SetArmPosition(arm, ScoringPosition.GroundFar.armExtension_m),
			new SetElevatorPosition(elevator, ScoringPosition.GroundFar.elevatorHeight_m),
			Commands.deadline(new GrabGrabberUntilGrabbed(grabber), goForward(swerve))));
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
		return deadlineSeconds(0.1, new GrabGrabber(grabber));
	}

	public static Command balance(Swerve swerve) {
		return new ChargeStationBalance(swerve);
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

	public static Command runAutoPath(PathPlannerTrajectory path, Swerve swerve) {
		return runAutoPath(path, swerve, false);
	}

	public static Command runAutoPath(PathPlannerTrajectory path, Swerve swerve, boolean firstPathEver) {
		return new PathPlannerCommand(path, swerve, true, firstPathEver);
	}

	/** 2 pice auto by field wall */
	public static Command twoPieceFW(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("2 piece fw",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCone),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(1), robo.swerve),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm));
	}

	/** 2 piece auto w/ charge station by field wall */
	public static Command twoPieceChargeStation(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("charge station 2 piece fw",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(1), robo.swerve),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(2), robo.swerve),
			balance(robo.swerve));
	}

	/** two piece by the loading zone wall */
	public static Command twoPieceLZ(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("2 piece lz",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCone),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.swerve, robo.elevator, robo.arm, robo.grabber),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(1), robo.swerve),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm));
	}

	/** start by loading zone, score high cube, and leave fast */
	public static Command loadingZone(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("loading zone",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			Commands.print("ready to go!"),
			runAutoPath(path.get(0), robo.swerve, true));
	}

	public static Command justScore(RobotContainer robo) {
		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm));
	}

	public static Command fieldWallOnePiece(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("field wall 1 piece",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			Commands.print("ready to go!"),
			runAutoPath(path.get(0), robo.swerve, true));
	}

	/** start in middle, backup over CS to get mobility, then balance on charge station */
	public static Command coopBackup(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coop backup",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			swerveAnglesTo0(robo.swerve),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			balance(robo.swerve));
	}
}

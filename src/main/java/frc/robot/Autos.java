package frc.robot;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
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

	public static Command allIn(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllIn, AutoConstants.tolerance_m);
	}

	public static Command intakeGroundClose(Elevator elevator, Arm arm, Grabber grabber) {
		return deadlineSeconds(3, Commands.sequence(
			// both sets are async
			new SetArmPosition(arm, ScoringPosition.Ground.armExtension_m),
			new SetElevatorPosition(elevator, ScoringPosition.Ground.elevatorHeight_m),
			new GrabGrabberUntilGrabbed(grabber)));
	}

	public static Command intakeGroundFar(Elevator elevator, Arm arm, Grabber grabber) {
		return Commands.sequence(
			// both sets are async
			new SetArmPosition(arm, ScoringPosition.GroundFar.armExtension_m),
			new SetElevatorPosition(elevator, ScoringPosition.GroundFar.elevatorHeight_m),
			new GrabGrabberUntilGrabbed(grabber));
	}

	public static Command score(Elevator elevator, Arm arm, Grabber grabber, ScoringPosition pos) {
		var sequence = new SequentialCommandGroup(
			new SetScoringPosition(elevator, arm, pos, AutoConstants.tolerance_m));
		if (pos.moveDown)
			sequence.addCommands(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m));
		sequence.addCommands(Commands.waitSeconds(0.8), new ReleaseGrabber(grabber), Commands.waitSeconds(0.3));
		return sequence;
	}

	public static Command grabTimed(Grabber grabber) {
		return deadlineSeconds(0.1, new GrabGrabber(grabber));
	}

	public static Command balance(Swerve swerve) {
		return new ChargeStationBalance(swerve);
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
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCone),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.elevator, robo.arm, robo.grabber),
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
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCone),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.elevator, robo.arm, robo.grabber),
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
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCone),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			intakeGroundClose(robo.elevator, robo.arm, robo.grabber),
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
			grabTimed(robo.grabber),
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true));
	}

	/** start in middle, backup, then go to charge station. LIKELY WILL NOT WORK! USE WITH CAUTION!!! */
	public static Command coopBackup(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coop backup",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		if (path == null)
			return new PrintCommand("no path group");

		return new SequentialCommandGroup(
			score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.HighCube),
			allIn(robo.elevator, robo.arm),
			runAutoPath(path.get(0), robo.swerve, true),
			balance(robo.swerve));
	}
}

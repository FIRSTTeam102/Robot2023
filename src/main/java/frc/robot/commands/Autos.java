package frc.robot.commands;

import static frc.robot.constants.AutoConstants.*;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.GrabConeOrCubeUntilGrabbed;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.PathPlannerCommand;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.GamePieceVision;
import frc.robot.commands.vision.RetroreflectiveVision;

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

import java.util.function.Supplier;

public final class Autos {
	// used by pathplanner
	public static final PIDController ppXController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppYController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public static final PIDController ppRotationController = new PIDController(autoAngleKp, autoAngleKi, autoAngleKd); // rad
	static {
		ppRotationController.enableContinuousInput(-Math.PI, Math.PI);
		ppRotationController.setTolerance(0.01, 0.01);
	}

	public static Command deadlineSeconds(double time_s, Command... commands) {
		return Commands.deadline(Commands.waitSeconds(time_s), commands);
	}

	public static Command allIn(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllIn, 0.4, 0.2);
	}

	public static Command allInHigher(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllInHigher, 0.4, 0.2);
	}

	public static Command allInBumper(Elevator elevator, Arm arm) {
		return new SetScoringPosition(elevator, arm, ScoringPosition.AllInBumper, 0.5, 0.2);
	}

	public static Command goForward(Swerve swerve, double speed_mps) {
		return Commands.startEnd(
			() -> swerve.drive(new Translation2d(speed_mps, 0), 0, false),
			() -> swerve.stop(), swerve);
	}

	public static Command gamePieceAlign(Swerve swerve, Vision vision, double blueStartAngle_rad,
		double redStartAngle_rad) {
		// return Commands.sequence(
		// deadlineSeconds(1,
		// new ProxyCommand(() -> new TurnToAngle(Robot.isBlue() ? blueStartAngle_rad : redStartAngle_rad, swerve))),
		return Commands.sequence(Commands.race(
			Commands.waitSeconds(1.5),
			Commands.waitUntil(() -> !vision.inputs.gamePieceVisionTarget), // if we lost the target, just go on
			Commands.sequence(
				new GamePieceVision(GamePieceVision.Routine.GamePieceGround, vision, swerve),
				new GamePieceVision(GamePieceVision.Routine.GamePieceGround, vision, swerve))),
			Commands.waitSeconds(0.5));
	}

	public static Command waitForElevator(Elevator elevator) {
		return Commands.waitUntil(() -> elevator.withinTargetPosition());
	}

	public static Command intakeGroundForAuto(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber,
		double deadline_s) {
		return intakeGroundForAuto(swerve, elevator, arm, grabber, deadline_s, GrabberConstants.cubeGrabSpeed);
	}

	/** will not cross the center line */
	public static Command intakeGroundForAuto(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber,
		double deadline_s, double grabSpeed) {
		return deadlineSeconds(deadline_s, Commands.sequence(
			new SetElevatorPosition(elevator, ScoringPosition.AllIn.elevatorHeight_m),
			waitForElevator(elevator),
			new SetScoringPosition(elevator, arm, ScoringPosition.Ground, elevatorTolerance_m, 0.2),
			Commands.runOnce(() -> swerve.autoAprilTag = false), // it shouldn't be seeing any tags rn, just to be safe
			Commands.race(
				Commands.waitUntil(() -> Robot.isBlue() // kill driving if we're crossing the center line
					? swerve.getPose().getX() > ((FieldConstants.fieldLengthX_m / 2) - centerClearance_m)
					: swerve.getPose().getX() < ((FieldConstants.fieldLengthX_m / 2) + centerClearance_m)),
				new GrabGrabberUntilGrabbed(grabber, grabSpeed, 12),
				goForward(swerve, 1.3)),
			Commands.runOnce(() -> swerve.autoAprilTag = true)));
	}

	public static Command intakeGroundUntimed(Swerve swerve, Elevator elevator, Arm arm, Grabber grabber, Vision vision) {
		return Commands.sequence(
			new SetElevatorPosition(elevator, ScoringPosition.AllInBumper.elevatorHeight_m),
			waitForElevator(elevator),
			new SetScoringPosition(elevator, arm, ScoringPosition.Ground, elevatorTolerance_m, 0.2),
			Commands.deadline(
				new GrabConeOrCubeUntilGrabbed(grabber, vision),
				Commands.sequence(
					Commands.waitUntil(() -> grabber.inputs.percentOutput >= 0.9 * GrabberConstants.cubeGrabSpeed),
					goForward(swerve, 1.3))),
			new SetScoringPosition(elevator, arm, ScoringPosition.AllIn));
	}

	/** if we don't have a game piece then don't go back */
	public static Command stopIfNoPiece(Grabber grabber) {
		return Commands.waitUntil(() -> grabber.hasGrabbed(GrabberConstants.grabbedTicks));
	}

	public static Command score(Elevator elevator, Arm arm, Grabber grabber, ScoringPosition pos) {
		var sequence = new SequentialCommandGroup(
			new SetScoringPosition(elevator, arm, pos, elevatorTolerance_m, armTolerance_m));
		if (pos.isCone)
			// todo: vision alignment
			sequence.addCommands(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m));
		sequence.addCommands(new ReleaseGrabber(grabber), Commands.waitSeconds(0.1));
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

	public static Command initAndScore(RobotContainer robo, ScoringPosition position) {
		return Commands.sequence(
			deadlineSeconds(5, Commands.sequence(
				swerveAnglesTo0(robo.swerve),
				grabTimed(robo.grabber),
				score(robo.elevator, robo.arm, robo.grabber, position),
				allInBumper(robo.elevator, robo.arm))),
			// make sure they get set anyways, though there should be enough time
			new SetArmPosition(robo.arm, ScoringPosition.AllInBumper.armExtension_m),
			new SetElevatorPosition(robo.elevator, ScoringPosition.AllInBumper.elevatorHeight_m));
	}

	/*
	 * routines
	 * for naming: start location (lz, fw, coop) then one-word description for each part (cube, cone, balance, ...)
	 */

	public static Supplier<Command> coopCubeBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coopCubeBalance",
			new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			balance(robo.swerve));
	}

	public static Supplier<Command> coopCubeMobilityBalance(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("coopCubeMobilityBalance",
			new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2),
			new PathConstraints(balanceMaxVelocity_mps + 0.4, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			Commands.runOnce(() -> robo.swerve.autoAprilTag = false),
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			Commands.waitSeconds(0.25), // wait for charge station to stabilize first
			pathCmds.get(1).get(),
			balance(robo.swerve));
	}

	public static Supplier<Command> lzCube(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lzCube",
			new PathConstraints(slowerMaxVelocity_mps, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get());
		// intakeAfter ? intakeGroundClose(robo.swerve, robo.elevator, robo.arm, robo.grabber) : Commands.none());
	}

	public static Supplier<Command> fwCube(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fwCube",
			new PathConstraints(slowerMaxVelocity_mps, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get());
	}

	/** 2 pice auto by field wall */
	public static Supplier<Command> fwCube2(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fwCube2",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			gamePieceAlign(robo.swerve, robo.vision, leftGamepieceAngle_rad, rightGamepieceAngle_rad),
			intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 3),
			stopIfNoPiece(robo.grabber),
			allIn(robo.elevator, robo.arm),
			pathCmds.get(1).get(),
			// score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
			new ReleaseGrabber(robo.grabber),
			allIn(robo.elevator, robo.arm));
	}

	/** two piece by the loading zone wall */
	public static Supplier<Command> lzCube2(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lzCube2",
			new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			gamePieceAlign(robo.swerve, robo.vision, rightGamepieceAngle_rad, leftGamepieceAngle_rad),
			intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 3.5),
			stopIfNoPiece(robo.grabber),
			allIn(robo.elevator, robo.arm),
			pathCmds.get(1).get(),
			new ReleaseGrabber(robo.grabber));
	}

	public static Supplier<Command> lzCubeAimCone(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lzCubeAimCone",
			new PathConstraints(3, maxAcceleration_mps2));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			gamePieceAlign(robo.swerve, robo.vision, rightGamepieceAngle_rad, leftGamepieceAngle_rad),
			intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 3.5, GrabberConstants.coneGrabSpeed),
			stopIfNoPiece(robo.grabber),
			allIn(robo.elevator, robo.arm),
			pathCmds.get(1).get(),
			new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, robo.vision, robo.swerve),
			new SetElevatorPosition(robo.elevator, ScoringPosition.HighCone.elevatorHeight_m));
	}

	public static Supplier<Command> lzCubePickupCone(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("lzCubePickupCone",
			new PathConstraints(3, 2),
			new PathConstraints(1.5, 0.75));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			gamePieceAlign(robo.swerve, robo.vision, rightGamepieceAngle_rad, leftGamepieceAngle_rad),
			intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 4, GrabberConstants.coneGrabSpeed),
			stopIfNoPiece(robo.grabber),
			allInHigher(robo.elevator, robo.arm),
			new GrabGrabber(robo.grabber, GrabberConstants.coneGrabSpeed),
			pathCmds.get(1).get());
	}

	public static Supplier<Command> fwCubePickupCone(RobotContainer robo) {
		var path = PathPlanner.loadPathGroup("fwCubePickupCone",
			new PathConstraints(2, 2),
			new PathConstraints(1.5, 0.75));
		var pathCmds = PathPlannerCommand.pregenerateAutoPathSuppliers(path, robo.swerve);

		return () -> new SequentialCommandGroup(
			initAndScore(robo, ScoringPosition.HighCube),
			pathCmds.get(0).get(),
			gamePieceAlign(robo.swerve, robo.vision, leftGamepieceAngle_rad, rightGamepieceAngle_rad),
			intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 4, GrabberConstants.coneGrabSpeed),
			stopIfNoPiece(robo.grabber),
			allInHigher(robo.elevator, robo.arm),
			new GrabGrabber(robo.grabber, GrabberConstants.coneGrabSpeed),
			pathCmds.get(1).get());
	}

	// public static Command fwCubeBalance(RobotContainer robo) {
	// var path = PathPlanner.loadPathGroup("fwCubeBalance",
	// new PathConstraints(maxVelocity_mps, maxAcceleration_mps2),
	// new PathConstraints(balanceMaxVelocity_mps, maxAcceleration_mps2));

	// return new SequentialCommandGroup(
	// // initAndScore(robo, ScoringPosition.HighCube),
	// autoPath(robo.swerve, path.get(0), true),
	// autoPath(robo.swerve, path.get(1)),
	// balance(robo.swerve));
	// }

	// /** 2 piece auto w/ charge station by field wall */
	// @Deprecated
	// public static Command fw2CubeBalance(RobotContainer robo) {
	// var path = PathPlanner.loadPathGroup("fw2CubeBalance",
	// new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

	// return new SequentialCommandGroup(
	// initAndScore(robo, ScoringPosition.HighCube),
	// autoPath(robo.swerve, path.get(0), true),
	// intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 2),
	// allIn(robo.elevator, robo.arm),
	// autoPath(robo.swerve, path.get(1)),
	// score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
	// allIn(robo.elevator, robo.arm),
	// autoPath(robo.swerve, path.get(2)),
	// balance(robo.swerve));
	// }

	// @Deprecated
	// public static Command lz2CubeBalance(RobotContainer robo) {
	// var path = PathPlanner.loadPathGroup("lz2CubeBalance",
	// new PathConstraints(maxVelocity_mps, maxAcceleration_mps2));

	// return new SequentialCommandGroup(
	// initAndScore(robo, ScoringPosition.HighCube),
	// autoPath(robo.swerve, path.get(0), true),
	// intakeGroundForAuto(robo.swerve, robo.elevator, robo.arm, robo.grabber, 2),
	// stopIfNoPiece(robo.grabber),
	// allIn(robo.elevator, robo.arm),
	// autoPath(robo.swerve, path.get(1)),
	// score(robo.elevator, robo.arm, robo.grabber, ScoringPosition.MidCube),
	// allIn(robo.elevator, robo.arm),
	// autoPath(robo.swerve, path.get(2)),
	// balance(robo.swerve));
	// }
}

package frc.robot.commands.scoring;

import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.SetElevatorPosition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetScoringPosition extends ProxyCommand {
	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos) {
		this(elevator, arm, pos, 0, 0);
	}

	/**
	 * set tolerance to 0 to ignore it
	 */
	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos,
		double elevatorTolerance_m, double armTolerance_m) {
		this(elevator, arm, pos.elevatorHeight_m, pos.armExtension_m, elevatorTolerance_m, armTolerance_m);
	}

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m,
		double elevatorTolerance_m, double armTolerance_m) {
		super(() -> new SetElevatorArmPositionProxied(elevator, arm, elevatorPos_m, armPos_m,
			elevatorTolerance_m, armTolerance_m));
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation())
			return true; // temp: until we actually sim it, for now we want to test auto
		return super.isFinished();
	}

	@Override
	public void end(boolean interrupted) {
		super.end(true);
		System.out.println("SetScoringPosition ended");
	}

	@Override
	public void initialize() {
		super.initialize();
		System.out.println("SetScoringPosition init");
	}

	private static class SetElevatorArmPositionProxied extends SequentialCommandGroup {
		SetElevatorArmPositionProxied(Elevator elevator, Arm arm, double elevatorTarget_m, double armTarget_m,
			double elevatorTolerance_m, double armTolerance_m) {
			elevator.inManualMode = false;
			arm.inManualMode = false;
			if (elevatorTarget_m > elevator.inputs.position_m) {
				// if we're on the ground, send arm a bit out first so cone lip doesn't get stuck on bumper
				// if (Math.abs(elevator.inputs.position_m - ScoringPosition.Ground.elevatorHeight_m) < 0.08)
				// addCommands(new MoveArmBy(arm, 0.08));

				// going up -> elevator first
				addCommands(
					new SetElevatorPosition(elevator, elevatorTarget_m),
					new WaitUntilCommand(() -> {
						if (elevator.inManualMode || arm.inManualMode)
							return true;

						if (elevator.inputs.position_m < ElevatorConstants.dangerZone_m)
							return false; // until we get above

						return elevator.inputs.position_m > 0.9 * elevatorTarget_m;
					}), // Units.inchesToMeters(30)
					new SetArmPosition(arm, armTarget_m));
			} else {
				// going down -> arm first
				addCommands(
					new SetArmPosition(arm, armTarget_m),
					new WaitUntilCommand(() -> {
						if (elevator.inManualMode || arm.inManualMode)
							return true;

						// Units.inchesToMeters(36)

						// arm going out
						if (armTarget_m > arm.getArmDist_m())
							return arm.getArmDist_m() > 0.6 * armTarget_m;
						// arm going in
						else
							return arm.getArmDist_m() < armTarget_m + 0.15; // when going to 0

						// return Math.abs(arm.getArmDist_m() - armTarget_m) <= 0.1;
					}),
					new SetElevatorPosition(elevator, elevatorTarget_m));
			}

			if (elevatorTolerance_m > 0)
				addCommands(
					new WaitUntilCommand(() -> {
						return Math.abs(elevator.inputs.position_m - elevatorTarget_m) < elevatorTolerance_m;
					}));
			if (armTolerance_m > 0)
				addCommands(new WaitUntilCommand(() -> {
					return Math.abs(arm.getArmDist_m() - armTarget_m) < armTolerance_m;
				}));
			addCommands(Commands.print("SetScoringPosition done"));
		}
	}
}

// @formatter:off
/*
public class SetScoringPosition extends InstantCommand {
	private Elevator elevator;
	private Arm arm;
	private double elevatorTarget_m;
	private double armTarget_m;

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorTarget_m, double armTarget_m) {
		this.elevator = elevator;
		this.arm = arm;
		this.elevatorTarget_m = elevatorTarget_m;
		this.armTarget_m = armTarget_m;
		addRequirements(elevator, arm);
	}

	@Override
	public void initialize() {
		if ((armTarget_m <= ArmConstants.gridSafeZone_m)
			&& (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) > ArmConstants.gridSafeZone_m)) {
			new SequentialCommandGroup(
				new SetArmPosition(arm, armTarget_m),
				new WaitUntilCommand(() -> (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) < ArmConstants.gridSafeZone_m)),
				new SetElevatorPosition(elevator, elevatorTarget_m))
					.schedule();
		} else if ((armTarget_m >= ArmConstants.gridSafeZone_m)
			&& (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) < ArmConstants.gridSafeZone_m)) {
			new SequentialCommandGroup(
				new SetElevatorPosition(elevator, elevatorTarget_m),
				new WaitUntilCommand(() -> (elevator.inputs.position_m >= ElevatorConstants.gridSafeZone_m)),
				new SetArmPosition(arm, armTarget_m))
					.schedule();
		} else {
			new SetElevatorPosition(elevator, elevatorTarget_m).schedule();
			new SetArmPosition(arm, armTarget_m).schedule();
		}
	}
}
*/
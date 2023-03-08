package frc.robot.commands.scoring;

import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.SetElevatorPosition;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetScoringPosition extends ProxyCommand {
	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos) {
		this(elevator, arm, pos, 0);
	}

	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos, double tolerance_m) {
		this(elevator, arm, pos.elevatorHeight_m, pos.armExtension_m, tolerance_m);
	}

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m, double tolerance) {
		super(() -> new SetElevatorArmPositionProxied(elevator, arm, elevatorPos_m, armPos_m, 0));
	}

	private static class SetElevatorArmPositionProxied extends SequentialCommandGroup {
		SetElevatorArmPositionProxied(Elevator elevator, Arm arm, double elevatorTarget_m, double armTarget_m,
			double tolerance) {
			elevator.inManualMode = false;
			arm.inManualMode = false;
			if (elevatorTarget_m > elevator.inputs.position_m) {
				// going up -> elevator first
				addCommands(
					new SetElevatorPosition(elevator, elevatorTarget_m),
					new WaitUntilCommand(() -> arm.inManualMode || elevator.inputs.position_m > 0.6 * elevatorTarget_m), // Units.inchesToMeters(30)
					new SetArmPosition(arm, armTarget_m));
			} else {
				// going down -> arm first
				addCommands(
					new SetArmPosition(arm, armTarget_m),
					new WaitUntilCommand(() -> {
						if (elevator.inManualMode)
							return true;

						// Units.inchesToMeters(36)

						// arm going out
						if (armTarget_m > arm.getArmDist_m())
							return arm.getArmDist_m() > 0.6 * armTarget_m;
						// arm going in
						else
							return arm.getArmDist_m() < armTarget_m + 0.08; // when going to 0

						// return Math.abs(arm.getArmDist_m() - armTarget_m) <= 0.1;
					}),
					new SetElevatorPosition(elevator, elevatorTarget_m));
			}

			if (tolerance > 0) {
				addCommands(new WaitUntilCommand(() -> Math.abs(elevator.inputs.position_m - elevatorTarget_m) < tolerance
					&& Math.abs(arm.getArmDist_m() - armTarget_m) < tolerance));
			}
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
package frc.robot.commands.scoring;

import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetScoringPosition extends CommandBase {
	Elevator elevator;
	Arm arm;
	double elevatorTarget_m;
	double armTarget_m;
	double elevatorTolerance_m;
	double armTolerance_m;

	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos) {
		this(elevator, arm, pos, 0, 0);
	}

	/** set tolerance to 0 to ignore it */
	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos,
		double elevatorTolerance_m, double armTolerance_m) {
		this(elevator, arm, pos.elevatorHeight_m, pos.armExtension_m, elevatorTolerance_m, armTolerance_m);
	}

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m,
		double elevatorTolerance_m, double armTolerance_m) {
		this.elevator = elevator;
		this.arm = arm;
		this.elevatorTarget_m = elevatorPos_m;
		this.armTarget_m = armPos_m;
		this.elevatorTolerance_m = elevatorTolerance_m;
		this.armTolerance_m = armTolerance_m;
		addRequirements(elevator, arm);
	}

	private enum Mech {
		Elevator, Arm, None;
	}

	Mech nextSet;

	@Override
	public void initialize() {
		elevator.inManualMode = false;
		arm.inManualMode = false;

		if (elevatorTarget_m > elevator.inputs.position_m) {
			// going up
			elevator.setPosition(elevatorTarget_m);
			nextSet = Mech.Arm;
		} else {
			// going down
			arm.setPosition(armTarget_m);
			nextSet = Mech.Elevator;
		}
	}

	@Override
	public void execute() {
		switch (nextSet) {
			case Elevator -> {
				if (elevator.inputs.position_m < ElevatorConstants.dangerZone_m)
					return; // until we get above

				if (elevator.inputs.position_m > 0.9 * elevatorTarget_m) {
					arm.setPosition(armTarget_m);
					nextSet = Mech.None;
				}
			}

			case Arm -> {
				if (armTarget_m > arm.getArmDist_m()
					? arm.getArmDist_m() > 0.6 * armTarget_m // arm going out
					: arm.getArmDist_m() < armTarget_m + 0.15 // arm going in, offset for when going to 0
				) {
					elevator.setPosition(elevatorTarget_m);
					nextSet = Mech.None;
				}
			}

			case None -> {
				// both sets already called
			}
		}
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation())
			return true; // temp: until we actually sim it, for now we want to test auto

		// operator aborted
		if (elevator.inManualMode || arm.inManualMode)
			return true;

		// keep running until we're at both positions (auto)
		if ((elevatorTolerance_m > 0 && Math.abs(elevator.inputs.position_m - elevatorTarget_m) < elevatorTolerance_m)
			|| (armTolerance_m > 0 && Math.abs(arm.getArmDist_m() - armTarget_m) < armTolerance_m))
			return false;

		// normally just end
		return true;
	}
}

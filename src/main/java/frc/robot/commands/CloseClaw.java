package frc.robot.commands;

import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseClaw extends CommandBase {
	private Claw claw;
	private int counter;
	private int counterTarget;

	/** Creates a new Close. 
	 * 
	 * @param time_ms how long to close
	*/
	public CloseClaw(Claw claw, int time_ms) {
		this.claw = claw;
		this.counterTarget = time_ms / 20; // time will be the amounts of ticks/miliseconds per tick to get time
		this.counter = 0; // each incrementaion is one tick
		addRequirements(claw);
	}

	@Override
	public void initialize() {
		if (!claw.isClosed) {
			claw.moveInward();
		} else {
			counter = counterTarget;
		}
	}

	@Override
	public void execute() {
		counter++;
	}

	@Override
	public void end(boolean interrupted) {
		claw.isClosed = true;
		claw.stop();
	}

	@Override
	public boolean isFinished() {
		return counter >= counterTarget;
	}
}

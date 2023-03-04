package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;
	private double speed;

	private double counter;

	public CloseGrabber(Grabber grabber, double speed) {
		this.grabber = grabber;
		this.speed = speed;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		counter = 0;
	}

	@Override
	public void execute() {
		counter++;

		if (counter % 4 == 0)
			grabber.stop();
		else
			grabber.move(speed);
	}

	@Override
	public boolean isFinished() {
		return counter >= 12;
	}

	@Override
	public void end(boolean interrupted) {
		// if (interrupted)
		grabber.stop();
		// else
		// grabber.hold();
	}
}

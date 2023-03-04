package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;
	private Timer timer = new Timer();
	private double runTime_s;
	private double speed;

	private double counter;

	public CloseGrabber(Grabber grabber, double speed, double runTime_s) {
		this.grabber = grabber;
		this.speed = speed;
		this.runTime_s = runTime_s;
		addRequirements(grabber);
	}

	public CloseGrabber(Grabber grabber, double speed) {
		this(grabber, speed, 0);
	}

	@Override
	public void initialize() {
		counter = 0;
		grabber.move(speed);
		timer.reset();
		timer.start();
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
	public void end(boolean interrupted) {
		// if (interrupted)
		grabber.stop();
		timer.stop();
		// else
		// grabber.hold();
	}

	@Override
	public boolean isFinished() {
		return counter >= 12 || (runTime_s > 0 && timer.hasElapsed(runTime_s));
	}
}

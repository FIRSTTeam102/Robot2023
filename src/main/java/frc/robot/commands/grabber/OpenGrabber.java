package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenGrabber extends CommandBase {
	private Grabber grabber;
	private Timer timer = new Timer();
	private double runTime_s;
	private double speed;

	public OpenGrabber(Grabber grabber) {
		this(grabber, 0.2, 0.5);
	}

	/**
	 * @param time_s how long to close
	*/
	public OpenGrabber(Grabber grabber, double speed, double runTime_s) {
		this.grabber = grabber;
		this.speed = speed;
		this.runTime_s = runTime_s;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.move(-speed);
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		grabber.stop();
		timer.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(runTime_s);
	}
}

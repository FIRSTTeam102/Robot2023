package frc.robot.commands.grabber;

import static frc.robot.constants.GrabberConstants.*;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReleaseGrabber extends CommandBase {
	private Grabber grabber;
	private Timer timer = new Timer();
	private double runTime_s;
	private double speed;

	public ReleaseGrabber(Grabber grabber) {
		this(grabber, releaseSpeed, releaseTime_s);
	}

	/**
	 * @param time_s how long to close
	 */
	public ReleaseGrabber(Grabber grabber, double speed, double runTime_s) {
		this.grabber = grabber;
		this.speed = speed;
		this.runTime_s = runTime_s;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.move(-speed);
		timer.restart();
	}

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

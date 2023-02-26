package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ScoringMechanism2d {
	public static final Color8Bit orange102 = new Color8Bit(244, 113, 0);
	public static final Color8Bit light102 = new Color8Bit(248, 245, 244);
	public static final Color8Bit red = new Color8Bit(Color.kRed); // reverse
	public static final Color8Bit green = new Color8Bit(Color.kGreen); // forward

	public static final Mechanism2d mech = new Mechanism2d(4, 4, new Color8Bit(17, 17, 17));
	public static final MechanismRoot2d root = mech.getRoot("base", 1, 0);
	public static final MechanismLigament2d elevator = root
		.append(new MechanismLigament2d("elevator", 0, 90, 6, orange102));
	public static final MechanismLigament2d arm = elevator
		.append(new MechanismLigament2d("arm", 0, -90, 6, orange102));
	public static final MechanismLigament2d grabber = arm
		.append(new MechanismLigament2d("grabber", .25, 0, 4, orange102));

	public static void setGrabber(double grabSpeed) {
		grabber.setColor((grabSpeed < 0) ? red : (grabSpeed > 0) ? green : light102);
	}
}

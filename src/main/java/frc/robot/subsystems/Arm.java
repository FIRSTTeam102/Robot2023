package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import java.util.Map;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch frontLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch backLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	private GenericEntry shuffleboardLength;

	// is within swerve module bounds so elevator doesn't go down too far
	private static boolean inDangerZone = false;

	public static boolean inDangerZone() {
		return inDangerZone;
	}

	public Arm() {
		var armGroup = Shuffleboard.getTab("Drive").getLayout("Arm", BuiltInLayouts.kList);
		shuffleboardLength = armGroup
			.add("length", 0)
			.withWidget(BuiltInWidgets.kNumberBar)
			.withProperties(Map.of("min", 0, "max", 40))
			.getEntry();

		frontLimitSwitch.enableLimitSwitch(true);
		backLimitSwitch.enableLimitSwitch(true);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);
		pidController.setIZone(kIZone);
		pidController.setFF(kF);

		pidController.setOutputRange(minOutput, maxOutput);

		encoder.setPositionConversionFactor(conversionFactor_m_per_rotation);
	}

	public void setPosition(double armLength_m) {
		pidController.setReference(
			MathUtil.clamp(armDistToNutDist(armLength_m), Elevator.inDangerZone() ? armDistToNutDist(moduleDangerZone_m) : 0,
				maxNutDist_m - minNutDist_m),
			CANSparkMax.ControlType.kSmartMotion);
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	@Override
	public void periodic() {
		if (backLimitSwitch.isPressed())
			encoder.setPosition(maxNutDist_m - minNutDist_m);

		if (frontLimitSwitch.isPressed())
			encoder.setPosition(0);

		shuffleboardLength.setDouble(nutDistToArmDist(encoder.getPosition()));

		inDangerZone = (encoder.getPosition() < armDistToNutDist(moduleDangerZone_m));
	}

	public static double armDistToNutDist(double armDistance_m) {
		return Math.sqrt(Math.pow(armSectionLength_m, 2) - Math.pow(armDistance_m / sectionCount, 2))
			- minNutDist_m;
	}

	public static double nutDistToArmDist(double nutDistance_m) {
		return sectionCount * Math.sqrt(Math.pow(armSectionLength_m, 2) - Math.pow(nutDistance_m + minNutDist_m, 2));
	}
}

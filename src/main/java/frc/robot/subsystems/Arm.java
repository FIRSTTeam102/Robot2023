package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
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
	private CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch frontLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch backLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	private GenericEntry shuffleboardArmEntry;

	public Arm() {
		var armGroup = Shuffleboard.getTab("Drive").getLayout("Arm");
		shuffleboardArmEntry = armGroup
			.add("arm length", 0)
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

		pidController.setOutputRange(kMinOutput, kMaxOutput);

		encoder.setPositionConversionFactor(conversionFactor_in_per_rotation);
	}

	public void setPosition(double armLength_in) {
		double nutDist = armDistToNutDist(armLength_in);
		nutDist = MathUtil.clamp(nutDist, 0, maxNutDist_in - minNutDist_in);

		pidController.setReference(nutDist, CANSparkMax.ControlType.kSmartMotion);
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
			encoder.setPosition(maxNutDist_in - minNutDist_in);

		if (frontLimitSwitch.isPressed())
			encoder.setPosition(0);

		shuffleboardArmEntry.setDouble(nutDistToArmDist(encoder.getPosition()));
	}

	public static double armDistToNutDist(double armDistance) {
		return Math.sqrt(Math.pow(armSectionLength_in, 2) - Math.pow(armDistance / sectionCount, 2))
			- minNutDist_in;
	}

	public static double nutDistToArmDist(double nutDistance) {
		return sectionCount *
			Math.sqrt(Math.pow(armSectionLength_in, 2) - Math.pow(nutDistance + minNutDist_in, 2));
	}
}

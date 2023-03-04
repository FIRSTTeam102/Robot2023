package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import frc.robot.util.BuildManager;
import frc.robot.util.ScoringMechanism2d;
import frc.robot.util.SendableSparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	// private SparkMaxLimitSwitch backLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	private double targetPosition_m = 0;

	@Getter
	// is within swerve module bounds so elevator doesn't go down too far
	private static boolean inDangerZone = false;

	public boolean inManualMode = true;

	public Arm() {
		limitSwitch.enableLimitSwitch(true);
		motor.setSoftLimit(SoftLimitDirection.kReverse, (float) maxNutDist_m);
		motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
		// backLimitSwitch.enableLimitSwitch(true);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);
		pidController.setIZone(kIZone);
		pidController.setFF(kF);

		pidController.setOutputRange(minOutput, maxOutput);

		encoder.setPositionConversionFactor(conversionFactor_m_per_rotation);
		encoder.setVelocityConversionFactor(conversionFactor_mps_per_rpm);

		motor.setSecondaryCurrentLimit(40);

		BuildManager.burnSpark(motor);

		SmartDashboard.putData(new SendableSparkMaxPIDController(pidController, ControlType.kPosition, "arm pid"));
	}

	public void setPosition(double armLength_m) {
		targetPosition_m = armLength_m;

		pidController.setReference(
			MathUtil.clamp(armDistToNutDist(armLength_m),
				Elevator.isInDangerZone() ? armDistToNutDist(dangerZone_m) : maxNutDist_m,
				minNutDist_m),
			CANSparkMax.ControlType.kPosition);
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		var armDist_m = nutDistToArmDist(inputs.nutPosition_m);
		Logger.getInstance().recordOutput("Arm/armDist_m", armDist_m);
		ScoringMechanism2d.arm.setLength(armDist_m);

		Logger.getInstance().recordOutput("Arm/targetArmDist_m", targetPosition_m);

		// if (inputs.backLimitSwitch)
		// encoder.setPosition(maxNutDist_m - minNutDist_m);

		// if (inputs.limitSwitch)
		// encoder.setPosition(minNutDist_m);

		// todo: bake danger zone
		inDangerZone = (inputs.nutPosition_m < armDistToNutDist(dangerZone_m));
	}

	public static double armDistToNutDist(double armDistance_m) {
		return Math.sqrt(Math.pow(armSectionLength_m, 2) - Math.pow(armDistance_m / sectionCount, 2));
	}

	public static double nutDistToArmDist(double nutDistance_m) {
		return sectionCount * Math.sqrt(Math.pow(armSectionLength_m, 2) - Math.pow(nutDistance_m, 2));
	}

	/**
	 * inputs
	 */
	@AutoLog
	public static class ArmIOInputs {
		public double percentOutput = 0;
		public double current_A = 0;
		public double temperature_C = 0;
		public double nutPosition_m = 0;
		public double velocity_mps = 0;
		public boolean limitSwitch = false;
		public boolean softLimit = false;
		// public boolean backLimitSwitch = false;
	}

	public ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	public void updateInputs(ArmIOInputs inputs) {
		inputs.percentOutput = motor.getAppliedOutput();
		inputs.current_A = motor.getOutputCurrent();
		inputs.temperature_C = motor.getMotorTemperature();
		inputs.nutPosition_m = encoder.getPosition();
		inputs.velocity_mps = encoder.getVelocity();
		inputs.limitSwitch = limitSwitch.isPressed();
		inputs.softLimit = motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
		// inputs.backLimitSwitch = backLimitSwitch.isPressed();
	}
}

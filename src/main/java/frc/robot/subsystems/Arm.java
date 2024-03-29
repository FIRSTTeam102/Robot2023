package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.util.BuildManager;
import frc.robot.util.ScoringMechanism2d;
import frc.robot.util.SendableSparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.Setter;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	// private SparkMaxLimitSwitch backLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	@Getter
	private double targetNutPosition_m = 0;

	@Getter
	private double armDist_m = 0;

	@Getter
	// is within swerve module bounds so elevator doesn't go down too far
	private static boolean inDangerZone = false;

	@Setter
	private DoubleSupplier manualModeInput = null;
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

		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		BuildManager.burnSpark(motor);

		SmartDashboard.putData(new SendableSparkMaxPIDController(pidController, ControlType.kPosition, "arm pid"));
	}

	public void setPosition(double armLength_m) {
		targetNutPosition_m = MathUtil.clamp(armDistToNutDist(armLength_m),
			maxNutDist_m,
			Elevator.isInDangerZone() ? armDistToNutDist(dangerZone_m) : minNutDist_m);

		pidController.setReference(targetNutPosition_m, CANSparkMax.ControlType.kPosition);
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	public boolean withinTargetPosition() {
		return Math.abs(armDist_m - targetNutPosition_m) < AutoConstants.armTolerance_m;
	}

	boolean hasDoneLimitReset = false;

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		armDist_m = nutDistToArmDist(inputs.nutPosition_m);
		Logger.getInstance().recordOutput("Arm/armDist_m", armDist_m);
		ScoringMechanism2d.arm.setLength(armDist_m);

		Logger.getInstance().recordOutput("Arm/targetNutPosition_m", targetNutPosition_m);

		// if (inputs.backLimitSwitch)
		// encoder.setPosition(maxNutDist_m - minNutDist_m);
		if (inputs.limitSwitch && !hasDoneLimitReset) {
			hasDoneLimitReset = true;
			encoder.setPosition(minNutDist_m);
		} else if (hasDoneLimitReset && armDist_m > 0.01)
			hasDoneLimitReset = false;

		inDangerZone = armDist_m < dangerZone_m;

		if (manualModeInput != null
			&& Math.abs(manualModeInput.getAsDouble()) >= OperatorConstants.operatorJoystickDeadband)
			inManualMode = true;
	}

	public static double armDistToNutDist(double armDistance_m) {
		// make negative distances work
		var x = Math.pow(armSectionLength_m, 2) - Math.pow(armDistance_m / sectionCount, 2);
		return Math.copySign(Math.sqrt(Math.abs(x)), x);
	}

	public static double nutDistToArmDist(double nutDistance_m) {
		var x = Math.pow(armSectionLength_m, 2) - Math.pow(nutDistance_m, 2);
		return sectionCount * Math.copySign(Math.sqrt(Math.abs(x)), x);
	}

	public final Command stop = Commands.startEnd(() -> {
		// limitSwitch.enableLimitSwitch(false);
		// motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
		pidController.setOutputRange(0, 0);
		pidController.setReference(0, CANSparkMax.ControlType.kVoltage, 0, 0);
		inManualMode = true;
	}, () -> {
		// limitSwitch.enableLimitSwitch(true);
		// motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
		pidController.setOutputRange(minOutput, maxOutput);
	});

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

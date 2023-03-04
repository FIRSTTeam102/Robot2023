package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

/**
 * based on {@link edu.wpi.first.math.controller.PIDController}
 */
public class SendableSparkMaxPIDController implements Sendable, AutoCloseable {
	private SparkMaxPIDController controller;
	private CANSparkMax.ControlType controlType;
	private double setpoint;

	public SendableSparkMaxPIDController(SparkMaxPIDController controller, CANSparkMax.ControlType controlType,
		String name) {
		this.controller = controller;
		this.controlType = controlType;
		SendableRegistry.addLW(this, "PIDController", name);
	}

	@Override
	public void close() {
		SendableRegistry.remove(this);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("PIDController");
		builder.addDoubleProperty("p", controller::getP, controller::setP);
		builder.addDoubleProperty("i", controller::getI, controller::setI);
		builder.addDoubleProperty("d", controller::getD, controller::setD);
		builder.addDoubleProperty("setpoint",
			// todo: read actual values
			() -> setpoint,
			(double setpoint) -> {
				this.setpoint = setpoint;
				controller.setReference(setpoint, controlType);
			});
	}
}

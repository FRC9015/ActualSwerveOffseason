package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class IMU {
	public Pigeon2 imu;

	public IMU() {
		imu = new Pigeon2(23);
	}

	public double yaw() {
		return Rotation2d.fromDegrees(imu.getYaw().getValue());
	}

	public void zeroYaw() {
		imu.setYaw(0);
	}
}

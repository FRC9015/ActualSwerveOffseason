// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DefaultDrive() {
		addRequirements(swerve);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double xVelocity = driveController.getLeftX(); // X - Direction Velocity
		double yVelocity = -driveController.getLeftY(); // Y - Direction Velocity
		double w = -driveController.getRightX(); // Rotational Velocity 
		double mag = Math.hypot(xVelocity, yVelocity);
		double ma2 = MathUtil.applyDeadband(mag, 0.1);
		double w2 = MathUtil.applyDeadband(w, 0.1);
		double theta = Math.atan2(yVelocity, xVelocity);
		xVelocity = cos(theta) * ma2 * maxSpeed;
		yVelocity = sin(theta) * ma2 * maxSpeed;
		w= w2 * maxSpeed;

		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, w, imu.yaw());
		swerve.drive(speeds);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveModuleConfiguration;

public class SwerveModule {
	private CANSparkMax turn, drive;
	private CANcoder encoder;
	private Rotation2d encoder_offset;

	private RelativeEncoder driveEncoder;
	private SwerveModuleState targState;

	private PIDController drivePID, turnPPID;
	private String name;
	private double kV = 3;

	
	public SwerveModule(SwerveModuleConfiguration configs, String nameString) {
		turn = new CANSparkMax(configs.TURN_MOTOR, MotorType.kBrushless);
		drive = new CANSparkMax(configs.DRIVE_MOTOR, MotorType.kBrushless);
		name = nameString;
		encoder = new CANcoder(configs.ENCODER);
		drivePID = new PIDController(3, 0, 0);
		turnPPID = new PIDController(2, 0, 0);
        
		turnPPID.enableContinuousInput(-PI, PI);
		encoder_offset = configs.offset;
		drive.restoreFactoryDefaults();
		turn.restoreFactoryDefaults();

		drive.setCANTimeout(250);
		turn.setCANTimeout(250);

		driveEncoder = drive.getEncoder();
		driveEncoder.setPosition(0.0);

		drive.setSmartCurrentLimit(40);
		turn.setSmartCurrentLimit(30);
		drive.enableVoltageCompensation(12.0);
		turn.enableVoltageCompensation(12.0);

		drive.setCANTimeout(0);
		turn.setCANTimeout(0);

		drive.burnFlash();
		turn.burnFlash();
	}

	public Rotation2d getDirection() {
		return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue())
				.minus(encoder_offset);
	}

	public void setState(SwerveModuleState state) {
		this.targState = SwerveModuleState.optimize(state, getDirection());
	}

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: "
				+ getDirection().plus(encoder_offset).getRotations());
	}
	//drive command for telop
	public void telop() {
		if (targState == null) return;
		double curr_velocity =
				Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / gearRatio * wheelRatio;
		double target_vel = Math.abs(Math.cos((getDirection().getRadians() - targState.angle.getRadians())))
				* targState.speedMetersPerSecond;

		drive.setVoltage(drivePID.calculate(curr_velocity, target_vel) + target_vel * kV);
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targState.angle.getRadians()));
	}
	//command to make the bot spin 360 degrees
	public void spin360(){
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), Math.PI *2));
	}
	
	//for limelight movment(currently does not have drive code)
	public void followTag(double x, double y, double area, double TagDistance){
		//stops the command if their is no tag
		if(area < 0.1){
			return;
		}
		
		// when it wants the bot to stop(1 foot)
		double StopDistance = 12.0; //in inches

		// Calculate the error in distance(PIDs)
		double distanceError = TagDistance - StopDistance;

		// Define PID constants for distance control
		//needs to be tested with bot to adjust numbers
		double kP_distance = 0.1;//adjust later
		double kI_distance = 0.01;//adjust later
		double kD_distance = 0.1;//adjust later

		// Use PID control to adjust drive speed based on distance error
		double driveSpeed = kP_distance * distanceError;
							
		// Set a maximum drive speed to avoid bot going to fast
		double maxDriveSpeed = 0.5; // Adjust as needed

		// Limit drive speed to the maximum
		// driveSpeed = Math.min(Math.abs(driveSpeed), maxDriveSpeed) * Math.signum(driveSpeed);

		// Use Limelight Y value to adjust turn angle
		double turnAngleAdjustment = 0.1 * y; // Adjust as needed

		// Apply adjustments to drive and turn commands
		drive.setVoltage(driveSpeed);
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targState.angle.getRadians()) + turnAngleAdjustment);
	}
}

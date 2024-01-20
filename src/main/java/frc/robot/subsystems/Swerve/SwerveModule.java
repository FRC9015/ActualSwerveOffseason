package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(getDriveDistance(), getDirection());

	}
	public double getDriveDistance(){
		return driveEncoder.getPosition() / gearRatio*2*Math.PI*Units.inchesToMeters(2);
	}

	public void setState(SwerveModuleState state) {
		this.targState = SwerveModuleState.optimize(state, getDirection());
	}

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: "
				+ getDirection().plus(encoder_offset).getRotations());
	}
	//drive command for teleop
	public void teleop() {
		if (targState == null) return;
		double curr_velocity =
				Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / gearRatio * wheelRatio;
		double target_vel = Math.abs(Math.cos((getDirection().getRadians() - targState.angle.getRadians())))
				* targState.speedMetersPerSecond;

		drive.setVoltage(drivePID.calculate(curr_velocity, target_vel) + target_vel * kV);
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targState.angle.getRadians()));
	}
	//DONT PUT IN MASTER
	//command to make the bot spin 360 degrees
	
	public void spin360(){
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), Math.PI *2));
	}
	
	public void drivePID(double curr,double tar){
		drive.setVoltage(drivePID.calculate(curr, tar));

	}
	public void turnPID(double turns){
		turn.setVoltage(turnPPID.calculate(getDirection().getRadians(), targState.angle.getRadians())+ turns);
	}
	
	

}

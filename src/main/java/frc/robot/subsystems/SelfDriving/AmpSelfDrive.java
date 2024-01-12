package frc.robot.subsystems.SelfDriving;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotSelf;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AmpSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used
    private SwerveSubsystem drive;
    private LimelightInterface limelight;
    private CommandXboxController controller;
    private RobotSelf robotSelf;
    
    //makes the subsystems exist and usable
    public AmpSelfDrive(CommandXboxController drivecontroller, RobotSelf robotSelf, LimelightInterface limelight,SwerveSubsystem drive) {
        this.controller = drivecontroller;
        this.robotSelf = robotSelf;
        this.limelight = limelight;
        this.drive = drive;
        
    }
    
	@Override
    public void periodic(){
        //checks to see if the X button has been pressed
        if(controller.getHID().getBButtonPressed()){//need to find how the x button is pressed in the command controller.
            //toggles selfdrive boolean
            robotSelf.toggleAmpSelf();
            
            //puts selfdrive onto smartdashboard
            SmartDashboard.putBoolean("AmpSelfDrive", robotSelf.getAmpSelf());
        }
        //makes seperate X, Y, Area values outside of the limelightinterface
        double x = limelight.getX();
        double y = limelight.getY();
        double area = limelight.getArea();
        //brings in the distance away the tag is from the bot
        double distance = limelight.getDistance();
        
        //makes sure that selfdrive is true
        if(robotSelf.getAmpSelf()){
                
                //checks for a tag using the limelightinterface commands
                if(limelight.TagCheck()){
                    //updates values
                    x = limelight.getX();
                    y = limelight.getY();
                    area = limelight.getArea();
                    distance = limelight.getDistance();
                    // uses drive system to drive based on tag
                    drive.runFollowTag(x, y, area, distance);
                    
                }
            }
        }
    }


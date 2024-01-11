package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static java.lang.Math.*;
    
public class LimelightInterface extends SubsystemBase{
    
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static boolean tag = false;
    //takes the X,Y, and area values from the limelight networktable
    NetworkTableEntry tx = limelight.getEntry("tx");//Tag X value
    NetworkTableEntry ty = limelight.getEntry("ty");//Tag Y value
    NetworkTableEntry ta = limelight.getEntry("ta");//Tag Area

    //makes variables for the X Y and Area values of the limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //updates dashboard
    private void updateDashboard(double x, double y, double area) {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area); 
    }

    //Used to calculate the distance from a tag

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;//needs to be changed based on limelight placement

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;//needs to be changed based on limelight placement
 
    // distance from the target to the floor
    double goalHeightInches = 18;//this is set to the height of an amp
         
    //the degrees of the camera to the tag
    double limelightToGoalDegrees = limelightMountAngleDegrees + y;
    //makes the degrees into radians
    double goalDegreesToRadians = limelightToGoalDegrees * (PI / 180.0);
         
    //calculate distance
    double TagDistance = (goalHeightInches - limelightLensHeightInches) / Math.tan(goalDegreesToRadians);



    //updates limelight X, Y, and Area and puts them onto smartdashboard.
    @Override
    public void periodic() {
        //updates the X,Y,Area values
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        TagDistance = (goalHeightInches - limelightLensHeightInches) / Math.tan(goalDegreesToRadians);
        //updates smartdashboard with values
        updateDashboard(x, y, area);
        SmartDashboard.putNumber("TagDistance",TagDistance);
    }



    public double getX(){
        return tx.getDouble(0.0);
    }
    
    public double getY(){
        return ty.getDouble(0.0);
    }
    
    public double getArea(){
        return ta.getDouble(0.0);
    }
    
    public double getDistance(){
        return TagDistance;
    }
    
    public boolean TagCheck(){
        if(getArea() > 0.1){
            tag = true;
        }else{
            tag = false;
        }
        return tag;
    }

}

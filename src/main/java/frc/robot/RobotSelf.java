package frc.robot;

//used so that selfdrive can be used in any file
public class RobotSelf {
    private boolean AmpSelf = false;
    private boolean SpeakerSelf = false;

    public boolean getAmpSelf(){
        return AmpSelf;
    }
    public void toggleAmpSelf(){
        AmpSelf = !AmpSelf;
    }

    public boolean getSpeakerSelf(){
        return SpeakerSelf;
    }
    public void toggleSpeakerSelf(){
        SpeakerSelf = !SpeakerSelf;
    }

     

}


package frc.robot;
//DONT PUT IN MASTER
//used so that selfdrive can be used in any file
public class RobotSelf {
    
    public static class RobotSelves{
        private static boolean AmpSelf = false;
        private static boolean SpeakerSelf = false;

        public static boolean getAmpSelf(){
            return AmpSelf;
        }
        public static void toggleAmpSelf(){
            AmpSelf = !AmpSelf;
            System.out.println(getAmpSelf());
        }

        public static boolean getSpeakerSelf(){
            return SpeakerSelf;
        }
        public static void toggleSpeakerSelf(){
            SpeakerSelf = !SpeakerSelf;
        }

     
    }
}


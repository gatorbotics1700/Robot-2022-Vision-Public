package frc.robot.subsystems;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class ClimberSubsystem {
    public static enum ClimberStates{
        ACTUATING, 
        RETRACTING, 
        OFF; 
    }

    // public static DoubleSolenoid solenoidOne = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 9, 8); 
    // public static DoubleSolenoid solenoidTwo = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 15, 14);

    // public static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); 

    public static ClimberStates climberState = ClimberStates.OFF;

    public void init(){
        // solenoidOne.set(kOff);
        // solenoidTwo.set(kOff);
        System.out.println("In climber init, soleinoids set to off");
    }

    public void setStateClimber(ClimberStates newState){
        climberState = newState;
    }

    public void periodic(){
        if(climberState == ClimberStates.ACTUATING){
            //solenoidOne.set(kForward);
            //solenoidTwo.set(kForward);
            System.out.println("Actuating");
        } else if (climberState == ClimberStates.RETRACTING){
            //solenoidOne.set(kReverse);
            //solenoidTwo.set(kReverse);
            System.out.println("Retracting");
        } 
    }

    public boolean getPSI(){
        //System.out.println(compressor.getCurrent());
        //return compressor.getPressureSwitchValue();
        return false;
    }
}
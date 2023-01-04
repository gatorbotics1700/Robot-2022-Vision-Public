package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.subsystems.TransitionSubsystem.TransitionState;

public class SensorSubsystem {

    public static Ultrasonic ultrasonic = new Ultrasonic(0, 1); 
    //output is 1, input is 0 - parameters are wiring DIO for out and in
    public static TransitionSubsystem transitionSubsystem = new TransitionSubsystem();
    public static double setpoint;
    public static Boolean within8Inches = false; //new 4/2


    public void setup(){
        setpoint = 0.0;
        Ultrasonic.setAutomaticMode(true); 
        //Starts the ultrasonic sensor running in automatic mode
    }

    public static enum SensorStates{
        ON,
        OFF;
    }

    public static SensorStates state = SensorStates.OFF;

    public void periodic(){
        if(state == SensorStates.ON){
            //System.out.println("Sensor is sensing!");
            if (checkRange() == true) {
                within8Inches = true;
                System.out.println("getrangeinches" + ultrasonic.getRangeInches());
                transitionSubsystem.resetTareEncoder();
                setpoint = transitionSubsystem.getTransitionPosition() - TransitionSubsystem.tareEncoder;
                System.out.println("setpoint" + setpoint);
                transitionSubsystem.setState(TransitionState.STOP_PD);
                setState(SensorStates.OFF);
            }else {
                //System.out.println("getrangeinches" + ultrasonic.getRangeInches());
                transitionSubsystem.setState(TransitionState.ON);
            } 
        } 
    }

    public void setState(SensorStates newState){
        state = newState;
    }

    public boolean checkRange(){ 
        if(ultrasonic.getRangeInches() < 8.0) { 
            return true;
        } 
        return false;
    }
}

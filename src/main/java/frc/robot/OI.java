package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

public class OI{
    //joysticks (Ids are the numbers) 
    static public Joystick leftJoystick = new Joystick(0); 
    static public Joystick rightJoystick = new Joystick(1);
    static public Joystick buttonJoystick = new Joystick(2); 

    //joystick buttons:
    public static Button warmUpShooter = new JoystickButton(rightJoystick, 1);
    public static Button shooter = new JoystickButton(buttonJoystick, 1);
    public static Button shooterOff = new JoystickButton(buttonJoystick, 4);
    public static Button reverseShoot = new JoystickButton(buttonJoystick, 6);
  
    public static Button transition = new JoystickButton(buttonJoystick, 2); 
    public static Button transitionStop = new JoystickButton(buttonJoystick, 7);
    public static Button reverseIntakeTransition = new JoystickButton(buttonJoystick, 3);
  
    //public static Button allMech = new JoystickButton(buttonJoystick, 6);
    public static Button allMechOff = new JoystickButton(buttonJoystick, 5); 
  
    /*public static Button climberRetract = new JoystickButton(buttonJoystick, 10);
    public static Button climberActuate = new JoystickButton(buttonJoystick, 11);*/
    public static Button turretOn = new JoystickButton(buttonJoystick, 10);
    public static Button turretOff = new JoystickButton(buttonJoystick, 11);

    public static Button shootOnce = new JoystickButton(buttonJoystick, 8);
    public static Button shootTwice = new JoystickButton(buttonJoystick, 9);
}

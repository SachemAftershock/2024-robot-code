package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * 
 * <p>
 * Handle input from Flight Joysticks connected to the Driver Station.
 * </p>
 * 
 * <p>
 * This class handles standard input that comes from the Driver Station. Each
 * time a value is requested the most recent value is returned. There is a
 * single class instance for each joystick and the mapping of ports to hardware
 * buttons depends on the code in the Driver Station.
 * </p>
 * 
 * <p> <b> Note: Hover over .button members to visualize its location with JavaDoc </b> </p>
 */
public class CommandJoystick extends Joystick {
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html#trigger-bindings

    public CommandJoystick(int port) {
        super(port);
    }

    /** ;\
     * index finger trigger
     */
    public JoystickButton button1 = new JoystickButton(this, 1);

    /**
     * body of joystick, left side (thumb slot if you are right handed, index finger if you are left handed)
     */
    public JoystickButton button2 = new JoystickButton(this, 2);

    /**
     * top of joystick, bottom left button
     */
    public JoystickButton button3 = new JoystickButton(this, 3);

    /**
     * top of joystick, bottom right button
     */
    public JoystickButton button4 = new JoystickButton(this, 4);

    /**
     * top of joystick, left button
     */
    public JoystickButton button5 = new JoystickButton(this, 5);

    /**
     * top of joystick, right button
     */
    public JoystickButton button6 = new JoystickButton(this, 6);

    /**
     * left of joystick base, front left button
     */
    public JoystickButton button7 = new JoystickButton(this, 7);

    /**
     * left of joystick base, front right button
     */
    public JoystickButton button8 = new JoystickButton(this, 8);

    /**
     * left of joystick base, middle left button
     */
    public JoystickButton button9 = new JoystickButton(this, 9);

    /**
     * left of joystick base, middle right button
     */
    public JoystickButton button10 = new JoystickButton(this, 10);

    /**
     * left of joystick base, back left button
     */
    public JoystickButton button11 = new JoystickButton(this, 11);

    /**
     * left of joystick base, back right button
     */
    public JoystickButton button12 = new JoystickButton(this, 12);

    // slider is the CommandJoystick().getThrottle() method
}

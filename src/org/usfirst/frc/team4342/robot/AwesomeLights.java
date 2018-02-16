package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.logging.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Class to control the RGB Lights
 */
public class AwesomeLights {
    private static Solenoid power, red, green, blue;

    private static boolean started, running;
    private static LightController controller;

    /**
     * Starts the awesome lights
     */
    public static void start() {
        if(running)
            return;
        
        if(!started) {
        	controller = new LightController();
            controller.start();
            started = true;
        }
        
        running = true;
    }

    /**
     * Stops the awesome lights
     */
    public static void stop() {
        running = false;
        green.set(false);
        blue.set(false);
        power.set(false);
    }

    /**
     * Class to control the RGB Lights on a separate thread
     */
    private static class LightController extends Thread implements Runnable {
        private boolean[] on = { true, false, false };

        /**
         * Runs if robot is enabled
         */
        @Override
        public void run() {
            boolean crashed = false;
            
            try {
            	Logger.info("Creating the Awesome Lights...");
                power = new Solenoid(RobotMap.LED_POWER);
                red = new Solenoid(RobotMap.RED_LED);
                green = new Solenoid(RobotMap.GREEN_LED);
                blue = new Solenoid(RobotMap.BLUE_LED);
                
                Logger.info("Starting the Awesome Lights...");
                power.set(true);
                while(!crashed) {
                    if(running && RobotState.isEnabled()) {
                        cycleLights();
                    }

                    Thread.sleep(500);
                }
            } catch(Exception ex) {
                Logger.error("The Awesome Lights crashed :(", ex);

                red.set(false);
                green.set(false);
                blue.set(false);
                power.set(false);

                crashed = true;
            }
        }

        /**
         * Shifts and sets the lights by calling
         * {@link #shiftLights()} then {@link #setLights()}
         */
        private void cycleLights() {
            shiftLights();
            setLights();
        }

        /**
         * Sets the lights based on the state of the internal array
         */
        private void setLights() {
            red.set(on[0]);
            green.set(on[1]);
            blue.set(on[2]);
        }

        /**
         * Shifts the internal array
         */
        private void shiftLights() {
            boolean temp = on[on.length-1];
            for(int i = on.length-2; i >= 0; i++) {
                on[i+1] = on[i];
            }
            on[0] = temp;
        }
    }
}

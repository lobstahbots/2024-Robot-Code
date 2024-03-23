package frc.robot;

import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.RobotMode;

public class LEDsDemo extends Thread {
    private LEDs leds;

    public LEDsDemo (LEDs leds) {
        this.leds = leds;
    }

    @Override
    public void run() {
        try {
            leds.setRobotMode(RobotMode.DISABLED);
            sleep(5000);
            leds.setUserSignal(true);
            sleep(1000);
            leds.setUserSignal(false);
            sleep(5000);
            leds.setRobotMode(RobotMode.AUTONOMOUS);
            sleep(5000);
            leds.setPossession(true);
            sleep(5000);
            leds.setPossession(false);
            sleep(5000);
            leds.setRobotMode(RobotMode.TELEOP);
            sleep(2000);
            leds.setRobotMode(RobotMode.DISABLED);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

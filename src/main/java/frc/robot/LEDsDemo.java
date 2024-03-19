package frc.robot;

import frc.robot.subsystems.leds.LEDs;

public class LEDsDemo extends Thread {
    private LEDs leds;

    public LEDsDemo (LEDs leds) {
        this.leds = leds;
    }

    @Override
    public void run() {
        try {
            leds.setPossession(true);
            sleep(5000);
            leds.setPossession(false);
            sleep(5000);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

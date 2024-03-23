// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import stl.led.AlphaBuffer;
import stl.led.AnimationEasing;
import stl.led.LobstahLEDBuffer;

public class LEDs extends SubsystemBase {
    LEDIO io;

    public enum ConnectionState { DISCONNECTED, DS_ONLY, FMS }
    ConnectionState connectionState = ConnectionState.DISCONNECTED;
    DriverStation.Alliance alliance = DriverStation.Alliance.Red;
    public enum RobotMode { DISABLED, TELEOP, AUTONOMOUS, ESTOPPED }
    RobotMode robotMode = RobotMode.DISABLED;
    boolean intaking = false;
    boolean possession = false;
    public enum AutoAssistState { INACTIVE, ACTIVE, DONE }
    AutoAssistState autoAssist = AutoAssistState.INACTIVE;
    boolean shooterReady = false;
    boolean ampSignal = false;
    boolean userSignal = false;
    boolean driveOverheated = false;
    boolean otherSubsytemOverheated = false;
    boolean tipped = false;
    boolean coastMode = false;
    boolean lowBattery = false;

    public LEDs(LEDIO io) {
        this.io = io;
        
        loadingNotifier.startPeriodic(0.02);
    }

    private void setFMSState(ConnectionState value) { connectionState = value; }

    private void setAlliance(DriverStation.Alliance value) { alliance = value; }

    private void setRobotMode(RobotMode value) { 
        if (value == RobotMode.DISABLED && robotMode == RobotMode.AUTONOMOUS
                && connectionState == ConnectionState.FMS) {
            triggerTeleopCountdown();
        }
        robotMode = value;
    }

    public void setIntaking (boolean value) { intaking = value; }

    public void setPossession(boolean value) {
        if (value == true && possession == false) possessionSignalTimer.restart();
        possession = value;
    }

    public void setAutoAssist(AutoAssistState value) { autoAssist = value; }

    public void setShooterReady(boolean value) { shooterReady = value; }

    public void setAmpSignal(boolean value) { ampSignal = value; }

    public void setUserSignal(boolean value) { userSignal = value; }

    public void setDriveOverheated(boolean value) { driveOverheated = value; }

    public void setOtherSubsystemOverheated(boolean value) { otherSubsytemOverheated = value; }

    public void setTipped(boolean value) { tipped = value; }

    public void setCoastMode(boolean value) { coastMode = value; }

    public void setLowBattery(boolean value) { lowBattery = value; }

    public void triggerTeleopCountdown() { }

    public void triggerEndgameSignal() { }

    public void periodic() {
        loadingNotifier.stop();

        if (!DriverStation.isDSAttached()) {
            setFMSState(ConnectionState.DISCONNECTED);
        } else if (DriverStation.isFMSAttached()) {
            setFMSState(ConnectionState.FMS);
            if(DriverStation.getAlliance().isPresent()) {
                setAlliance(DriverStation.getAlliance().get());
            }
        } else {
            setFMSState(ConnectionState.DS_ONLY);
        }

        if (DriverStation.isEStopped()) {
            setRobotMode(RobotMode.ESTOPPED);
        } else if (DriverStation.isAutonomousEnabled()) {
            setRobotMode(RobotMode.AUTONOMOUS);
        } else if (DriverStation.isTeleopEnabled()) {
            setRobotMode(RobotMode.TELEOP);
        } else {
            setRobotMode(RobotMode.DISABLED);
        }

        io.setData(LobstahLEDBuffer.layer(LEDConstants.LED_LENGTH,
                userSignal(),
                posessionSignal(),
                posessionIndicator(),
                robotMode == RobotMode.AUTONOMOUS ? autonomous() : null,
                robotMode == RobotMode.DISABLED ? disabledStandby.get() : null
            ).toAdressableLEDBuffer());
    }

    private final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            System.out.println("Loading");
            io.setData(LobstahLEDBuffer.solid(3, Color.kWhite)
                    .opacity(AnimationEasing.sine(System.currentTimeMillis(), 1000, 0)).toAdressableLEDBuffer());
        }
    });
    
    Timer possessionSignalTimer = new Timer();
    LobstahLEDBuffer posessionSignal() {
        double t = possessionSignalTimer.get();
        if (t>= 2 || !possession) return null;

        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(77, 255, 79))
            .opacity(1-t/2);
    }

    LobstahLEDBuffer posessionIndicator() {
        if (!possession) return null;
        return LobstahLEDBuffer.solid(5, new Color(77, 255, 79))
            .wrappedShift(LEDConstants.LED_LENGTH, -5);
    }

    static class DisabledStandby {
        int prevHeight1; int prevHeight2; int nextHeight1; int nextHeight2;
        Timer timer = new Timer();

        DisabledStandby() {
            timer.start();
            generateHeights();
        }

        LobstahLEDBuffer get() {
            if (timer.advanceIfElapsed(0.2)) generateHeights();
            int height1 = (int) (prevHeight1 + (nextHeight1 - prevHeight1) * timer.get());
            int height2 = (int) (prevHeight2 + (nextHeight2 - prevHeight2) * timer.get());
            return LobstahLEDBuffer.solid(height1, new Color(255, 25, 25))
                    .layerAbove(LobstahLEDBuffer.solid(height2, new Color(255, 69, 118)));
        }

        void generateHeights() {
            prevHeight1 = nextHeight1;
            prevHeight2 = nextHeight2;
            nextHeight1 = (int) (Math.random() * 20);
            nextHeight2 = (int) (Math.random() * 20);
        }
    }
    DisabledStandby disabledStandby = new DisabledStandby();

    LobstahLEDBuffer autonomous() {
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 69, 118), 0.5)
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 30, 180))
                        .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 5, 7)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 25, 25)));
    }

    LobstahLEDBuffer userSignal() {
        if (userSignal == false) return null;
        if (Timer.getFPGATimestamp() % 0.2 < 0.1) {
            return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kWhite);
        } else {
            return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kBlack, 0.8);
        }
    }
}

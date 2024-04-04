// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import stl.led.AlphaBuffer;
import stl.led.AnimationEasing;
import stl.led.LobstahLEDBuffer;

public class LEDs extends SubsystemBase {
    private static LEDs instance = null;
    public static LEDs getInstance() { return instance; }

    LEDIO io;
    
    public LEDs(LEDIO io) {
        if (instance != null) throw new IllegalStateException("LEDs already initialized");
        instance = this;

        this.io = io;
        
        loadingNotifier.startPeriodic(0.02);
    }

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
    public Color debugColor = null;

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

    private void triggerTeleopCountdown() { }

    private void triggerEndgameSignal() { }

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
                 robotMode == RobotMode.DISABLED ? disabledStandby() : null,
                 robotMode == RobotMode.AUTONOMOUS ? autonomous() : null,
                 posessionIndicator(),
                 posessionSignal(),
                //  shooterReadyIndicator(),
                 debugColor == null? null : LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, debugColor),
                 userSignal()
            ).toAdressableLEDBuffer());
    }

    private final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            io.setData(LobstahLEDBuffer.solid(3, Color.kWhite)
                    .opacity(AnimationEasing.sine(System.currentTimeMillis(), 1000, 0)).crop(LEDConstants.LED_LENGTH)
                    .toAdressableLEDBuffer());
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
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 25, 25))
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(160, 170,255))
                        .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10 + 3)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 25, 25), 0.25));
    }

    LobstahLEDBuffer shooterReadyIndicator() {
        if (!shooterReady) return null;
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(77, 255, 79));
    }

    static class DisabledStandby {
        int prevHeight1; int prevHeight2; int nextHeight1; int nextHeight2;
        Timer timer = new Timer();

        DisabledStandby() {
            timer.start();
            generateHeights();
        }

        LobstahLEDBuffer get(Color color1, Color color2) {
            if (timer.hasElapsed(0.2)) {
                timer.restart();
                generateHeights();
            }
            double time = Math.min(timer.get() * 5, 1);
            int height1 = (int) (prevHeight1 + (nextHeight1 - prevHeight1) * time);
            int height2 = (int) (prevHeight2 + (nextHeight2 - prevHeight2) * time);
            return LobstahLEDBuffer.layer(LEDConstants.LED_LENGTH,
                    LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, color1, 0.5),
                    LobstahLEDBuffer.solid(height2, color2),
                    LobstahLEDBuffer.solid(height1, color1));
        }

        void generateHeights() {
            prevHeight1 = nextHeight1;
            prevHeight2 = nextHeight2;
            nextHeight1 = (int) (Math.random() * 20);
            nextHeight2 = (int) (Math.random() * 20);
        }
    }
    DisabledStandby disabledStandby = new DisabledStandby();
    LobstahLEDBuffer disabledStandby() {
        if (connectionState == ConnectionState.DISCONNECTED) {
            return disconnected();
        } else if (connectionState == ConnectionState.DS_ONLY) {
            return disabledStandby.get(new Color(255, 25, 25), new Color(255, 69, 70));
        } else if (alliance == Alliance.Red) {
            return disabledStandby.get(new Color(255, 25, 25), new Color(255, 69, 70));
        } else  {
            return disabledStandby.get(new Color(25, 25, 255), new Color(160, 170,255));
        }
    }

    LobstahLEDBuffer disconnected() {
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kWhite)
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, AnimationEasing.sine(Timer.getFPGATimestamp(), 3, 0)*10));
    }

    LobstahLEDBuffer autonomous() {
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 69, 118), 0.5)
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 30, 180))
                        .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 5, Timer.getFPGATimestamp() * 7)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(100, 25, 25)));
    }

    LobstahLEDBuffer userSignal() {
        if (userSignal == false) return null;
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kWhite, 0.5)
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 11))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kWhite, 0.7)
                        .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 5, Timer.getFPGATimestamp() * 6)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, Color.kWhite, 0.2));
    }
}

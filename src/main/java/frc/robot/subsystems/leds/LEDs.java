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

        io.setData(LobstahLEDBuffer.layer(LEDConstants.LENGTH_TOTAL,
                 robotMode == RobotMode.DISABLED ? disabledStandby() : null,
                 robotMode == RobotMode.AUTONOMOUS ? autonomous() : null,
                 posessionIndicator(),
                 posessionSignal(),
                //  shooterReadyIndicator(),
                 debugColor == null? null : LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, debugColor),
                 userSignal()
            ).toAdressableLEDBuffer());
    }

    LobstahLEDBuffer segments(LobstahLEDBuffer lowerLeft, LobstahLEDBuffer midSegment, LobstahLEDBuffer lowerRight, LobstahLEDBuffer upperRight, LobstahLEDBuffer upperLeft) {
        return LobstahLEDBuffer.concat(
            lowerLeft == null ? new LobstahLEDBuffer(LEDConstants.LENGTH_LOWER_LEFT) : lowerLeft.crop(LEDConstants.LENGTH_LOWER_LEFT),
            midSegment == null ? new LobstahLEDBuffer(LEDConstants.LENGTH_MID) : midSegment.crop(LEDConstants.LENGTH_MID),
            lowerRight == null ? new LobstahLEDBuffer(LEDConstants.LENGTH_LOWER_RIGHT) : lowerRight.crop(LEDConstants.LENGTH_LOWER_RIGHT).flip(),
            upperRight == null ? new LobstahLEDBuffer(LEDConstants.LENGTH_UPPER_RIGHT) : upperRight.crop(LEDConstants.LENGTH_UPPER_RIGHT),
            upperLeft == null ? new LobstahLEDBuffer(LEDConstants.LENGTH_UPPER_LEFT) : upperLeft.crop(LEDConstants.LENGTH_UPPER_LEFT).flip()
        );
    }

    private final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            double opacity = AnimationEasing.sine(System.currentTimeMillis(), 1000, 0);

            LobstahLEDBuffer buffer = LobstahLEDBuffer.solid(10, LEDConstants.COLOR_LOADING).mask(AlphaBuffer.sine(10, 20, -10)).opacity(opacity);

            buffer = segments(buffer, null, buffer, null, null);

            io.setData(buffer.toAdressableLEDBuffer());
        }
    });
    
    Timer possessionSignalTimer = new Timer();
    LobstahLEDBuffer posessionSignal() {
        double t = possessionSignalTimer.get();
        if (t>= 2 || !possession) return null;

        return LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_SUCCESS)
            .opacity(1-t/2);
    }

    LobstahLEDBuffer posessionIndicator() {
        if (!possession) return null;
        return prideFlagCycle(3, 10).tile(LEDConstants.LENGTH_TOTAL);

        // return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, LEDConstants.COLOR_RED)
        //         .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10))
        //         .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, LEDConstants.COLOR_TEAL)
        //                 .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10 + 3)))
        //         .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, LEDConstants.COLOR_RED, 0.25));
    }

    LobstahLEDBuffer shooterReadyIndicator() {
        if (!shooterReady) return null;
        return LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_SUCCESS);
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
            return LobstahLEDBuffer.layer(LEDConstants.LENGTH_TOTAL,
                    LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, color1, 0.5),
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
            return disabledStandby.get(LEDConstants.COLOR_RED, LEDConstants.COLOR_PINK);
        } else if (alliance == Alliance.Red) {
            return disabledStandby.get(LEDConstants.COLOR_RED, LEDConstants.COLOR_PINK);
        } else  {
            return disabledStandby.get(LEDConstants.COLOR_BLUE, LEDConstants.COLOR_TEAL);
        }
    }

    LobstahLEDBuffer disconnected() {
        return LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_LOADING)
                .mask(AlphaBuffer.sine(LEDConstants.LENGTH_TOTAL, 10, AnimationEasing.sine(Timer.getFPGATimestamp(), 3, 0)*10));
    }

    LobstahLEDBuffer autonomous() {
        return LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_AUTON_1, 0.5)
                .mask(AlphaBuffer.sine(LEDConstants.LENGTH_TOTAL, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_AUTON_2)
                        .mask(AlphaBuffer.sine(LEDConstants.LENGTH_TOTAL, 5, Timer.getFPGATimestamp() * 7)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_AUTON_3));
    }

    LobstahLEDBuffer userSignal() {
        if (userSignal == false) return null;
        return LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_USER_SIGNAL, 0.5)
                .mask(AlphaBuffer.sine(LEDConstants.LENGTH_TOTAL, 10, Timer.getFPGATimestamp() * 11))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_USER_SIGNAL, 0.7)
                        .mask(AlphaBuffer.sine(LEDConstants.LENGTH_TOTAL, 5, Timer.getFPGATimestamp() * 6)))
                .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LENGTH_TOTAL, LEDConstants.COLOR_USER_SIGNAL, 0.2));
    }

    LobstahLEDBuffer prideFlagCycle(int segmentLength, double speed) {
        int offset = (int) (Timer.getFPGATimestamp() * speed);
        return prideFlag(segmentLength).cycle(offset);

    }

    LobstahLEDBuffer prideFlag(int segmentLength) {
        return LobstahLEDBuffer.concat(
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_RED),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_ORANGE),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_YELLOW),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_GREEN),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_BLUE),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_PRIDE_PURPLE),
            LobstahLEDBuffer.solid(segmentLength, LEDConstants.COLOR_TRANS_PINK),
            LobstahLEDBuffer.solid(2 * segmentLength, Color.kWhite),
            LobstahLEDBuffer.solid(2 * segmentLength, LEDConstants.COLOR_TRANS_TEAL)
        );
    }
}

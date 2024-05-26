// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.*;
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

        io.setData(LobstahLEDBuffer.layer(LengthConstants.TOTAL,
                 robotMode == RobotMode.DISABLED ? disabledStandby() : null,
                 robotMode == RobotMode.AUTONOMOUS ? autonomous() : null,
                 posessionIndicator(),
                 posessionSignal(),
                //  shooterReadyIndicator(),
                 debugColor == null? null : LobstahLEDBuffer.solid(LengthConstants.TOTAL, debugColor),
                 userSignal()
            ).toAdressableLEDBuffer());
    }

    LobstahLEDBuffer segments(LobstahLEDBuffer lowerLeft, LobstahLEDBuffer midSegment, LobstahLEDBuffer lowerRight, LobstahLEDBuffer upperRight, LobstahLEDBuffer upperLeft) {
        return LobstahLEDBuffer.concat(
            lowerLeft == null ? new LobstahLEDBuffer(LengthConstants.LOWER_LEFT) : lowerLeft.crop(LengthConstants.LOWER_LEFT),
            midSegment == null ? new LobstahLEDBuffer(LengthConstants.MID) : midSegment.crop(LengthConstants.MID),
            lowerRight == null ? new LobstahLEDBuffer(LengthConstants.LOWER_RIGHT) : lowerRight.crop(LengthConstants.LOWER_RIGHT).flip(),
            upperRight == null ? new LobstahLEDBuffer(LengthConstants.UPPER_RIGHT) : upperRight.crop(LengthConstants.UPPER_RIGHT),
            upperLeft == null ? new LobstahLEDBuffer(LengthConstants.UPPER_LEFT) : upperLeft.crop(LengthConstants.UPPER_LEFT).flip()
        );
    }

    private final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            double opacity = AnimationEasing.sine(System.currentTimeMillis(), 1000, 0);

            LobstahLEDBuffer buffer = LobstahLEDBuffer.solid(10, ColorConstants.LOADING).mask(AlphaBuffer.sine(10, 20, -10)).opacity(opacity);

            buffer = segments(buffer, null, buffer, null, null);

            io.setData(buffer.toAdressableLEDBuffer());
        }
    });
    
    Timer possessionSignalTimer = new Timer();
    LobstahLEDBuffer posessionSignal() {
        double t = possessionSignalTimer.get();
        if (t>= 2 || !possession) return null;

        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.SUCCESS)
            .opacity(1-t/2);
    }

    LobstahLEDBuffer posessionIndicator() {
        if (!possession) return null;
        return prideFlagCycle(3, 10).tile(LengthConstants.TOTAL);

        // return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, ColorConstants.RED)
        //         .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10))
        //         .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, ColorConstants.TEAL)
        //                 .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 10 + 3)))
        //         .layerAbove(LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, ColorConstants.RED, 0.25));
    }

    LobstahLEDBuffer shooterReadyIndicator() {
        if (!shooterReady) return null;
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.SUCCESS);
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
            return LobstahLEDBuffer.layer(LengthConstants.TOTAL,
                    LobstahLEDBuffer.solid(LengthConstants.TOTAL, color1, 0.5),
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
            return disabledStandby.get(ColorConstants.RED, ColorConstants.PINK);
        } else if (alliance == Alliance.Red) {
            return disabledStandby.get(ColorConstants.RED, ColorConstants.PINK);
        } else  {
            return disabledStandby.get(ColorConstants.BLUE, ColorConstants.TEAL);
        }
    }

    LobstahLEDBuffer disconnected() {
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.LOADING)
                .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 10, AnimationEasing.sine(Timer.getFPGATimestamp(), 3, 0)*10));
    }

    LobstahLEDBuffer autonomous() {
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_1, 0.5)
                .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_2)
                        .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 5, Timer.getFPGATimestamp() * 7)))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_3));
    }

    LobstahLEDBuffer userSignal() {
        if (userSignal == false) return null;
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.USER_SIGNAL, 0.5)
                .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 10, Timer.getFPGATimestamp() * 11))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.USER_SIGNAL, 0.7)
                        .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 5, Timer.getFPGATimestamp() * 6)))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.USER_SIGNAL, 0.2));
    }

    LobstahLEDBuffer prideFlagCycle(int segmentLength, double speed) {
        int offset = (int) (Timer.getFPGATimestamp() * speed);
        return prideFlag(segmentLength).prepend(LobstahLEDBuffer.solid(1, Color.kBlack)).cycle(offset);

    }

    LobstahLEDBuffer prideFlag(int segmentLength) {
        return LobstahLEDBuffer.concat(
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_RED),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_ORANGE),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_YELLOW),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_GREEN),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_BLUE),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_PURPLE),
            LobstahLEDBuffer.solid(1, Color.kBlack),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_TEAL),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_PINK),
            LobstahLEDBuffer.solid(segmentLength, Color.kWhite),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_PINK),
            LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_TEAL)
        ).flip();
    }
}

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

    public enum FMSState { DISCONNECTED, DS_ONLY, RED, BLUE }
    FMSState fmsState = FMSState.DISCONNECTED;
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

    public void setPossession(boolean value) {
        possession = value;
        if (possession) possessionSignalTimer.restart();
    }

    public void setIntaking (boolean value) { intaking = value; }

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
            fmsState = FMSState.DISCONNECTED;
        } else if (DriverStation.isFMSAttached()) {
            if (DriverStation.getAlliance().ge == DriverStation.Alliance.Red) {
                fmsState = FMSState.RED;
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                fmsState = FMSState.BLUE;
            } else {
                fmsState = FMSState.DS_ONLY;
            }
        } else {
            fmsState = FMSState.DS_ONLY;
        }

        if (DriverStation.isEStopped()) {
            robotMode = RobotMode.ESTOPPED;
        } else if (DriverStation.isAutonomousEnabled()) {
            robotMode = RobotMode.AUTONOMOUS;
        } else if (DriverStation.isTeleopEnabled()) {
            robotMode = RobotMode.TELEOP;
        } else {
            robotMode = RobotMode.DISABLED;
        }

        io.setData(LobstahLEDBuffer.layer(LEDConstants.LED_LENGTH,
                posessionSignal(),
                posessionIndicator(),
                disabledStandby() ).toAdressableLEDBuffer());
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
        if (t>= 1 || !possession) return null;

        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(77, 255, 79))
            .opacity(1-t);
    }

    LobstahLEDBuffer posessionIndicator() {
        if (!possession) return null;
        return LobstahLEDBuffer.solid(5, new Color(77, 255, 79))
            .wrappedShift(LEDConstants.LED_LENGTH, -5);
    }

    LobstahLEDBuffer disabledStandby() {
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 69, 118))
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 5));
    }
}

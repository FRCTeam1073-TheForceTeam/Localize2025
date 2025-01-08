package frc.robot.subsystems;
import java.util.Arrays;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;


public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte triggerCommand[] = new byte[4];
    private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)

    double range = 0.0;
    double filtered_range = 0.0;
    double intensity = 0.0; // starts to die at under 0.005
    double timestamp = 0.0;

    public Lidar () {
        super.setSubsystem("Lidar");
        serialPort.setFlowControl(SerialPort.FlowControl.kNone);
        
        triggerCommand[0] = 0x5a;
        triggerCommand[1] = 0x04;
        triggerCommand[2] = 0x04;
        triggerCommand[3] = 0x62;

        Handshake();
    }

    public void Handshake () {
        byte FrameCommand[] = new byte[6];
        FrameCommand[0] = 0x5A;
        FrameCommand[1] = 0x06;
        FrameCommand[2] = 0x03;
        FrameCommand[3] = 0x0;
        FrameCommand[4] = 0x0;
        FrameCommand[5] = 0x63;
        serialPort.write(FrameCommand, 6);
        serialPort.flush();
        try {
            Thread.sleep(50);
        }
        catch (Exception e) {
            System.out.println(e);
        }
        serialPort.reset();
        serialPort.write(triggerCommand, 4);
    }

    public void parseData (byte message[]) {
        if (message.length < 9) {
            System.err.println("Rangefinder message format ERROR");
            return;
        }

        if (message[0] != 0x59 || message[1] != 0x59) {
            System.err.println("Rangefinder message format ERROR");
            return;
        }

        int dl = (message[2] & 0xFF);
        int dh = (message[3] & 0xFF);
        range = ((dl + 255 * dh) * 0.01) * elevationFactor;

        // IIR filter:
        filtered_range = 0.8 * filtered_range + 0.2 * range; 
        
        int il = (message[4] & 0xFF);
        int ih = (message[5] & 0xFF);

        intensity = (il + 255 * ih) * 1.5259e-5;
        timestamp = Timer.getFPGATimestamp();
    }

    public double getRange() {
        return range;
    }

    public double getFilteredRange() {
        return filtered_range;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public void periodic() {
        int bytestoread = serialPort.getBytesReceived();
        byte bytes[];

        if (bytestoread >= 9 ) {
            bytes = serialPort.read(bytestoread);

            parseData(bytes);

            //System.out.println(Arrays.toString(bytes));
            serialPort.write(triggerCommand, 4);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Range", this::getRange, null);
        builder.addDoubleProperty("Intensity", this::getIntensity, null);
        builder.addDoubleProperty("Timestamp", this::getTimestamp, null);
    }

    @Override
    public boolean updateDiagnostics() {
        double now = Timer.getFPGATimestamp();
        boolean OK = true;
        if (now - timestamp > 2.0) {
            OK = false;
        }

        else {
            OK = false;
        }
        return setDiagnosticsFeedback("", OK);
    }
}
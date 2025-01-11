package frc.robot.subsystems;
import java.util.Arrays;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort = new SerialPort(1000000, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte startCommand[] = new byte[2];
    //private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)

    double range = 0.0;
    double filtered_range = 0.0;
    double intensity = 0.0; // starts to die at under 0.005
    double timestamp = 0.0;

    public Lidar () {
        super.setSubsystem("Lidar");
        serialPort.setFlowControl(SerialPort.FlowControl.kNone);
        
        startCommand[0] = 0xA5;
        startCommand[1] = 0x20;

        Handshake();
    }

    public void Handshake () {
        byte StopCommand[] = new byte[2];
        StopCommand[0] = 0xA5;
        StopCommand[1] = 0x25;

        // TODO: fill out stop command
        //send stop
        serialPort.write(StopCommand, StopCommand.length);
        try {
            Thread.sleep(50);
        }
        catch (Exception e) {
            System.out.println(e);
        }
        serialPort.flush();
        serialPort.reset();
        // ?? add GET_HEALTH Request?
        //startCommand
        serialPort.write(startCommand, startCommand.length);
    }

    public void parseData (byte message[]) {
        
        timestamp = Timer.getFPGATimestamp();
    }

    // public double getRange() {
    //     return range;
    // }

    // public double getFilteredRange() {
    //     return filtered_range;
    // }

    // public double getIntensity() {
    //     return intensity;
    // }

    public double getTimestamp() {
        return timestamp;
    }

    public void read(){
        //clockwise is positive - opposite for robot?
        //angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]

    }

    @Override
    public void periodic() {
        // wait until descriptor is available
        // loop until full scan data has been processed
        //      poll until bytes available >= one entry
        //         read one entry
        //read()
        //      parse one entry
        //parse()
        // if full scan has been complete (S = 1)
        //    Stop/Restart Scan (Handshake)
        //      stop scan
        //      send get_healty request, verify not in Protection Stop (HW Failure) state
        //      start scan
        //  note that data response packets will stop if HW failure is detected
        //      eventually, add timeout and reset

        
        int bytestoread = serialPort.getBytesReceived();
        byte bytes[];

        if (bytestoread >= 9 ) {
            bytes = serialPort.read(bytestoread);

            parseData(bytes);
            //request a new scan
            //System.out.println(Arrays.toString(bytes));
            serialPort.write(startCommand, startCommand.length);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addDoubleProperty("Range", this::getRange, null);
        // builder.addDoubleProperty("Intensity", this::getIntensity, null);
        // builder.addDoubleProperty("Timestamp", this::getTimestamp, null);
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
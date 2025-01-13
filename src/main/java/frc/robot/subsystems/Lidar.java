package frc.robot.subsystems;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort = new SerialPort(1000000, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte startCommand[] = new byte[2];
    boolean measureMode = false;
    boolean arrayMode = false;
    private final int bytesPerScan = 5;
    int numScansRead;
    int numBytesAvail = 0;
    int numScansToRead;
    Array [byte[]] dataArray;
    ArrayList <byte[]> dataArrayList;
    float quality;
    float range_mm;
    float angle_deg;
    float range_m;
    float angle_rad;
    //TODO: add scan descriptor to check descriptor string
    byte scanDescriptor[] = {5A, A5, 05, 00, 00, 40, 81};
    //private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)

    double filtered_range = 0.0;
    double intensity = 0.0; // starts to die at under 0.005
    double timestamp = 0.0;
    ArrayList <Scan> one;
    ArrayList <Scan> two;

    public Lidar () {
        super.setSubsystem("Lidar");
        serialPort.setFlowControl(SerialPort.FlowControl.kNone);
        
        startCommand[0] = (byte) 0xa5;
        startCommand[1] = 0x20;

        Handshake();
    }

    public void Handshake () {
        byte StopCommand[] = new byte[2];
        StopCommand[0] = (byte) 0xa5;
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
    public ArrayList getArray(){
        if(arrayMode){
            return two;
        }

        if(!arrayMode){
            return one;
        }
    }

    public double getTimestamp() {
        return timestamp;
    }


    public boolean parseDescriptor(){
        return serialPort.read(7) == scanDescriptor;
    }

    // what is most efficient?
    public void readAndParseMeasurements(int numAvail){
        //put data into scan class - return array list of scan object to add to arrayList?
        // round down to determine number of full scans available
        numScansToRead = numAvail/bytesPerScan;
        dataArray = serialPort.read(numScansToRead * bytesPerScan);
        //dataArrayList = tbd
        // TODO: put parse data into arraylist
        //clockwise is positive - opposite for robot - convert to radians
        //angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
        //distance = Math.pow(2, 8) * distance[15:8] + distance[7:0]
        // TODO: Format Data Packets
        for(int i = 0; i < numScansToRead; i ++){
            // divide by 4 because to drop the lower two bits
            quality = dataArrayList.get(0 + (i * bytesPerScan)) / 4;
            // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            angle_deg = Math.pow(2, 7) * dataArrayList.get(2 + (i * bytesPerScan)) + (dataArrayList.get(1 + (i * bytesPerScan)) / 2);
            //range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            range_mm = Math.pow(2, 8) * dataArrayList.get(4 + (i * bytesPerScan)) + dataArrayList.get(3 + (i * bytesPerScan));
            // TODO: convert to right thing
            // best way to 
            if(arrayMode){
                one.add(new Scan(range_m, angle_rad, quality));
            }

            if(!arrayMode){
                two.add(new Scan(range_m, angle_rad, quality));
            }

        }
        // don't need anymore
        dataArrayList.clear();
    }

    @Override
    public void periodic() {
        numBytesAvail = serialPort.getBytesReceived();
        if(measureMode){
            // read and parse all available scan data to read - fill array
            readAndParseMeasurements(numBytesAvail);
        }

        if(numBytesAvail >= 7){
            // read and check the first 7 bytes of response
            if(parseDescriptor()){
                // expected descriptor received, switch to read data
                measureMode = true;
            }
        }
        
        // TODO transform 3D
        // loop until full scan data has been processed
        //      poll until bytes available >= one entry
        //         read one entry
        //read()
        //      parse one entry
        //parse()
        // if full scan has been complete (S = 1) -> switch to the other array
        //  note that data response packets will stop if HW failure is detected

        
        // int bytestoread = serialPort.getBytesReceived();
        // byte bytes[];

        // if (bytestoread >= 9 ) {
        //     bytes = serialPort.read(bytestoread);

        //     parseData(bytes);
        //     //request a new scan
        //     //System.out.println(Arrays.toString(bytes));
        //     serialPort.write(startCommand, startCommand.length);
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
package frc.robot.subsystems;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort = new SerialPort(1000000, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte startCommand[] = {(byte) 0xa5, 0x20};
    boolean measureMode = false;
    boolean writeToOne = false;
    private final int bytesPerScan = 5;
    int numScansRead;
    int numBytesAvail = 0;
    int numScansToRead;
    int offset;
    int rawDataByte;
    float quality;
    float range_mm;
    float angle_deg;
    float range_m;
    float angle_rad;
    double arrayOneTimestamp;
    double arrayTwoTimestamp;
    byte scanDescriptor[] = {(byte) 0x5A, (byte) 0xA5, (byte) 0x05, 0x00, 0x00, 0x40, (byte) 0x81};
    //private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)

    double filtered_range = 0.0;
    double intensity = 0.0; // starts to die at under 0.005
    double timestamp = 0.0;
    ArrayList <Scan> one = new ArrayList<>();
    ArrayList <Scan> two = new ArrayList<>();

    public Lidar () {
        super.setSubsystem("Lidar");
        serialPort.setFlowControl(SerialPort.FlowControl.kNone);
        System.out.println("Lidar constructor");
        Handshake();
    }

    public void Handshake () {
        System.out.println("Beginning of handshake method for Lidar sensor");
        byte StopCommand[] = new byte[2];
        StopCommand[0] = (byte) 0xa5;
        StopCommand[1] = 0x25;

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
        System.out.println("Sent start command to Lidar sensor");
        for(int i = 0; i < startCommand.length; i++){
            System.out.println(String.format("0x%02x", startCommand[i]));
        }
        // while(serialPort.getBytesReceived() == 0){
        //     System.out.println("Waiting for bytes from LiDAR");
        // }
        byte data[] = serialPort.read(serialPort.getBytesReceived());
        System.out.println("Length of LiDAR sent from start command " + data.length);
        for(int i = 0; i < data.length; i++){
            System.out.println(String.format("0x%02x", data[i]));
        }
        // How to print out hex in java - String.format("0x%02x", what you're printing out)

    }

    public ArrayList<Scan> getScan(){
        if(writeToOne){
            return two; 
        } else {
            return one;
        }
    }

    public double getTimestamp() {
        // return the opposite that is being filled
        if (writeToOne) return arrayTwoTimestamp;
        else return arrayOneTimestamp;
    }


    public boolean parseDescriptor(){
        System.out.println("Parsing lidar scan descriptor");
        byte [] received = serialPort.read(7);
        return Arrays.equals(received, scanDescriptor);
    }

    // what is most efficient?
    public void readAndParseMeasurements(int numAvail){
        // TODO: promote byte to an int - do something to make it from 0 to 255 - look through old rangefinder for converting values
        //put data into scan class - return array list of scan object to add to arrayList?
        // round down to determine number of full scans available
        numScansToRead = numAvail/bytesPerScan;
        byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
        // TODO: clockwise is positive - opposite for robot - convert to radians
        for(int i = 0; i < numScansToRead; i ++){
            offset = i * bytesPerScan;
           if((rawData[0] & 0x01) == 1){
                // TODO: Write timestamp when switch
                System.out.println("Switching LiDAR arrays");
                if(writeToOne) {
                    //done writing to one, switching to writing to two
                    two.clear();
                    arrayTwoTimestamp = Timer.getFPGATimestamp();
                }

                else {
                    //done writing to two, switching to writing to one
                    one.clear();
                    arrayOneTimestamp = Timer.getFPGATimestamp();
                }
                writeToOne = !writeToOne;
            } 
            // divide by 4 because to drop the lower two bits
            quality = (rawData[offset + 0] & 0xFF) / 4;
            // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            angle_deg = -128 * (rawData[(offset) + 2] & 0xFF) + ((rawData[offset + 1] & 0xFF)/ 2);
            //range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            range_mm = 256 * (rawData[offset + 4] & 0xFF) + (rawData[offset + 3] & 0xFF);
            angle_rad = angle_deg / 180.0f * 3.141592f;
            range_m = range_mm / 1000;
            if(writeToOne){
                one.add(new Scan(range_m, angle_rad, quality));
            } 
            else{
                two.add(new Scan(range_m, angle_rad, quality));
            }

        }
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
                System.out.println("Started Lidar measurement mode");
            }
            else System.out.println("Lidar handshake error");
        }
        
        // TODO transform 3D
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
package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort = new SerialPort(460800, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    private final int bytesPerScan = 5;
    //private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)
    ArrayList <Scan> one = new ArrayList<>();
    ArrayList <Scan> two = new ArrayList<>();
    ArrayList <Scan> data = new ArrayList<>();
    byte getInfo[] = {(byte) 0xa5, (byte) 0x52};
    byte scanDescriptor[] = {(byte) 0xa5, (byte) 0x5a, (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x40, (byte) 0x81};
    byte stopCommand[] = {(byte) 0xa5, (byte) 0x25};
    byte startCommand[] = {(byte) 0xa5, (byte) 0x20};
    boolean arrayTwoFilled = false;
    boolean recordScan = true;
    boolean measureMode = false;
    boolean writeToOne = true;
    double arrayOneTimestamp;
    double arrayTwoTimestamp;
    double filtered_range = 0.0;
    double intensity = 0.0; // starts to die at under 0.005
    double timestamp = 0.0;
    float angle_deg;
    float angle_rad;
    float quality;
    float range_m;
    float range_mm;
    int minAcceptedRange; // In meters, TODO: set min accepted range
    int maxAcceptedRange; // In meters, TODO: set max accepted range
    int minAcceptedAngle; // In radians, TODO: set min accepted angle
    int maxAcceptedAngle; // In radians, TODO: set min accepted angle
    int numBytesAvail = 0;
    int numTimesLidarArraySwitch = 0;
    int numScansRead;
    int numScansToRead;
    int offset;
    int rawDataByte;

    public Lidar () {
        // before
        serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
        super.setSubsystem("Lidar");
        serialPort.setFlowControl(SerialPort.FlowControl.kNone);
        System.out.println("Lidar constructor");
        Handshake();
    }

    public void Handshake () {
        System.out.println("Beginning of handshake method for Lidar sensor");

        //send stop
        serialPort.write(stopCommand, stopCommand.length);
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
        // wait loop, while getBytesReceived is less, sleep
        while(serialPort.getBytesReceived() < 7){
            try {
                Thread.sleep(50);
            }
            catch (Exception e) {
                System.out.println(e);
            }
        }
        
        // How to print out hex in java - String.format("0x%02x", what you're printing out)

    }

    public ArrayList<Scan> getLidarArray(){
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
        System.out.println("Scan descriptor received from serialPort");
        for(int i = 0; i < received.length; i++){
            System.out.println(String.format("0x%02x", received[i]));
        }
        return Arrays.equals(received, scanDescriptor);
    }

    // what is most efficient?
    public void readAndParseMeasurements(int numBytesAvail){

        // round down to determine number of full scans available
        numScansToRead = numBytesAvail/bytesPerScan;
        byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
        // TODO: clockwise is positive - opposite for robot
        for(int i = 0; i < numScansToRead; i ++){
            offset = i * bytesPerScan;
           if((rawData[offset] & 0x003) == 1){
                // print out bytes
                System.out.println(String.format("Angle Bytes: 0x%02x 0x%02x", rawData[offset + 1], rawData[offset + 2]));

                if(writeToOne) {
                    //done writing to one, switching to writing to two - sets the timestamp for array two
                    two.clear();
                    arrayTwoTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    writeToOne = false;
                }

                else {
                    //done writing to two, switching to writing to one - sets the timestamp for array one
                    one.clear();
                    arrayOneTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    arrayTwoFilled = true;
                    writeToOne = true;
                }
            } 
            // TODO: print out number of scans we got, map out scans?, put angle filter (only care abt certain angles), range filter
            
            // divide by 4 to drop the lower two bits
            quality = ((rawData[offset + 0] & 0x0FC) >> 2);
            // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            angle_deg = ((Byte.toUnsignedInt(rawData[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(rawData[offset + 1]) & 0x0FE) >> 1);
            //range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            range_mm = ((rawData[offset + 4] & 0x0FF) << 8) | (rawData[offset + 3] & 0x0FF);
            //angle_rad = 3.141592f * (angle_deg/64.0f) / 180.0f;
            angle_rad = angle_deg/64.0f;
            range_m = (range_mm / 4.0f) / 1000;
            //if(range_m > maxAcceptedRange || range_m < minAcceptedRange) recordScan = false;
            //if(angle_rad > maxAcceptedAngle || angle_rad < minAcceptedAngle) recordScan = false;

            // TODO: Error output if we lost sync with scanner - try turning off/handshake
            // TODO: Add quality thresh-hold
            if(writeToOne && one.size() < 512){
                one.add(new Scan(range_m, angle_rad, quality));
            } 
            else if(!writeToOne && two.size() < 512){
                two.add(new Scan(range_m, angle_rad, quality));
            }
        }
    }

    public int getNumberScansToRead(){
        return numScansToRead;
    }

    public int getNumberScans(){
        if(writeToOne){
            return two.size(); 
        } else {
            return one.size();
        }
    }

    @Override
    public void periodic() {
        if(arrayTwoFilled){
            SmartDashboard.putNumber("Times Lidar Array Switched", getTimesArraySwitch());
            SmartDashboard.putNumber("Range", getRange());
            SmartDashboard.putNumber("Angle", getAngle());
            SmartDashboard.putNumber("Quality", getQuality());
            SmartDashboard.putNumber("Number of Scans in LiDAR Array", getNumberScans());
            SmartDashboard.putNumber("Number of Scans to Read", getNumberScansToRead());
        }
        numBytesAvail = serialPort.getBytesReceived();
        if(measureMode){
            // read and parse all available scan data to read - fill array
            readAndParseMeasurements(numBytesAvail);
        }

        if(numBytesAvail >= 7 && !measureMode){
            // read and check the first 7 bytes of response
            if(parseDescriptor()){
                // expected descriptor received, switch to read data
                measureMode = true;
                System.out.println("Started Lidar measurement mode");
            }
            else{
                System.out.println("Lidar handshake error");
            }
        }
        
        // TODO transform 3D
        }

    public int getTimesArraySwitch(){
        return numTimesLidarArraySwitch;
    }

    public double getRange(){
        return getLidarArray().get(1).getRange();
    }

    public double getAngle(){
        return getLidarArray().get(1).getAngle();
    }
    
    public double getQuality(){
        return getLidarArray().get(1).getQuality();
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
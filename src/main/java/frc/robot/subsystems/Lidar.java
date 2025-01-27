package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    ArrayList <Scan> ransac = new ArrayList<>();
    ArrayList <Scan> inliers = new ArrayList<>();
    ArrayList <Scan> bestInliers = new ArrayList<>(); // list of the best inliers for RANSAC
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
    double x_l;
    double y_l;
    double a; // a value for standard form of a line
    double b; // b value for standard form of a line
    double c; // c value for standard form of a line
    double[] bestLine = new double[3]; // a, b, and c value of the best line
    final double distanceThreshold = 0; // maximum distance a point can be from the line to be considered an inlier
    double distance;
    float angle_deg;
    float angle_rad;
    float quality;
    float range_m;
    float range_mm;
    final float minAcceptedRange = 0.07f; // In meters
    final int maxAcceptedRange = 3; // In meters
    final int minAcceptedAngle1 = 0; // In degrees, for the first range of accepted angles
    final int maxAcceptedAngle1 = 80; // In degrees, for the first range of accepted angles
    final int minAcceptedAngle2 = 280; // In degrees, for the second range of accepted angles
    final int maxAcceptedAngle2 = 360; // In degrees, for the second range of accepted angles
    private final int minAcceptedQuality = 5;
    final int minInliers = 10; // minimum number of inliers for a model to be considered valid
    final int maxIterations = 100; // maximum number of iterations to find a model
    int minSamples;
    int numBytesAvail = 0;
    int numTimesLidarArraySwitch = 0;
    int numScansRead;
    int numScansToRead;
    int offset;
    int rawDataByte;
    Transform2d robotToLidar = new Transform2d(new Translation2d(0.27, 0), new Rotation2d()); // Translation needs x and y, rotation needs 
    Matrix<N3,N3> T;
    Scan point1;
    Scan point2;

    public Lidar () {
        // before
        T = robotToLidar.toMatrix();
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
    /*Start with the input data - ArrayLists
    * Pick random samples
    * Fit a mathematical model (line, curve, 3d shape, ...)
    * Compute a cost by checking how many points fit the model
    * Repeat until you found the model with the lowest cost */
    public void lidarRANSAC(){
        ransac = getLidarArray();
        for(int i = 0; i < maxIterations; i++){
            // select two random points
            point1 = ransac.get((int) (Math.random() * ransac.size()));
            point2 = ransac.get((int) (Math.random() * ransac.size()));
            while(point1 == point2){
                point2 = ransac.get((int) Math.random() * ransac.size());
            }
            a = point2.getY() - point1.getY();
            b = point1.getX() - point2.getX();
            c = point1.getY() * point2.getX() - point2.getY() * point1.getX();
            for(Scan scan : ransac){
                distance = Math.abs(a * scan.getX() + b * scan.getY() + c) / Math.sqrt(a * a + b * b);
                if(distance < distanceThreshold){
                    inliers.add(scan);
                }
            }
            if(inliers.size() > bestInliers.size()){
                bestInliers = inliers;
                bestLine = new double[] {a, b, c};
            }
            inliers.clear();
        }
        if(bestLine != null){
            System.out.printf("Best line: %.2fx + %.2fy + %.2f = 0%n", bestLine[0], bestLine[1], bestLine[2]);
        }
        else System.out.println("No valid line found.");
        
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

    public boolean isAngleGood(float angle){
        if((angle > minAcceptedAngle1 && angle < maxAcceptedAngle1) || (angle > minAcceptedAngle2 && angle < maxAcceptedAngle2)) return true;
        else return false;
    }

    public boolean isRangeGood(float range){
        if((range < maxAcceptedRange) && (range > minAcceptedRange)) return true;
        else return false;
    }

    // what is most efficient?
    public void readAndParseMeasurements(int numBytesAvail){

        // round down to determine number of full scans available
        numScansToRead = numBytesAvail/bytesPerScan;
        byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
        // TODO: clockwise is positive - opposite for robot
        for(int i = 0; i < numScansToRead; i ++){
            offset = i * bytesPerScan;
            recordScan = true;
           if((rawData[offset] & 0x003) == 1){
                // print out bytes
                System.out.println(String.format("Angle Bytes: 0x%02x 0x%02x", rawData[offset + 1], rawData[offset + 2]));

                if(writeToOne){
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
            if(quality < minAcceptedQuality) recordScan = false;
            // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            angle_deg = ((Byte.toUnsignedInt(rawData[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(rawData[offset + 1]) & 0x0FE) >> 1);
            angle_deg /= 64.0f;
            //range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            range_mm = ((rawData[offset + 4] & 0x0FF) << 8) | (rawData[offset + 3] & 0x0FF);
            angle_rad = 3.141592f * angle_deg / 180.0f;
            range_m = (range_mm / 4.0f) / 1000;
            //quality, range, and angle filter
            if(quality < minAcceptedQuality) recordScan = false;
            if(isAngleGood(angle_deg) == false) recordScan = false;
            if(isRangeGood(range_m) == false) recordScan = false;

            // TODO: Error output if we lost sync with scanner - try turning off/handshake
            if(recordScan){
                x_l = Math.cos(-angle_rad) * range_m;
                y_l = Math.sin(-angle_rad) * range_m;
                Matrix<N3, N1> lidarPoint = VecBuilder.fill(x_l, y_l, 1.0);
                Matrix<N3, N1> robotPoint = T.times(lidarPoint);
                if(writeToOne && one.size() < 512){
                one.add(new Scan(range_m, angle_rad, quality, robotPoint.get(0,0), robotPoint.get(1, 0)));
                } 
                else if(!writeToOne && two.size() < 512){
                two.add(new Scan(range_m, angle_rad, quality, robotPoint.get(0,0), robotPoint.get(1, 0)));
                }
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
            SmartDashboard.putNumber("LiDAR X Value", getXVal());
            SmartDashboard.putNumber("LiDAR Y Value", getYVal());
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

    public double getXVal(){
        return getLidarArray().get(1).getX();
    }
    
    public double getYVal(){
        return getLidarArray().get(1).getY();
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
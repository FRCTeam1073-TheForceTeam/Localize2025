package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lidar extends DiagnosticsSubsystem {
    SerialPort serialPort;
    private final int bytesPerScan = 5;
    //private final double elevationFactor = 0.94293; //0.97237; //compensating for the fact that it's not level: cos(angle of rangeFinder)
    public class Point{
        double x;
        double y;
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }

        public double getX(){
            return x;
        }

        public double getY(){
            return y;
        }

        public void setX(double x){
            this.x = x;
        }
        
        public void setY(double y){
            this.y = y;
        }
    }
    
    ArrayList <Scan> one = new ArrayList<Scan>();
    ArrayList <Scan> two = new ArrayList<Scan>();
    ArrayList <Scan> data = new ArrayList<Scan>();
    ArrayList <Scan> ransac = new ArrayList<Scan>();
    ArrayList <Scan> inliers = new ArrayList<Scan>();
    ArrayList <Scan> points  = new ArrayList<Scan>();
    ArrayList <Scan> bestInliers = new ArrayList<Scan>(); // list of the best inliers for RANSAC
    Point[] startAndEnd = new Point[2];
    byte getInfo[] = {(byte) 0xa5, (byte) 0x52};
    byte scanDescriptor[] = {(byte) 0xa5, (byte) 0x5a, (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x40, (byte) 0x81};
    byte stopCommand[] = {(byte) 0xa5, (byte) 0x25};
    byte startCommand[] = {(byte) 0xa5, (byte) 0x20};
    boolean arrayTwoFilled = false;
    boolean recordScan = true;
    boolean measureMode = false;
    boolean writeToOne = true;
    boolean searchingForStart = true;
    boolean searchingForEnd = false;
    boolean foundLine = false;
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
    double lidarSlope;
    double robotSlope;
    double angleToRotate;
    double thetaVelocity;
    double[] bestLine = new double[3]; // a, b, and c value of the best line
    final double distanceThreshold = 0.05; // maximum distance a point can be from the line to be considered an inlier
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
    final int maxIterations = 20; // maximum number of iterations to find a model
    int indexOfEnd;
    int minSamples;
    int numBytesAvail = 0;
    int numTimesLidarArraySwitch = 0;
    int numScansRead;
    int numScansToRead;
    int offset;
    int rawDataByte;
    int pointsOnLine;
    int rand1;
    int rand2;
    Transform2d robotToLidar = new Transform2d(new Translation2d(0.27, 0), new Rotation2d()); // Translation needs x and y, rotation needs 
    Matrix<N3,N3> T;
    Point start = new Point(0, 0);
    Point end = new Point(0, 0);
    Pose2d targetRotationPose;
    Pose2d currentPose;
    PIDController thetaController;
    Random randy = new Random();
    Scan point1;
    Scan point2;

    public Lidar () {
        T = robotToLidar.toMatrix();

        thetaController = new PIDController(1.2, 0.0, 0.01);

        try{
            serialPort = new SerialPort(460800, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        }
        catch(Exception e){
            System.out.println("No LiDAR found");
            serialPort = null;
        }
        if(serialPort != null){
            serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
            super.setSubsystem("Lidar");
            serialPort.setFlowControl(SerialPort.FlowControl.kNone);
            System.out.println("Lidar constructor");
            Handshake();
        }
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

    public ArrayList<Scan> lidarRANSAC(){
        ransac = new ArrayList<Scan>(getLidarArray());
        bestInliers.clear();
        if(ransac.size() <= 0){
            return null;
        }
        for(int i = 0; i < 10; i++){
            // select two random points

            inliers.clear();
            rand1 = Math.abs(randy.nextInt(ransac.size()));
            rand2 = Math.abs(randy.nextInt(ransac.size()));
            point1 = ransac.get(rand1);
            point2 = ransac.get(rand2);
            while(point1 == point2){
                rand2 = Math.abs(randy.nextInt(ransac.size()));
                point2 = ransac.get(rand2);
            }

            a = point2.getY() - point1.getY();
            b = point1.getX() - point2.getX();
            c = point1.getY() * point2.getX() - point2.getY() * point1.getX();
            for(Scan scan : ransac){
                distance = Math.abs(a * scan.getX() + b * scan.getY() + c) / Math.sqrt(a * a + b * b);
                if(distance < distanceThreshold){
                    inliers.add(scan);
                    // give first and last inlier
                    // when there's a large gap, drop points
                }
            }
            // if it's greater than the threshold and better
            if(inliers.size() > bestInliers.size() && inliers.size() > 15){
                bestInliers = inliers;
                bestLine = new double[] {a, b, c};
            }
        }
        if(bestLine != null){
            SmartDashboard.putBoolean("Found a Line", true);
            SmartDashboard.putNumber("Line A Value", a);
            SmartDashboard.putNumber("Line B Value", b);
            SmartDashboard.putNumber("Line C Value", c);
            SmartDashboard.putNumber("Slope of Lidar Line", getSlope());
            SmartDashboard.putNumber("Lidar Angle to Rotate", Math.atan(getSlope()));
            return bestInliers;
        }
        else {
            SmartDashboard.putBoolean("Found a Line", false);
            return null;
        }
    }
    // test if points are close together ( 5- 10 cm), searching and finding segment
    // searching for beginning or searching for the end
    // index of beginning and end
    // outputs endpoint
    public double[] getLine(){
        return bestLine;
    }

    public double getSlope(){
        if(getLine() != null){
            return -bestLine[0]/bestLine[1];
        }
        return 0.0;
    }

    public Point[] findLineSegment(ArrayList<Scan> arr){
            points.clear();
            if(arr == null){
                return null;
            }
            points = new ArrayList<Scan>(arr);
        if(points != null && points.size() >= 15){
            System.out.println(points.size());
            pointsOnLine = 0;
            searchingForStart = true;
            searchingForEnd = false;
            foundLine = false;
            SmartDashboard.putBoolean("Found Line", foundLine);
            for(int i = 0; i < points.size() - 1; i++){
                if(searchingForStart){
                    // checks the distance of a point in the inlier set to the next point
                    // if the distance between the two points is within 0.05 m, then the points are close enough
                    double xPoint = points.get(i).getX();
                    double yPoint = points.get(i).getY();
                    if(Math.sqrt(Math.pow((yPoint - points.get(i + 1).getY()), 2) + Math.pow((xPoint - points.get(i + 1).getX()), 2)) <= 0.05){
                        searchingForStart = false;
                        searchingForEnd = true;
                        pointsOnLine = 1;
                        start.setX(xPoint);
                        start.setY(yPoint);
                    }
                }
                if(searchingForEnd){
                    if(Math.sqrt(Math.pow((points.get(i).getY() - points.get(i + 1).getY()), 2) + Math.pow((points.get(i).getX() - points.get(i + 1).getX()), 2)) <= 0.05){
                        pointsOnLine ++;
                    }
                    else if(Math.sqrt(Math.pow((points.get(i).getY() - points.get(i + 1).getY()), 2) + Math.pow((points.get(i).getX() - points.get(i + 1).getX()), 2)) > 0.05){
                        if(pointsOnLine >= 10){
                            indexOfEnd = i;
                            foundLine = true;
                            end.setX(points.get(i).getX());
                            end.setY(points.get(i).getY());
                            break;
                        }
                        else{
                            searchingForStart = true;
                            searchingForEnd = false;
                            pointsOnLine = 0;
                        }
                    }
                }
            }
            if(pointsOnLine >= 10 && indexOfEnd < points.size()){
                //TODO: implement point class to return array of start and end point, not index
                startAndEnd[0] = start;
                startAndEnd[1] = end;
                SmartDashboard.putNumber("Start Point X", startAndEnd[0].getX());
                SmartDashboard.putNumber("Start Point Y", startAndEnd[0].getY());
                SmartDashboard.putNumber("End Point X", startAndEnd[1].getX());
                SmartDashboard.putNumber("End Point Y", startAndEnd[1].getY());
                SmartDashboard.putBoolean("Found Line", true);
                System.out.println("Lidar slope " + getSlope());
                System.out.println("Angles to rotate " + Math.atan(getSlope()));
                return startAndEnd;
            }
            return null;
        }
        return null;
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
        // for(int i = 0; i < received.length; i++){
        //     System.out.println(String.format("0x%02x", received[i]));
        // }
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
        for(int i = 0; i < numScansToRead; i ++){
            offset = i * bytesPerScan;
            recordScan = true;
           if((rawData[offset] & 0x003) == 1){
                if(writeToOne){
                    //done writing to one, switching to writing to two - sets the timestamp for array two
                    two.clear();
                    arrayTwoTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    writeToOne = false;
                    if(arrayTwoFilled && getLidarArray() != null){
                        findLineSegment(lidarRANSAC());
                    }
                }
                else {
                    //done writing to two, switching to writing to one - sets the timestamp for array one
                    one.clear();
                    arrayOneTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    arrayTwoFilled = true;
                    writeToOne = true;
                    if(getLidarArray() != null){
                        findLineSegment(lidarRANSAC());
                    }
                }
            } 
            
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
        if(serialPort == null){
            return;
        }
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
        }

    public int getTimesArraySwitch(){
        return numTimesLidarArraySwitch;
    }

    public double getRange(){
        return getLidarArray().get(0).getRange();
    }

    public double getAngle(){
        return getLidarArray().get(0).getAngle();
    }
    
    public double getQuality(){
        return getLidarArray().get(0).getQuality();
    }

    public double getXVal(){
        return getLidarArray().get(0).getX();
    }
    
    public double getYVal(){
        return getLidarArray().get(0).getY();
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
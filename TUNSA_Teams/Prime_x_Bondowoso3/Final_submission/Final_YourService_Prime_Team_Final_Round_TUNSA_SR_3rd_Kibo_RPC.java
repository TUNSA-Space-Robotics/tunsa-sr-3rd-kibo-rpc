package jp.jaxa.iss.kibo.rpc.indonesia;
import android.util.Log;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static java.lang.Math.*;
// opencv library
import java.util.ArrayList;
import java.util.List;
// java library


public class YourService extends KiboRpcService {
    int photoNbr = 0;
    double xCenter, yCenter;
    double maxY = 0, minY = 10000, minX = 10000, maxX = 0;
    Mat camMatrix, dstMatrix;
    @Override

    protected void runPlan1() {
        // the mission starts
        api.startMission ();

        Point Target1 = new Point (10.71000, - 7.777479200, 4.48000);
        Quaternion qTarget1 = new Quaternion (0f, 0.707f, 0f, 0.707f);

        go(Target1, qTarget1, true);
        api.reportPoint1Arrival ( );

        api.laserControl (true);
        api.takeTarget1Snapshot ( );
        api.laserControl (false);

        Point Move2 = new Point (11.2, - 9.5111000, 4.5);
        Quaternion qMove2 = new Quaternion (0.0000f, 0.0000f, - 0.70700f, 0.70700f);

        go(Move2, qMove2, true);
        double[] dimanskuy = posit();
        if (dimanskuy[1] < - 9.92284 + 0.2) {
            Log.i ("way", "maaanjiingg dimanskuy");
            Point Move2_1 = new Point (11.3, -9.5111000, 4.5);
            Quaternion qMove2_1 = new Quaternion (0.0000f, 0.0000f, -0.70700f, 0.70700f);

            go(Move2_1, qMove2_1, true);
        }

        final double x = 11.27460 - 0.066332362 - 0.0008892719 - 3.25886e-05, y = - 9.92284, z = 5.492200 + 0.0014507618 + 0.0001393025;
        Point Target2_1 = new Point (x, y, z);
        Quaternion qTarget2_1 = new Quaternion (0f, 0f, - 0.707f, 0.707f);

        go(Target2_1, qTarget2_1, true);

        sleepFunction (7000);
        api.laserControl (true);
        dealWithCenter (x-0.027, y, z-0.0106, 2839, 1811, 0.00026760263125, 0.00026760263125); //npl ori

        if(xCenter == 0 || yCenter == 0){
            Log.i ("way", "mletre salapan");
            Point pipip = new Point (x, y, z+0.1);
            go(pipip, qTarget2_1, true);
            dealWithCenter (x-0.027, y, z-0.0106, 2839, 1811, 0.00026760263125, 0.00026760263125); //npl ori
        }

        if(xCenter == 0 || yCenter == 0){
            Log.i ("way", "mletre tilu");
            Point popop = new Point (x, y, z-0.2);
            go(popop, qTarget2_1, true);
            dealWithCenter (x-0.027, y, z-0.0106, 2839, 1811, 0.00026760263125, 0.00026760263125); //npl ori
        }

        api.takeTarget2Snapshot ();
        api.laserControl (false);

        Point Moveb1 = new Point (10.82150, -9.6511000, 5.4615);
        Quaternion qMoveb1 = new Quaternion (0.0000f, 0.0000f, -0.70700f, 0.70700f);
        go(Moveb1, qMoveb1, true);

        Point point2 = new Point (11.27460, -7.89178, 4.96538);
        Quaternion quaternion2 = new Quaternion (0f, 0f, -0.707f, 0.707f);
        go(point2, quaternion2, true);

        api.reportMissionCompletion ();
    }

    private void sleepFunction(long delay) {
        try {
            Thread.sleep (delay);
        } catch (Exception e) {
        }
    }

    private void go(Point p ,Quaternion q , boolean b) {
        Result result;
        int LOOP=5;
        result=api.moveTo(p,q,b);
        int i=0;

        while(!result.hasSucceeded() && i< LOOP){
            result = api.moveTo(p,q,b);
            i++;

        }
    }


    private void printArucoData(int ar, List<Mat> corners) {

        int x1=(int) corners.get (ar).get (0, 0)[0];
        int y1=(int) corners.get (ar).get (0, 0)[1];
        Log.i ("---", "X1 = " + x1 + "Y1 = "+ y1);

        if (x1 < minX){
            minX = x1;
        }if (x1 > maxX){
            maxX = x1;
        }

        if (y1 < minY){
            minY = y1;
        }if (y1 > maxY){
            maxY = y1;
        }

        // Data of the second corner point
        int x2=(int) corners.get (ar).get (0, 1)[0];
        int y2=(int) corners.get (ar).get (0, 1)[1];
        Log.i ("---", "X2 = " + x2 + "Y2 = "+ y2);

        if (x2 < minX){
            minX = x2;
        }if (x2 > maxX){
            maxX = x2;
        }

        if (y2 < minY){
            minY = y2;
        }if (y2 > maxY){
            maxY = y2;
        }

        // Data of the third corner point
        int x3=(int) corners.get (ar).get (0, 2)[0];
        int y3=(int) corners.get (ar).get (0, 2)[1];
        Log.i ("---", "X3 = " + x3 + "Y3 = "+ y3);

        if (x2 < minX){
            minX = x2;
        }else if (x2 > maxX){
            maxX = x2;
        }

        if (y2 < minY){
            minY = y2;
        }if (y2 > maxY){
            maxY = y2;
        }

        // Data of the fourth corner point
        int x4=(int) corners.get (ar).get (0, 3)[0];
        int y4=(int) corners.get (ar).get (0, 3)[1];
        Log.i ("---", "X4 = " + x4 + "Y4 = "+ y4);

        if (x2 < minX){
            minX = x2;
        }if (x2 > maxX){
            maxX = x2;
        }

        if (y2 < minY){
            minY = y2;
        }if (y2 > maxY){
            maxY = y2;
        }

    }

    private void detectCenterOfTarget() {
        for ( int oo = 0 ; oo < 1 ; oo++ ) {
            getCalibrationParameters ( );
            Mat image2 = api.getMatNavCam ( );

            Imgproc.medianBlur (image2, image2, 3);
            Mat image = new Mat ( );
            Imgproc.undistort (image2, image, camMatrix, dstMatrix);

            Mat resizeImage = new Mat (3840, 5120, image.type ());
            Imgproc.resize (image, resizeImage, resizeImage.size ( ), 0, 0, Imgproc.INTER_CUBIC);

            Mat ids = new Mat ( );

            Dictionary dictionary = Aruco.getPredefinedDictionary (Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<> ( );

            try {
                Aruco.detectMarkers(resizeImage, dictionary, corners, ids);
                if (ids.rows () == 4) {
                    Log.i ("IDDDD", "All 4 ids are found.");
                    for (int i = 0; i < 4; i++) {
                        printArucoData(i, corners);
                    }

                } else {
                    Log.i ("IDDDDD", "--Fail: Only found " + ids.rows ( ) + " markers");
                    break;
                }

                Log.i ("Status:", "Done detecting Aruco Markers");

            } catch (Exception e) {
                Log.i ("AR[status]:", " Not detected");
            }
            Log.i ("ARR", "atas = " + maxY + " bawah = " + minY
                    + " kiri = " + minX + "kanan = " + maxX);



            Mat gray = new Mat ( );

            Mat circles = new Mat ( );

            Imgproc.HoughCircles (resizeImage, circles, Imgproc.HOUGH_GRADIENT, 1,
                    (double) 500,
                    55.5,
                    70,
                    50,
                    150);


            double moyX = 0;
            double moyY = 0;
            int centersLen = 0;


            for (int x = 0; x < circles.cols () ; x++) {
                double[] c = circles.get (0, x);
                int radius = (int) round (c[2]);
                int xAb = (int) round (c[0]);
                int yAb = (int) round (c[1]);

                Log.i ("CEK", "XAB = " + xAb + "YAB = " + yAb);
                Log.i ("CEK", "YMAX = " + maxY + " YMIN = " + minY
                        + " XMIN = " + minX + "XMAX = " + maxX);
                //teg
                if (xAb > minX && xAb < maxX && yAb > minY && yAb < maxY) {
                    moyX += xAb;
                    moyY += yAb;
                    centersLen++;

                    org.opencv.core.Point xx = new org.opencv.core.Point (Math.round (c[0]), Math.round (c[1]));

                    // circle center
                    Imgproc.circle (resizeImage, xx, 1, new Scalar (0, 100, 100), 3, 8, 0);

                    // circle outline
                    int buletan = (int) Math.round (c[2]);
                    Imgproc.circle (resizeImage, xx, buletan, new Scalar (0, 100, 100), 3, 8, 0);
                    Log.i ("Center", "X : " + Integer.toString (xAb) + "  Y : " + Integer.toString (yAb) + "  Radius : " + Integer.toString (radius));
                }
            }

            xCenter = moyX / centersLen;
            yCenter = moyY / centersLen;

            Log.i ("Center Moyenne ", "X CENTER = " + xCenter + "Y CENTER = " + yCenter);

            if (centersLen == 0) {
                break;
            }
            else {

            }

            Log.i ("Center Moyenne ", "X : " + Double.toString (moyX) + "  Y : " + Double.toString (moyY));
        }

    }

    private void dealWithCenter(double x, double y, double z, double xRef, double yRef, double kpX, double kpZ) {

        double xx = x;
        double zz = z;
        double xideal = 0, zideal = 0, minerx = 1000, minerz = 1000;


        boolean xultim = false, yultim = false;


        detectCenterOfTarget ();

        double XError = xCenter - xRef;
        double YError = yCenter - yRef;

        Log.i ("XError", Double.toString (XError));
        Log.i ("YError", Double.toString (YError));

        xx = (kpX * XError) + x;
        zz = (kpZ * YError) + z;
        Log.i ("Supposed X ", Double.toString (xx));
        Log.i ("Supposed Z ", Double.toString (zz));

        //trying to correct pose
        Point newTarget = new Point (xx+0.0002, y, zz-0.000105);
        Point calib = new Point (xx, y+0.15, zz);
        Quaternion quat = new Quaternion (0f, 0f, - 0.707f, 0.707f);
        go(calib, quat, true);
        go(newTarget, quat, true);
    }

    private void resize_save(Mat gambar, String nama) {
        Mat resizeImage = new Mat (960, 1280, gambar.type ( ));
        int interpolation = Imgproc.INTER_CUBIC;
        Imgproc.resize (gambar, resizeImage, resizeImage.size ( ), 0, 0, interpolation);

        api.saveMatImage (resizeImage, nama);
    }

    @Override
    protected void runPlan2() {
        runPlan1();
    }

    @Override
    protected void runPlan3() {
        runPlan1();
    }

    public void getCalibrationParameters() {
        double[][] cameraParameters = api.getNavCamIntrinsics ( );
        double[] cameraMatrix = cameraParameters[0];
        double[] distortionCoeffs = cameraParameters[1];
        camMatrix = new Mat (3, 3, CvType.CV_64FC1);
        dstMatrix = new Mat (1, 5, CvType.CV_64FC1); //64F
        camMatrix.put (0, 0, cameraMatrix[0]);
        camMatrix.put (0, 1, cameraMatrix[1]);
        camMatrix.put (0, 2, cameraMatrix[2]);
        camMatrix.put (1, 0, cameraMatrix[3]);
        camMatrix.put (1, 1, cameraMatrix[4]);
        camMatrix.put (1, 2, cameraMatrix[5]);
        camMatrix.put (2, 0, cameraMatrix[6]);
        camMatrix.put (2, 1, cameraMatrix[7]);
        camMatrix.put (2, 2, cameraMatrix[8]);
        dstMatrix.put (0, 0, distortionCoeffs[0]);
        dstMatrix.put (0, 1, distortionCoeffs[1]);
        dstMatrix.put (0, 2, distortionCoeffs[2]);
        dstMatrix.put (0, 3, distortionCoeffs[3]);
        dstMatrix.put (0, 4, distortionCoeffs[4]);
    }

    private double[] posit() {

        final String TAG = "log_position";
        long end1 = System.currentTimeMillis ( );

        Point p = api.getRobotKinematics ( ).getPosition ( );

        Log.i (TAG, "Position= " + p.getX ( ) + ", " + p.getY ( ) + ", " + p.getZ ( ));
        Log.i (TAG, "time = " + (System.currentTimeMillis ( ) - end1));

        return new double[]{ p.getX ( ), p.getY ( ), p.getZ ( ) };
    }


}

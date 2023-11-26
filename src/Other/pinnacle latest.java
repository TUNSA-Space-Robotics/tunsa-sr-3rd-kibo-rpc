package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
//
import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.ResultPoint;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.detector.FinderPattern;
//
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
//
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    Bitmap gambar, gambar1, gambar2;
    @Override

    protected void runPlan1(){
        // the mission starts
        api.startMission();

        final String TAG = "text";
        // move to a point1  (10.71000, -7.70000, 4.48000)
        Point point1 = new Point(10.71000, -7.90000, 4.48000);
        Quaternion quaternion1 = new Quaternion(0f, 0.707f, 0f, 0.707f) ;
        api.moveTo(point1, quaternion1, true);
        api.reportPoint1Arrival();
        // Tembak Target 1
        Point Target1 = new Point(10.71000, -7.750000, 4.48000);
        Quaternion qTarget1 = new Quaternion(0f, 0.707f, 0f, 0.707f) ;
        api.moveTo(Target1, qTarget1, true);

        api.laserControl(true);
        api.takeTarget1Snapshot();
        api.laserControl(false);

        //OTW Target 2
        Point Move1 = new Point(11, -8.28813, 4.4800);
        Quaternion qMove1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
        api.moveTo(Move1, qMove1, true);

        Point Move2 = new Point(11.2, -9.5111000, 4.4800000);
        Quaternion qMove2 = new Quaternion(0.0000f, 0.0000f, -0.70700f, 0.70700f);
        api.moveTo(Move2, qMove2, true);

        //TEMBAK TARGET 2
        Point Target2 = new Point(11.20535, -10, 5.460700);
        Quaternion qTarget2 = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
        api.moveTo(Target2, qTarget2, true);

        try {
            Thread.sleep(22000);
        } catch (Exception e) {}


        for(int pattern =1; pattern < 8; pattern++) {
            final Mat AR_Center = ar_read(pattern);

            final Mat undistortAr = undistortPoints(AR_Center);
            final double[] laser = {711, 455};
            final double[] angleToTurn = pixelDistanceToAngle(undistortAr.get(0, 0), laser, pattern);
            final Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);
            final Quaternion qToTurn = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
            Log.i(TAG, "LOOP KE-"+pattern);
        }

        Point Target2_1 = new Point(11.20535, -10.1, 5.460700);
        Quaternion qTarget2_1 = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
        api.moveTo(Target2_1, qTarget2_1, true);

        api.laserControl(true);
        Mat gambar2 = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(gambar2, "coba_gambar2.jpeg");
        api.takeTarget2Snapshot();

        api.laserControl(false);

        Log.d(TAG, "CEPHEUS_SNAPDONE");

        //OTW PULANG
        Point Moveb1 = new Point(10.71000, -9.8111000, 5.4615);
        Quaternion qMoveb1 = new Quaternion(0.0000f, 0.0000f, -0.70700f, 0.70700f);
        api.moveTo(Moveb1, qMoveb1, true);

//        Point Moveb2 = new Point(10.71000, -8.28813, 5.4615);
//        Quaternion qMoveb2 = new Quaternion(0.0000f, 0.0000f, -0.70700f, 0.70700f);
//        api.moveTo(Moveb2, qMoveb2, true);

        //Sampe
        Point point2 = new Point(11.27460, -7.89178, 4.96538);
        Quaternion quaternion2 = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
        api.moveTo(point2, quaternion2, true);

        // report point1 arrival
//        api.reportPoint1Arrival();


        // get a camera image
//        Mat image = api.getMatNavCam();

        // irradiate the laser
//        api.laserControl(true);

        // take target1 snapshots
//        api.takeTarget1Snapshot();

        // turn the laser off
//        api.laserControl(false);

        /* ******************************************** */
        /* write your own code and repair the air leak! */
        /* ******************************************** */

        // send mission completion
        api.reportMissionCompletion();
    }

    final int LOOP_MAX = 3;
    /*
    final int NAV_MAX_WIDTH = 1280;
    final int NAV_MAX_HEIGHT = 960;
    */

    final double[] CAM_MATSIM = {
            567.229305, 0.0, 659.077221,
            0.0, 574.192915, 517.007571,
            0.0, 0.0, 1.0
    };

    final double[] DIST_COEFFSIM = {
            -0.216247, 0.03875, -0.010157, 0.001969, 0.0
    };

    class imagePoint {
        float x, y;
        imagePoint(float x, float y) {
            this.x = x;
            this.y = y;
        }

        String dump() {
            return ("[" + x + ", " + y + "]");
        }
    }

    private Mat ar_read(int pattern) {


        final String TAG = "ar_read";
        final long start = System.currentTimeMillis();

        final Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        if (pattern == 7) {
            try {
                Thread.sleep(22000);
            } catch (Exception e) {}
        }
        else if (pattern == 1) {
            try {
                Thread.sleep(24000);
            } catch (Exception e) {}
        }
        else {
            try {
                Thread.sleep(23000);
            } catch (Exception e) {}

        }

        long end = System.currentTimeMillis();
        Log.i(TAG, "sleep_times=" + (end-start));

        int counter = 0;
        Log.i(TAG, "Reading AR");
        while (ids.rows() != 4 && counter < LOOP_MAX) {
            Mat pic = new Mat(api.getMatNavCam(), new Rect(320, 240, 640, 500));
            Aruco.detectMarkers(pic, dict, corners, ids);
            counter++;
        }

        for (int j = 0; j < corners.size(); j++) {
            Mat temp = corners.get(j);

            for (int z = 0; z < temp.cols(); z++) {
                double[] t = temp.get(0, z);
                t[0] = t[0] + 320;
                t[1] = t[1] + 240;
                temp.put(0, z, t);
            }

            Log.i(TAG, "corners[" + j + "]=" + temp.dump());
        }

        Log.i(TAG, "ids= " + ids.dump());
        end = System.currentTimeMillis();
        Log.i(TAG, "ar_read_time=" + (end-start));
        //

        imagePoint[] markersCenter = new imagePoint[4];

        if(ids.rows() == 4) {
            Log.i(TAG, "All 4 ids are found.");
            for (int i = 0; i < 4; i++) {
                markersCenter[i] = findCenterRect(corners.get(i));
                Log.i(TAG, "Marker Center[" + i + "](id: " + ids.get(i, 0)[0] + ")=" + markersCenter[i].dump());
            }

        } else {
            Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
        }

        final Mat AR_Center = findCenterRect(markersCenter[0], markersCenter[1], markersCenter[2], markersCenter[3]);
        Log.i(TAG, "distorted=" + AR_Center.dump());

        end = System.currentTimeMillis();
        Log.i(TAG, "ar_read+process_time=" + (end-start));
        return  AR_Center;
    }

    private Mat undistortPoints(Mat points) {
        final String TAG = "undistortCorner";

        // in -> rows:1, cols:4
        // in -> 1xN 2 Channel
        Log.i(TAG, "Start");

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        Mat out = new Mat(points.rows(), points.cols(), points.type());

        Imgproc.undistortPoints(points, out, cameraMat, distCoeffs, new Mat(), cameraMat);

        Log.i(TAG, "undistort=" + out.dump());
        // out -> 1xN 2 Channel
        return out;
    }

    private Mat findCenterRect(imagePoint p1, imagePoint p2,
                               imagePoint p3, imagePoint p4) {
        float xCenter = (p1.x + p2.x + p3.x + p4.x) / 4.0f;
        float yCenter = (p1.y + p2.y + p3.y + p4.y) / 4.0f;

        Mat out = new Mat(1, 1, CvType.CV_32FC2);
        float[] point = {xCenter, yCenter};
        out.put(0, 0, point);

        return out;
    }

    private imagePoint findCenterRect(Mat corners) {
        double xCenter;
        double yCenter;

        xCenter = (corners.get(0, 0)[0] + corners.get(0, 1)[0] +
                corners.get(0, 2)[0] + corners.get(0, 3)[0]) / 4.0f;

        yCenter = (corners.get(0, 0)[1] + corners.get(0, 1)[1] +
                corners.get(0, 2)[1] + corners.get(0, 3)[1]) / 4.0f;

        return new imagePoint((float)xCenter, (float)yCenter);
    }

    // OTHER MATHS OPERATION

    private double[] pixelDistanceToAngle(double[] target, double[] ref, int pattern) {
        final String TAG = "pixelDistanceToAngle";

        final double xDistance = target[0] - ref[0];
        final double yDistance = ref[1] - target[1];
        //final double anglePerPixel = 130 / Math.sqrt(Math.pow(NAV_MAX_WIDTH, 2) + Math.pow(NAV_MAX_HEIGHT, 2));
        final double anglePerPixel = 0.08125;
        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "anglePerPixel=" + anglePerPixel);

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;

        if (pattern == 7) {
//            xAngle -= 2.2;
        }

        if (pattern == 1 || pattern == 8) {
            xAngle -= 2.2;
            yAngle -= 2.0;
        }

        if (pattern == 2) {
            yAngle -= 2.0;
        }

        if (pattern == 3) {
            xAngle += 0.5;
            yAngle -= 2.0;
        }

        if (pattern == 4) {
            xAngle += 0.4;
            yAngle -= 2.0;
        }

        if (pattern == 5 || pattern == 6) {
            xAngle += 0.5;
        }

        Log.i(TAG, "xAngle=" + xAngle);
        Log.i(TAG, "yAngle=" + yAngle);

        double[] out = {xAngle, yAngle};
        return out;
    }

    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        final String TAG = "Convert euler angle to quaternion";

        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
        double c1 = Math.cos(yAngle/2);
        double c2 = Math.cos(zAngle/2);
        double c3 = Math.cos(xAngle/2);
        double s1 = Math.sin(yAngle/2);
        double s2 = Math.sin(zAngle/2);
        double s3 = Math.sin(xAngle/2);

        double w = c1*c2*c3 - s1*s2*s3;
        double x = s1*s2*c3 + c1*c2*s3;
        double y = s1*c2*c3 + c1*s2*s3;
        double z = c1*s2*c3 - s1*c2*s3;

        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float) y, (float)z, (float)w);
    }

    // For multiply quaternion, Apply q2(new) to q1(old)
    // q1 * q2= a*e - b*f - c*g- d*h + i (b*e + a*f + c*h - d*g) + j (a*g - b*h + c*e + d*f) + k (a*h + b*g - c*f + d*e)
    private Quaternion combineQuaternion(Quaternion newOrientation, Quaternion oldOrientation) {
        String TAG = "combineQuaternion";
        double x =  newOrientation.getX() * oldOrientation.getW() + newOrientation.getY() * oldOrientation.getZ()
                - newOrientation.getZ() * oldOrientation.getY() + newOrientation.getW() * oldOrientation.getX();
        double y = -newOrientation.getX() * oldOrientation.getZ() + newOrientation.getY() * oldOrientation.getW()
                + newOrientation.getZ() * oldOrientation.getX() + newOrientation.getW() * oldOrientation.getY();
        double z =  newOrientation.getX() * oldOrientation.getY() - newOrientation.getY() * oldOrientation.getX()
                + newOrientation.getZ() * oldOrientation.getW() + newOrientation.getW() * oldOrientation.getZ();
        double w = -newOrientation.getX() * oldOrientation.getX() - newOrientation.getY() * oldOrientation.getY()
                - newOrientation.getZ() * oldOrientation.getZ() + newOrientation.getW() * oldOrientation.getW();
        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }







































    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        api.moveTo(point, quaternion, true);
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        api.relativeMoveTo(point, quaternion, true);
    }

}


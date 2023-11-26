package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;


import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import static java.lang.Math.*;
import static org.opencv.android.Utils.matToBitmap;
// opencv library
import java.util.ArrayList;
import java.util.List;
// java library

import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    Bitmap gambar, gambar1, gambar2,target1Mat,target2Mat;
    double fixgolx,fixgoly;

    @Override

    protected void runPlan1(){
        // the mission starts
        api.startMission();

        final String TAG = "text";

        // Tembak Target 1
        Point Target1 = new Point(10.71000, -7.750000, 4.48000);
        Quaternion qTarget1 = new Quaternion(0f, 0.707f, 0f, 0.707f) ;
        api.moveTo(Target1, qTarget1, true);
        api.reportPoint1Arrival();

//        analyseAruco();
//        detectCenterOfTarget();

        api.laserControl(true);

        api.takeTarget1Snapshot();


        api.laserControl(false);

        //OTW Target 2
//        Point Move1 = new Point(11, -8.28813, 4.4800);
//        Quaternion qMove1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
//        api.moveTo(Move1, qMove1, true);

        Point Move2 = new Point(11.2, -9.5111000, 4.8);
        Quaternion qMove2 = new Quaternion(0.0000f, 0.0000f, -0.70700f, 0.70700f);
        api.moveTo(Move2, qMove2, true);

        //TEMBAK TARGET 2
        /*Point Target2 = new Point(11.20535, -10, 5.460300);
        Quaternion qTarget2 = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
        api.moveTo(Target2, qTarget2, true);*/

        Point Target2_1 = new Point(11.20535, -10.1, 5.560300);
        Quaternion qTarget2_1 = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
        api.moveTo(Target2_1, qTarget2_1, true);
        Mat gambar = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(gambar, "coba_gambar.jpeg");

        try {
            Thread.sleep(10000);
        } catch (Exception e) {}

        Log.i(TAG, "Masuk 1");
        bismillah_bunder();
        Log.i(TAG, "Masuk 2");
        final double[] laser = {768, 459};
        final double[] angleToTurn = bismillah_Ngepas(fixgolx , fixgoly , laser);
        Log.i(TAG, "Masuk 3");
        final Quaternion imageQ = bismillah_tengah(angleToTurn[1], 0, angleToTurn[0]);
        Log.i(TAG, "Masuk 4");
        final Quaternion bismillah_headshoot  = combineQuaternion(imageQ, new Quaternion(0, 0, -0.707f, 0.707f));
        Log.i(TAG, "Masuk 5");

        api.moveTo(Target2_1, bismillah_headshoot, true);
        try {
            Thread.sleep(5000);
        } catch (Exception e) {}

        api.laserControl(true);
        Mat tt = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(tt, "coba_gambar1.jpeg");
        api.takeTarget2Snapshot();

        api.laserControl(false);
        Mat yy = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(yy, "coba_gambar2.jpeg");

        Log.i(TAG, "CEPHEUS_SNAPDONE");NAV_MAX_WIDTH

//        OTW PULANG
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


        // send mission completion
        api.reportMissionCompletion();
    }

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

    private Quaternion bismillah_tengah(double xAngle, double yAngle, double zAngle) {
        final String TAG = "Convert euler angle to quaternion";

        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
        double c1 = Math.cos(yAngle/2);
        double s1 = Math.sin(yAngle/2);
        double c2 = Math.cos(zAngle/2);
        double s2 = Math.sin(zAngle/2);
        double c3 = Math.cos(xAngle/2);
        double s3 = Math.sin(xAngle/2);
        double w = c1*c2*c3 - s1*s2*s3;
        double x = s1*s2*c3 + c1*c2*s3;
        double y = s1*c2*c3 + c1*s2*s3;
        double z = c1*s2*c3 - s1*c2*s3;

        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float) y, (float)z, (float)w);
    }

    private double[] bismillah_Ngepas(double targetX,double targetY, double[] ref) {
        final String TAG = "pixelDistanceToAngle";

        final double xDistance = targetX - ref[0];
        final double yDistance = ref[1] - targetY;
        //final double anglePerPixel = 130 / Math.sqrt(Math.pow(NAV_MAX_WIDTH, 2) + Math.pow(NAV_MAX_HEIGHT, 2));
        final double anglePerPixel = 0.08125;
        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "anglePerPixel=" + anglePerPixel);

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;



        Log.i(TAG, "xAngle=" + xAngle);
        Log.i(TAG, "yAngle=" + yAngle);

        double[] out = {xAngle, yAngle};
        return out;
    }


    private void bismillah_bunder(){

        double TengahX=0;
        double TengahY=0;
        int mbuh=0;
        Mat bunder = new Mat();

        Mat img= api.getMatNavCam();

        Imgproc.medianBlur(img, img, 5);

        Imgproc.HoughCircles(img, bunder, Imgproc.HOUGH_GRADIENT,
                1,
                (double)1,
                100,
                30,
                1,
                100);





        for (int x = 0; x < bunder.cols(); x++) {

            double[] data = bunder.get(0, x);

            int xTuru = (int) round(data[0]);
            int yTuru = (int) round(data[1]);

            if(xTuru>700){

                Log.i("Tengah"," Nilai X : " + xTuru + " Nilai Y : " + yTuru);

                TengahX += xTuru;
                TengahY += yTuru;
                mbuh++;
            }

        }

        fixgolx = TengahX / mbuh;
        fixgoly = TengahY / mbuh;
        Log.i("TITIK "," Nilai X :  : "+fixgolx+"  Nilai Y : "+fixgoly);
        



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
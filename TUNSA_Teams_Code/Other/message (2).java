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
    int maxX,minX,maxY,minY;
    int photoNbr=0;
    double xCenter,yCenter;
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

        //analyseAruco();
        //detectCenterOfTarget();

        api.laserControl(true);

        api.takeTarget1Snapshot();


        api.laserControl(false);

        //OTW Target 2
//        Point Move1 = new Point(11, -8.28813, 4.4800);
//        Quaternion qMove1 = new Quaternion(0f, 0.707f, 0f, 0.707f);
//        api.moveTo(Move1, qMove1, true);

        Point Move2 = new Point(11.2, -9.5111000, 4.4800000);
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
        api.saveMatImage(gambar, "coba_gambar1.jpeg");

        try {
            Thread.sleep(10000);
        } catch (Exception e) {}


        dealWithCenter(11.20535, -10.1, 5.560300,768.26797,459,0.0008,1);


        api.laserControl(true);
        Mat tt = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(tt, "coba_gambar1.jpeg");
        api.takeTarget2Snapshot();

        api.laserControl(false);
        Mat yy = api.getMatNavCam();

        Log.i(TAG, "BONDOWOSO3 LAGI SIMPEN GAMBAR");
        api.saveMatImage(yy, "coba_gambar2.jpeg");

        Log.i(TAG, "CEPHEUS_SNAPDONE");

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


        // send mission completion
        api.reportMissionCompletion();
    }

//    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
//                               double qua_x, double qua_y, double qua_z,
//                               double qua_w){
//
//        final Point point = new Point(pos_x, pos_y, pos_z);
//        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
//                (float)qua_z, (float)qua_w);
//
//        api.moveTo(point, quaternion, true);
//    }
//
//    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
//                                       double qua_x, double qua_y, double qua_z,
//                                       double qua_w) {
//
//        final Point point = new Point(pos_x, pos_y, pos_z);
//        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
//                (float) qua_z, (float) qua_w);
//
//        api.relativeMoveTo(point, quaternion, true);
//    }

    private void analyseAruco(){
        photoNbr++;
        Mat source = api.getMatNavCam();
        api.saveMatImage(source, "target"+Integer.toString(photoNbr)+".jpeg");
        Mat ids = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        try
        {

            Aruco.detectMarkers(source, dictionary, corners, ids);

            minX=0;
            minY=10000;
            maxX=10000;
            maxY=0;

            printArucoData(0,ids,corners);
            Log.i("---","-----");

            printArucoData(1,ids,corners);
            Log.i("---","-----");

            printArucoData(2,ids,corners);
            printArucoData(3,ids,corners);


            Log.i("Status:", "Done detecting Aruco Markers");
        }
        catch (Exception e)
        {
            Log.i("AR[status]:", " Not detected");
        }

    }
    private void printArucoData(int id , Mat ids , List<Mat>  corners){

        //aruco IDs
        Log.i("---","---");

        int idMarker=(int) ids.get(id, 0)[0];
        Log.i("---","---");
//        int id2=(int) ids.get(0, 1)[0];
//        int id3=(int) ids.get(0, 2)[0];
//        int id4=(int) ids.get(0, 3)[0];

        // Data of the first corner point

        int x1=(int) corners.get(id).get(0, 0)[0];
        Log.i("---","---");

        int y1=(int) corners.get(id).get(0, 0)[1];
        Log.i("---","---");

        // Data of the second corner point

        int x2=(int) corners.get(id).get(0, 1)[0];
        Log.i("---","---");

        int y2=(int) corners.get(id).get(0, 1)[1];
        Log.i("---","---");

        // Data of the third corner point

        int x3=(int) corners.get(id).get(0, 2)[0];
        int y3=(int) corners.get(id).get(0, 2)[1];

        // Data of the fourth corner point

        int x4=(int) corners.get(id).get(0, 3)[0];
        int y4=(int) corners.get(id).get(0, 3)[1];

        if(idMarker==12 || idMarker==13 || idMarker==2 || idMarker==3){
            if(x1>minX){
                minX=x1;}
            if(x2>minX){
                minX=x2;}
            if(x3>minX){
                minX=x3;}
            if(x4>minX){
                minX=x4;}
        }
        if(idMarker==11 || idMarker==14 || idMarker==1 || idMarker==4) {
            if(x1<maxX){
                maxX=x1;}
            if(x2<maxX){
                maxX=x2;}
            if(x3<maxX){
                maxX=x3;}
            if(x4<maxX){
                maxX=x4;}

        }


        if(y1<minY){
            minY=y1;
        }else if(y1>maxY){
            maxY=y1;
        }
        if(y2<minY){
            minY=y2;
        }else if(y2>maxY){
            maxY=y2;
        }
        if(y3<minY){
            minY=y3;
        }else if(y3>maxY){
            maxY=y3;
        }
        if(y4<minY){
            minY=y4;
        }else if(y4>maxY){
            maxY=y4;
        }
        Log.i("---","---");

        Log.i("Aruco Data","Aruco Aruco Aruco Aruco Aruco Aruco Aruco Aruco");
        Log.i("ID_Marker",Integer.toString(idMarker));
        Log.i("First point in corner","  X : "+Integer.toString(x1)+"  Y : "+Integer.toString(y1));
        Log.i("Second point in corner","  X : "+Integer.toString(x2)+"  Y : "+Integer.toString(y2));
        Log.i("Third point in corner","  X : "+Integer.toString(x3)+"  Y : "+Integer.toString(y3));
        Log.i("Fourth point in corner","  X : "+Integer.toString(x4)+"  Y : "+Integer.toString(y4));
        Log.i("Aruco Data","Aruco Aruco Aruco Aruco Aruco Aruco Aruco Aruco");

    }
    private void detectCenterOfTarget(){
        Mat image= api.getMatNavCam();
        Mat gray = new Mat();
        Log.i("Circles"," Circles Circles Circles Circles Circles Circles Circles Circles ");

//        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        Log.i("Circles"," Circles Circles Circles Circles Circles Circles Circles Circles ");

        Imgproc.medianBlur(image, image, 5);
        Log.i("Circles"," Circles Circles Circles Circles Circles Circles Circles Circles ");

        Mat circles = new Mat();
        Log.i("Circles"," Circles Circles Circles Circles Circles Circles Circles Circles ");

        Imgproc.HoughCircles(image, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)1,100.0, 30.0, 1, 100);
        Log.i("Circles"," Circles Circles Circles Circles Circles Circles Circles Circles ");
        double moyX=0;
        double moyY=0;
        int centersLen=0;

        Log.i("Area Circle","minX : " + Integer.toString(minX) + "  maxX : " + Integer.toString(maxX) + "minY : " + Integer.toString(minY) + "maxY : " + Integer.toString(maxY));


        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            int radius = (int) round(c[2]);
            int xAb = (int) round(c[0]);
            int yAb = (int) round(c[1]);

            if(xAb > minX && xAb < maxX && yAb > minY && yAb < maxY){
                moyX+=xAb;
                moyY+=yAb;
                centersLen++;

                Log.i("Center","X : "+Integer.toString(xAb)+"  Y : "+Integer.toString(yAb) +"  Radius : "+Integer.toString(radius ));

            }

        }

        Log.i("Center Moyenne ","centersLen = "+ centersLen);
        if(centersLen==0){

            xCenter=-1;
            yCenter=-1;
        }else{
            moyX = moyX / centersLen;
            moyY = moyY / centersLen;
            xCenter = moyX;
            yCenter = moyY;
        }
        Log.i("Center Moyenne ","X : "+Double.toString(moyX)+"  Y : "+Double.toString(moyY));



    }
    private void dealWithCenter
            (double x,double y, double z,double xRef,double yRef,double kp,int numberTry){

        double xx = x;
        double zz = z;
        int tryyy = 0;
        double XError = 1000;
        double YError = 1000;
        while(tryyy < numberTry && (abs(XError) >5 || abs(YError) > 5)){
            tryyy++;
            analyseAruco();
            detectCenterOfTarget();
            if(xCenter==-1 || yCenter==-1){
                XError = 10000;
                YError = 10000;
                continue;
            }
            XError=this.xCenter-xRef;
            YError=this.yCenter-yRef;
            Log.i("Correction try",Integer.toString(tryyy));
            Log.i("XError",Double.toString(XError));
            Log.i("YError",Double.toString(YError));
            xx = xx+kp*XError;
            zz = zz+kp*YError;
            xx = xx-(0.005/8*5.5); //nopal
            Log.i("ZZ sebelum : ",Double.toString(zz));
//            zz = zz+0.005;
            Log.i("ZZ sesudah : ",Double.toString(zz));
//            xx = xx+0.005;
            //trying to correct pose
            Point newTarget = new Point( xx ,y, zz );
            Quaternion quat = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
            api.moveTo(newTarget, quat, true);

            try {
                Thread.sleep(10000);
            } catch (Exception e) {}


            //regle de trois
            /*double MXERROR=0.175*XError/(maxX-minX);
            double MYERROR=0.133*YError/(maxY-minY);

            Point intermTagrget = new Point(x-0.2,y, z-0.2);
            Point exactTagrget = new Point(x+MXERROR,y, z+MYERROR);
            Quaternion quat = new Quaternion(0f, 0f, -0.707f, 0.707f) ;
            api.moveTo(intermTagrget, quat, true);
            api.moveTo(exactTagrget, quat, true);
            api.moveTo(exactTagrget, quat, true);*/


        }


    }

    private Quaternion toQuaternion(float x ,float y, float z){




        float c1 = (float) Math.cos(z/2.0);
        float s1 = (float) Math.sin(z/2.0);
        float c2 = (float) Math.cos(y/2.0);
        float s2 = (float) Math.sin(y/2.0);
        float c3 = (float) Math.cos(x/2.0);
        float s3 = (float) Math.sin(x/2.0);
        float c1c2 = c1*c2;
        float s1s2 = s1*s2;

        Log.i("Quaternion after conversion == > ","X: "+Double.toString(c1c2*c3 - s1s2*s3)
                +"Y: "+Double.toString(c1c2*s3 + s1s2*c3)
                +"Z: "+Double.toString(s1*c2*c3 + c1*s2*s3)
                +"W: "+Double.toString(c1*s2*c3 - s1*c2*s3));

        return new Quaternion(c1c2*c3 - s1s2*s3,c1c2*s3 + s1s2*c3,s1*c2*c3 + c1*s2*s3,c1*s2*c3 - s1*c2*s3);
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


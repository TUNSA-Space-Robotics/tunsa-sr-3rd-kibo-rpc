package jp.jaxa.iss.kibo.rpc.thailand;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.os.SystemClock;
import android.util.Log;
// android library

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1() {



        // irradiate the laser and take snapshots
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);

    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    public void alignLaser(Point3 target){
        double[] robot_pos = new double[]{10.71000D, -7.70000D, 4.48000D};
        Point3 laser = new Point3 (0.1302D, 0.0572D, -0.1111D);
        //Point3 Rcenter = new Point3 (0D,0D,0D);
        double r1 = 0.1711585522;
        double pivot_laser_target_angle = 139.525787D;

        // law of cosines: c2=a2+b2﹣2abcosγ
        // γ = pivot_laser_target_angle
        // a = R2 (unknown)
        // b = R1
        // c= pivot_to_target_length
        double[] current_orientation = {-90, 0, 0};
        double[] target_point = new double[2];
        target_point[0] = 0.785;
//        target_point[1] = target_pos[1] + 0.0826;
        target_point[1] = target.y + 0.0926;
        Log.i("AR[target_point(xy plane)]", target_point[0] + ", " + target_point[1]);

        // square root(delta x square + delta y square)
        double pivot_to_target_length = Math.sqrt(Math.pow(target_point[0], 2) + Math.pow(target_point[1], 2));
        Log.i("AR[pivot_to_target_length]", "" + pivot_to_target_length);

        double a = 1;
        double b = 2 * r1 * Math.cos(Math.toRadians(180 - pivot_laser_target_angle));
        double c = Math.pow(r1, 2) - Math.pow(pivot_to_target_length, 2);
        double r2 = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / 2 * a;

        Log.i("AR[radius]", r1 + ", " + r2);

        double[] laser_shooting_coord = find_laser_point(target_point, r1, r2);
        double laser_origin_to_shooting_length = Math.sqrt(Math.pow(laser.x - laser_shooting_coord[0], 2) + Math.pow(laser.y - laser_shooting_coord[1], 2));

        Log.i("AR[laser_shooting_coord]", laser_shooting_coord[0] + ", " + laser_shooting_coord[1]);
        Log.i("AR[laser_origin_to_shooting_length]", "" + laser_origin_to_shooting_length);

        double pitch = 2 * Math.toDegrees(Math.asin((0.5 * laser_origin_to_shooting_length) / r1));
        Log.i("AR[pitch]", "" + pitch);
        if (target_point[1] != 0 && pitch < 45) {
            current_orientation[1] = current_orientation[1] - pitch;
        } else {
            //Just to be safe
            current_orientation[1] = current_orientation[1] - 30;
        }

//        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.0994);
//        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.092);
        robot_pos[0] = robot_pos[0] + (target.x - 0.1094);
        Log.i("AR[Robot Position]", robot_pos[0] + ", " + current_orientation[1]);
        Quaternion q = eulerToQuaternion( current_orientation[0], current_orientation[1], current_orientation[2]);
        result = moveToWrapper(robot_pos[0],robot_pos[1],robot_pos[2], q.getX(), q.getY(), q.getZ(), q.getW());
    }


    private double[] calTargetPos() {
        long start_time = SystemClock.elapsedRealtime();
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);

        Mat ids = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        double xdiff = 0, ydiff = 0;

        Log.d("AR[status]:", "start");
        byte ar_count = 0;
        Rect ar_roi;
        while (ids.size().height != 4 && ar_count < 3) {
            try {
                if (ar_count == 0) {
                    ar_roi = new Rect(470, 680, 360, 280);
                }
                //fall back if the aruco tag is outside of cropped boundary.
                else {
                    ar_roi = new Rect(100, 660, 1080, 300);
                }
                Mat source = new Mat(api.getMatNavCam(), ar_roi);
                Aruco.detectMarkers(source, dictionary, corners, ids);
                for (int i = 0; i < corners.size(); i++) {
                    //Get shifted aruco tag corners
                    Mat corrected_corner = corners.get(i);
                    //Shift it by the position that it get cropped.
                    corrected_corner.put(0, 0, corrected_corner.get(0, 0)[0] + ar_roi.x, corrected_corner.get(0, 0)[1] + ar_roi.y);
                    corrected_corner.put(0, 1, corrected_corner.get(0, 1)[0] + ar_roi.x, corrected_corner.get(0, 1)[1] + ar_roi.y);
                    corrected_corner.put(0, 2, corrected_corner.get(0, 2)[0] + ar_roi.x, corrected_corner.get(0, 2)[1] + ar_roi.y);
                    corrected_corner.put(0, 3, corrected_corner.get(0, 3)[0] + ar_roi.x, corrected_corner.get(0, 3)[1] + ar_roi.y);
                    //tmp mat to store undistorted corners.
                    Mat tmp = new Mat(1, 4, CvType.CV_32FC2);
                    //undistort the corners.
                    Imgproc.undistortPoints(corners.get(i), tmp, cameraMatrix, dstMatrix, new Mat(), cameraMatrix);
                    //put it back in to the same array list.
                    corners.set(i, tmp);
                }
            } catch (Exception e) {
                Log.e("AR[status]:", "error", e);
            }
            ar_count++;
        }
        if (ids.size().height == 4) {
            float markerSize = 0.05f;
            double avg_ar_size = 0;
            double tx_undistort = 0, ty_undistort = 0;
            for (Mat corner : corners) {
                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corner.size().width; j++) {
                    _x = _x + corner.get(0, j)[0];
                    _y = _y + corner.get(0, j)[1];
                }
                avg_ar_size += Math.abs(corner.get(0, 0)[0] - corner.get(0, 1)[0]);
                avg_ar_size += Math.abs(corner.get(0, 2)[0] - corner.get(0, 3)[0]);
                avg_ar_size += Math.abs(corner.get(0, 0)[1] - corner.get(0, 3)[1]);
                avg_ar_size += Math.abs(corner.get(0, 1)[1] - corner.get(0, 2)[1]);
                tx_undistort += _x / 4.0;
                ty_undistort += _y / 4.0;
            }
            tx_undistort /= 4;
            ty_undistort /= 4;
            avg_ar_size /= 16;
            double pixelPerM = avg_ar_size / markerSize;
            Log.d("AR[pixelperM]", "" + pixelPerM);
            //find diff from the center of the image
            xdiff = (tx_undistort - 640) / pixelPerM;
            ydiff = (480 - ty_undistort) / pixelPerM;
        }
        long stop_time = SystemClock.elapsedRealtime();
        Log.d("AR[count]", "" + ar_count);
        Log.d("AR[total_time]:", " " + (stop_time - start_time));
        Log.d("AR[target_pos]", xdiff + ", " + ydiff);
        return new double[]{xdiff, ydiff};
    }


}
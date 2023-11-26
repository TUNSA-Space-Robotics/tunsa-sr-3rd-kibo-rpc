
package imageProcessing;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.Rect;

import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import java.lang.Math;

public class imgProcessing {
	public static Mat processedImg;
	public static Mat originalImg;
	public Mat processedCircleImg;
	public static Mat threshImg;
	public static Mat grayImg;
	public Mat sharpenImg;
	private static int kernelSize = 3;
	private static Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,
			new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new Point(kernelSize, kernelSize));
	public Mat cropped_img;
	public Mat warped_img;
	public Point text_position;
	public Rect target_rect;
	public Mat inv_rvec;
	public Mat inv_tvec;

	private static Point[] sortingPoints(MatOfPoint pts, int x, int y) {
		Point[] sortedPoints = new Point[4];
		double data[];
		for (int i = 0; i < pts.size().height; i++) {
			data = pts.get(i, 0);
			double datax = data[0];
			double datay = data[1];
//		    0-------1
//		    |		|
//		    |  x,y  |
//		    |		|
//		    2-------3
			if (datax < x && datay < y) {
				sortedPoints[0] = new Point(datax, datay);
			} else if (datax > x && datay < y) {
				sortedPoints[1] = new Point(datax, datay);
			} else if (datax < x && datay > y) {
				sortedPoints[2] = new Point(datax, datay);
			} else if (datax > x && datay > y) {
				sortedPoints[3] = new Point(datax, datay);
			}
		}
		return sortedPoints;

	}

	private static Mat sharpeningImg(Mat src) {
		Mat dst = new Mat(src.rows(), src.cols(), src.type());
		Imgproc.medianBlur(src, dst, 7);
		Core.subtract(src, dst, dst);
		Core.add(dst, src, dst);
		return dst;
	}

	public void initProcImg(Mat img) {
		processedImg = new Mat(img.rows(), img.cols(), img.type());
		originalImg = new Mat(img.rows(), img.cols(), img.type());
		img.copyTo(processedImg);
		img.copyTo(originalImg);
	}

	public void find_paper(Mat img, List<Point> src_pts) {

//		img.copyTo(processedImg);
		List<Integer> list_x = new ArrayList<Integer>();
		List<Integer> list_y = new ArrayList<Integer>();

		for (int i = 0; i < 4; i++) {
			Imgproc.circle(processedImg, src_pts.get(i), 5, new Scalar(255, 0, 0), -1);
			list_x.add((int) src_pts.get(i).x);
			list_y.add((int) src_pts.get(i).y);
		}
		Collections.sort(list_x);
		Collections.sort(list_y);

		double max_w = list_x.get(3) - list_x.get(0);
		double max_h = list_y.get(3) - list_y.get(0);
//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3		
		MatOfPoint2f dst_pts = new MatOfPoint2f(new Point(max_w - 1, 0), new Point(0, 0), new Point(0, max_h - 1),
				new Point(max_w - 1, max_h - 1));
		MatOfPoint2f _pts = new MatOfPoint2f();
		_pts.fromList(src_pts);
		cropped_ROI(originalImg, _pts);

		Mat perspective_tf = Imgproc.getPerspectiveTransform(_pts, dst_pts);
		warped_img = new Mat();
		Imgproc.warpPerspective(originalImg, warped_img, perspective_tf, new Size(max_w, max_h), Imgproc.INTER_LINEAR);
	}

	public void cropped_ROI(Mat img, MatOfPoint2f _pts) {
		target_rect = Imgproc.boundingRect(_pts);
		cropped_img = new Mat();
		cropped_img = img.submat(target_rect);
	}
}

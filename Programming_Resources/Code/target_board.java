
package imageProcessing;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

public class target_board {

	private List<Mat> objPoints1;
	private List<Mat> objPoints2;
	private MatOfInt board1ID;
	private MatOfInt board2ID;

	private List<MatOfPoint3f> offset_c;

	public List<MatOfPoint3f> get_offset() {
		return offset_c;

	}

	public List<Mat> getObjP1() {
		return objPoints1;
	}

	public MatOfInt getBoard1ID() {
		return board1ID;
	}

	public List<Mat> getObjP2() {
		return objPoints2;
	}

	public MatOfInt getBoard2ID() {
		return board2ID;
	}

	double multiplyMatricesCell(double[][] firstMatrix, double[][] secondMatrix, int row, int col) {
		double cell = 0;
		for (int i = 0; i < secondMatrix.length; i++) {
			cell += firstMatrix[row][i] * secondMatrix[i][col];
		}
		return cell;
	}

	double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
		double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

		for (int row = 0; row < result.length; row++) {
			for (int col = 0; col < result[row].length; col++) {
				result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
			}
		}
		return result;
	}

	public void find_ROI3D(Mat rvec, Mat tvec) {

		List<Point3> global_corner = new ArrayList<>();

		// define paper_corner offset from its center

		Mat rot = new Mat();
		Calib3d.Rodrigues(rvec, rot);

//		for(int i = 0;i<3;i++) {
//			for(int j=0;j<3;j++) {
//				System.out.print(rot.get(i, j)[0]+" ");	
//			}
//			System.out.println();
//		}
		// find global offset

//		double[][] offset_corner = {
//				  new double[]{0.135f,-0.135f,-0.135f,0.135f},
//				  new double[]{0.075f,0.075f ,-0.075f,-0.075f},
//				  new double[]{0f    ,0f     ,0f     ,0f},
//				};
		double[][] offset_corner = { new double[] { 0.075f, -0.075f, -0.075f, 0.075f },
				new double[] { 0.0625f, 0.0625f, -0.0625f, -0.0625f }, new double[] { 0f, 0f, 0f, 0f }, };

		double[][] rotationMatrix = { new double[] { rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0] },
				new double[] { rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0] },
				new double[] { rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0] } };

		double[][] global_offset = multiplyMatrices(rotationMatrix, offset_corner);

		// c1------c0
		// | |
		// | |
		// c2------c3
//		for(int i = 0;i<3;i++) { //x,y,z
//			for(int j=0;j<4;j++) { //c0,c1,c2,c3
//				System.out.print(global_offset[i][j]+" ");	
//			}
//			System.out.println();
//		}
		// ex. global_offset[0][0] = c0 x
		// global_offset[1][0] = c0 y
		// global_offset[2][0] = c0 z

		// ex. global_offset[0][1] = c1 x
		// global_offset[1][1] = c1 y
		// global_offset[2][1] = c1 z

		Point3 tar_c0 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][0],
				(double) tvec.get(1, 0)[0] + global_offset[1][0], (double) tvec.get(2, 0)[0] + global_offset[2][0]);

		Point3 tar_c1 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][1],
				(double) tvec.get(1, 0)[0] + global_offset[1][1], (double) tvec.get(2, 0)[0] + global_offset[2][1]);

		Point3 tar_c2 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][2],
				(double) tvec.get(1, 0)[0] + global_offset[1][2], (double) tvec.get(2, 0)[0] + global_offset[2][2]);

		Point3 tar_c3 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][3],
				(double) tvec.get(1, 0)[0] + global_offset[1][3], (double) tvec.get(2, 0)[0] + global_offset[2][3]);

		MatOfPoint3f offset_c0 = new MatOfPoint3f();
		offset_c0.fromArray(tar_c0);

		MatOfPoint3f offset_c1 = new MatOfPoint3f();
		offset_c1.fromArray(tar_c1);

		MatOfPoint3f offset_c2 = new MatOfPoint3f();
		offset_c2.fromArray(tar_c2);

		MatOfPoint3f offset_c3 = new MatOfPoint3f();
		offset_c3.fromArray(tar_c3);

		offset_c = new ArrayList<MatOfPoint3f>();
		offset_c.add(offset_c0);
		offset_c.add(offset_c1);
		offset_c.add(offset_c2);
		offset_c.add(offset_c3);

	}

	public void set_target_board1() {

		int[] id = new int[] { 1, 2, 3, 4 };
		board1ID = new MatOfInt();
		board1ID.fromArray(id);

		List<Point3> c0 = new ArrayList<>();
		List<Point3> c1 = new ArrayList<>();
		List<Point3> c2 = new ArrayList<>();
		List<Point3> c3 = new ArrayList<>();

		c0.add(new Point3(0.075f, 0.0625f, 0.0f));
		c0.add(new Point3(0.125f, 0.0625f, 0.0f));
		c0.add(new Point3(0.125f, 0.0125f, 0.0f));
		c0.add(new Point3(0.075f, 0.0125f, 0.0f));

		c1.add(new Point3(-0.125f, 0.0625f, 0.0f));
		c1.add(new Point3(-0.075f, 0.0625f, 0.0f));
		c1.add(new Point3(-0.075f, 0.0125f, 0.0f));
		c1.add(new Point3(-0.125f, 0.0125f, 0.0f));

		c2.add(new Point3(-0.125f, -0.0125f, 0.0f));
		c2.add(new Point3(-0.075f, -0.0125f, 0.0f));
		c2.add(new Point3(-0.075f, -0.0625f, 0.0f));
		c2.add(new Point3(-0.125f, -0.0625f, 0.0f));

		c3.add(new Point3(0.075f, -0.0125f, 0.0f));
		c3.add(new Point3(0.125f, -0.0125f, 0.0f));
		c3.add(new Point3(0.125f, -0.0625f, 0.0f));
		c3.add(new Point3(0.075f, -0.0625f, 0.0f));

		MatOfPoint3f c_id0 = new MatOfPoint3f();
		MatOfPoint3f c_id1 = new MatOfPoint3f();
		MatOfPoint3f c_id2 = new MatOfPoint3f();
		MatOfPoint3f c_id3 = new MatOfPoint3f();

		c_id0.fromList(c0);
		c_id1.fromList(c1);
		c_id2.fromList(c2);
		c_id3.fromList(c3);
		objPoints1 = new ArrayList<Mat>();
		objPoints1.add(c_id0);
		objPoints1.add(c_id1);
		objPoints1.add(c_id2);
		objPoints1.add(c_id3);
	}

	public void set_target_board2() {

		int[] id = new int[] { 11, 12, 13, 14 };
		board2ID = new MatOfInt();
		board2ID.fromArray(id);

		List<Point3> c0 = new ArrayList<>();
		List<Point3> c1 = new ArrayList<>();
		List<Point3> c2 = new ArrayList<>();
		List<Point3> c3 = new ArrayList<>();

		c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
		c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
		c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
		c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

		c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
		c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
		c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
		c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

		c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
		c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
		c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
		c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

		c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
		c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
		c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
		c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

		MatOfPoint3f c_id0 = new MatOfPoint3f();
		MatOfPoint3f c_id1 = new MatOfPoint3f();
		MatOfPoint3f c_id2 = new MatOfPoint3f();
		MatOfPoint3f c_id3 = new MatOfPoint3f();

		c_id0.fromList(c0);
		c_id1.fromList(c1);
		c_id2.fromList(c2);
		c_id3.fromList(c3);
		objPoints2 = new ArrayList<Mat>();
		objPoints2.add(c_id0);
		objPoints2.add(c_id1);
		objPoints2.add(c_id2);
		objPoints2.add(c_id3);
	}
}

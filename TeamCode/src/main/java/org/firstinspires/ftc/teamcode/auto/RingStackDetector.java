package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;
import android.util.Pair;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.apache.commons.math3.distribution.NormalDistribution;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingStackDetector {
  public enum RingStackResult {
    ZERO(0),
    ONE(1),
    FOUR(4);

    public final int ringCount;

    RingStackResult(int ringCount) {
      this.ringCount = ringCount;
    }
  }

  private final OpenCvCamera camera;
  private volatile Pair<RingStackResult, Double> result = null;

  public RingStackDetector(OpMode opMode) {
    int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
    camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
        cameraMonitorViewId);
    camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.SIDEWAYS_LEFT);
  }

  public Optional<Pair<RingStackResult, Double>> currentlyDetected() {
    return Optional.ofNullable(result);
  }

  class RingDetectionPipeline extends OpenCvPipeline {

    static final double
        Y_LOWER = 25, Y_UPPER = 220,
        CB_LOWER = 0, CB_UPPER = 130,
        CR_LOWER = 140, CR_UPPER = 220;
    final Scalar lowerRange = new Scalar(Y_LOWER, CR_LOWER, CB_LOWER);
    final Scalar upperRange = new Scalar(Y_UPPER, CR_UPPER, CB_UPPER);

    //TODO: replace these numbers with the actual thresholds
    static final double ONE_RING_AREA = 100, FOUR_RING_AREA = 400;
    static final double ST_DEV = 10;
    NormalDistribution one_nd = new NormalDistribution(ONE_RING_AREA, ST_DEV);
    NormalDistribution four_nd = new NormalDistribution(FOUR_RING_AREA, ST_DEV);

    final Mat test = new Mat(),
        edgeDetector = new Mat(),
        smoothEdges = new Mat(),
        contourDetector = new Mat();
    final MatOfPoint2f polyDpResult = new MatOfPoint2f();
    final List<Rect> bounds = new ArrayList<>();

    @SuppressLint("SdCardPath")
    @Override
    public Mat processFrame(Mat input) {

      Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2YCrCb);
      Core.inRange(test, lowerRange, upperRange, edgeDetector);
      Imgproc.GaussianBlur(edgeDetector, smoothEdges, new Size(13, 13), 0, 0);

      ArrayList<MatOfPoint> contours = new ArrayList<>();
      Imgproc.findContours(smoothEdges, contours, contourDetector,
          Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      extractRectBounds(contours);

      for (Rect t : bounds) {
        Imgproc.rectangle(input, t, lowerRange, 2);
      }

      result = identifyStackFromBounds().orElse(null);
      return input;
    }

    // returns a pair containing verdict and confidence from 0 to 1
    private Optional<Pair<RingStackResult, Double>> identifyStackFromBounds() {
      if (bounds.size() == 0) {
        return Optional.of(Pair.create(RingStackResult.ZERO, 0.7));
      }
      // TODO if bounds.size() == 1, compare the rectangle's position on the camera and area to preset constants for 1 vs 4 rings
      // TODO if bounds.size() > 1, maybe set a horizontal threshold (like orange above y=100 ignored), decide on each bound, and pick the decision with the highest confidence
      if (bounds.size() == 1) {
        Rect r = bounds.get(0);
        //the way this is right now, ONE_RING_AREA and FOUR_RING_AREA should each represent the peak of
        //a normal confidence distribution
        if (Math.abs(ONE_RING_AREA - r.area()) > Math.abs(FOUR_RING_AREA - r.area())) {
          //one ring
          double conf = one_nd.density(r.area());
          //double conf = 1 - (Math.abs(ONE_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
          return Optional.of(Pair.create(RingStackResult.ONE, conf));
        }else{
          //four rings
          double conf = four_nd.density(r.area());
          //double conf = 1 - (Math.abs(FOUR_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
          return Optional.of(Pair.create(RingStackResult.FOUR, conf));
        }
      } else {
        //this just chooses the largest rectangle for now since it's probably the most likely... that's what i had in mind originally lol
        Rect maxRect = new Rect();
        double maxArea = 0;
        for (Rect r : bounds) {
          if (r.area() > maxArea) {
            maxArea = r.area();
            maxRect = r;
          }
        }
        Rect r = maxRect;
        if (Math.abs(ONE_RING_AREA - r.area()) > Math.abs(FOUR_RING_AREA - r.area())) {
          //one ring
          double conf = one_nd.density(r.area());
          //double conf = 1 - (Math.abs(ONE_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
          return Optional.of(Pair.create(RingStackResult.ONE, conf));
        }else{
          //four rings
          double conf = four_nd.density(r.area());
          //double conf = 1 - (Math.abs(FOUR_RING_AREA - r.area())/Math.abs(FOUR_RING_AREA - ONE_RING_AREA));
          return Optional.of(Pair.create(RingStackResult.FOUR, conf));
        }
      }
    }

    private void extractRectBounds(ArrayList<MatOfPoint> contours) {
      bounds.clear();
      for (MatOfPoint contour : contours) {
        // if polydp fails, switch to a local new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyDpResult, 3, true);
        Rect r = Imgproc.boundingRect(new MatOfPoint(polyDpResult.toArray()));
        addCombineRectangle(bounds, r, bounds.size() - 1);
      }
    }

    private boolean overlaps(Rect a, Rect b) {
      return a.tl().inside(b) || a.br().inside(b) || b.tl().inside(a) || b.br().inside(a);
    }

    private Rect combineRect(Rect a, Rect b) {
      int topY = (int) Math.min(a.tl().y, b.tl().y);
      int leftX = (int) Math.min(a.tl().x, b.tl().x);
      int bottomY = (int) Math.max(a.br().y, b.br().y);
      int rightX = (int) Math.max(a.br().x, b.br().x);
      return new Rect(leftX, topY, rightX - leftX, bottomY - topY);
    }

    private void addCombineRectangle(List<Rect> list, Rect newRect, int ptr) {
      for (int i = ptr; i >= 0; i--) {
        Rect existing = list.get(i);
        if (overlaps(newRect, existing)) {
          list.remove(i);
          addCombineRectangle(list, combineRect(existing, newRect), i - 1);
          return;
        }
      }
      list.add(newRect);
    }
  }
}

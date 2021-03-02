package org.firstinspires.ftc.teamcode.testing;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class StreamingDataWriter {
  private final Timer timer = new Timer();
  private final Supplier<double[]> source;
  private final int freq;
  private final PrintWriter out;

  StreamingDataWriter(int freq, Supplier<double[]> source, String filename)
      throws FileNotFoundException {
    this.source = source;
    this.freq = freq;
    out = new PrintWriter(filename);
  }

  void startStreaming() {
    timer.scheduleAtFixedRate(new TimerTask() {
      @Override
      public void run() {
        loop();
      }
    }, 0, 1000 / freq);
  }

  void stopStreaming() {
    timer.cancel();
    out.close();
  }

  void loop() {
    String line = DoubleStream.of(source.get())
        .mapToObj(it -> String.format(Locale.US, "%.5f", it))
        .collect(Collectors.joining(","));
    out.println(line);
  }

  void writeLine(String line) {
    out.println(line);
  }
}

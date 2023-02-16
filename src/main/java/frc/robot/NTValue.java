package frc.robot;

import java.util.EnumSet;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NTValue {
  public NTValue(double initialValue, String key) {
    this(initialValue, key, "NTValues");
  }

  public NTValue (double initialValue, String key, String tabName) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    topic = tab.add(key, initialValue)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kTextView) // specify the widget here
      .getEntry().getTopic();
    topic.genericPublish("double").setDouble(initialValue);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        value = event.valueData.value.getDouble();
    });
    value = initialValue;
  }

  public void addListener(DoubleConsumer callback) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
        callback.accept(event.valueData.value.getDouble());
    });
  }
  public double value;
  private Topic topic;
public void addListener(Object object) {
}
}

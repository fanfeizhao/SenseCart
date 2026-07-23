import processing.serial.*;

Serial myPort;

int leftDist = -1;
int leftMainDist = -1;
int leftExtraDist = -1;
int centerDist = -1;
int rightDist = -1;
int downDist = -1;

int leftPWM = 0;
int rightPWM = 0;

String detected = "NONE";
String rawLine = "No serial yet";

void setup() {
  size(1000, 650);

  println("PORTS:");
  printArray(Serial.list());

  myPort = new Serial(this, "/dev/cu.usbmodem2101", 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(245);

  fill(0);
  textAlign(CENTER);
  textSize(32);
  text("Sense Cart Live View", width / 2, 45);

  drawZone("LEFT", 180, 300, leftDist, detected.indexOf("LEFT") >= 0 || detected.indexOf("ALL") >= 0);
  drawZone("CENTER", 500, 180, centerDist, detected.indexOf("CENTER") >= 0 || detected.indexOf("ALL") >= 0);
  drawZone("RIGHT", 820, 300, rightDist, detected.indexOf("RIGHT") >= 0 || detected.indexOf("ALL") >= 0);
  drawZone("DOWN / EDGE", 500, 500, downDist, detected.indexOf("DOWN") >= 0 || detected.indexOf("EDGE") >= 0);

  rectMode(CENTER);
  stroke(0);
  strokeWeight(3);
  fill(210);
  rect(500, 330, 180, 220, 25);

  fill(0);
  textSize(22);
  text("CART", 500, 335);

  textSize(20);
  text("D9 L0X: " + leftMainDist + " mm", 180, 400);
  text("D12 L0X: " + leftExtraDist + " mm", 180, 430);

  textSize(22);
  text("LEFT PWM: " + leftPWM, 250, 580);
  text("RIGHT PWM: " + rightPWM, 750, 580);

  textSize(28);
  text("Detected: " + detected, width / 2, 615);

  textSize(14);
  textAlign(LEFT);
  text("Raw: " + rawLine, 20, 640);
}

void drawZone(String label, int x, int y, int dist, boolean active) {
  stroke(0);
  strokeWeight(2);

  if (active) {
    fill(255, 80, 80);
  } else {
    fill(180, 220, 255);
  }

  ellipse(x, y, 150, 150);

  fill(0);
  textAlign(CENTER);
  textSize(20);
  text(label, x, y - 25);

  textSize(22);
  if (dist > 0) {
    text(dist + " mm", x, y + 10);
  } else {
    text("No Data", x, y + 10);
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');

  if (line == null) {
    return;
  }

  line = trim(line);
  rawLine = line;
  println(line);

  if (line.startsWith("DATA,")) {
    String[] parts = split(line, ',');

    if (parts.length >= 10) {
      leftDist = int(parts[1]);
      leftMainDist = int(parts[2]);
      leftExtraDist = int(parts[3]);

      centerDist = int(parts[4]);
      rightDist = int(parts[5]);
      downDist = int(parts[6]);

      leftPWM = int(parts[7]);
      rightPWM = int(parts[8]);

      detected = parts[9];
    }
  }
}

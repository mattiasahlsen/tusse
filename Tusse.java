package tusse;
import robocode.*;
import java.awt.Color;
import static java.lang.Math.*;


// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * Tusse - a robot by (your name here)
 */
public class Tusse extends Robot
{
  double width;
  double height;
  ScannedRobotEvent lastHit = null;
  long lastHitTime = 0;
  ScannedRobotEvent lastHitRef = null;
  long refTime = 0;

  final static int MAX_TIMEDIFF_TURNINGSPEED = 20;


  /**
   * run: Tusse's default behavior
   */
  public void run() {
    int count = 0;
    boolean doTurnLeft = true;
    boolean doTurnRadarLeft = true;
    setAdjustGunForRobotTurn(true);
    setAdjustRadarForGunTurn(true);
    width = getBattleFieldWidth();
    height = getBattleFieldHeight();

    setColors(Color.red,Color.blue,Color.green); // body,gun,radar

    // Robot main loop
    while(true) {
      // Replace the next 4 lines with any behavior you would like
      count = (count + 1) % 10;
      dodgeWall();


      if (count == 0) {
        doTurnLeft = random() < 0.5;
        doTurnRadarLeft = random() < 0.5;
      }

      if (doTurnLeft) {
        turnLeft(5);
      } else {
        turnRight(5);
      }

      if (lastHit != null && getTime() - lastHitTime < 20) scanAt(lastHit);
      else if (doTurnRadarLeft) turnRadarLeft(45);
      else turnRadarRight(45);

      ahead(10);
    }
  }

  void dodgeWall() {
    double x = getX();
    double y = getY();
    double angle = getHeading();

    if (x / width < 0.1) {
      if (y / height < 0.1) {
        turnLeft(angle - 45);
        ahead(100);
      } else if (y / height > 0.9) {
        turnLeft(angle - 135);
        ahead(100);
      } else {
        turnLeft(angle - 110 - random() * 50);
        ahead(100);
      }
    } else if (x / width > 0.9) {
      if (y / width < 0.1) {
        turnLeft(angle + 45);
        ahead(100);
      } else if (y / height > 0.9) {
        turnLeft(angle + 135);
        ahead(100);
      } else {
        turnLeft(angle + 110 + random() * 50);
        ahead(100);
      }
    } else if (y / height < 0.1) {
      turnLeft(angle -  25 + random() * 50);
      ahead(100);
    } else if (y / height > 0.9) {
      turnLeft(angle + 155 + random() * 50);
      ahead(100);
    }
  }

  double predictAngle(ScannedRobotEvent e, double mySpeed, double turningSpeed) {
    double targetAngle = toRadians(-(e.getBearing() + getHeading() - 90)); // normalized
    double diffAngle = mod(e.getHeading() - targetAngle);

    double enemyX = getX() + cos(targetAngle) * e.getDistance();
    double enemyY = getY() + sin(targetAngle) * e.getDistance();

    int tMin = 0;
    int tMax = (int) floor(max(width, height) / abs(e.getVelocity()));
    int t = tMax / 2;
    int lastT = t;
    double tX;
    double tY;
    double lastSpeedDiff = Integer.MAX_VALUE;
    double minSpeedDiff = Integer.MAX_VALUE;

    while (true) {
      double tAngle;
      double realSpeed;
      double speedDiff;
      double absSpeedDiff;

      double startTargetHeadingAngle = toRadians(-(e.getHeading() - 90)
        + (e.getVelocity() > 0 ? 0 : 180));
      double endTargetHeadingAngle = startTargetHeadingAngle - t * turningSpeed;
      double totalTurn = endTargetHeadingAngle - startTargetHeadingAngle;
      double enemyDX = (totalTurn == 0 ?
        cos(startTargetHeadingAngle) :
        (sin(endTargetHeadingAngle) - sin(startTargetHeadingAngle)) / totalTurn) *
        abs(e.getVelocity() * t); // wrong?
      double enemyDY = (totalTurn == 0 ?
        sin(startTargetHeadingAngle) :
        (-cos(endTargetHeadingAngle) + cos(startTargetHeadingAngle)) / totalTurn) *
        abs(e.getVelocity() * t); // wrong?

      tX = enemyX + enemyDX;
      tY = enemyY + enemyDY;
      realSpeed = sqrt(pow(tX - getX(), 2) + pow(tY - getY(), 2)) / t;
      speedDiff = realSpeed - mySpeed;
      absSpeedDiff = abs(speedDiff);

      if (absSpeedDiff < minSpeedDiff) minSpeedDiff = absSpeedDiff;
      if (tMax == tMin || (t > lastT && speedDiff > lastSpeedDiff)) {
        if (minSpeedDiff * t > 20) return getHeading() + e.getBearing();
        else {
          double offset = toDegrees(atan(abs(tY - getY()) / abs(tX - getX())));
          if (tX > getX()) {
            if (tY > getY()) tAngle = 90 - offset;
            else tAngle = 90 + offset;
          } else if (tY > getY()) tAngle = -90 + offset;
          else tAngle = -90 - offset;

          System.out.println("In predictangle");
          System.out.println("enemyDX: " + enemyDX);
          System.out.println("enemyDY: " + enemyDY);
          System.out.println("tAngle: " + tAngle);
          System.out.println("t: " + t);
          System.out.println("turning speed: " + turningSpeed);
          if (totalTurn != 0) {
            System.out.println("avg cosx: " + (sin(endTargetHeadingAngle) - sin(startTargetHeadingAngle)) / totalTurn);
            System.out.println("avg sinx: " + (-cos(endTargetHeadingAngle) + cos(startTargetHeadingAngle)) / totalTurn);
          }
          return tAngle;
        }
      }

      if (speedDiff < 0) {
        if (tMax == t) tMax = tMin; // last step
        else tMax = t;
      } else {
        if (tMin == t) tMin = tMax;
        else tMin = t;
      }
      lastT = t;
      lastSpeedDiff = speedDiff;
      t = (tMax + tMin) / 2;
    }
  }

  double predictAngle(ScannedRobotEvent e, double mySpeed) {
    return predictAngle(e, mySpeed, getTurningSpeed(e));
  }

  void turnAt(ScannedRobotEvent e) {
    turn(norm(predictAngle(e, getVelocity()) - getHeading()));
  }

  void aimAt(ScannedRobotEvent e, double firePower) {
    turnGun(predictAngle(e, Rules.getBulletSpeed(firePower)) - getGunHeading());
  }

  // returns turning angle of e
  void scanAt(ScannedRobotEvent e) {
    double turnAngle = norm(predictAngle(e,
      e.getDistance() / 3) - getRadarHeading());
    double angle = 45;

    if (turnAngle > 0) {
      turnRadarRight(angle);
      scan();

      turnRadarLeft(angle);
      scan();

      turnRadarLeft(angle);
      scan();
    } else {
      turnRadarLeft(angle);
      scan();

      turnRadarRight(angle);
      scan();

      turnRadarRight(angle);
      scan();
    }

  }

  double getTurningSpeed(ScannedRobotEvent e) {
    long timeDiff = e.getTime() - refTime;
    if (lastHitRef == null ||
      e.getName() != lastHitRef.getName() ||
      timeDiff > MAX_TIMEDIFF_TURNINGSPEED ||
      timeDiff == 0) return 0;
    double turningSpeed = norm(e.getHeading() - lastHitRef.getHeading()) /
      (double) (e.getTime() - refTime);
    return toRadians(turningSpeed);
  }

  void lockAim(ScannedRobotEvent e) {
    while (getGunHeading() != getRadarHeading()) {
      turnGun(Math.min(getRadarHeading() - getGunHeading(), 20));
    }
    aimAt(e, 3);
    double angle = norm(predictAngle(e, 3) - getGunHeading());
    if (angle > 0) {
      for (int i = 0; i < 5; i++) {
        turnGunRight(5);
        scan();
      }
      for (int i = 0; i < 10; i++) {
        turnGunLeft(5);
        scan();
      }
    }
    else {
      for (int i = 0; i < 5; i++) {
        turnGunLeft(5);
        scan();
      }
      for (int i = 0; i < 10; i++) {
        turnGunRight(5);
        scan();
      }
    }
  }

  void follow(ScannedRobotEvent e) {
    turnAt(e);
    ahead(min(50, e.getDistance() / 2));
  }

  /**
   * onScannedRobot: What to do when you see another robot
   */
  public void onScannedRobot(ScannedRobotEvent e) {
    setLastHit(e);

    if (getGunHeat() != 0) {
      if (e.getDistance() > 200) {
        turnAt(e);
        ahead(20);
        scanAt(e);
      } else lockAim(e);

      return;
    }

    double turnAngle = e.getBearing() + getHeading() - getGunHeading();
    double absDiffAngle = abs(norm(e.getHeading() - (e.getBearing() + getHeading())));
    double absAngle = abs(turnAngle);
    double turningSpeed;

    long timeDiff = getTime() - refTime;
    if (lastHitRef == null || timeDiff < 4 || timeDiff > MAX_TIMEDIFF_TURNINGSPEED) {
      if (timeDiff < 4) ahead(5);
      scanAt(e);
    }
    turningSpeed = getTurningSpeed(e);

    if (e.getDistance() < 50) {
      if (absAngle > 30) aimAt(e, 3);
      fire(3);
    } else if (e.getDistance() < 100) {
      if (absAngle > 25) aimAt(e, 2);
      fire(2);
    } else if (e.getDistance() < 200) {
      if (absAngle > 20) aimAt(e, 1);
      fire(1);
    } else if (e.getDistance() < 500) {
      if (absAngle > 10) aimAt(e, 1);
      fire(1);
    }

    follow(e);
  }

  /**
   * onHitByBullet: What to do when you're hit by a bullet
   */
  public void onHitByBullet(HitByBulletEvent e) {
    // Replace the next line with any behavior you would like
    double angle = e.getBearing();
    if (angle > 0) turn(angle - 90);
    else turn (angle + 90);
    if (random() > 0.5) ahead(50);
    else back(50);
  }

  /**
   * onHitWall: What to do when you hit a wall
   */
  public void onHitWall(HitWallEvent e) {
    back(20);
  }

  void turn(double angle) {
    turnRight(norm(angle));
  }
  void turnGun(double angle) {
    turnGunRight(norm(angle));
  }
  void turnRadar(double angle) {
    turnRadarRight(norm(angle));
  }

  double mod(double n) {
    return (n % 360 + 360) % 360;
  }

  // returns an angle between -180 and 180
  double norm(double angle) {
    angle = mod(angle);
    return angle <= 180 ? angle : angle - 360;
  }

  void setLastHit(ScannedRobotEvent e) {
    lastHit = e;
    lastHitTime = e.getTime();
    setLastHitRef(e);
  }

  void setLastHitRef(ScannedRobotEvent e) {
    if (lastHitRef == null || getTime() - lastHitRef.getTime() > MAX_TIMEDIFF_TURNINGSPEED) {
      lastHitRef = e;
      refTime = e.getTime();
    }
  }
}

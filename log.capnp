using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("have.some.delicious.cereal");
$Java.outerClassname("Log");

using Machine = import "machine.capnp";

@0xf3b1f17e25a4285b;

const logVersion :Int32 = 1;

struct Map(Key, Value) {
  entries @0 :List(Entry);
  struct Entry {
    key @0 :Key;
    value @1 :Value;
  }
}

struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @1 :Bool;

  union {
    # *********** log metadata ***********
    initData @2 :InitData;
    sentinel @3 :Sentinel;

    # *********** bootlog ***********
    boot @4 :Boot;

    # ********** openpilot daemon msgs **********
    gpsNMEA @11 :GPSNMEAData;
    sendcan @12 :List(CanData);
    can @13 :List(CanData);
    controlsState @14 :ControlsState;
    sensorEvents @15 :List(SensorEventData);
    pandaState @16 :PandaState;

    # camera stuff, each camera state has a matching encode idx
    cameraState @17 :FrameData;
    
    # systems stuff
    androidLog @5 :AndroidLogEntry;
    managerState @6 :ManagerState;
    procLog @7 :ProcLog;
    clocks @8 :Clocks;
    deviceState @9 :DeviceState;
    logMessage @10 :Text;


    # *********** debug ***********
    joystick1 @18 :Joystick;
    joystick2 @19 :Joystick;

  }
}

struct InitData {
  kernelArgs @0 :List(Text);
  kernelVersion @1 :Text;

  gctx @2 :Text;
  dongleId @3 :Text;

  deviceType @4 :DeviceType;
  version @5 :Text;
  gitCommit @6 :Text;
  gitBranch @7 :Text;
  gitRemote @8 :Text;

  androidProperties @9 :Map(Text, Text);

  pandaInfo @10 :PandaInfo;

  dirty @11 :Bool;
  passive @12 :Bool;
  params @13 :Map(Text, Data);

  enum DeviceType {
    unknown @0;
    pc @1;
  }

  struct PandaInfo {
    hasPanda @0 :Bool;
    dongleId @1 :Text;
    stVersion @2 :Text;
    espVersion @3 :Text;
  }

}

struct Sentinel {
  enum SentinelType {
    endOfSegment @0;
    endOfRoute @1;
    startOfSegment @2;
    startOfRoute @3;
  }
  type @0 :SentinelType;
}

struct Boot {
  wallTimeNanos @0 :UInt64;
  lastKmsg @1 :Data;
  lastPmsg @2 :Data;
  launchLog @3 :Text;
}

struct GPSNMEAData {
  timestamp @0 :Int64;
  localWallTime @1 :UInt64;
  nmea @2 :Text;
}

struct CanData {
  address @0 :UInt32;
  busTime @1 :UInt16;
  dat     @2 :Data;
  src     @3 :UInt8;
}

struct ControlsState @0x97ff69c53601abf1 {
  startMonoTime @0 :UInt64;
  canMonoTimes @1 :List(UInt64);
  enabled @2 :Bool;
  active @3 :Bool;
  frisky @4 :Bool;
}


# android sensor_event_t
struct SensorEventData {
  version @0 :Int32;
  sensor @1 :Int32;
  type @2 :Int32;
  timestamp @3 :Int64;
  uncalibratedDEPRECATED @10 :Bool;

  union {
    acceleration @4 :SensorVec;
    magnetic @5 :SensorVec;
    orientation @6 :SensorVec;
    gyro @7 :SensorVec;
    pressure @9 :SensorVec;
    magneticUncalibrated @11 :SensorVec;
    gyroUncalibrated @12 :SensorVec;
    proximity @13: Float32;
    light @14: Float32;
    temperature @15: Float32;
  }
  source @8 :SensorSource;

  struct SensorVec {
    v @0 :List(Float32);
    status @1 :Int8;
  }

  enum SensorSource {
    android @0;
    iOS @1;
    fiber @2;
    velodyne @3;  # Velodyne IMU
    bno055 @4;    # Bosch accelerometer
    lsm6ds3 @5;   # accelerometer (c2)
    bmp280 @6;    # barometer (c2)
    mmc3416x @7;  # magnetometer (c2)
    bmx055 @8;
    rpr0521 @9;
  }
}

# android struct GpsLocation
struct GpsLocationData {
  # Contains GpsLocationFlags bits.
  flags @0 :UInt16;

  # Represents latitude in degrees.
  latitude @1 :Float64;

  # Represents longitude in degrees.
  longitude @2 :Float64;

  # Represents altitude in meters above the WGS 84 reference ellipsoid.
  altitude @3 :Float64;

  # Represents speed in meters per second.
  speed @4 :Float32;

  # Represents heading in degrees.
  bearingDeg @5 :Float32;

  # Represents expected accuracy in meters. (presumably 1 sigma?)
  accuracy @6 :Float32;

  # Timestamp for the location fix.
  # Milliseconds since January 1, 1970.
  timestamp @7 :Int64;

  source @8 :SensorSource;

  # Represents NED velocity in m/s.
  vNED @9 :List(Float32);

  # Represents expected vertical accuracy in meters. (presumably 1 sigma?)
  verticalAccuracy @10 :Float32;

  # Represents bearing accuracy in degrees. (presumably 1 sigma?)
  bearingAccuracyDeg @11 :Float32;

  # Represents velocity accuracy in m/s. (presumably 1 sigma?)
  speedAccuracy @12 :Float32;

  enum SensorSource {
    android @0;
    iOS @1;
    car @2;
    velodyne @3;  # Velodyne IMU
    fusion @4;
    external @5;
    ublox @6;
    trimble @7;
  }
}

struct PandaState @0xa7649e2575e4591e {
  # from can health
  voltage @0 :UInt32;
  current @1 :UInt32;
  ignitionLine @2 :Bool;
  controlsAllowed @3 :Bool;
  gasInterceptorDetected @4 :Bool;
  safetyParam @5 :Int16;
  hasGps @6 :Bool;
  canSendErrs @7 :UInt32;
  canFwdErrs @8 :UInt32;
  canRxErrs @9 :UInt32;
  gmlanSendErrs @10 :UInt32;
  pandaType @11 :PandaType;
  fanSpeedRpm @12 :UInt16;
  usbPowerMode @13 :UsbPowerMode;
  ignitionCan @14 :Bool;
  faultStatus @15 :FaultStatus;
  powerSaveEnabled @16 :Bool;
  uptime @17 :UInt32;
  faults @18 :List(FaultType);

  enum FaultStatus {
    none @0;
    faultTemp @1;
    faultPerm @2;
  }

  enum FaultType {
    relayMalfunction @0;
    unusedInterruptHandled @1;
    interruptRateCan1 @2;
    interruptRateCan2 @3;
    interruptRateCan3 @4;
    interruptRateTach @5;
    interruptRateGmlan @6;
    interruptRateInterrupts @7;
    interruptRateSpiDma @8;
    interruptRateSpiCs @9;
    interruptRateUart1 @10;
    interruptRateUart2 @11;
    interruptRateUart3 @12;
    interruptRateUart5 @13;
    interruptRateUartDma @14;
    interruptRateUsb @15;
    interruptRateTim1 @16;
    interruptRateTim3 @17;
    registerDivergent @18;
    interruptRateKlineInit @19;
    interruptRateClockSource @20;
    interruptRateTim9 @21;
    # Update max fault type in boardd when adding faults
  }

  enum PandaType @0x8a58adf93e5b3751 {
    unknown @0;
    whitePanda @1;
    greyPanda @2;
    blackPanda @3;
    pedal @4;
    uno @5;
    dos @6;
  }

  enum UsbPowerMode {
    none @0;
    client @1;
    cdp @2;
    dcp @3;
  }

  
}


struct Joystick {
  # convenient for debug and live tuning
  axes @0: List(Float32);
  buttons @1: List(Bool);
}

struct FrameData {
  frameId @0 :UInt32;
  encodeId @1 :UInt32; # DEPRECATED
  timestampEof @2 :UInt64;
  frameLength @3 :Int32;
  integLines @4 :Int32;
  globalGain @5 :Int32;
  lensPos @11 :Int32;
  lensSag @12 :Float32;
  lensErr @13 :Float32;
  lensTruePos @14 :Float32;
  image @6 :Data;
  gainFrac @15 :Float32;
  focusVal @16 :List(Int16);
  focusConf @17 :List(UInt8);
  sharpnessScore @18 :List(UInt16);
  recoverState @19 :Int32;

  frameType @7 :FrameType;
  timestampSof @8 :UInt64;
  transform @10 :List(Float32);

  androidCaptureResult @9 :AndroidCaptureResult;

  enum FrameType {
    unknown @0;
    neo @1;
    chffrAndroid @2;
    front @3;
  }

  struct AndroidCaptureResult {
    sensitivity @0 :Int32;
    frameDuration @1 :Int64;
    exposureTime @2 :Int64;
    rollingShutterSkew @3 :UInt64;
    colorCorrectionTransform @4 :List(Int32);
    colorCorrectionGains @5 :List(Float32);
    displayRotation @6 :Int8;
  }
}

struct AndroidLogEntry {
  id @0 :UInt8;
  ts @1 :UInt64;
  priority @2 :UInt8;
  pid @3 :Int32;
  tid @4 :Int32;
  tag @5 :Text;
  message @6 :Text;
}

struct ManagerState {
  processes @0 :List(ProcessState);

  struct ProcessState {
    name @0 :Text;
    pid @1 :Int32;
    running @2 :Bool;
    exitCode @3 :Int32;
  }
}

struct ProcLog {
  cpuTimes @0 :List(CPUTimes);
  mem @1 :Mem;
  procs @2 :List(Process);

  struct Process {
    pid @0 :Int32;
    name @1 :Text;
    state @2 :UInt8;
    ppid @3 :Int32;

    cpuUser @4 :Float32;
    cpuSystem @5 :Float32;
    cpuChildrenUser @6 :Float32;
    cpuChildrenSystem @7 :Float32;
    priority @8 :Int64;
    nice @9 :Int32;
    numThreads @10 :Int32;
    startTime @11 :Float64;

    memVms @12 :UInt64;
    memRss @13 :UInt64;

    processor @14 :Int32;

    cmdline @15 :List(Text);
    exe @16 :Text;
  }

  struct CPUTimes {
    cpuNum @0 :Int64;
    user @1 :Float32;
    nice @2 :Float32;
    system @3 :Float32;
    idle @4 :Float32;
    iowait @5 :Float32;
    irq @6 :Float32;
    softirq @7 :Float32;
  }

  struct Mem {
    total @0 :UInt64;
    free @1 :UInt64;
    available @2 :UInt64;
    buffers @3 :UInt64;
    cached @4 :UInt64;
    active @5 :UInt64;
    inactive @6 :UInt64;
    shared @7 :UInt64;
  }
}

struct Clocks {
  bootTimeNanos @0 :UInt64;
  monotonicNanos @1 :UInt64;
  monotonicRawNanos @2 :UInt64;
  wallTimeNanos @3 :UInt64;
  modemUptimeMillis @4 :UInt64;
}

struct DeviceState @0xa4d8b5af2aa492eb {
  alive @0 :Bool;
  freeSpacePercent @1 :Float32;
  memoryUsagePercent @2 :Int8;
  cpuUsagePercent @3 :Int8;
  usbOnline @4 :Bool;
  networkType @5 :NetworkType;
  offroadPowerUsageUwh @6 :UInt32;
  networkStrength @7 :NetworkStrength;
  carBatteryCapacityUwh @8 :UInt32;

  fanSpeedPercentDesired @9 :UInt16;
  started @10 :Bool;
  startedMonoTime @11 :UInt64;

  # power
  batteryPercent @12 :Int16;
  batteryStatus @13 :Text;
  batteryCurrent @14 :Int32;
  batteryVoltage @15 :Int32;
  chargingError @16 :Bool;
  chargingDisabled @17 :Bool;

  # device thermals
  cpuTempC @18 :List(Float32);
  gpuTempC @19 :List(Float32);
  memoryTempC @20 :Float32;
  batteryTempC @21 :Float32;
  ambientTempC @22 :Float32;
  thermalStatus @23 :ThermalStatus;

  enum ThermalStatus {
    green @0;
    yellow @1;
    red @2;
    danger @3;
  }

  enum NetworkType {
    none @0;
    wifi @1;
    cell2G @2;
    cell3G @3;
    cell4G @4;
    cell5G @5;
  }

  enum NetworkStrength {
    unknown @0;
    poor @1;
    moderate @2;
    good @3;
    great @4;
  }

}
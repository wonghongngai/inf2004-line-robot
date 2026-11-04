Buddy 4 
=================================

Required shared configuration changes (coordinate with team):
1) config/config_device.h
   #define DEVCNF_USE_IIC 1

2) device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h
   #define DEVCNF_I2C0_SDA_PIN 4
   #define DEVCNF_I2C0_SCL_PIN 5

3) TEAM.md
   Claim GP4/GP5 for Buddy 4 and I2C0 for imu_task before integration.

The build already uses wildcard ../app_program/*/*.c, so adding task_imu.c and
removing task_monitor.c is enough; there is no manual source-file list to edit.

Important:
- Keep the robot stationary for the first ~1 second after Buddy 4 starts so
  baseline calibration is meaningful.
- Detection thresholds are initial placeholders and must be tuned from actual
  IMU logs before they are treated as final requirements/evidence.
- This is intentionally the PRE minimum-bit-change version.

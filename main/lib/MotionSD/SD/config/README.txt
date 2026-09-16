Copy this config directory to the root of the robot's SD card.

Required paths:
  /config/options.ini
  /config/update_rate_gain.csv
  /config/procedure.csv

The files are loaded after the start switch is pressed. If any file is
missing or invalid, the LED remains red and the walking task is not started.

feedback_gain_mode accepts:
  FIXED_MINIMUM
  FIXED_MAXIMUM
  T_SUP_DEPENDENT

disturbance_type accepts:
  NONE
  PUSH
  STEP

pitch_foot_kp and pitch_foot_kd are optional for version-1 option files.
When omitted, both gains default to zero. The pitch-foot correction is
limited to +/-0.060 m in SensorFB.

error_action accepts:
  RESTART
  SKIP

content accepts:
  WARMUP
  WALK
  ENDING

procedure.csv columns:
  t_sup_start,t_sup_end,steps,error_action,content

When t_sup_start and t_sup_end differ, content must be WARMUP and steps must
be at least 2. T_sup is linearly interpolated from the first through the last
WARMUP step. Adjacent procedure rows must be continuous.

For T_SUP_DEPENDENT, gain-table rows must be sorted by increasing t_sup.
Kp and Kd are linearly interpolated between adjacent rows.

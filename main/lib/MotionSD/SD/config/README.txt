Copy this config directory to the root of the robot's SD card.

Required path:
  /config/options.ini

When single_T_sup_exp is false, these paths are also required:
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

Single-T_sup experiment options:
  single_T_sup_exp=true or false
  single_T_sup=<fixed T_sup in seconds>
  single_gain_p=<fixed update-rate proportional gain>
  single_gain_d=<fixed update-rate derivative gain>

When single_T_sup_exp is true, update_rate_gain.csv and procedure.csv are not
loaded. The firmware uses the specified fixed T_sup and gains, and generates
this procedure:
  30 steps, RESTART, WARMUP
  50 steps, SKIP, WALK
  5 steps, RESTART, WARMUP

The generated gain table and procedure are copied to the experiment_config
output directory with the options file.

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

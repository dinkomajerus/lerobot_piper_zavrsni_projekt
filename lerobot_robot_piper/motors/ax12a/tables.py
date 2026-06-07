# AX-12A Control Table (Protocol 1.0)
# Source: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table

AX12A_CONTROL_TABLE = {
  "Model_Number":           (0,  2),
  "Firmware_Version":       (2,  1),
  "ID":                     (3,  1),
  "Baud_Rate":              (4,  1),
  "Return_Delay_Time":      (5,  1),
  "CW_Angle_Limit":         (6,  2),
  "CCW_Angle_Limit":        (8,  2),
  "Temperature_Limit":      (11, 1),
  "Min_Voltage_Limit":      (12, 1),
  "Max_Voltage_Limit":      (13, 1),
  "Max_Torque":             (14, 2),
  "Status_Return_Level":    (16, 1),
  "Alarm_LED":              (17, 1),
  "Shutdown":               (18, 1),

  "Torque_Enable":          (24, 1),
  "LED":                    (25, 1),
  "CW_Compliance_Margin":   (26, 1),
  "CCW_Compliance_Margin":  (27, 1),
  "CW_Compliance_Slope":    (28, 1),
  "CCW_Compliance_Slope":   (29, 1),
  "Goal_Position":          (30, 2),
  "Moving_Speed":           (32, 2),
  "Torque_Limit":           (34, 2),
  "Present_Position":       (36, 2),
  "Present_Speed":          (38, 2),
  "Present_Load":           (40, 2),
  "Present_Voltage":        (42, 1),
  "Present_Temperature":    (43, 1),
  "Registered":             (44, 1),
  "Moving":                 (46, 1),
  "Lock":                   (47, 1),
  "Punch":                  (48, 2),
  }

AX12A_BAUDRATE_TABLE = {
  9_600:     6,
  19_200:    5,
  57_600:    3,
  115_200:   2,
  1_000_000: 1,
  }

AX12A_RESOLUTION   = 1024
AX12A_MODEL_NUMBER = 12
AX12A_MIN_POSITION = 0
AX12A_MAX_POSITION = 1023

# MPSS

# Todo list:
- Change hard coded 10 s wait times after a scan
- Option to redefine Dp list to have 64 or 32 bins per decade, instead of current log spaced list
- Improve COM port connections - bring over changes from Multilogger
- gas_voltage_limit should be changed to a variable for different DMA's - now limit hard coded to 9kV for air
- improve system data quality flagging and error indicators (status_monitor -function)
- replace the hard coded blower maximum power with something more appropriate (blowerValue). E.g. limit from flow meters range (0 - 20 lpm), as current upper limit can be too small.
- Add a message box for errors and other user information, if needed

# Software 0.1.2 - 2025.07.31
- Minor bugfixes

# Software 0.1.1 - 2025.06.25
- Changed monitor voltage from the 1hz data file to setpoint voltage, as the monitor voltage is not working

# Firmware - 2025.06.25
Renamed from "Combined_dmps_v1.10" to Airmodus DMA controller

# Firmware 1.1.1 - 2025.05.26
- Added firmware versioning
- Added calibration commands for storing HV and flow parameters on MPSS -box
- Added support for N2 and Argon

New commands
*VER? - get firmware version
SET_VOLTAGE_CAL
SET_SHEATH_CAL_AIR
SET_SHEATH_CAL_N2
SET_SHEATH_CAL_AR
GET_VOLTAGE_CAL
GET_SHEATH_CAL_AIR
GET_SHEATH_CAL_N2
GET_SHEATH_CAL_AR
GET_CAL

# Software 0.1.0 - 2025.05.26
- Added set Dp
- Improved gas dependent breakdown voltage protection
- Added software versioning
- Added calibration storing of HV and flow parameters
- Added support for N2 and Argon
- Fixed downscan figures
- Disabled automatic SI prefixes on parameters
- Improved rounding of numbers
- Hidden monitor voltage (not working properly at the moment)

New commands
*VER? - get firmware version
SET_VOLTAGE_CAL
SET_SHEATH_CAL_AIR
SET_SHEATH_CAL_N2
SET_SHEATH_CAL_AR
GET_VOLTAGE_CAL
GET_SHEATH_CAL_AIR
GET_SHEATH_CAL_N2
GET_SHEATH_CAL_AR
GET_CAL
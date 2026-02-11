builtin_interfaces/Time stamp

<<<<<<< HEAD
SensorStatus gps
SensorStatus pcd
SensorStatus camera
SensorStatus vehicle
=======
#gps
bool gps_connected
string gps_reason
builtin_interfaces/Time gps_last_publish

# PCD
string pcd_reason
builtin_interfaces/Time pcd_last_publish

# Camera
string camera_reason
builtin_interfaces/Time camera_last_publish

# Vehicle status
string vehicle_status_reason
builtin_interfaces/Time vehicle_status_last_publish
>>>>>>> 0ae66ec (하트비트 추가, 하트비트용 메시지 추가)

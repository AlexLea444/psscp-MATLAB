## SeniorDesign README
Alex Lea, 10/27/24

In order to use the RPLIDAR device in MATLAB, we choose to use the ```mex``` method to create methods to access the SDK with simple functions. This currently includes 'setup', 'startScan', 'getData', and 'close'.

This has only been tested on Linux thusfar, but the provided sdk directory should be correctly modified for all operating systems. To create a ```mex``` file that can be used with your OS of choice, use the below method (change linux for mac or win32, if necessary):

```MATLAB
(Within MATLAB)
 >> cd /path/to/SeniorDesign
 >> mex -I"/home/alex/Code/SeniorDesign/sdk/include" project/rplidar_mex.cpp sdk/src/rplidar_driver.cpp sdk/src/thread.cpp sdk/src/linux_timer.cpp sdk/src/linux_net_serial.cpp sdk/src/linux_net_socket.cpp -outdir project
```

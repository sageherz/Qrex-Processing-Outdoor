# Qrex-Processing-Outdoor
Repository containing codes and directions for processing outdoor forward flight test data. All processing codes are included in the repository, but are also included on the OneDrive (AFCAD Research Group --> VLRCOE --> Qrex Post Processing). Do not make any large changes to the codes on the OneDrive!!! If you need to make changes either create a copy file or process the data in a copied directory and not in the folder on the OneDrive.

# Processing Steps
Below is the outlined order that the processing codes should be executed.

## Step 1: Preliminary Processing
Use the code ```processDataQREX_2025.mat```. This code will preliminarily process the recorded data, including extracting data from the .bag or .db3 file from the pi and save as a .mat file, apply scale factors, and align data. Run this code section by section and enter information where indicated. Include the following functions in your working library:
  * ```air_density```: Computes local air density from input air pressure, temperature, and percent humidity (RECORD THESE VALUES EVERY TIME YOU CONDUCT A FLIGHT TEST).
  * ```saveData_outdoor``` and ```saveData_ROS2```: Extracts data from pi and converts to structure called ```rawData```, and saves all raw data as a .mat file.
  * ```Ulog_Parser_Function```: Extracts data from Pixhawk log file and saves as a structure called ```logData```.
  * ```unbiasThrust_constant``` and ```unbiasThrust_linear```: Unbiases thrust data by either one constant value or linear fit of two values at beginning and end of data. This is based on whether you select 1 or 2 unbiasing points when prompted in MATLAB (recommended to choose two points at beginning and end of flight test to do a linear unbias).

Run this code for every flight test data file you have. 

## Step 2: Averaging Inbound/Outbound Flight Legs
Use the function ```averageEWlegs_updated.mat```. This code will average data from inbound and outbound flight legs. The function takes in the following parameters:
* ```data```: Data to be averaged (Thrust, RPM, ...)
* ```pos```: Position of the vehicle (this will be how inbound/outbound legs will be identified).
* ```startMission```: Start of flight mission (index).
* ```endMission```: End of flight mission (index).
* ```qCrop```: Boolean value (0 or 1) indicating if there is a section of any flight leg that needs to be cropped (such as if the vehicle unexpectedly enters a loiter mid-flight leg). You will need to manually define these indices in the function itself (currently in progress working on a better way to handle this).

Note: This function could use some updating to better support current and future outdoor flight testing. 

## Step #3

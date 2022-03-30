# PCL_Lidarvisualization
Project for visualization of lidar data with a YDLIDAR X4 lidarsensor from YDLIDAR (www.ydlidar.com) and the pointcloudlibrary(https://pointclouds.org/)
The X4 is a belt-driven 360° rangefinder. It has range frequency of 5K, and detection radius of 10m.
The project is developed under Ubuntu 20.04. 

Build Instruction: 

**Install:**  
The pcl lib is mandatory does not matter if you build YDLidar from source or use the prebuilt library. (If you are using a mac-computer keep in mind that there are some issues with pcl visualization on mac)
> sudo apt install libpcl-dev 

**Option 1:**
Build YDLidar from source
Get YLidar sdk frome them https://github.com/YDLIDAR/YDLidar-SDK  
 in YLidar folder:
> mkdir build   

> cmake ..
 
> make -j 

> sudo make install 

In PCL_Lidarvisualization root CMakeLists.txt **set(UseLidarInstall ON)**
In PCL_Lidarvisualization root folder:
> mkdir build 

> cmake ..

> make -j 

**Option 2:**
Use prebuilt static library (was built with Ubuntu 20.04)
In PCL_Lidarvisualization CMakeLists.txt set "UseLidarInstall off"
> mkdir build 

> cmake ..
 
> make -j 

> sudo make install 

**Starting the Lidar**
 Go to PCL_Lidarvisualization build folder and execute with root-privilegs (thats mandatory):
> sudo ./LidarVisu 

> Baudrate: 128000

> one-way communication: no

> set scan frequency to 8 (others value in range are also working)

With PCLVISUALIZER:
![Screenshot from 2022-03-30 22-31-45](https://user-images.githubusercontent.com/42981587/160927186-8c0ef823-d986-4934-8df6-5cde0373cb02.png)
With RANGE points which are nearer than others become more redish
![Screenshot from 2022-03-30 22-39-15](https://user-images.githubusercontent.com/42981587/160927194-14b9d2d7-512e-42c0-bd6a-1913d68efef5.png)
With no visualization in main.cpp set visuMode_ to visualization::IVisualizationHandler::visuMode::NONE
![Screenshot from 2022-03-30 22-41-29](https://user-images.githubusercontent.com/42981587/160927199-7346c0f7-7d2d-4372-8e8d-af369ed58403.png)


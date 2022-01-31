# Structure from motion  
####Result:  
![3d visualization](./markdown_site/3d_visualization.gif)  
You can see pink flower on the doll's belly and yellow back.  
Also you can see cylinder of the doll.  
Input images are in [doll_selected](./doll_selected) folder.  
Here is combined image:  
![combined image](./markdown_site/input_pano.png)  
  
Bottom view:  
![scan direction](./markdown_site/doll_scan_2.png)  
We ca see that noise increase along scan direction.  
This is because this SfM is intremetal and errors are accumultated from frame to frame.  
Also we can see that doll cross section is not perfect circle.  
You can open 3d scan with Meshlab.
Data is in [pointCloud.ply](./pointCloud.ply) file.  
####How to use:
```sfm --video2frames path_to_video```  
   converts video to frames in png format  
   png will be stored in to a folder with name of the video file  
  
```sfm --calibrate path_to_images```  
   calibrate camera  
   path_to_images is directory with images  
   images are 6x9 chessboard images  
   images are png  
   then it prints calibration parameters  
  
```sfm --reconstruct path_to_images```  
   reconstruct 3d scene  
   path_to_images is directory with images  
   result will be saved to pointCloud.ply  
  
Images should have different enought point of view.  
Otherwise it would have low parralax and low accuracy.
    
####Description:  
Android Xiaomi redmi note 5 smartphone was used to get videos. 
First camera was calibrated with chess board images.  
Distortion was neglected. 
Found camera matrix:  
```text
[[1487.886270357746, 0, 547.1524898799552], 
[0, 1488.787677381604, 979.9460018614599],
[0, 0, 1]]
```
The algoritm works in follow way:
1. Find SIFT keypoints on an image.
2. Compare descriptors of the keypoints with descriptors from previouse image. Get correspondance.  
3. Find essential matrix with findEssentialMat
4. Find relative camera position and 3d point clound with recoverPose functon. Camera matrix is used.  
5. 
 


  

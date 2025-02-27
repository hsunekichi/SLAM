/**
 * mono_tumvi.cc
 *
 * A demo application that runs Mini-SLAM on a TUM-VI room1 sequence using a fish-eye camera.
 *
 * Usage:
 *    ./mono_tumvi <dataset_path>
 */

 #include "DatasetLoader/TUMVILoader.h"  // Use the loader for the TUM-VI dataset.
 #include "System/MiniSLAM.h"
 #include <opencv2/opencv.hpp>
 #include <fstream>
 #include <iomanip>
 #include <iostream>
 
 using namespace std;
 
 int main(int argc, char **argv) {
     // Ensure the program is invoked with the dataset path.
     if (argc != 2) {
         cerr << "[Error]: You must invoke the program with 1 parameter:" << endl;
         cerr << "\t./mono_tumvi <dataset_path>" << endl;
         return -1;
     }
 
     // Load the dataset sequence.
     string datasetPath = argv[1];
     // The TUMVILoader is assumed to be provided (similar to TUMRGBDLoader)
     // and will load the fish-eye room1 sequence (for example, from room1/rgb.txt).
     TUMVILoader sequence(datasetPath, datasetPath + "/rgb.txt");
 
     // Create the MiniSLAM system using a configuration file that supports the fish-eye camera.
     // Make sure Data/TUMVI.yaml defines the camera type as "FishEye" and includes the 8 parameters.
     MiniSLAM SLAM("Data/TUMVI.yaml");
 
     // Open a file to store the estimated trajectory.
     ofstream trajectoryFile("trajectory.txt");
     if (!trajectoryFile.is_open()) {
         cerr << "[Error]: Could not open trajectory file, aborting..." << endl;
         return -2;
     }
 
     cv::Mat currIm;
     double currTs;
     // Process each image in the sequence.
     for (int i = 0; i < sequence.getLenght(); i++) {
         sequence.getRGBImage(i, currIm);
         sequence.getTimeStamp(i, currTs);
 
         Sophus::SE3f Tcw;
         if (SLAM.processImage(currIm, Tcw)) {
             // Convert the camera pose Tcw to world pose Twc.
             Sophus::SE3f Twc = Tcw.inverse();
             // Save the predicted pose to the file.
             trajectoryFile << setprecision(17) << currTs << ","
                            << setprecision(7) << Twc.translation()(0) << ","
                            << Twc.translation()(1) << ","
                            << Twc.translation()(2) << ","
                            << Twc.unit_quaternion().x() << ","
                            << Twc.unit_quaternion().y() << ","
                            << Twc.unit_quaternion().z() << ","
                            << Twc.unit_quaternion().w() << endl;
         }
     }
 
     trajectoryFile.close();
     return 0;
 }
 
/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Mapping/LocalMapping.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"

using namespace std;

LocalMapping::LocalMapping() {

}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    checkDuplicatedMapPoints();

    //Run a local Bundle Adjustment
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());
}

void LocalMapping::mapPointCulling() {
    /*
     * Your code for Lab 4 - Task 4 here!
     */
    
    // Iterate through all MapPoints in the map
    for (auto it = pMap_->getMapPoints().begin(); it != pMap_->getMapPoints().end(); ++it) {
        // Get the MapPoint
        shared_ptr<MapPoint> pMP = it->second;

        bool isBadMapPoint = false;
        // Implement your criteria for determining if a MapPoint is bad
        // Return true if the MapPoint is bad, false otherwise
        // Example criteria: check if the MapPoint has very few observations

        // Get the number of observations of the MapPoint
        int numObservations = pMap_->getNumberOfObservations(pMP->getId());
        if (numObservations < 2) {
            isBadMapPoint =  true; // Consider MapPoint as bad if it has less than 2 observations
        }

        // Check if the MapPoint meets the criteria for being a bad MapPoint
        if (isBadMapPoint) {
            // Remove the bad MapPoint from the map
            pMap_->removeMapPoint(pMP->getId());
        }
    }
    
}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        // Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if (vMatches[i] != -1){
                /*
                 * Your code for Lab 4 - Task 2 here!
                 * Note that the last KeyFrame inserted is stored at this->currKeyFrame_
                 */

                

                // Triangulate the 3D point
                cv::KeyPoint keypoint1 = currKeyFrame_->getKeyPoint(i);
                cv::KeyPoint keypoint2 = pKF->getKeyPoint(vMatches[i]);
                Eigen::Vector3f point3D;
                triangulate(calibration1->unproject(keypoint1.pt), pKF->getCalibration()->unproject(keypoint2.pt), T1w, T2w, point3D);

                // Check if the triangulated point is geometrically correct
                Eigen::Vector3f normal1 = point3D - T1w.translation();
                Eigen::Vector3f normal2 = point3D - T2w.translation();
                float cosParallax = cosRayParallax(normal1, normal2);

                cv::Point2f uv1= calibration1->project(point3D);
                float reprojectionError = squaredReprojectionError(keypoint1.pt, uv1);
                
                if(cosParallax < settings_.getMinCos() || reprojectionError > 0.01)//settings_.getMaxReprojectionError())
                {
                    // cout << "Triangulation faile - reprojectionError = " << reprojectionError << " cosParallax = " << cosParallax << endl;
                    continue;
                }

                // Insert the MapPoint into the map
                shared_ptr<MapPoint> pMP(new MapPoint(point3D));

                // Add the observation of the MapPoint in the KeyFrames
                pMap_->addObservation(currKeyFrame_->getId(), pMP->getId(), i);
                pMap_->addObservation(pKF->getId(), pMP->getId(), vMatches[i]);

                nTriangulated++;

                
            }
        }
    }
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}

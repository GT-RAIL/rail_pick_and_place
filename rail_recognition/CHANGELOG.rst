^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_recognition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2015-03-30)
------------------

1.0.1 (2015-03-27)
------------------
* Removed some old code
* Contributors: David Kent

1.0.0 (2015-03-27)
------------------
* object recognition listener now checks if segmentation passes in updated information vs. new object information
* Moved actions from recognition to msgs
* Removed unused messages
* Message generation dependency
* Removed some unused imports
* Added a recognition listener that allows for shared recognition of a list of detected objects
* Removed some unused stuff from recognition
* Updated rail recognition to use the database, fixed a point cloud transform bug in grasp collection
* Updates to rviz panels
* Switched registration to use the graspdb, fixed point cloud selection in grasp collector
* Added a vision panel that can handle segmentation and recognition, minor documentation updates to other panels
* Moved rviz panels to rail_pick_and_place_tools, added an rviz panel for grasp collection
* Updated grasp requests to use stamped poses instead of base_footprint frame poses
* Switched model generation from a service to an action, and updated the rviz plugin so that it does not freeze during the model generation call
* Updated to reflect moving some messages from rail_segmentation to rail_manipulation_messages
* rviz plugin launch/install, new models, and some general cleanup
* Rviz plugin updates
* Initial rviz plugin for model generation
* Contributors: David Kent

0.0.2 (2015-02-18)
------------------
* catkin cleanup
* Updated metapackage
* Grasp collection, model building, basic recognition and grasping
* Contributors: David Kent, Russell Toris

0.0.1 (2014-10-22)
------------------

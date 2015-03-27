^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_grasp_collection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2015-03-27)
------------------

1.0.0 (2015-03-27)
------------------
* minor cleanup
* small fix
* Updated rail recognition to use the database, fixed a point cloud transform bug in grasp collection
* Switched registration to use the graspdb, fixed point cloud selection in grasp collector
* minor bug fixes
* include cleanup
* fixed default link
* global database params
* load functions for grasp models
* add grasp added
* ID returned from add
* major refactor
* virtual added to destructor
* model added
* Moved rviz panels to rail_pick_and_place_tools, added an rviz panel for grasp collection
* typo
* adds back private node handle
* removes unused members
* bug fixes after initial testing
* small edits
* small edits
* merge conflict
* allow for copy of client
* get array of objects based on object name
* extract from database added for demonstrations
* Updated to reflect moving some messages from rail_segmentation to rail_manipulation_messages
* added node to load and publish grasps
* Updated to use the new rail_manipulation_msgs
* travis fix
* missing dep
* minor edits
* added param for segmented object topic
* launch file added
* added mutex for list
* more comments
* first pass of new collector
* consty const const const
* insert grasp added
* graspdb started (creates tables)
* all but storing in the SQL database redone for grasp collection
* Contributors: David Kent, Russell Toris

0.0.2 (2015-02-18)
------------------
* catkin cleanup
* Updated metapackage
* Grasp collection, model building, basic recognition and grasping
* Contributors: David Kent, Russell Toris

0.0.1 (2014-10-22)
------------------

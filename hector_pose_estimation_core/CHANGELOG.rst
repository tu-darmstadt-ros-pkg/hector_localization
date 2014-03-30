^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_pose_estimation_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-03-30)
------------------
* Fixed boost 1.53 issues
  changed boost::shared_dynamic_cast to boost::dynamic_pointer_cast and
  boost::shared_static_cast to boost::static_pointer_cast
* hector_pose_estimation_core: rotate rate vector to nav frame in PoseEstimation::getState()
  All vectors in state messages (e.g. on topic /state) are given in nav frame. The rate vector
  has not been converted from body until now.
* Contributors: Christopher Hrabia, Johannes Meyer

0.1.0 (2013-09-03)
-----------
* catkinized stack hector_localization

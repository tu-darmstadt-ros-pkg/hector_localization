^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_pose_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-03-30)
------------------
* Fixed boost 1.53 issues
  changed boost::shared_dynamic_cast to boost::dynamic_pointer_cast and
  boost::shared_static_cast to boost::static_pointer_cast
* readded tf_prefix support (was removed from tf::TransformBroadcaster in hydro)
* hector_pose_estimation/rtt_hector_pose_estimation: removed optional usage of package hector_uav_msgs and created a separate package hector_quadrotor_pose_estimation instead
* fixed unresolved symbols when loading the nodelet, renamed plugin description xml file
* Contributors: Christopher Hrabia, Johannes Meyer

0.1.0 (2013-09-03)
------------------
* catkinized stack hector_localization

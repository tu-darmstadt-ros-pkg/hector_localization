^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_pose_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2014-10-02)
------------------
* hector_pose_estimation: make private variables protected to allow access from specialized pose_estimation classes
* hector_pose_estimation: fixed wrong indicies bug in GPS pose publisher
* Contributors: Johannes Meyer

0.1.4 (2014-08-28)
------------------

0.1.3 (2014-07-09)
------------------

0.1.2 (2014-06-02)
------------------

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

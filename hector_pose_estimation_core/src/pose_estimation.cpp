//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/measurements/gravity.h>

namespace hector_pose_estimation {

namespace {
  static PoseEstimation *the_instance = 0;
}

PoseEstimation::PoseEstimation(SystemModel *system_model)
  : system_(0), filter_(0)
  , state_is_dirty_(true)
  , covariance_is_dirty_(true)
  , status_()
  , gravity_(new GravityModel, "gravity")
  , zerorate_(new ZeroRateModel, "zerorate")
{
  if (!the_instance) the_instance = this;

  base_frame_ = "base_link";
  nav_frame_ = "nav";
  alignment_time_ = 0.0;

  parameters().add("base_frame", base_frame_);
  parameters().add("nav_frame", nav_frame_);
  parameters().add("reference_latitude",  global_reference_.latitude);
  parameters().add("reference_longitude", global_reference_.longitude);
  parameters().add("reference_altitude",  global_reference_.altitude);
  parameters().add("reference_heading",   global_reference_.heading);
  parameters().add("alignment_time",      alignment_time_);

  // initialize system model
  setSystemModel(system_model);

  // add default measurements
  addMeasurement(&gravity_);
  addMeasurement(&zerorate_);
}

PoseEstimation::~PoseEstimation()
{
  cleanup();
}

PoseEstimation *PoseEstimation::Instance() {
  if (!the_instance) the_instance = new PoseEstimation();
  return the_instance;
}

void PoseEstimation::setSystemModel(SystemModel *new_system_model) {
  if (system_) {
    cleanup();
    delete system_;
    system_ = 0;
  }

  if (!new_system_model) return;
  system_ = new System(new_system_model);
}

System *PoseEstimation::getSystem() const {
  return system_;
}

bool PoseEstimation::init()
{
  // cleanup everything
  if (filter_) cleanup();
  if (!system_) return false;

  // reset extended Kalman filter
  filter_ = new BFL::ExtendedKalmanFilter(system_->getPrior());
  state_is_dirty_ = covariance_is_dirty_ = true;

  // set initial status
  if (alignment_time_ > 0) {
    status_ = STATE_ALIGNMENT;
    alignment_start_ = ros::Time();
  } else {
    status_ = static_cast<SystemStatus>(0);
  }

  // initialize all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->init();

  return true;
}

void PoseEstimation::cleanup()
{
  // delete filter instance
  if (filter_) {
    delete filter_;
    filter_ = 0;
  }

  // cleanup all measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->cleanup();
}

bool PoseEstimation::reset()
{
  // just forward to init() for now
  return init();
}

void PoseEstimation::update(const InputVector& input, ros::Time new_timestamp)
{
  ros::Duration dt;

  // set input and calculate time diff dt
  system_->setInput(input);
  if (!timestamp_.isZero()) dt = new_timestamp - timestamp_;
  timestamp_ = new_timestamp;

  // do the update step
  update(dt.toSec());
}

void PoseEstimation::update(double dt)
{
  // check dt
  if (dt < -1.0)
    reset();
  else if (dt < 0.0)
    return;

  // check if filter is initialized
  if (!system_ || !filter_) return;

  // time update step
  ROS_DEBUG("Updating with system model %s", system_->getName().c_str());
  system_->update(*filter_, getSystemStatus(), dt);
  system_->limitState(*filter_);

//  state_is_dirty_ = true;
//  std::cout << "     u = [" << system_->getInput().transpose() << "]" << std::endl;
//  std::cout << "x_pred = [" << getState().transpose() << "]" << std::endl;
//  std::cout << "P_pred = [" << filter_->PostGet()->CovarianceGet() << "]" << std::endl;

  // iterate through measurements and do the measurement update steps
  SystemStatus measurement_status = 0;
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    Measurement *measurement = *it;
    if (!measurement->active(getSystemStatus())) continue;

    // process the incoming queue
    measurement->process(*filter_, getSystemStatus());
    system_->limitState(*filter_);

    measurement_status |= measurement->getStatusFlags();
    measurement->increase_timer(dt);
  }

  // pseudo updates
  if (!(measurement_status & STATE_XY_VELOCITY) && !(measurement_status & STATE_XY_POSITION)) {
    ROS_DEBUG("Updating with pseudo measurement model %s", gravity_.getName().c_str());
    GravityModel::MeasurementVector y(system_->getInput().sub(ACCEL_X, ACCEL_Z).normalized());
    gravity_.update(*filter_, getSystemStatus(), y);
    system_->limitState(*filter_);
    measurement_status |= gravity_.getStatusFlags();
  }
  if (!(measurement_status & STATE_YAW)) {
    ROS_DEBUG("Updating with pseudo measurement model %s", zerorate_.getName().c_str());
    ZeroRateModel::MeasurementVector y(system_->getInput().sub(GYRO_Z, GYRO_Z));
    zerorate_.update(*filter_, getSystemStatus(), y);
    system_->limitState(*filter_);
    measurement_status |= zerorate_.getStatusFlags();
  }

  // update the system status
  updateSystemStatus(measurement_status, STATE_ROLLPITCH | STATE_YAW | STATE_XY_POSITION | STATE_XY_VELOCITY | STATE_Z_POSITION | STATE_Z_VELOCITY);

//  state_is_dirty_ = true;
//  std::cout << "x_est = [" << getState().transpose() << "]" << std::endl;
//  std::cout << "P_est = [" << filter_->PostGet()->CovarianceGet() << "]" << std::endl;

  // switch overall system state
  if (status_ & STATE_ALIGNMENT) {
    if (alignment_start_.isZero()) alignment_start_ = timestamp_;
    if ((timestamp_ - alignment_start_).toSec() >= alignment_time_) {
      updateSystemStatus(STATE_DEGRADED, STATE_ALIGNMENT);
    }
  } else if ((status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION)) {
    // if (!(status_ & STATE_READY) && (status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION)) {
    updateSystemStatus(STATE_READY, STATE_DEGRADED);
  } else {
    // if ((status_ & STATE_READY) && !((status_ & STATE_ROLLPITCH) && (status_ & STATE_YAW) && (status_ & STATE_XY_POSITION) && (status_ & STATE_Z_POSITION))) {
    updateSystemStatus(STATE_DEGRADED, STATE_READY);
  }

  state_is_dirty_ = covariance_is_dirty_ = true;
}

void PoseEstimation::addMeasurement(Measurement *measurement) {
  measurements_.push_back(measurement);
}

Measurement *PoseEstimation::getMeasurement(const std::string &name) const {
  for(Measurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    if ((*it)->getName() == name) return *it;
  }
  return 0;
}

const StateVector& PoseEstimation::getState() {
  if (state_is_dirty_) {
    state_ = filter_->PostGet()->ExpectedValueGet();
    state_is_dirty_ = false;
  }
  return state_;
}

const StateCovariance& PoseEstimation::getCovariance() {
  if (covariance_is_dirty_) {
    covariance_ = filter_->PostGet()->CovarianceGet();
    covariance_is_dirty_ = false;
  }
  return covariance_;
}

void PoseEstimation::setState(const StateVector& state) {
  filter_->PostGet()->ExpectedValueSet(state);
  state_is_dirty_ = true;
}

void PoseEstimation::setCovariance(const StateCovariance& covariance) {
  filter_->PostGet()->CovarianceSet(covariance);
  covariance_is_dirty_ = true;
}


SystemStatus PoseEstimation::getSystemStatus() const {
  SystemStatus copy = status_;
  if (copy & STATE_XY_POSITION) copy |= STATE_XY_VELOCITY;
  if (copy & STATE_Z_POSITION)  copy |= STATE_Z_VELOCITY;
  return copy;
}

bool PoseEstimation::updateSystemStatus(SystemStatus set, SystemStatus clear) {
  return setSystemStatus((status_ & ~clear) | set);
}

bool PoseEstimation::setSystemStatus(SystemStatus new_status) {
  if (status_callback_ && !status_callback_(new_status)) return false;

  SystemStatus set = new_status & ~status_;
  SystemStatus cleared = status_ & ~new_status;
  if (set)     ROS_INFO_STREAM("Set system state " << getSystemStatusString(set));
  if (cleared) ROS_INFO_STREAM("Cleared system state " << getSystemStatusString(cleared));

  status_ = new_status;
  return true;
}

ros::Time PoseEstimation::getTimestamp() const {
  return timestamp_;
}

void PoseEstimation::setTimestamp(ros::Time timestamp) {
  timestamp_ = timestamp;
}

void PoseEstimation::getPose(tf::Pose& pose) {
  tf::Quaternion quaternion;
  getPosition(pose.getOrigin());
  getOrientation(quaternion);
  pose.setRotation(quaternion);
}

void PoseEstimation::getPose(tf::Stamped<tf::Pose>& pose) {
  getPose(static_cast<tf::Pose &>(pose));
  pose.stamp_ = timestamp_;
  pose.frame_id_ = nav_frame_;
}

void PoseEstimation::getPosition(tf::Point& point) {
  getState();
  point = tf::Point(state_(POSITION_X), state_(POSITION_Y), state_(POSITION_Z));
}

void PoseEstimation::getPosition(tf::Stamped<tf::Point>& point) {
  getPosition(static_cast<tf::Point &>(point));
  point.stamp_ = timestamp_;
  point.frame_id_ = nav_frame_;
}

void PoseEstimation::getOrientation(tf::Quaternion& quaternion) {
  getState();
  quaternion = tf::Quaternion(state_(QUATERNION_X), state_(QUATERNION_Y), state_(QUATERNION_Z), state_(QUATERNION_W));
}

void PoseEstimation::getOrientation(tf::Stamped<tf::Quaternion>& quaternion) {
  getOrientation(static_cast<tf::Quaternion &>(quaternion));
  quaternion.stamp_ = timestamp_;
  quaternion.frame_id_ = nav_frame_;
}

void PoseEstimation::getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity) {
  getState();
  const InputVector &input = system_->getInput();
  linear_acceleration.x = input(ACCEL_X) + state_(BIAS_ACCEL_X);
  linear_acceleration.y = input(ACCEL_Y) + state_(BIAS_ACCEL_Y);
  linear_acceleration.z = input(ACCEL_Z) + state_(BIAS_ACCEL_Z);
  angular_velocity.x    = input(GYRO_X)  + state_(BIAS_GYRO_X);
  angular_velocity.y    = input(GYRO_Y)  + state_(BIAS_GYRO_Y);
  angular_velocity.z    = input(GYRO_Z)  + state_(BIAS_GYRO_Z);
}

void PoseEstimation::getVelocity(tf::Vector3& vector) {
  getState();
  vector = tf::Vector3(state_(VELOCITY_X), state_(VELOCITY_Y), state_(VELOCITY_Z));
}

void PoseEstimation::getVelocity(tf::Stamped<tf::Vector3>& vector) {
  getVelocity(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = timestamp_;
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getBias(tf::Vector3& angular_velocity, tf::Vector3& linear_acceleration) {
  getState();
  angular_velocity.setX(state_(BIAS_GYRO_X));
  angular_velocity.setY(state_(BIAS_GYRO_Y));
  angular_velocity.setZ(state_(BIAS_GYRO_Z));
  linear_acceleration.setX(state_(BIAS_ACCEL_X));
  linear_acceleration.setY(state_(BIAS_ACCEL_Y));
  linear_acceleration.setZ(state_(BIAS_ACCEL_Z));
}

void PoseEstimation::getBias(tf::Stamped<tf::Vector3>& angular_velocity, tf::Stamped<tf::Vector3>& linear_acceleration) {
  getBias(static_cast<tf::Vector3 &>(angular_velocity), static_cast<tf::Vector3 &>(linear_acceleration));
  angular_velocity.stamp_ = timestamp_;
  angular_velocity.frame_id_ = base_frame_;
  linear_acceleration.stamp_ = timestamp_;
  linear_acceleration.frame_id_ = base_frame_;
}

void PoseEstimation::getTransforms(std::vector<tf::StampedTransform>& transforms) {
  transforms.resize(3);
  tf::Quaternion orientation;
  tf::Point position;
  getOrientation(orientation);
  getPosition(position);
  btMatrix3x3 rotation(orientation);
  double y,p,r;
  rotation.getEulerYPR(y,p,r);

  transforms[0].stamp_ = timestamp_;
  transforms[0].frame_id_ = nav_frame_;
  transforms[0].child_frame_id_ = "base_footprint";
  transforms[0].setOrigin(tf::Point(position.x(), position.y(), 0.0));
  rotation.setEulerYPR(y,0.0,0.0);
  transforms[0].setBasis(rotation);

  transforms[1].stamp_ = timestamp_;
  transforms[1].frame_id_ = "base_footprint";
  transforms[1].child_frame_id_ = "base_stabilized";
  transforms[1].setIdentity();
  transforms[1].setOrigin(tf::Point(0.0, 0.0, position.z()));

  transforms[2].stamp_ = timestamp_;
  transforms[2].frame_id_ = "base_stabilized";
  transforms[2].child_frame_id_ = base_frame_;
  transforms[2].setIdentity();
  rotation.setEulerYPR(0.0,p,r);
  transforms[2].setBasis(rotation);
}


ParameterList PoseEstimation::getParameters() const {
  ParameterList parameters = parameters_;

  if (system_) {
    parameters.copy(system_->getName(), system_->parameters());
  }

  for(Measurements::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    parameters.copy((*it)->getName(), (*it)->parameters());
  }

  return parameters;
}

} // namespace hector_pose_estimation

//	bool Navigation::configureHook()
//	{
//		RTT::Logger::In in(getName());

//		CurrentTime = 0;
//		LastTime = 0;
//		CycleTime = 0;

//		//--> Set time intervals for sensors and navigation FSM
//		//-------------------------------------------------------
//		GPSTimer				= 0;
//		BaroTimer				= 0;
//		MagTimer				= 0;
//		GPSTimeout				= 5*1000;	//ms
//		BaroTimeout				= 1*1000;	//ms
//		MagTimeout				= 1*1000;	//ms

//		AlignmentTimer			= 0;
//		FIX_POS_Timer			= 0;
//		ZVEL_NE_Timer			= 0;
//		ZVEL_D_Timer			= 0;
//		EST_AZI_Timer			= 0;
//		AlignmentTimeout		= 0; // 15*1000;	//ms
//		FIX_POS_Interval		= 0;		//ms
//		ZVEL_NE_Interval		= 0;		//ms
//		ZVEL_D_Interval			= 0;		//ms
//		EST_AZI_Interval		= 0;		//ms
//		//-------------------------------------------------------

//		//--> Set start position
//		//-------------------------------------------------------
//		GPSLastLat = 49.86196863 * M_PI/180.0;
//		GPSLastLon =  8.68275853 * M_PI/180.0;
//		InitialHeight = 0.0;
//		//-------------------------------------------------------

//		//--> Set reference position and altitude
//		//--------------------------------------------------------------------------------------------------
//		INS_Interface.Reference = propertyReference.get();
//		INS_Interface.Reference.lat *= M_PI/180.0;
//		INS_Interface.Reference.lon *= M_PI/180.0;
//		if (INS_Interface.Reference.lat == 0.0 && INS_Interface.Reference.lon == 0.0) {
//		  INS_Interface.Reference.lat = GPSLastLat;
//		  INS_Interface.Reference.lon = GPSLastLon;
//		  INS_Interface.Reference.altitude = InitialHeight;
//		}
//		INS_Interface.NavData.lat = INS_Interface.Reference.lat;
//		INS_Interface.NavData.lon = INS_Interface.Reference.lon;
//		INS_Interface.NavData.altitude = INS_Interface.Reference.altitude;

//		setReferenceAltitude(INS_Interface.Reference.altitude);
//		setReferencePosition(INS_Interface.Reference.lat, INS_Interface.Reference.lon);

//		//--> Initialize flags and operating state
//		//-------------------------------------------------------
//		PredictWithIMU		= false;

//		UpdateWithBaro		= false;
//		UpdateWithGPS		= false;
//		UpdateWithMag		= false;
//		UpdateWithFIX_POS	= false;
//		UpdateWithZVEL_NE	= false;
//		UpdateWithZVEL_D	= false;
//		UpdateWithEST_AZI	= false;

//		GPSError			= true;
//		BaroError			= true;
//		MagError			= true;
//		ExternalPoseUpdate              = false;

//		SetNewGPSPosition	= true;

//		INS_Interface.NavData_available  = noData;
//		INS_Interface.IMUData_available  = noData;
//		INS_Interface.GPSData_available  = noData;
//		INS_Interface.BaroData_available = noData;
//		INS_Interface.MagData_available  = noData;

//		OperatingStatus = INS_STATUS_ALIGNMENT;
//		//-------------------------------------------------------

//		//--> Initialize navigation filter
//		//-------------------------------------------------------

//		//--> BARO measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector baro_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_BARO);
//		baro_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix baro_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_BARO);
//		baro_meas_noise_Cov		  =	   0.0;
//		baro_meas_noise_Cov(1,1)  =  sqr(5.0);

//		baro_meas_pdf.AdditiveNoiseMuSet(baro_meas_noise_Mu);
//		baro_meas_pdf.AdditiveNoiseSigmaSet(baro_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> GPS measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector gps_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_GPS);
//		gps_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix gps_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_GPS);
//		gps_meas_noise_Cov		  =	   0.0;
//		gps_meas_noise_Cov(1,1)   =   sqr(5.0);
//		gps_meas_noise_Cov(2,2)   =   sqr(5.0);
//		gps_meas_noise_Cov(3,3)   =   sqr(1.0);
//		gps_meas_noise_Cov(4,4)   =   sqr(1.0);

//		gps_meas_pdf.AdditiveNoiseMuSet(gps_meas_noise_Mu);
//		gps_meas_pdf.AdditiveNoiseSigmaSet(gps_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> MAG measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector mag_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_MAG);
//		mag_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix mag_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_MAG);
//		mag_meas_noise_Cov		  =	   0.0;
//		mag_meas_noise_Cov(1,1)   =   sqr(5.0 * M_PI/180.0);
//		mag_meas_noise_Cov(2,2)   =   sqr(5.0 * M_PI/180.0);
//		mag_meas_noise_Cov(3,3)   =   sqr(5.0 * M_PI/180.0);

//		mag_meas_pdf.AdditiveNoiseMuSet(mag_meas_noise_Mu);
//		mag_meas_pdf.AdditiveNoiseSigmaSet(mag_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> FIX_POS measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector fix_pos_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_FIX_POS);
//		fix_pos_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix fix_pos_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_FIX_POS);
//		fix_pos_meas_noise_Cov		  =	   0.0;
//		fix_pos_meas_noise_Cov(1,1)   =  sqr(1.0);
//		fix_pos_meas_noise_Cov(2,2)   =  sqr(1.0);

//		fix_pos_meas_pdf.AdditiveNoiseMuSet(fix_pos_meas_noise_Mu);
//		fix_pos_meas_pdf.AdditiveNoiseSigmaSet(fix_pos_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> ZVEL_NE measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector zvel_ne_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ZVEL_NE);
//		zvel_ne_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix zvel_ne_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ZVEL_NE);
//		zvel_ne_meas_noise_Cov		  =	   0.0;
//		zvel_ne_meas_noise_Cov(1,1)   =  sqr(1.0 * 100.0);
//		zvel_ne_meas_noise_Cov(2,2)   =  sqr(1.0 * 100.0);

//		zvel_ne_meas_pdf.AdditiveNoiseMuSet(zvel_ne_meas_noise_Mu);
//		zvel_ne_meas_pdf.AdditiveNoiseSigmaSet(zvel_ne_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> ZVEL_D measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector zvel_d_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ZVEL_D);
//		zvel_d_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix zvel_d_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ZVEL_D);
//		zvel_d_meas_noise_Cov		  =	   0.0;
//		zvel_d_meas_noise_Cov(1,1)    =  sqr(1.0 * 100.0);

//		zvel_d_meas_pdf.AdditiveNoiseMuSet(zvel_d_meas_noise_Mu);
//		zvel_d_meas_pdf.AdditiveNoiseSigmaSet(zvel_d_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> EST_AZI measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector est_azi_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_EST_AZI);
//		est_azi_meas_noise_Mu		  =    0.0;
	
//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix est_azi_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_EST_AZI);
//		est_azi_meas_noise_Cov		  =	   0.0;
//		est_azi_meas_noise_Cov(1,1)   =  sqr(5.0 * M_PI/180.0);

//		est_azi_meas_pdf.AdditiveNoiseMuSet(est_azi_meas_noise_Mu);
//		est_azi_meas_pdf.AdditiveNoiseSigmaSet(est_azi_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> GYRO_Z measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector gyro_z_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS);
//		gyro_z_meas_noise_Mu		  =    0.0;

//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix gyro_z_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS);
//		gyro_z_meas_noise_Cov		  =	   0.0;
//		gyro_z_meas_noise_Cov(1,1)   =   sqr(180.0 * M_PI/180.0);

//		gyro_z_meas_pdf.AdditiveNoiseMuSet(gyro_z_meas_noise_Mu);
//		gyro_z_meas_pdf.AdditiveNoiseSigmaSet(gyro_z_meas_noise_Cov);
//		//---------------------------------------------------

//		//--> ACCEL measurement uncertainty R
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector accel_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ACCEL);
//		accel_meas_noise_Mu		  =    0.0;

//		//--> Covariance
//		MatrixWrapper::SymmetricMatrix accel_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ACCEL);
//		accel_meas_noise_Cov		  =	   0.0;
//		accel_meas_noise_Cov(1,1)   =   sqr(1.0);
//		accel_meas_noise_Cov(2,2)   =   sqr(1.0);
//		accel_meas_noise_Cov(3,3)   =   sqr(1.0);

//		accel_meas_pdf.AdditiveNoiseMuSet(accel_meas_noise_Mu);
//		accel_meas_pdf.AdditiveNoiseSigmaSet(accel_meas_noise_Cov);
//		//---------------------------------------------------


//		//--> System uncertainty Q
//		//---------------------------------------------------
//		//--> Mean
//		MatrixWrapper::ColumnVector sys_noise_Mu(NUMBER_OF_STATES);
//		sys_noise_Mu		 =    0.0;

//		//--> Covariance
//		sys_noise_Cov	     =  0.0;
////		sys_noise_Cov(ROLL,ROLL)	         =   1E+1 * 1e-3;
////		sys_noise_Cov(PITCH,PITCH)           =   1E+1 * 1e-3;
////		sys_noise_Cov(YAW,YAW)               =   1E+1 * 1e-3;
////		sys_noise_Cov(PX,PX)                 =   0; // 1E+2 * 1e-3; // 1E-2;
////		sys_noise_Cov(PY,PY)                 =   0; // 1E+2 * 1e-3; // 1.5E-2;
////		sys_noise_Cov(PZ,PZ)                 =   0; // 1E+2 * 1e-3;
////		sys_noise_Cov(VX,VX)                 =   1E+1 * 1e-3;
////		sys_noise_Cov(VY,VY)                 =   1E+1 * 1e-3;
////		sys_noise_Cov(VZ,VZ)                 =   1E+1 * 1e-3;
////		sys_noise_Cov(BIAS_AZ,BIAS_AZ)       =   1E-3 * 1e-3;
////		sys_noise_Cov(BIAS_WX,BIAS_WX)       =   1E-4 * 1e-3;
////		sys_noise_Cov(BIAS_WY,BIAS_WY)       =   1E-4 * 1e-3;
////		sys_noise_Cov(BIAS_WZ,BIAS_WZ)       =   1E-4 * 1e-3;
//		sys_noise_Cov(ROLL,ROLL)	         =  sqr(5.0 * M_PI/180.0);
//		sys_noise_Cov(PITCH,PITCH)         =  sqr(5.0 * M_PI/180.0);
//		sys_noise_Cov(YAW,YAW)             =  sqr(5.0 * M_PI/180.0);
//		sys_noise_Cov(PX,PX)               =  sqr(1.0);
//		sys_noise_Cov(PY,PY)               =  sqr(1.0);
//		sys_noise_Cov(PZ,PZ)               =  sqr(1.0);
//		sys_noise_Cov(VX,VX)               =  sqr(1.0);
//		sys_noise_Cov(VY,VY)               =  sqr(1.0);
//		sys_noise_Cov(VZ,VZ)      	       =  sqr(1.0);
//		sys_noise_Cov(BIAS_AZ,BIAS_AZ)     =  sqr(1.0e-6);
//		sys_noise_Cov(BIAS_WX,BIAS_WX)     =  sqr(5.0e-6 * M_PI/180.0);
//		sys_noise_Cov(BIAS_WY,BIAS_WY)     =  sqr(5.0e-6 * M_PI/180.0);
//		sys_noise_Cov(BIAS_WZ,BIAS_WZ)     =  sqr(5.0e-6 * M_PI/180.0);

//		sys_noise_Cov_ALIGNMENT = sys_noise_Cov;
//		sys_noise_Cov_ALIGNMENT(BIAS_AZ,BIAS_AZ) =  sqr(1.0e-3);
//		sys_noise_Cov_ALIGNMENT(BIAS_WX,BIAS_WX) =  sqr(5.0e-3 * M_PI/180.0);
//		sys_noise_Cov_ALIGNMENT(BIAS_WY,BIAS_WY) =  sqr(5.0e-3 * M_PI/180.0);
//		sys_noise_Cov_ALIGNMENT(BIAS_WZ,BIAS_WZ) =  sqr(5.0e-3 * M_PI/180.0);

//		sys_pdf.AdditiveNoiseMuSet(sys_noise_Mu);
//		sys_pdf.AdditiveNoiseSigmaSet(sys_noise_Cov_ALIGNMENT);
//		sys_pdf.setEarthData(RmH, RnH, -LocalGravity);
//		//---------------------------------------------------
	
//		//-->  Initialize EKF
//		//---------------------------------------------------
//		//--> Continuous Gaussian prior (for Kalman filter)
//		//--> X0

//		MatrixWrapper::ColumnVector prior_Mu(NUMBER_OF_STATES);
//		prior_Mu = 0.0;
	
//		//--> P0
//		MatrixWrapper::SymmetricMatrix prior_Cov(NUMBER_OF_STATES);
//		prior_Cov		 =    0.0;
////		prior_Cov(ROLL,ROLL)             =   1E-1 * 1e-3;
////		prior_Cov(PITCH,PITCH)		     =   1E-1 * 1e-3;
////		prior_Cov(YAW,YAW)               =   1E-1 * 1e-3;
////		prior_Cov(PX,PX)                 =   1E-3 * 1e-3;
////		prior_Cov(PY,PY)                 =   1E-3 * 1e-3;
////		prior_Cov(PZ,PZ)                 =   1E-3 * 1e-3;
////		prior_Cov(VX,VX)                 =    0.0 * 1e-3;
////		prior_Cov(VY,VY)                 =    0.0 * 1e-3;
////		prior_Cov(VZ,VZ)                 =    0.0 * 1e-3;
////		prior_Cov(BIAS_AZ,BIAS_AZ)       =   5E-1 * 1e-3;
////		prior_Cov(BIAS_WX,BIAS_WX)       =   5E-1 * 1e-3;
////		prior_Cov(BIAS_WY,BIAS_WY)       =   5E-1 * 1e-3;
////		prior_Cov(BIAS_WZ,BIAS_WZ)       =   5E-1 * 1e-3;
//		prior_Cov(ROLL,ROLL)		     =  sqr(1e2 * M_PI/180.0);
//		prior_Cov(PITCH,PITCH)		   =  sqr(1e2 * M_PI/180.0);
//		prior_Cov(YAW,YAW)		       =  sqr(1e2 * M_PI/180.0);
//		prior_Cov(PX,PX)             =  sqr(1e3);
//		prior_Cov(PY,PY)             =  sqr(1e3);
//		prior_Cov(PZ,PZ)             =  sqr(1e3);
//		prior_Cov(VX,VX)             =  sqr(1e3);
//		prior_Cov(VY,VY)             =  sqr(1e3);
//		prior_Cov(VZ,VZ)             =  sqr(1e3);
//		prior_Cov(BIAS_AZ,BIAS_AZ)   =  sqr(0.1);
//		prior_Cov(BIAS_WX,BIAS_WX)   =  sqr(1.0 * M_PI/180.0);
//		prior_Cov(BIAS_WY,BIAS_WY)   =  sqr(1.0 * M_PI/180.0);
//		prior_Cov(BIAS_WZ,BIAS_WZ)   =  sqr(1.0 * M_PI/180.0);

//		prior_cont.ExpectedValueSet(prior_Mu);
//		prior_cont.CovarianceSet(prior_Cov);
//		pFilter = new ExtendedKalmanFilter(&prior_cont);
//		//---------------------------------------------------

//		//--> Initialize IMU-Filter
//		imuFilter = new IMUFilter;
	
//		return true;
//	}

//	void Navigation::cleanupHook()
//	{
//		RTT::Logger::In in(getName());

//		delete pFilter;
//		delete imuFilter;
//	}

//	bool Navigation::startHook()
//	{
//		RTT::Logger::In in(getName());
//		return true;
//	}

//	void Navigation::stopHook()
//	{
//		RTT::Logger::In in(getName());
//	}

//	bool Navigation::reset()
//	{
//	  stop();
//	  cleanup();
//		if (!configure()) return false;
//		if (!start()) return false;
//	  return true;
//	}

//	void Navigation::setReferenceAltitude(double altitude) {
//	  INS_Interface.Reference.altitude = altitude;
//	  propertyReference.set(INS_Interface.Reference);
//	}

//	void Navigation::resetAltitude() {
//	  setReferenceAltitude(INS_Interface.NavData.altitude);

//		x_est = (pFilter->PostGet())->ExpectedValueGet();
//		x_est(PZ) = 0.0;
//		pFilter->PostGet()->ExpectedValueSet(x_est);
//	}

//	void Navigation::setReferencePosition(double lat, double lon) {
//		INS_Interface.Reference.lat = lat * 180.0/M_PI;
//		INS_Interface.Reference.lon = lon * 180.0/M_PI;
//	  propertyReference.set(INS_Interface.Reference);

//		INS_Interface.Reference.lat = lat;
//		INS_Interface.Reference.lon = lon;

//	  //--> Calculate Earth data
//	  //--------------------------------------------------------------------------------------------------
//	  EarthCalc.CalcLocalGravity(INS_Interface.Reference.lat, INS_Interface.Reference.altitude, &LocalGravity);
//	  EarthCalc.CalcLocalRadii(INS_Interface.Reference.lat, INS_Interface.Reference.altitude, &RmH, &RnH);
//	  //--------------------------------------------------------------------------------------------------

//	  //--> Get magnetic declination and field vector from magnetic world model
//	  Magnetometer3D.GetDeclination(INS_Interface.Reference.lat,INS_Interface.Reference.lon,INS_Interface.NavData.altitude,CURRENT_YEAR);
//	  Magnetometer3D.GetNormalizedMagnetFieldVector(&normalizedMagneticFieldVector(1),
//												    &normalizedMagneticFieldVector(2),
//												    &normalizedMagneticFieldVector(3));

//	  //--> Set normalized magnetic field vector from world magnetic model
//	  normalizedMagneticFieldVector(2) = -normalizedMagneticFieldVector(2);
//	  normalizedMagneticFieldVector(3) = -normalizedMagneticFieldVector(3);
//	  mag_meas_pdf.setNormalizedMagneticFieldVector(normalizedMagneticFieldVector);
//	  //-------------------------------------------------------
//	}

//	void Navigation::resetPosition() {
//		setReferencePosition(INS_Interface.NavData.lat, INS_Interface.NavData.lon);

//		x_est = (pFilter->PostGet())->ExpectedValueGet();
//		x_est(PX) = 0.0;
//		x_est(PY) = 0.0;
//		pFilter->PostGet()->ExpectedValueSet(x_est);
//	}

//	void Navigation::updateHook()
//	{
//		CycleTime = 0.0;

//		//--> Load inertial data
//		//--------------------------------------------------------------------------------------------------
//								Data::Sensor::IMU imuData;
//                if (portIMU.read(imuData) == RTT::NewData) {
//                        INS_Interface.IMUData_available = newData;

//			//--> Charge u vector with inertial data
//			//--------------------------------------
//			if (imuFilterEnabled.get())
//			{
//				u = imuFilter->Update(imuData.AccelX,-imuData.AccelY,-imuData.AccelZ,imuData.GyroX,-imuData.GyroY,-imuData.GyroZ);
//			}
//			else
//			{
//				u(AX) = imuData.AccelX;
//				u(AY) = -imuData.AccelY;
//				u(AZ) = -imuData.AccelZ;
//				u(WX) = imuData.GyroX;
//				u(WY) = -imuData.GyroY;
//				u(WZ) = -imuData.GyroZ;
//			}
//			//--------------------------------------

//			/*//--> Synthetic IMU data generator
//			//--------------------------------------
//			u(AX) = 0.0;
//			u(AY) = 0.0;
//			u(AZ) = LocalGravity;
//			u(WX) = 0.0;
//			u(WY) = 0.0;
//			u(WZ) = 0.0;

//			static Data::Timestamp StartTime;
//			if (OperatingStatus == INS_STATUS_ALIGNMENT) {
//				StartTime = CurrentTime;
//			} else if (OperatingStatus != INS_STATUS_ALIGNMENT) {
//				u(AX) = cos(2.0 * M_PI * 0.1 * (CurrentTime - StartTime));
//			}
//			*/

//			//--> Charge IMU-Structure from INS-Interface with inertial data
//			//--------------------------------------------------------------
//			INS_Interface.IMUData.accelX = u(AX);
//			INS_Interface.IMUData.accelY = -u(AY);
//			INS_Interface.IMUData.accelZ = -u(AZ);
//			INS_Interface.IMUData.gyroX  = u(WX);
//			INS_Interface.IMUData.gyroY  = -u(WY);
//			INS_Interface.IMUData.gyroZ  = -u(WZ);
//			INS_Interface.IMUData.localGravity = LocalGravity;
//			INS_Interface.IMUData.setTimestamp(imuData);
//			//--------------------------------------------------------------

//			y_gyro_z_bias(1) = -u(WZ);
//			y_accel(1) = u(AX);
//			y_accel(2) = u(AY);
//			y_accel(3) = u(AZ);

//			CurrentTime = INS_Interface.IMUData.getTimestamp();
//			if (LastTime) CycleTime = CurrentTime - LastTime;
//			// assert(CycleTime >= 0 && CycleTime < 1.0);

//			PredictWithIMU = true;
//		}
//		else
//		{
//			//--> If no IMU data available, then stop navigtion update
//			return;
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Load barometer data
//		//--------------------------------------------------------------------------------------------------
//		Data::Sensor::Baro baroData;
//		if (portBaro.read(baroData) == RTT::NewData) {
//			INS_Interface.BaroData_available = newData;

//			//--> Calculate heights from barometer data
//			if (Barometer.SetMeasurement(baroData.pressure)) {
		
//				//--> Charge Baro-Structure from INS-Interface with barometric data
//				INS_Interface.BaroData.Pressure = baroData.pressure;
//				INS_Interface.BaroData.HeightQNH = Barometer.GetAltitudeWithQNH();
//				INS_Interface.BaroData.HeightQFE = INS_Interface.BaroData.HeightQNH - INS_Interface.Reference.altitude;
//				INS_Interface.BaroData.setTimestamp(baroData);

//				y_baro(1) = INS_Interface.BaroData.HeightQFE;

//				UpdateWithBaro = true;

//				//--> Clear BaroError flag
//				BaroError = false;

//				//--> Clear BaroTimer
//				BaroTimer = 0;

//			} else {

//				//--> Set BaroError flag due to illegal pressure value
//				BaroError = true;
//			}
//		}
//		else
//		{
//			//--> Check state of barometer
//			if (BaroTimer < BaroTimeout)
//			{
//				BaroTimer += static_cast<unsigned int>(CycleTime * 1000);
//			}
//			else
//			{
//				if (!BaroError) RTT::log(RTT::Warning) << "Baro failed." << RTT::endlog();

//				//--> Set BaroError flag
//				BaroError = true;
//			}
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Load gps data
//		//--------------------------------------------------------------------------------------------------
//                if (portGPS.read(INS_Interface.GPSData) == RTT::NewData) {
//                        INS_Interface.GPSData_available = newData;
//                        // RTT::log( RTT::Debug ) << "new GPS data (t = " << INS_Interface.GPSData.getTimestamp() << ")" << RTT::endlog();
//                }

//                if ((INS_Interface.GPSData_available == newData) &&
//                    (INS_Interface.GPSData.signalQuality >= 3) &&
//                    (INS_Interface.GPSData.numberOfSatellites >= 4))
//                {
//                        y_gps(1) = (INS_Interface.GPSData.lat - INS_Interface.Reference.lat) * RmH;
//                        y_gps(2) = -(INS_Interface.GPSData.lon - INS_Interface.Reference.lon) * RnH;
//                        y_gps(3) = INS_Interface.GPSData.v_n;
//                        y_gps(4) = -INS_Interface.GPSData.v_e;

//                        GPSLastLat = INS_Interface.GPSData.lat;
//                        GPSLastLon = INS_Interface.GPSData.lon;

//                        UpdateWithGPS = true;

//                        //--> Clear GPSError flag
//                        GPSError = false;

//                        //--> Clear Timer
//                        GPSTimer = 0;
//		}
//		else
//		{
//			//--> Check state of gps
//			if (GPSTimer < GPSTimeout)
//			{
//				GPSTimer += static_cast<unsigned int>(CycleTime * 1000);
//			}
//			else
//			{
//				if (!GPSError) RTT::log(RTT::Warning) << "GPS failed." << RTT::endlog();

//				//--> Set GPSError flag
//				GPSError = true;
				
//				//--> Set next received GPS position
//				SetNewGPSPosition = true;
//			}
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Load magnetometer data
//		//--------------------------------------------------------------------------------------------------
//                if (portMagnetic.read(INS_Interface.MagData) == RTT::NewData) {
//                        INS_Interface.MagData_available = newData;

//			//--> Correct deviation
//			INS_Interface.Compass = compass.convert(INS_Interface.MagData);

//			//--> Caclulate norm of measured magnetic field vector
//			double normOfMeasuredMagneticField = sqrt(INS_Interface.Compass.Bx*INS_Interface.Compass.Bx +
//													  INS_Interface.Compass.By*INS_Interface.Compass.By +
//													  INS_Interface.Compass.Bz*INS_Interface.Compass.Bz);

//			y_mag(1) = INS_Interface.Compass.Bx/normOfMeasuredMagneticField;
//			y_mag(2) = -INS_Interface.Compass.By/normOfMeasuredMagneticField;
//			y_mag(3) = -INS_Interface.Compass.Bz/normOfMeasuredMagneticField;

//			//--> Caclulate compass information from magnetometer data
//			Magnetometer3D.SetMeasurements(&y_mag(1),&y_mag(2),&y_mag(3),&INS_Interface.NavData.rol, &INS_Interface.NavData.pitch);
		
//			//--> Charge Compass-Structure from INS-Interface with magnetometer data
//			INS_Interface.Compass.MagHeading = -Magnetometer3D.GetMagHeading();
//			INS_Interface.Compass.Declination = -Magnetometer3D.GetDeclination();
//			INS_Interface.Compass.setTimestamp(INS_Interface.MagData);

//			UpdateWithMag = true;

//			//--> Clear MagError flag
//			MagError = false;

//			//--> Clear MagTimer
//			MagTimer = 0;
//		}
//		else
//		{
//			//--> Check state of magnetometer
//			if (MagTimer < MagTimeout)
//			{
//				MagTimer += static_cast<unsigned int>(CycleTime * 1000);
//			}
//			else
//			{
//				if (!MagError) RTT::log(RTT::Warning) << "Magnetometer failed." << RTT::endlog();

//				//--> Set MagError flag
//				MagError = true;
//			}
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Load supply data
//		//--------------------------------------------------------------------------------------------------
//		Data::Sensor::Supply supply;
//		if (portSupply.read(supply) == RTT::NewData) {
//			INS_Interface.NavData.inputVoltage = supply.voltage;
//			INS_Interface.NavData.inputCurrent = static_cast<unsigned short>(supply.currentMain + supply.currentMotors);
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Navigation Control FSM
//		//--------------------------------------------------------------------------------------------------
//		if (OperatingStatus == INS_STATUS_ALIGNMENT)
//			{
//				AlignmentTimer += static_cast<unsigned int>(CycleTime * 1000);
				
//				//--> Do alignment
//				if (AlignmentTimer < AlignmentTimeout)
//				{
//					//--> Do fix position updates
//					//--------------------------------------------------------------------------
////					if (true)
////					{
////						FIX_POS_Timer += static_cast<unsigned int>(CycleTime * 1000);

////						//--> Force fix position updates (lat,lon) at a certain interval
////						//--------------------------------------------------------------------------
////						if (FIX_POS_Timer >= FIX_POS_Interval)
////						{
////							FIX_POS_Timer = 0;
////							UpdateWithFIX_POS = true;
////						}
////						//--------------------------------------------------------------------------
////					}
//					//--------------------------------------------------------------------------
					
//					//--> Do zero velocity updates
//					//--------------------------------------------------------------------------
//					//--> Avoid updates with real GPS data during alignment
//					UpdateWithGPS = false;
//					UpdateWithZVEL_NE = true;
////					if (true)
////					{
////						ZVEL_NE_Timer += static_cast<unsigned int>(CycleTime * 1000);

////						//--> Force zero velocity updates (v_North,v_East) at a certain interval
////						//--------------------------------------------------------------------------
////						if (ZVEL_NE_Timer >= ZVEL_NE_Interval)
////						{
////							ZVEL_NE_Timer = 0;
////							UpdateWithZVEL_NE = true;
////						}
////						//--------------------------------------------------------------------------
////					}
//					//--------------------------------------------------------------------------

//					//--> Check state of baro
//					//--------------------------------------------------------------------------
//					UpdateWithBaro = false;
//					UpdateWithZVEL_D = true;
////					if (true) // (BaroError)
////					{
////						ZVEL_D_Timer += static_cast<unsigned int>(CycleTime * 1000);

////						//--> Force zero velocity updates (v_Down) at a certain interval
////						//--------------------------------------------------------------------------
////						if (ZVEL_D_Timer >= ZVEL_D_Interval)
////						{
////							ZVEL_D_Timer = 0;
////							UpdateWithZVEL_D = true;
////						}
////						//--------------------------------------------------------------------------
////					}
////					else
////						ZVEL_D_Timer = 0;
//					//--------------------------------------------------------------------------
				
//					//--> Check state of magnetometer
//					//--------------------------------------------------------------------------
//					UpdateWithMag = false;
//					UpdateWithEST_AZI = true;
////					if (true) // (MagError)
////					{
////						EST_AZI_Timer += static_cast<unsigned int>(CycleTime * 1000);

////						//--> Force update with estimated azimuth at a certain interval
////						//--------------------------------------------------------------------------
////						if (EST_AZI_Timer >= EST_AZI_Interval)
////						{
////							EST_AZI_Timer = 0;
////							UpdateWithEST_AZI = true;
////						}
////						//--------------------------------------------------------------------------
////					}
////					else
////						EST_AZI_Timer = 0;
//					//--------------------------------------------------------------------------
//				}
//				else
//				{
//					x_est = (pFilter->PostGet())->ExpectedValueGet();

//					//--> Set magnetic heading as azimuth after alignment cycle only if magnetometer is ok
//					if (!MagError)
//					{
//						double magHeading = INS_Interface.Compass.MagHeading + INS_Interface.Compass.Declination;
//						x_est(YAW) = -magHeading;			      // azi
//					}

//					if (!BaroError) {
//						setReferenceAltitude(INS_Interface.BaroData.HeightQNH);
//						x_est(PZ) = 0.0;
//						y_baro(1) = 0.0;
//					}

//					pFilter->PostGet()->ExpectedValueSet(x_est);

//					//--> Set new system noise covariance
//					sys_pdf.AdditiveNoiseSigmaSet(sys_noise_Cov);
					
//					//--> Clear Timer
//					AlignmentTimer = 0;
//					FIX_POS_Timer = 0;
//					ZVEL_NE_Timer = 0;
//					ZVEL_D_Timer = 0;
//					EST_AZI_Timer = 0;

//					//--> Change operating state
//					OperatingStatus = INS_STATUS_DEGRADED_NAV;

//					RTT::log(RTT::Info) << "Alignment finished." << RTT::endlog();
//				}
//			}
			
//		if (OperatingStatus == INS_STATUS_DEGRADED_NAV)
//			{
//				//--> Check state of GPS
//				//--------------------------------------------------------------------------
//				if (UpdateWithGPS)
//				{
//					//--> Set new GPS postion if GPS data is available after a GPSError
//					//--------------------------------------------------------------------------
//					if (SetNewGPSPosition)
//					{
//						x_est = (pFilter->PostGet())->ExpectedValueGet();
//						x_est(PX) = 0.0;
//						x_est(PY) = 0.0;
//						x_est(VX) = INS_Interface.GPSData.v_n;
//						x_est(VY) = -INS_Interface.GPSData.v_e;
//						pFilter->PostGet()->ExpectedValueSet(x_est);

//						setReferencePosition(INS_Interface.GPSData.lat, INS_Interface.GPSData.lon);

//						SetNewGPSPosition = false;
//					}
//					//--------------------------------------------------------------------------
//				}
//				else
//				{
//					if (GPSError && enableZeroVelocityNE)
//					{
//						ZVEL_NE_Timer += static_cast<unsigned int>(CycleTime * 1000);

//						//--> Force zero velocity updates (v_North,v_East) at a certain interval
//						//--------------------------------------------------------------------------
//						if (ZVEL_NE_Timer >= ZVEL_NE_Interval)
//						{
//							ZVEL_NE_Timer = 0;
//							UpdateWithZVEL_NE = true;
//						}
//						//--------------------------------------------------------------------------
//					}
//					else
//						ZVEL_NE_Timer = 0;
//				}
//				//--------------------------------------------------------------------------
				
//				//--> Check state of baro
//				//--------------------------------------------------------------------------
//				if (BaroError && enableZeroVelocityD)
//				{
//					ZVEL_D_Timer += static_cast<unsigned int>(CycleTime * 1000);

//					//--> Force zero velocity updates (v_Down) at a certain interval
//					//--------------------------------------------------------------------------
//					if (ZVEL_D_Timer >= ZVEL_D_Interval)
//					{
//						ZVEL_D_Timer = 0;
//						UpdateWithZVEL_D = true;
//					}
//					//--------------------------------------------------------------------------
//				}
//				else
//					ZVEL_D_Timer = 0;
//				//--------------------------------------------------------------------------
				
//				//--> Check state of magnetometer
//				//--------------------------------------------------------------------------
//				if (MagError && enableZeroGyroZ)
//				{
//					EST_AZI_Timer += static_cast<unsigned int>(CycleTime * 1000);

//					//--> Force update with estimated azimuth until magnetometer data is available
//					//--------------------------------------------------------------------------
//					if (EST_AZI_Timer >= EST_AZI_Interval)
//					{
//						EST_AZI_Timer = 0;
//						UpdateWithEST_AZI = true;
//					}
//					//--------------------------------------------------------------------------
//				}
//				else
//					EST_AZI_Timer = 0;
//				//--------------------------------------------------------------------------
				
//				//--> Change operating state from DEGRADED_NAV to NAV_READY if there are no sensor errors
//				//--------------------------------------------------------------------------
//				if (!(GPSError) && !(BaroError) && !(MagError)) {
//				  OperatingStatus = INS_STATUS_NAV_READY;
//				  RTT::log(RTT::Info) << "Navigation ready." << RTT::endlog();
//				}

//				//--------------------------------------------------------------------------
//			}

//		if (OperatingStatus == INS_STATUS_NAV_READY)
//			{
//				//--> Change operating status from NAV_READY to DEGRADED_NAV and start with
//				//--> zero velocity updates (v_North,v_East) if GPSError is detected
//				//--------------------------------------------------------------------------
//				if (GPSError)
//				{
//					ZVEL_NE_Timer = 0;
//					UpdateWithZVEL_NE = true;

//					//--> Change operating state
//					OperatingStatus = INS_STATUS_DEGRADED_NAV;
//					RTT::log(RTT::Info) << "Navigation degraded (no GPS)." << RTT::endlog();
//				}
//				//--------------------------------------------------------------------------
				
//				//--> Change operating status from NAV_READY to DEGRADED_NAV and start with
//				//--> zero velocity updates (v_Down) if BaroError is detected
//				//--------------------------------------------------------------------------
//				if (BaroError)
//				{
//					ZVEL_D_Timer = 0;
//					UpdateWithZVEL_D = true;

//					//--> Change operating state
//					OperatingStatus = INS_STATUS_DEGRADED_NAV;
//					RTT::log(RTT::Info) << "Navigation degraded (no Baro)." << RTT::endlog();
//				}
//				//--------------------------------------------------------------------------
				
//				//--> Change operating status from NAV_READY to DEGRADED_NAV and start with
//				//--> updates using estimated azimuth if MagError is detected
//				//--------------------------------------------------------------------------
//				if (MagError)
//				{
//					EST_AZI_Timer = 0;
//					UpdateWithEST_AZI = true;

//					//--> Change operating state
//					OperatingStatus = INS_STATUS_DEGRADED_NAV;
//					RTT::log(RTT::Info) << "Navigation degraded (no Magnetometer)." << RTT::endlog();
//				}
//				//--------------------------------------------------------------------------
//			}

//		//--> Set timestamp for NavData
//		//--------------------------------------------------------------------------------------------------
//		INS_Interface.NavData.time = static_cast<unsigned long>(CurrentTime.toSeconds() * 1000);
//		//--------------------------------------------------------------------------------------------------

//		//--> Remember a-posteriori heading for EST_AZI updates BEFORE time update
//		//--------------------------------------------------------------------------------------------------
//		x_est = (pFilter->PostGet())->ExpectedValueGet();
//		y_est_azi(1) = x_est(YAW);

//		//--> Check if we have an external pose update and therefore don't need the zero velocity updates
//		//--------------------------------------------------------------------------------------------------
//		if (ExternalPoseUpdate) {
//			UpdateWithZVEL_NE = false;
//			UpdateWithEST_AZI = false;
//		}

//		//--> Propagation step
//		//--------------------------------------------------------------------------------------------------
//		if (PredictWithIMU)
//		{
//			RTT::log(RTT::RealTime) << "Predict (ZVEL_NE = " << (UpdateWithZVEL_NE ? "true" : "false") << ", ZVEL_D = " << (UpdateWithZVEL_D ? "true" : "false") << ")" << RTT::endlog();

//			sys_pdf.enableZVEL_NE(UpdateWithZVEL_NE);
//			sys_pdf.enableZVEL_D(UpdateWithZVEL_D);

//			sys_pdf.setDt(CycleTime);
//			pFilter->Update(&sys_model,u);

//			PredictWithIMU = false;
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Update filter with measurements
//		//--------------------------------------------------------------------------------------------------
//#ifdef HAVE_POSEUPDATE
//		if (enablePoseUpdate && OperatingStatus != INS_STATUS_ALIGNMENT) {
//			poseUpdate.update();
//		}
//#endif // HAVE_POSEUPDATE

//		if (UpdateWithBaro) {
//			RTT::log(RTT::RealTime) << "UpdateWithBaro y = " << y_baro(1) << RTT::endlog();

//			x_est = (pFilter->PostGet())->ExpectedValueGet();
//			if (baroMaxFilterError > 0.0 && fabs(y_baro(1) - x_est(6)) > baroMaxFilterError) {
//				x_est(PZ) = y_baro(1);
//				pFilter->PostGet()->ExpectedValueSet(x_est);
//			} else {
//				pFilter->Update(&baro_meas_model,y_baro);
//			}

//			UpdateWithBaro = false;
//		}

//		if (UpdateWithGPS)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithGPS" << RTT::endlog();
//			pFilter->Update(&gps_meas_model,y_gps);

////			// use v_down from GPS too
////			y_zvel_d(1) = -INS_Interface.GPSData.v_d;
////			pFilter->Update(&zvel_d_meas_model,y_zvel_d);

//			UpdateWithGPS = false;
//		}

//		if (UpdateWithMag)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithMag" << RTT::endlog();
//			pFilter->Update(&mag_meas_model,y_mag);
//			UpdateWithMag = false;
//		}

//		if (UpdateWithFIX_POS)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithFIX_POS" << RTT::endlog();
//			y_fix_pos(1) = 0.0;
//			y_fix_pos(2) = 0.0;
//			pFilter->Update(&fix_pos_meas_model,y_fix_pos);
//			UpdateWithFIX_POS = false;
//		}

//		if (UpdateWithZVEL_NE)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithZVEL_NE" << RTT::endlog();
//			y_zvel_ne(1) = 0.0;
//			y_zvel_ne(2) = 0.0;
//			// pFilter->Update(&zvel_ne_meas_model,y_zvel_ne);
//			pFilter->Update(&accel_meas_model, y_accel);
//			UpdateWithZVEL_NE = false;
//		}

//		if (UpdateWithZVEL_D)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithZVEL_D" << RTT::endlog();
//			y_zvel_d(1) = 0.0;
//			// pFilter->Update(&zvel_d_meas_model,y_zvel_d);
//			UpdateWithZVEL_D = false;
//		}

//		if (UpdateWithEST_AZI)
//		{
//			RTT::log(RTT::RealTime) << "UpdateWithEST_AZI" << RTT::endlog();
//			// pFilter->Update(&est_azi_meas_model,y_est_azi);
//			pFilter->Update(&gyro_z_meas_model,y_gyro_z_bias);

//			UpdateWithEST_AZI = false;
//		}
//		//--------------------------------------------------------------------------------------------------

//		//--> Save filtered data and write navigation solution
//		//--------------------------------------------------------------------------------------------------
//		x_est = pFilter->PostGet()->ExpectedValueGet();
//		P_est = pFilter->PostGet()->CovarianceGet();

//		INS_Interface.NavData.rol                = x_est(ROLL);
//		INS_Interface.NavData.pitch              = -x_est(PITCH);
//		INS_Interface.NavData.azimuth            = -x_est(YAW);
//		Conversion::Rotation::body2euler(INS_Interface.NavData, &INS_Interface.NavData.rol_dot, u(WX) + x_est(BIAS_WX), -(u(WY) + x_est(BIAS_WY)), -(u(WZ) + x_est(BIAS_WZ)));
////		INS_Interface.NavData.rol_dot            =  (u(WX) + x_est(BIAS_WX));
////		INS_Interface.NavData.pitch_dot          = -(u(WY) + x_est(BIAS_WY));
////		INS_Interface.NavData.azimuth_dot        = -(u(WZ) + x_est(BIAS_WZ));
//		INS_Interface.NavData.lat                =  x_est(PX) / RmH + INS_Interface.Reference.lat;
//		INS_Interface.NavData.lon                = -x_est(PY) / RnH + INS_Interface.Reference.lon;
//		INS_Interface.NavData.altitude           =  x_est(PZ) + INS_Interface.Reference.altitude;
//		INS_Interface.NavData.v_n                =  x_est(VX);
//		INS_Interface.NavData.v_e                = -x_est(VY);
//		INS_Interface.NavData.v_d                = -x_est(VZ);
//		Conversion::Vector::body2nav(INS_Interface.NavData, &INS_Interface.NavData.a_n, (u(AX) + x_est(BIAS_AX)), -(u(AY) + x_est(BIAS_AY)),  -(u(AZ) + x_est(BIAS_AZ)) + LocalGravity);
////		INS_Interface.NavData.a_x                =  (u(AX) + x_est(BIAS_AX));
////		INS_Interface.NavData.a_y                = -(u(AY) + x_est(BIAS_AY));
////		INS_Interface.NavData.a_z                = -(u(AZ) + x_est(BIAS_AZ));
//		INS_Interface.NavData.maxEstSigmaPos     = sqrt(P_est(PX,PX) + P_est(PY,PY));
//		INS_Interface.NavData.maxEstSigmaHeight  = sqrt(P_est(PZ,PZ));
//		INS_Interface.NavData.maxEstSigmaAtt     = sqrt(P_est(ROLL,ROLL) + P_est(PITCH,PITCH));
//		INS_Interface.NavData.maxEstSigmaAzimuth = sqrt(P_est(YAW,YAW));
//		INS_Interface.NavData.insStatus          = OperatingStatus;
//		INS_Interface.NavData_available          = newData;

//		INS_Interface.IMUData.estBiasAccelX      =  x_est(BIAS_AX);
//		INS_Interface.IMUData.estBiasAccelY      = -x_est(BIAS_AY);
//		INS_Interface.IMUData.estBiasAccelZ      = -x_est(BIAS_AZ);
//		INS_Interface.IMUData.estBiasGyroX       =  x_est(BIAS_WX);
//		INS_Interface.IMUData.estBiasGyroY       = -x_est(BIAS_WY);
//		INS_Interface.IMUData.estBiasGyroZ       = -x_est(BIAS_WZ);

//		INS_Interface.NavData.setTimestamp(INS_Interface.IMUData);
//        portNavigationSolution.write(INS_Interface.NavData);
//		//--------------------------------------------------------------------------------------------------

//		//--> Write local navigation solution
//		//--------------------------------------------------------------------------------------------------
//		INS_Interface.LocalData.time		           = INS_Interface.NavData.time;
//		INS_Interface.LocalData.rol                = x_est(ROLL);
//		INS_Interface.LocalData.pitch              = x_est(PITCH);
//		INS_Interface.LocalData.azimuth            = x_est(YAW);
//		INS_Interface.LocalData.rol_dot            = u(WX) + x_est(BIAS_WX);
//		INS_Interface.LocalData.pitch_dot          = u(WY) + x_est(BIAS_WY);
//		INS_Interface.LocalData.azimuth_dot        = u(WZ) + x_est(BIAS_WZ);
//		INS_Interface.LocalData.x                  = x_est(PX);
//		INS_Interface.LocalData.y                  = x_est(PY);
//		INS_Interface.LocalData.z                  = x_est(PZ);
//		INS_Interface.LocalData.v_x                = x_est(VX);
//		INS_Interface.LocalData.v_y                = x_est(VY);
//		INS_Interface.LocalData.v_z                = x_est(VZ);
//		INS_Interface.LocalData.a_x                = u(AX) + x_est(BIAS_AX);
//		INS_Interface.LocalData.a_y                = u(AY) + x_est(BIAS_AY);
//		INS_Interface.LocalData.a_z                = u(AZ) + x_est(BIAS_AZ);
//		INS_Interface.LocalData.maxEstSigmaPos     = INS_Interface.NavData.maxEstSigmaPos;
//		INS_Interface.LocalData.maxEstSigmaHeight  = INS_Interface.NavData.maxEstSigmaHeight;
//		INS_Interface.LocalData.maxEstSigmaAtt     = INS_Interface.NavData.maxEstSigmaAtt;
//		INS_Interface.LocalData.maxEstSigmaAzimuth = INS_Interface.NavData.maxEstSigmaAzimuth;
//		INS_Interface.LocalData.inputVoltage       = INS_Interface.NavData.inputVoltage;
//		INS_Interface.LocalData.inputCurrent       = INS_Interface.NavData.inputCurrent;
//		INS_Interface.LocalData.insStatus          = INS_Interface.NavData.insStatus;

//		INS_Interface.LocalData.setTimestamp(INS_Interface.IMUData);
//		portLocalNavigationSolution.write(INS_Interface.LocalData);
//		//--------------------------------------------------------------------------------------------------

//// 		//--> Write navigation variance
//// 		//--------------------------------------------------------------------------------------------------
//// 		for(int i = 0; i < NUMBER_OF_STATES; ++i)
//// 			for(int j = 0; j < NUMBER_OF_STATES; ++j)
//// 				INS_Interface.NavVariance.vector[i*NUMBER_OF_STATES+j] = P_est(i+1,j+1);
//// 		INS_Interface.NavVariance.setTimestamp(INS_Interface.NavData);
//// 		portNavigationVariance.write(INS_Interface.NavVariance);

//		//--> Reset cycle
//		//--------------------------------------------------------------------------------------------------
//		if (INS_Interface.IMUData_available == newData)
//			portBiasedIMU.write(INS_Interface.IMUData);

//		if (INS_Interface.MagData_available == newData)
//			portCompass.write(INS_Interface.Compass);

//		if (INS_Interface.BaroData_available == newData)
//			portAltimeter.write(INS_Interface.BaroData);

//		if (INS_Interface.NavData_available == newData)
//			INS_Interface.NavData_available = usedData;

//		if (INS_Interface.IMUData_available == newData)
//			INS_Interface.IMUData_available = usedData;

//		if (INS_Interface.GPSData_available == newData)
//			INS_Interface.GPSData_available = usedData;

//		if (INS_Interface.BaroData_available == newData)
//			INS_Interface.BaroData_available = usedData;

//		if (INS_Interface.MagData_available == newData)
//			INS_Interface.MagData_available = usedData;

//		LastTime = CurrentTime;
//		//--------------------------------------------------------------------------------------------------
//	}
//}; // namespace Navigation

//#include <rtt/Component.hpp>
//ORO_LIST_COMPONENT_TYPE( Navigation::Navigation )
//ORO_CREATE_COMPONENT_LIBRARY()


#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <ar_pose/ARMarkers.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace pattern_pose_estimation {

class MarkerFilterNode {

private:

	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	// best:	use the pose of the marker with highest confidence
	// wmean:	compute a weighted mean using the confidence to compute the weight of each marker pose
	// kbags:	cluster the markers with similar pose and get the most confident marker from the biggest bag
	std::string filter_mode_;

	std::string parent_frame_id_;
	std::string frame_id_;
	std::string marker_msg_;
	bool publish_tf_;

	double kbags_lin_thd_;// factor used when computing the maximum distance between elements of the same bag in mbags mode
	double kbags_ang_thd_;// threshold used when computing the maximum difference in terms of roll, pitch and yaw in mbags mode

	bool use_wwf_;				// filter the resultant pose using a weighted window filter (wwf)
	double ramp_wwf_;			// ramp used for the wwf
	int num_samples_wwf_;		// number of samples used for the wwf
	bool xyz_limits_;			// scene filter limits
	double x_lim_;
	double y_lim_;
	double z_lim_;


	double * x_array, * y_array, * z_array/*, * roll_array, * pitch_array, * yaw_array*/;
	double * sR_array, * cR_array, * sP_array, * cP_array, * sY_array, * cY_array;
	double * t_array, * w_array; // times and weights
	int wwf_index;
	bool full_arrays;

	ros::Subscriber markers_sub_;
	ros::Publisher pose_pub_;

	tf::TransformBroadcaster broadcaster_;
	tf::Transform prev_transform_;

public:

	MarkerFilterNode() : nh_(), nh_priv_("~") {
		init();
	}

	void init() {
		nh_priv_.param("filter_mode", filter_mode_, std::string("best"));
		ROS_INFO("\tFilter mode: %s", filter_mode_.c_str());

		nh_priv_.param("kbags_lin_thd", kbags_lin_thd_, 0.1); // error of 10cm at 1 meter distance
		ROS_INFO("\tkbags factor for lin. thd: %f", kbags_lin_thd_);

		nh_priv_.param("kbags_ang_thd", kbags_ang_thd_, 10 * M_PI / 180.0); // error of 10 degrees
		ROS_INFO("\tkbags angular thd: %f", kbags_ang_thd_);

		nh_priv_.param("use_wwf", use_wwf_, false);
		//ROS_INFO ("\tUse Weighted Window Filter: %d", use_wwf_);
		ROS_INFO_STREAM("\tUse Weighted Window Filter? " << (use_wwf_ ? "true" : "false"));

		nh_priv_.param("ramp_wwf", ramp_wwf_, 0.5);
		ROS_INFO("\tRamp WWF: %f", ramp_wwf_);

		nh_priv_.param("num_samples_wwf", num_samples_wwf_, 10);
		ROS_INFO("\tNum samples WWF: %d", num_samples_wwf_);

		nh_priv_.param("publish_tf", publish_tf_, true);
		ROS_INFO_STREAM("\tPublish transforms? " << (publish_tf_ ? "true" : "false"));

		nh_priv_.param("parent_frame_id", parent_frame_id_, std::string(""));
		ROS_INFO("\tParent frame id: %s", parent_frame_id_.c_str());

		nh_priv_.param("frame_id", frame_id_, std::string("marker"));
		ROS_INFO("\tFrame id: %s", frame_id_.c_str());

		nh_priv_.param("marker_msg", marker_msg_, std::string("ar_pose_markers"));
  		ROS_INFO ("\tMarker Message: %s", marker_msg_.c_str());

		nh_priv_.param("xyz_limits", xyz_limits_, false);
		nh_priv_.param("x_lim", x_lim_, 0.5);
		nh_priv_.param("y_lim", y_lim_, 0.5);
		nh_priv_.param("z_lim", z_lim_, 1.2);
  		ROS_INFO_STREAM ("\tFilter limirs: " << xyz_limits_);
  		ROS_INFO_STREAM ("\t\tX: " << x_lim_);
  		ROS_INFO_STREAM ("\t\tY: " << y_lim_);
  		ROS_INFO_STREAM ("\t\tZ: " << z_lim_);

		if (publish_tf_) {
			markers_sub_ = nh_.subscribe(marker_msg_, 0, &MarkerFilterNode::markersCallback, this);
		}

		//fill circular arrays with zeros
		wwf_index = 0;
		x_array = new double[num_samples_wwf_];
		y_array = new double[num_samples_wwf_];
		z_array = new double[num_samples_wwf_];
		/*roll_array = new double[num_samples_wwf_];
		pitch_array = new double[num_samples_wwf_];
		yaw_array = new double[num_samples_wwf_];*/
		sR_array = new double[num_samples_wwf_];
		cR_array = new double[num_samples_wwf_];
		sP_array = new double[num_samples_wwf_];
		cP_array = new double[num_samples_wwf_];
		sY_array = new double[num_samples_wwf_];
		cY_array = new double[num_samples_wwf_];
		t_array = new double[num_samples_wwf_];
		w_array = new double[num_samples_wwf_];
		full_arrays = false;
		for(int i = 0; i< num_samples_wwf_; i++){
			x_array[i] = 0.0;
			y_array[i] = 0.0;
			z_array[i] = 0.0;
			/*roll_array[i] = 0.0;
			pitch_array[i] = 0.0;
			yaw_array[i] = 0.0;*/
			sR_array[i] = 0.0;
			cR_array[i] = 0.0;
			sP_array[i] = 0.0;
			cP_array[i] = 0.0;
			sY_array[i] = 0.0;
			cY_array[i] = 0.0;
			t_array[i] = 0.0;	//times
			w_array[i] = 0.0;	//weights
		}

		ros::SubscriberStatusCallback connect_cb = boost::bind(&MarkerFilterNode::connectCallback, this, _1);
		pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseWithCovarianceStamped>("marker_pose", 0, connect_cb, connect_cb);
		prev_transform_.setIdentity();
	}

	void connectCallback(const ros::SingleSubscriberPublisher&) {
		ROS_DEBUG("Connection callback");
		if (!publish_tf_) {
			bool has_subscribers = pose_pub_.getNumSubscribers() > 0;
			if (has_subscribers) {
				ROS_INFO("Subscribing to marker msg.");
				markers_sub_ = nh_.subscribe(marker_msg_, 0, &MarkerFilterNode::markersCallback, this);
			} else {
				ROS_INFO("Unsubscribing from marker msg.");
				markers_sub_.shutdown();
			}
		}
	}

	void markersCallback(const ar_pose::ARMarkersConstPtr& markers_msg) {
		geometry_msgs::PoseWithCovarianceStamped pose;
		pose.header = markers_msg->header;
		if (getMarkerPose(*markers_msg, pose.pose)) {

			geometry_msgs::PoseWithCovarianceStamped filt_pose;
			calculateFilteredPose(pose, filt_pose);

			pose_pub_.publish(filt_pose);
			if (publish_tf_) {

				// Sanity check
				double x = fabs(filt_pose.pose.pose.position.x);
				double y = fabs(filt_pose.pose.pose.position.y);
				double z = fabs(filt_pose.pose.pose.position.z);
				tf::Transform transform;
				if ((x >= x_lim_ || y >= y_lim_ || z >= z_lim_) && (xyz_limits_))
				{
					ROS_WARN("Marker pose outside scene!");
					transform = prev_transform_;
				}
				else
				{	
					tf::poseMsgToTF(filt_pose.pose.pose, transform);
				}			

				// If no parent frame id, use the marker message
				if (parent_frame_id_ == "")
					parent_frame_id_ = markers_msg->header.frame_id;

				tf::StampedTransform camToMarker(transform, markers_msg->header.stamp, parent_frame_id_, frame_id_);
				broadcaster_.sendTransform(camToMarker);
				prev_transform_ = transform;
			}
			//TODO: publish status ko
		} else {
			ROS_INFO("No confident marker pose found");
			//TODO: publish status ko
		}
	}

	bool getMarkerPose(const ar_pose::ARMarkers& markers_msg, geometry_msgs::PoseWithCovariance& pose_msg) {

		if (markers_msg.markers.size() == 0) {
			return false;
		}

		bool confident_marker_found = false;

		// simply take the marker pose with highest confidence,
		// this could be replaced by some weighted mean or median
		// to gain more stability
		if (filter_mode_ == "best") {

			double best_confidence = 0;
			for (size_t i = 0; i < markers_msg.markers.size(); ++i) {
				if (markers_msg.markers[i].confidence > best_confidence) {
					best_confidence = markers_msg.markers[i].confidence;
					pose_msg = markers_msg.markers[i].pose;
					confident_marker_found = true;
				}
			}
		} else if (filter_mode_ == "wmean") {

			double total_conf = 0.0;
			double mean_x, mean_y, mean_z, mean_c_roll, mean_s_roll, mean_c_pitch, mean_s_pitch, mean_c_yaw, mean_s_yaw;
			mean_x = mean_y = mean_z = 0.0;
			mean_c_roll = mean_s_roll = mean_c_pitch = mean_s_pitch = mean_c_yaw = mean_s_yaw = 0.0;

			for (size_t i = 0; i < markers_msg.markers.size(); ++i) {
				double conf = markers_msg.markers[i].confidence;
				if (conf > 0) {
					confident_marker_found = true;
				}

				mean_x += conf * markers_msg.markers[i].pose.pose.position.x;
				mean_y += conf * markers_msg.markers[i].pose.pose.position.y;
				mean_z += conf * markers_msg.markers[i].pose.pose.position.z;

				tf::Quaternion quat;
				tf::quaternionMsgToTF(markers_msg.markers[i].pose.pose.orientation, quat);
				tf::Matrix3x3 m(quat);
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);

				mean_c_roll += conf * cos(roll);
				mean_s_roll += conf * sin(roll);
				mean_c_pitch += conf * cos(pitch);
				mean_s_pitch += conf * sin(pitch);
				mean_c_yaw += conf * cos(yaw);
				mean_s_yaw += conf * sin(yaw);

				total_conf += conf;
			}

			mean_x = mean_x / total_conf;
			mean_y = mean_y / total_conf;
			mean_z = mean_z / total_conf;
			mean_c_roll = mean_c_roll / total_conf;
			mean_s_roll = mean_s_roll / total_conf;
			mean_c_pitch = mean_c_pitch / total_conf;
			mean_s_pitch = mean_s_pitch / total_conf;
			mean_c_yaw = mean_c_yaw / total_conf;
			mean_s_yaw = mean_s_yaw / total_conf;

			double mean_roll =  atan2(mean_s_roll, mean_c_roll);
			double mean_pitch =  atan2(mean_s_pitch, mean_c_pitch);
			double mean_yaw =  atan2(mean_s_yaw, mean_c_yaw);

			pose_msg.pose.position.x = mean_x;
			pose_msg.pose.position.y = mean_y;
			pose_msg.pose.position.z = mean_z;
			tf::Quaternion btQ;
			btQ = tf::createQuaternionFromRPY(mean_roll, mean_pitch, mean_yaw);
			tf::quaternionTFToMsg(btQ, pose_msg.pose.orientation);

			//TODO: compute the mean covariance matrix

		} else if (filter_mode_ == "kbags") {

			std::vector<ar_pose::ARMarker> marker_bags[markers_msg.markers.size()];
			geometry_msgs::Pose centroids[markers_msg.markers.size()];

			// 1.- group the markers using the euclidean distance (kmeans)

			for (size_t iterMarker = 0; iterMarker < markers_msg.markers.size(); ++iterMarker) {
				bool bagFound = false;
				int iterBag = 0;

				// obtaining the marker roll, pitch and yaw
				tf::Quaternion quatMarker;
				tf::quaternionMsgToTF(markers_msg.markers[iterMarker].pose.pose.orientation, quatMarker);
				tf::Matrix3x3 mMarker(quatMarker);
				double rollMarker, pitchMarker, yawMarker;
				mMarker.getRPY(rollMarker, pitchMarker, yawMarker);

				while (!bagFound) {
					if (marker_bags[iterBag].size() == 0) { // create a new bag for this marker
						marker_bags[iterBag].push_back(markers_msg.markers[iterMarker]);
						centroids[iterBag] = markers_msg.markers[iterMarker].pose.pose;
						bagFound = true;
					} else {    // the bag is not empty
						double diffX = centroids[iterBag].position.x - markers_msg.markers[iterMarker].pose.pose.position.x;
						double diffY = centroids[iterBag].position.y - markers_msg.markers[iterMarker].pose.pose.position.y;
						double diffZ = centroids[iterBag].position.z - markers_msg.markers[iterMarker].pose.pose.position.z;
						double eucDist = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

						if (eucDist <= kbags_lin_thd_ * centroids[iterBag].position.z) { // the marker is close to the bag centroid

							// obtaining the centroid roll, pitch and yaw
							tf::Quaternion quatCentroid;
							tf::quaternionMsgToTF(centroids[iterBag].orientation, quatCentroid);
							tf::Matrix3x3 mCentroid(quatCentroid);
							double rollCentroid, pitchCentroid, yawCentroid;
							mCentroid.getRPY(rollCentroid, pitchCentroid, yawCentroid);

							/*double diffRoll = abs(rollMarker - rollCentroid);
							double diffPitch = abs(pitchMarker - pitchCentroid);
							double diffYaw = abs(yawMarker - yawCentroid);*/

							double diffRoll = abs(acos(cos(rollMarker)*cos(rollCentroid) + sin(rollMarker)*sin(rollCentroid)));
							double diffPitch = abs(acos(cos(pitchMarker)*cos(pitchCentroid) + sin(pitchMarker)*sin(pitchCentroid)));
							double diffYaw = abs(acos(cos(yawMarker)*cos(yawCentroid) + sin(yawMarker)*sin(yawCentroid)));

							if ((diffRoll <= kbags_ang_thd_)
									&& (diffPitch <= kbags_ang_thd_)
									&& (diffYaw <= kbags_ang_thd_)) {
								// the marker roughly has the same orientation than the bag centroid

								marker_bags[iterBag].push_back(markers_msg.markers[iterMarker]);

								// compute the new bag centroid position (we keep the same orientation for simplicity)
								centroids[iterBag].position.x = 0.0;
								centroids[iterBag].position.y = 0.0;
								centroids[iterBag].position.z = 0.0;
								for (size_t iAux = 0; iAux < marker_bags[iterBag].size(); ++iAux) {
									centroids[iterBag].position.x += markers_msg.markers[iAux].pose.pose.position.x;
									centroids[iterBag].position.y += markers_msg.markers[iAux].pose.pose.position.y;
									centroids[iterBag].position.z += markers_msg.markers[iAux].pose.pose.position.z;
								}
								centroids[iterBag].position.x = centroids[iterBag].position.x / marker_bags[iterBag].size();
								centroids[iterBag].position.y = centroids[iterBag].position.y / marker_bags[iterBag].size();
								centroids[iterBag].position.z = centroids[iterBag].position.z / marker_bags[iterBag].size();
								bagFound = true;
							}
						}
					}
					iterBag++;
				}
			}

			//2.- look in the biggest bag for the marker with the highest confidence

			unsigned int bigBagMarkers = 0;
			double maxConfBigBag = 0.0;
			ar_pose::ARMarker best_marker;

			printf("Bags:");

			for (unsigned int iterBag = 0; iterBag < markers_msg.markers.size(); iterBag++) { // for all the bags (empty or not)

				printf(" %d", (int)(marker_bags[iterBag].size()));

				if (marker_bags[iterBag].size() >= bigBagMarkers) { // the bag is bigger or equal
					if (marker_bags[iterBag].size() > bigBagMarkers) { // the bag is bigger
						bigBagMarkers = marker_bags[iterBag].size();
						maxConfBigBag = 0.0; // max confidence must be recomputed for the new biggest bag
					}

					printf(":(");

					for (unsigned int iterMarker = 0; iterMarker < bigBagMarkers; iterMarker++) {

						printf("%d ", marker_bags[iterBag][iterMarker].id);

						if (marker_bags[iterBag][iterMarker].confidence > maxConfBigBag) {
							maxConfBigBag =	marker_bags[iterBag][iterMarker].confidence;
							best_marker = marker_bags[iterBag][iterMarker];
						}
					}
					printf(")");
				}
			}

			printf("\n");
			printf("Selected marker: %d\n", best_marker.id);

			confident_marker_found = true;
			pose_msg = best_marker.pose;
		} else {
			ROS_ERROR("Mode not available");
		}
		return confident_marker_found;
	}

	void calculateFilteredPose(geometry_msgs::PoseWithCovarianceStamped& new_pose, geometry_msgs::PoseWithCovarianceStamped& filt_pose){

		if(use_wwf_){

			filt_pose = new_pose; // the header and covariance matrix are taken from the new pose

			//save vX and vY into circular arrays of N elements
			x_array[wwf_index] = new_pose.pose.pose.position.x;
			y_array[wwf_index] = new_pose.pose.pose.position.y;
			z_array[wwf_index] = new_pose.pose.pose.position.z;

			tf::Quaternion quatN;
			tf::quaternionMsgToTF(new_pose.pose.pose.orientation, quatN);
			tf::Matrix3x3 mCentroid(quatN);
			double roll, pitch, yaw;
			mCentroid.getRPY(roll, pitch, yaw);
			sR_array[wwf_index] = sin(roll);
			cR_array[wwf_index] = cos(roll);
			sP_array[wwf_index] = sin(pitch);
			cP_array[wwf_index] = cos(pitch);
			sY_array[wwf_index] = sin(yaw);
			cY_array[wwf_index] = cos(yaw);

			//save msg_time into circular array
			ros::Time now = new_pose.header.stamp;	//<-- use of msg time
			t_array[wwf_index] = now.toSec();

			//compute weights for every element of the arrays
			int num_elem = num_samples_wwf_;
			if(!full_arrays){
				num_elem = wwf_index + 1;	//to leave weights of empty cells to zero
			}
			double w_total = 0.0;
			for(int i = 0; i < num_elem; i++){
				double t_aux = t_array[i] - t_array[wwf_index]; //<=0
				w_array[i] = std::max(0.0,ramp_wwf_*t_aux + 1);
				w_total += w_array[i];
			}
			//compute and save estimated pose
			double w_aux; // weight
			double posX, posY, posZ, sinR, cosR, sinP, cosP, sinY, cosY;
			posX = posY = posZ = sinR = cosR = sinP = cosP = sinY = cosY = 0.0;

			for(int i = 0; i < num_elem; i++){
				w_aux = w_array[i]/w_total;
				posX += w_aux * x_array[i];
				posY += w_aux * y_array[i];
				posZ += w_aux * z_array[i];
				sinR += w_aux * sR_array[i];
				cosR += w_aux * cR_array[i];
				sinP += w_aux * sP_array[i];
				cosP += w_aux * cP_array[i];
				sinY += w_aux * sY_array[i];
				cosY += w_aux * cY_array[i];
				/*oriR += w_aux * roll_array[i];
				oriP += w_aux * pitch_array[i];
				oriY += w_aux * yaw_array[i];*/
				/*oriR = atan2(sin(oriR)*cos(w_aux * roll_array[i]) + cos(oriR)*sin(w_aux * roll_array[i]),
						cos(oriR)*cos(w_aux * roll_array[i]) - sin(oriR)*sin(w_aux * roll_array[i]));
				oriP = atan2(sin(oriP)*cos(w_aux * pitch_array[i]) + cos(oriP)*sin(w_aux * pitch_array[i]),
						cos(oriP)*cos(w_aux * pitch_array[i]) - sin(oriP)*sin(w_aux * pitch_array[i]));
				oriY = atan2(sin(oriY)*cos(w_aux * yaw_array[i]) + cos(oriY)*sin(w_aux * yaw_array[i]),
						cos(oriY)*cos(w_aux * yaw_array[i]) - sin(oriY)*sin(w_aux * yaw_array[i]));*/
			}

			filt_pose.pose.pose.position.x = posX;
			filt_pose.pose.pose.position.y = posY;
			filt_pose.pose.pose.position.z = posZ;

			double oriR = atan2(sinR, cosR);
			double oriP = atan2(sinP, cosP);
			double oriY = atan2(sinY, cosY);

			tf::Quaternion btQ;
			btQ = tf::createQuaternionFromRPY(oriR, oriP, oriY);
			tf::quaternionTFToMsg(btQ, filt_pose.pose.pose.orientation);

			//increase the iterator
			wwf_index ++;
			if (wwf_index==num_samples_wwf_){
				full_arrays = true;
				wwf_index = 0;
			}
		}else{
			filt_pose = new_pose;
		}

	}
};

}

int main(int argc, char* argv[]) {
ros::init(argc, argv, "marker_filter");
pattern_pose_estimation::MarkerFilterNode mfn;
ros::spin();
}


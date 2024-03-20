#include <pc_localizer.h>
#include <chrono>
#include <stdlib.h>
#include <time.h>

 Eigen::Matrix4f randomTransform()
	{
		srand(time(NULL));

		float angle = 3.1415926 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float ax_x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float ax_y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float ax_z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		Eigen::Vector3f axis(ax_x, ax_y, ax_z);

		Eigen::AngleAxis<float> rot(angle, axis.normalized());
		Eigen::Matrix3f rotation = rot.toRotationMatrix();

		float t_x = 3.*(-0.5+static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		float t_y = 3.*(-0.5+static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		float t_z = 3.*(-0.5+static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		

		Eigen::Matrix4f T_init;
		T_init << rotation(0, 0), rotation(0, 1), rotation(0, 2), t_x,
		          rotation(1, 0), rotation(1, 1), rotation(1, 2), t_y,
		          rotation(2, 0), rotation(2, 1), rotation(2, 2), t_z,
		          0, 0, 0, 1;

		
		// pcl::transformPointCloud (*target_cloud, *target_cloud, T_init);
		return T_init;
	}

namespace pc_localizer{
	PCLocalizer::PCLocalizer(ros::NodeHandle nh):
	target_cloud (new pcl::PointCloud<pcl::PointXYZ>),
	model_cloud (new pcl::PointCloud<pcl::PointXYZ>),
	model_cloud_n (new pcl::PointCloud<pcl::PointNormal>),
	target_cloud_n (new pcl::PointCloud<pcl::PointNormal>),
	model_FPFH (new pcl::PointCloud<pcl::FPFHSignature33>),
	target_FPFH (new pcl::PointCloud<pcl::FPFHSignature33>)
	{	
		ROS_INFO("[Localizer] Creating localizer node");
		
		got_transform = false;
		param_loader(nh);

		sub_ = nh.subscribe(input_cloud_topic.c_str(), 1, &PCLocalizer::cloudCallback, this);

		pub_after_t = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud_t", 1);
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile(template_file.c_str(), *input_cloud) == -1)
		{
			ROS_FATAL("[Localizer] Unable to load model");
		}
		else
		{
			*model_cloud = *input_cloud;
			ROS_INFO("[Localizer] Input model loaded, listening for cloud on topic %s",input_cloud_topic.c_str());
		}

	}

	void PCLocalizer::process()
	{
		pcl::UniformSampling<pcl::PointXYZ> voxelize;
        voxelize.setRadiusSearch(m_leaf_size);
        voxelize.setInputCloud(model_cloud);
        voxelize.filter(*model_cloud);
 
        voxelize.setRadiusSearch(t_leaf_size);
        voxelize.setInputCloud(target_cloud);
        voxelize.filter(*target_cloud);

		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*target_cloud, *target_cloud_n);
		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*model_cloud, *model_cloud_n);

		pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> normalEst;
		normalEst.setRadiusSearch(m_norm_search);
		normalEst.setInputCloud(model_cloud_n);
		normalEst.compute(*model_cloud_n);

		normalEst.setRadiusSearch(t_norm_search);
		normalEst.setInputCloud(target_cloud_n);
		normalEst.compute(*target_cloud_n);

		pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> featureEst;
		featureEst.setRadiusSearch(fpfh_search);

		// trigger someones OCD by switching the order
		featureEst.setInputCloud(target_cloud_n);
		featureEst.setInputNormals(target_cloud_n);
		featureEst.compute(*target_FPFH);

		featureEst.setInputCloud(model_cloud_n);
		featureEst.setInputNormals(model_cloud_n);
		featureEst.compute(*model_FPFH);

		pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> localizer;
		localizer.setInputSource(model_cloud_n);
		localizer.setSourceFeatures(model_FPFH);

		localizer.setInputTarget(target_cloud_n);
		localizer.setTargetFeatures(target_FPFH);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_transformed_n(new pcl::PointCloud<pcl::PointNormal>);
		localizer.setMaximumIterations(scp_iter);
		localizer.setNumberOfSamples(scp_samples);
		localizer.setSimilarityThreshold(scp_similarity);
		localizer.setMaxCorrespondenceDistance(scp_distance);
		localizer.setInlierFraction(scp_inliers);
		localizer.align(*cloud_transformed_n);

		Eigen::Matrix4f transformationMatrix = localizer.getFinalTransformation();
		ROS_INFO("[Localizer] SampleConsensus fit %f", localizer.getFitnessScore());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*cloud_transformed_n, *cloud_transformed);

		if(localizer.getFitnessScore() < scp_threshold)
		{
			ROS_INFO("[Localizer] SCP fit is ok, do ICP");
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputSource(cloud_transformed);
			icp.setInputTarget(target_cloud);
			icp.setMaxCorrespondenceDistance (icp_distance);
			icp.setMaximumIterations (icp_iter);
			icp.setTransformationEpsilon (icp_transform_e);
			icp.setEuclideanFitnessEpsilon (icp_euclidian_fit_e);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_icp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud_transformed, *cloud_transformed_icp);
			icp.align(*cloud_transformed_icp);
			Eigen::Matrix4f transformationICP = icp.getFinalTransformation ();

			ROS_INFO("[Localizer] ICP fit %f", icp.getFitnessScore());

			final_transform = transformationICP*transformationMatrix;
			got_transform = true;
			ROS_INFO("[Localizer] Final transform");
        	std::cout << final_transform << std::endl;       	
        
			// publish using transform
			sensor_msgs::PointCloud2 final_cloud_msg;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud (*model_cloud, *cloud_final, final_transform);
			pcl::toROSMsg(*cloud_final,final_cloud_msg);
			final_cloud_msg.header.frame_id = base_frame_name.c_str();
			pub_after_t.publish(final_cloud_msg);
		}
		else
		{
			ROS_INFO("[Localizer] Fit not ok >%f, no ICP, redo sample consensus", scp_threshold);
			process();
		}
	}

	void PCLocalizer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		ROS_INFO("[Localizer] Got cloud");
		pcl::fromROSMsg(*cloud_msg, *target_cloud);
		std::vector<int> ind;
		pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, ind);
		if (target_cloud->size() > 0) 
		{
			process();			
		}
		else
		{
			ROS_WARN("[Localizer] Cloud empty after NaN removal");
		}
	}

	void PCLocalizer::param_loader(ros::NodeHandle nh) 
	{
		nh.getParam("template_file", template_file);
		nh.getParam("input_cloud_topic", input_cloud_topic);
		nh.getParam("model_leaf_size", m_leaf_size);
		nh.getParam("scan_leaf_size", t_leaf_size);
		nh.getParam("model_normals_search_radius", m_norm_search);
		nh.getParam("scan_normals_search_radius", t_norm_search);
		nh.getParam("fpfh_search_radius", fpfh_search);
		nh.getParam("max_iterations_prerej", scp_iter);
		nh.getParam("num_samples_prerej", scp_samples);
		nh.getParam("similarity_thresh_prerej", scp_similarity);
		nh.getParam("max_corr_distance_prerej", scp_distance);
		nh.getParam("inlier_fract_prerej", scp_inliers);
		nh.getParam("threshold_prerej", scp_threshold);
		nh.getParam("max_corr_distance_icp", icp_distance);
		nh.getParam("max_iterations_icp", icp_iter);
		nh.getParam("transf_e_icp", icp_transform_e);
		nh.getParam("euclidean_fit_e_icp", icp_euclidian_fit_e);
		nh.getParam("threshold_icp", icp_threshold);
		nh.getParam("base_frame_name", base_frame_name);
		nh.getParam("object_frame_name", object_frame_name);
	}
}
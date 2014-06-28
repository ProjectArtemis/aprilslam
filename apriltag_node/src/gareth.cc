struct tag {
  cv::Point2f p[4];
};

map<int, tag> tag_positions;

void init_map_mat() {
  float x = 0, y = 0;

  static bool init = false;
  if (!init) {
    int tag_id = 0;

    for (int i = 0; i < 9; i++) {
      x = 0.0f;

      for (int j = 0; j < 12; j++) {
        tag_positions[tag_id].p[0] = cv::Point2f(x + 0.152f, y);
        tag_positions[tag_id].p[1] = cv::Point2f(x + 0.152f, y + 0.152f);
        tag_positions[tag_id].p[2] = cv::Point2f(x, y + 0.152f);
        tag_positions[tag_id].p[3] = cv::Point2f(x, y);

        x += 0.152f * 2.0f;
        tag_id++;
      }

      if (i == 2 || i == 5) {
        y += 0.178f + 0.152f;
      } else {
        y += 0.152f * 2.0f;
      }
    }

    init = true;
  }
}

void refinePose(const vector<cv::Point3f>& p3D, const vector<cv::Point2f>& p2D,
                const cv::Mat& F, Pose& pose, bool& unstable, int max_iter = 20,
                float tol = 1e-4f) {
  assert(p3D.size() == p2D.size() && max_iter > 0 && tol > 0.0f);

  Matrix<float, Dynamic, 1> r(p3D.size() * 2, 1);    //  residuals
  Matrix<float, Dynamic, 6> jac(p3D.size() * 2, 6);  //  jacobians
  Matrix<float, 6, Dynamic> jacT;

  //  initial pose
  mat4 SE3 = pose.homogenous();

  unstable = false;
  int iter = 0;

  const float fx = F.at<float>(0, 0);  //  camera parameters
  const float fy = F.at<float>(1, 1);
  const float cx = F.at<float>(0, 2);
  const float cy = F.at<float>(1, 2);

  while (iter < max_iter) {
    for (size_t k = 0; k < p3D.size(); k++) {
      const vec4 pf_cam = SE3 * vector4(p3D[k].x, p3D[k].y, p3D[k].z, 1.0f);
      const float invz = 1.0f / pf_cam[2];

      vec3 h;
      h[0] = pf_cam[0] * invz;  //  project
      h[1] = pf_cam[1] * invz;
      h[2] = 1.0f;

      //  projection jacobian
      matrix<2, 3> Jproj;
      Jproj(0, 0) = invz;  //  1/z
      Jproj(1, 1) = invz;
      Jproj(0, 2) = -pf_cam[0] * (invz * invz);  //  1/(z^2)
      Jproj(1, 2) = -pf_cam[1] * (invz * invz);

      //  camera jacobian
      matrix<3, 6> Jcam;
      Jcam(0, 0) = 1.0f;
      Jcam(1, 1) = 1.0f;
      Jcam(2, 2) = 1.0f;

      Jcam.set<0, 3>(-cross_skew(vector3(pf_cam[0], pf_cam[1], pf_cam[2])));

      auto row = k * 2;
      r(row, 0) = (p2D[k].x - cx) / fx - h[0];
      r(row + 1, 0) = (p2D[k].y - cy) / fy - h[1];

      jac.block<2, 6>(row, 0) = (Jproj * Jcam).to_eigen();
    }

    //  calculate hessian
    jacT = jac.transpose();
    Matrix<float, 6, 6> H = jacT * jac;

    //  marquardt scaling
    float lambda = 0.01f;
    for (int i = 0; i < 6; i++) {
      H(i, i) *= (1.0f + lambda);
    }

    Matrix<float, 6, 6> Hinv;
    auto LU = H.fullPivLu();
    if (LU.isInvertible()) {
      Hinv = LU.inverse();
    } else {
      //  diverged
      unstable = true;
      log_w("diverged!!\n");
      return;
    }

    Matrix<float, 6, 1> twist = Hinv * jacT * r;

    vec3 t = vector3(twist[0], twist[1], twist[2]);
    vec3 w = vector3(twist[3], twist[4], twist[5]);

    mat4 update = make_homogenous_r3(rodrigues_exp(w), t);
    SE3 = update * SE3;

    if (twist.norm() < tol) {
      //  converged
      break;
    }

    iter++;
  }

  //  update the pose
  pose.q = quat::from_matrix(SE3.get<0, 0, 3, 3>());
  pose.q.normalize();

  pose.p = -(pose.q.conjugate().to_matrix() * SE3.get<0, 3, 3, 1>());
}

vector<cv::Point3f> tagPosWorld;
vector<cv::Point2f> tagPosImage;
cv::Mat tagImage;
bool hasPose = false;
Pose pnpPose;
Pose sparsePose;

void cam_callback(const sensor_msgs::Image::ConstPtr& img) {
  const uint64_t TIME =
      static_cast<uint64_t>(img->header.stamp.sec) * 1000000000 +
      img->header.stamp.nsec;

  //  camera properties
  static cv::Mat camF;
  static vector<double> distCoeffs;

  if (camF.empty()) {
    camF = cv::Mat(3, 3, CV_32FC1);
    memcpy(camF.ptr(), F_MATRIX, sizeof(float) * 3 * 3);

    for (int i = 0; i < 8; i++) {
      distCoeffs.push_back(DIST_COEFFS[i]);
    }
    ROS_INFO("Initialized camera intrinsics");
  }

  cv::Mat temp(cv::Size(img->width, img->height), CV_8UC1,
               const_cast<uchar*>(&img->data[0]), img->step);

  //  undistort
  cv::Mat currentImage(cv::Size(img->width, img->height), CV_8UC1);
  // temp.copyTo(currentImage);
  cv::undistort(temp, currentImage, camF, distCoeffs);

  //  search for april tags
  if (tagPosWorld.size() < 40) {
    log_e("performing tag search!\n");

    tagPosWorld.clear();
    tagPosImage.clear();

    static AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
    vector<AprilTags::TagDetection> detections =
        tag_detector.extractTags(currentImage);

    init_map_mat();

    for (const AprilTags::TagDetection& det : detections) {
      const int id = det.id;
      for (int i = 0; i < 4; i++) {
        cv::Point3f p;
        p.x = tag_positions[id].p[i].x;
        p.y = tag_positions[id].p[i].y;
        p.z = 0.0;
        tagPosWorld.push_back(p);
        tagPosImage.push_back(cv::Point2f(det.p[i].first, det.p[i].second));
      }
    }
  } else {
    std::vector<cv::Point2f> trackedTagImage;
    vector<unsigned char> status;
    vector<float> errors;

    //  track features
    cv::Size winSize(21, 21);
    cv::calcOpticalFlowPyrLK(
        tagImage, currentImage, tagPosImage, trackedTagImage, status, errors,
        winSize, 3,
        cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 15, 0.03));

    //  get rid of misses
    tagPosImage = trackedTagImage;

    auto it_im = tagPosImage.begin();
    auto it_w = tagPosWorld.begin();
    for (int k = 0; k < (int)tagPosImage.size(); k++) {
      cv::Point2f p = trackedTagImage[k];

      if ((p.x < 8.0f) || (p.y < 8.0f) || (p.x > currentImage.cols - 8.0f) ||
          (p.y > currentImage.rows - 8.0f)) {
        status[k] = 0;
      }

      if (!status[k]) {
        it_im = tagPosImage.erase(it_im);
        it_w = tagPosWorld.erase(it_w);
      } else {
        it_im++;
        it_w++;
      }
    }
  }

  if (tagPosWorld.size() >= 4) {
    if (!hasPose) {
      //  get pose from tags
      cv::Mat rvec(3, 1, cv::DataType<double>::type);
      cv::Mat tvec(3, 1, cv::DataType<double>::type);
      double* r = rvec.ptr<double>();
      double* t = tvec.ptr<double>();

      t[0] = t[1] = t[2] = 0.0;

      std::vector<double> temp(5, 0.0);
      cv::solvePnPRansac(tagPosWorld, tagPosImage, camF, temp, rvec, tvec,
                         false, 200, 8.0, tagPosWorld.size() * 0.6);

      pnpPoseRef.q = quat::rotation(r[0], r[1], r[2]);
      pnpPoseRef.p =
          -(pnpPoseRef.q.conjugate().to_matrix() * vector3(t[0], t[1], t[2]));

      if (!hasPose) {
        pnpPose = pnpPoseRef;
        hasPose = true;
      }
    } else {
      //  update from past iteration
      bool unstable;
      refinePose(tagPosWorld, tagPosImage, camF, pnpPose, unstable);
      if (!unstable) {
        //      do something with pose here
      }
    }
  }

  //  run other stuf here...
}

#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API
#include <opencv2/core/core.hpp>
#include <pyboostcvconverter/pyboostcvconverter.hpp>
#include <ORB_SLAM3/Atlas.h>
#include <ORB_SLAM3/ImuTypes.h>
#include <ORB_SLAM3/KeyFrame.h>
#include <ORB_SLAM3/Map.h>
#include <ORB_SLAM3/MapPoint.h>
#include <ORB_SLAM3/Tracking.h>
#include "ORBSlamPython.h"

namespace
{
// ORB_SLAM3::System keeps its tracker and atlas private and offers no accessors,
// but the full per-frame trajectory (Tracking::mlRelativeFramePoses and friends)
// and the keyframes only live there. Explicit template instantiations are exempt
// from access checking ([temp.explicit]), which lets the bindings name those
// members without patching ORB_SLAM3.
template <typename Tag, typename Tag::type Member>
struct PrivateMember
{
    friend typename Tag::type memberPointer(Tag) { return Member; }
};

struct SystemTracker
{
    typedef ORB_SLAM3::Tracking* ORB_SLAM3::System::*type;
    friend type memberPointer(SystemTracker);
};

struct SystemAtlas
{
    typedef ORB_SLAM3::Atlas* ORB_SLAM3::System::*type;
    friend type memberPointer(SystemAtlas);
};

template struct PrivateMember<SystemTracker, &ORB_SLAM3::System::mpTracker>;
template struct PrivateMember<SystemAtlas, &ORB_SLAM3::System::mpAtlas>;
}

#if (PY_VERSION_HEX >= 0x03000000)
static void* init_ar() {
#else
static void init_ar() {
#endif
    Py_Initialize();

    // import_array() returns NULL (Py3) / void (Py2) on failure. Older NumPy
    // exposed NUMPY_IMPORT_ARRAY_RETVAL for the trailing return, but it was
    // dropped in newer releases, so return the value directly.
    import_array();
#if (PY_VERSION_HEX >= 0x03000000)
    return NULL;
#endif
}

BOOST_PYTHON_MODULE(orbslam3)
{
    using boost::python::arg;

    init_ar();

    boost::python::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    boost::python::enum_<ORB_SLAM3::Tracking::eTrackingState>("TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM3::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM3::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM3::Tracking::eTrackingState::OK)
        .value("RECENTLY_LOST", ORB_SLAM3::Tracking::eTrackingState::RECENTLY_LOST)
        .value("LOST", ORB_SLAM3::Tracking::eTrackingState::LOST)
        .value("OK_KLT", ORB_SLAM3::Tracking::eTrackingState::OK_KLT);

    boost::python::enum_<ORB_SLAM3::System::eSensor>("Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD);

    // IMU measurements are optional: only the IMU_* sensor modes use them.
    const boost::python::object noImu;

    boost::python::class_<ORBSlamPython, boost::noncopyable>("System", boost::python::init<const char*, const char*, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def(boost::python::init<std::string, std::string, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def("initialize", &ORBSlamPython::initialize)
        .def("load_and_process_mono", &ORBSlamPython::loadAndProcessMono,
            (arg("image_file"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("process_image_mono", &ORBSlamPython::processMono,
            (arg("image"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("load_and_process_stereo", &ORBSlamPython::loadAndProcessStereo,
            (arg("left_image_file"), arg("right_image_file"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("process_image_stereo", &ORBSlamPython::processStereo,
            (arg("left_image"), arg("right_image"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("load_and_process_rgbd", &ORBSlamPython::loadAndProcessRGBD,
            (arg("image_file"), arg("depth_image_file"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("process_image_rgbd", &ORBSlamPython::processRGBD,
            (arg("image"), arg("depth_image"), arg("timestamp"), arg("imu_measurements") = noImu))
        .def("shutdown", &ORBSlamPython::shutdown)
        .def("is_running", &ORBSlamPython::isRunning)
        .def("reset", &ORBSlamPython::reset)
        .def("set_mode", &ORBSlamPython::setMode)
        .def("set_use_viewer", &ORBSlamPython::setUseViewer)
        .def("get_keyframe_points", &ORBSlamPython::getKeyframePoints)
        .def("get_trajectory_points", &ORBSlamPython::getTrajectoryPoints)
        .def("get_tracked_mappoints", &ORBSlamPython::getTrackedMappoints)
        .def("get_tracked_features", &ORBSlamPython::getTrackedFeatures)
        .def("get_tracking_state", &ORBSlamPython::getTrackingState)
        .def("get_num_features", &ORBSlamPython::getNumFeatures)
        .def("get_num_matched_features", &ORBSlamPython::getNumMatches)
        .def("save_settings", &ORBSlamPython::saveSettings)
        .def("load_settings", &ORBSlamPython::loadSettings)
        .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPython::loadSettingsFile)
        .staticmethod("load_settings_file");
}

ORBSlamPython::ORBSlamPython(std::string vocabFile, std::string settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
    settingsFile(settingsFile),
    sensorMode(sensorMode),
    system(nullptr),
    bUseViewer(false),
    bUseRGB(true)
{

}

ORBSlamPython::ORBSlamPython(const char* vocabFile, const char* settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
    settingsFile(settingsFile),
    sensorMode(sensorMode),
    system(nullptr),
    bUseViewer(false),
    bUseRGB(true)
{

}

ORBSlamPython::~ORBSlamPython()
{
}

bool ORBSlamPython::initialize()
{
    system = std::make_shared<ORB_SLAM3::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer);
    return true;
}

bool ORBSlamPython::isRunning()
{
    return system != nullptr;
}

void ORBSlamPython::reset()
{
    if (system)
    {
        system->Reset();
    }
}

// Helper: convert the optional python IMU measurements into ORB_SLAM3 IMU points.
// Accepts None or any sequence of 7-element rows (e.g. an Nx7 numpy array) laid
// out as (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp), with the
// accelerometer in m/s^2, the gyroscope in rad/s and the timestamp in seconds.
static std::vector<ORB_SLAM3::IMU::Point> imuPointsFromPython(const boost::python::object& measurements)
{
    std::vector<ORB_SLAM3::IMU::Point> points;
    if (measurements.is_none())
    {
        return points;
    }

    const long count = boost::python::len(measurements);
    points.reserve(count);
    for (long i = 0; i < count; i++)
    {
        boost::python::object row = measurements[i];
        if (boost::python::len(row) != 7)
        {
            PyErr_SetString(PyExc_ValueError, "IMU measurements must be rows of (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp)");
            boost::python::throw_error_already_set();
        }
        points.emplace_back(
            boost::python::extract<float>(row[0]),
            boost::python::extract<float>(row[1]),
            boost::python::extract<float>(row[2]),
            boost::python::extract<float>(row[3]),
            boost::python::extract<float>(row[4]),
            boost::python::extract<float>(row[5]),
            boost::python::extract<double>(row[6]));
    }
    return points;
}

bool ORBSlamPython::loadAndProcessMono(std::string imageFile, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processMono(im, timestamp, imuMeasurements);
}

bool ORBSlamPython::processMono(cv::Mat image, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    if (image.data)
    {
        // ORB_SLAM3 always returns a pose (identity when there is none), so
        // success is judged by the tracking state instead.
        system->TrackMonocular(image, timestamp, imuPointsFromPython(imuMeasurements));
        return system->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK;
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB) {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processStereo(leftImage, rightImage, timestamp, imuMeasurements);
}

bool ORBSlamPython::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    if (leftImage.data && rightImage.data) {
        system->TrackStereo(leftImage, rightImage, timestamp, imuPointsFromPython(imuMeasurements));
        return system->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK;
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    return this->processRGBD(im, imDepth, timestamp, imuMeasurements);
}

bool ORBSlamPython::processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp, boost::python::object imuMeasurements)
{
    if (!system)
    {
        return false;
    }
    if (image.data && depthImage.data)
    {
        system->TrackRGBD(image, depthImage, timestamp, imuPointsFromPython(imuMeasurements));
        return system->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK;
    }
    else
    {
        return false;
    }
}

void ORBSlamPython::shutdown()
{
    if (system)
    {
        system->Shutdown();
        system.reset();
    }
}

ORB_SLAM3::Tracking::eTrackingState ORBSlamPython::getTrackingState() const
{
    if (system)
    {
        return static_cast<ORB_SLAM3::Tracking::eTrackingState>(system->GetTrackingState());
    }
    return ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY;
}

unsigned int ORBSlamPython::getNumFeatures() const
{
    if (system)
    {
        // Read the feature count from the keypoints of the most recently tracked frame.
        return system->GetTrackedKeyPointsUn().size();
    }
    return 0;
}

unsigned int ORBSlamPython::getNumMatches() const
{
    if (system)
    {
        // The tracker clears the outlier slots of the current frame, so the
        // valid map points it still holds are the matched features.
        std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = system->GetTrackedMapPoints();
        unsigned int matches = 0;
        for (size_t i = 0; i < trackedMapPoints.size(); ++i)
        {
            ORB_SLAM3::MapPoint* pMP = trackedMapPoints[i];
            if (pMP && !pMP->isBad() && pMP->Observations() > 0)
            {
                ++matches;
            }
        }
        return matches;
    }
    return 0;
}

// Helper: convert a camera-to-world pose into a python tuple of the timestamp
// followed by the row-major 3x4 [R | t] matrix.
static boost::python::tuple poseToTuple(double timestamp, const Sophus::SE3f& Twc)
{
    const Eigen::Matrix3f R = Twc.rotationMatrix();
    const Eigen::Vector3f t = Twc.translation();
    return boost::python::make_tuple(
        timestamp,
        R(0,0), R(0,1), R(0,2), t(0),
        R(1,0), R(1,1), R(1,2), t(1),
        R(2,0), R(2,1), R(2,2), t(2));
}

boost::python::list ORBSlamPython::getKeyframePoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // This is copied from the ORB_SLAM3 System.SaveKeyFrameTrajectoryTUM function, with some changes to output a python tuple.
    // The atlas only returns the keyframes of the active map.
    ORB_SLAM3::Atlas* atlas = system.get()->*memberPointer(SystemAtlas());
    vector<ORB_SLAM3::KeyFrame*> vpKFs = atlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    boost::python::list trajectory;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM3::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        trajectory.append(poseToTuple(pKF->mTimeStamp, pKF->GetPoseInverse()));
    }

    return trajectory;
}

// Helper: convert a vector of map points into a python list of (x, y, z) tuples,
// skipping null / bad points.
static boost::python::list mapPointsToList(const std::vector<ORB_SLAM3::MapPoint*>& mps)
{
    boost::python::list map_points;
    for (size_t i = 0; i < mps.size(); i++)
    {
        ORB_SLAM3::MapPoint* pMP = mps[i];
        if (!pMP || pMP->isBad())
        {
            continue;
        }
        Eigen::Vector3f wp = pMP->GetWorldPos();
        map_points.append(boost::python::make_tuple(wp(0), wp(1), wp(2)));
    }
    return map_points;
}

boost::python::list ORBSlamPython::getTrackedMappoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // GetTrackedMapPoints() returns the current frame's map-point slots, which
    // include null entries (keypoints with no associated landmark) and bad
    // points, so the conversion helper skips those rather than dereferencing them.
    return mapPointsToList(system->GetTrackedMapPoints());
}

boost::python::list ORBSlamPython::getTrackedFeatures() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // GetTrackedKeyPointsUn() and GetTrackedMapPoints() are filled from the same
    // current-frame slots (mCurrentFrame.mvKeysUn / mvpMapPoints) and therefore
    // stay index-aligned; a feature only has a 3D point once it has been
    // triangulated and associated with a landmark, so null/bad entries (2D-only
    // features) are skipped here.
    std::vector<cv::KeyPoint> keypoints = system->GetTrackedKeyPointsUn();
    std::vector<ORB_SLAM3::MapPoint*> mapPoints = system->GetTrackedMapPoints();

    boost::python::list features;
    size_t n = std::min(keypoints.size(), mapPoints.size());
    for (size_t i = 0; i < n; i++)
    {
        ORB_SLAM3::MapPoint* pMP = mapPoints[i];
        if (!pMP || pMP->isBad())
        {
            continue;
        }
        Eigen::Vector3f wp = pMP->GetWorldPos();
        features.append(boost::python::make_tuple(
            keypoints[i].pt.x,
            keypoints[i].pt.y,
            wp(0),
            wp(1),
            wp(2)
            ));
    }
    return features;
}

boost::python::list ORBSlamPython::getTrajectoryPoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // Reconstructs the full per-frame trajectory the same way ORB_SLAM3's
    // System::SaveTrajectoryTUM / SaveTrajectoryEuRoC do: each frame's pose is
    // stored relative to its reference keyframe, which is refined by BA and loop
    // closure, so concatenating the two gives the up-to-date frame pose.
    // Unlike those functions the poses are left in the map's world frame (rather
    // than re-anchored on the first keyframe) so they line up with the map points.
    ORB_SLAM3::Tracking* tracker = system.get()->*memberPointer(SystemTracker());
    ORB_SLAM3::Atlas* atlas = system.get()->*memberPointer(SystemAtlas());

    // ORB_SLAM3 starts a new map (with its own world frame) when tracking is lost,
    // so only frames anchored in the active map are comparable.
    ORB_SLAM3::Map* activeMap = atlas->GetCurrentMap();

    boost::python::list trajectory;

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = tracker->mlpReferences.begin();
    list<double>::iterator lT = tracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = tracker->mlbLost.begin();
    for(list<Sophus::SE3f>::iterator lit=tracker->mlRelativeFramePoses.begin(),
        lend=tracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        // While RECENTLY_LOST without a pose estimate, the tracker records a copy
        // of the previous entry (same timestamp, reference keyframe and relative
        // pose) instead of flagging the frame as lost, so skip those copies.
        if(lit != tracker->mlRelativeFramePoses.begin() && *lT == *std::prev(lT) &&
           *lRit == *std::prev(lRit) && lit->params() == std::prev(lit)->params())
            continue;

        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF && pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if(!pKF || pKF->GetMap() != activeMap)
            continue;

        Trw = Trw * pKF->GetPose();

        Sophus::SE3f Tcw = (*lit) * Trw;
        trajectory.append(poseToTuple(*lT, Tcw.inverse()));
    }

    return trajectory;
}

void ORBSlamPython::setMode(ORB_SLAM3::System::eSensor mode)
{
    sensorMode = mode;
}

void ORBSlamPython::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

void ORBSlamPython::setRGBMode(bool rgb)
{
    bUseRGB = rgb;
}

bool ORBSlamPython::saveSettings(boost::python::dict settings) const
{
    return ORBSlamPython::saveSettingsFile(settings, settingsFile);
}

boost::python::dict ORBSlamPython::loadSettings() const
{
    return ORBSlamPython::loadSettingsFile(settingsFile);
}

bool ORBSlamPython::saveSettingsFile(boost::python::dict settings, std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);
    
    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index)
    {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check())
        {
            continue;
        }
        std::string key = extractedKey;
        
        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check())
        {
            fs << key << int(intValue);
            continue;
        }
        
        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check())
        {
            fs << key << float(floatValue);
            continue;
        }
        
        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check())
        {
            fs << key << std::string(stringValue);
            continue;
        }
    }
    
    return true;
}

// Helpers for reading cv::FileNode objects into python objects.
boost::python::list readSequence(cv::FileNode fn, int depth=10);
boost::python::dict readMap(cv::FileNode fn, int depth=10);

boost::python::dict ORBSlamPython::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    cv::FileNode root = fs.root();
    if (root.isMap()) 
    {
        return readMap(root);
    }
    else if (root.isSeq())
    {
        boost::python::dict settings;
        settings["root"] = readSequence(root);
        return settings;
    }
    return boost::python::dict();
}


// ----------- HELPER DEFINITIONS -----------
boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    if (fn.isMap()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            std::string key = item.name();
            
            if (item.isNone())
            {
                map[key] = boost::python::object();
            }
            else if (item.isInt())
            {
                map[key] = int(item);
            }
            else if (item.isString())
            {
                map[key] = std::string(item);
            }
            else if (item.isReal())
            {
                map[key] = float(item);
            }
            else if (item.isSeq() && depth > 0)
            {
                map[key] = readSequence(item, depth-1);
            }
            else if (item.isMap() && depth > 0)
            {
                map[key] = readMap(item, depth-1);  // Depth-limited recursive call to read inner maps
            }
        }
    }
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    if (fn.isSeq()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            
            if (item.isNone())
            {
                sequence.append(boost::python::object());
            }
            else if (item.isInt())
            {
                sequence.append(int(item));
            }
            else if (item.isString())
            {
                sequence.append(std::string(item));
            }
            else if (item.isReal())
            {
                sequence.append(float(item));
            }
            else if (item.isSeq() && depth > 0)
            {
                sequence.append(readSequence(item, depth-1)); // Depth-limited recursive call to read nested sequences
            }
            else if (item.isMap() && depth > 0)
            {
                sequence.append(readMap(item, depth-1));
            }
        }
    }
    return sequence;
}

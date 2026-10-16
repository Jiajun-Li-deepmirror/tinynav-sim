// The C++ looper bridge of the sim rigs — the replacement for the fleet's
// tinynav/tool/looper_bridge_node.py. One binary, two modes:
//
//   tinynav_bridge --hil
//       FULL BRIDGE MODE, what the mujoco HIL rig runs. The emu's camera face
//       publishes the raw looper contract (stereo grays + mono16 depth + vio
//       poses), so this side owns the per-frame work a real camera box would
//       do: exact-stamp sync of (depth, vio_image, infra1 image), keyframing
//       (0.03 m / 1 deg / 3 s criteria), depth 16U -> 32F meters, vio_100hz
//       -> /slam/odometry, and the per-pose /slam/odometry_visual with
//       velocity. A faithful port of the Python original: outputs are
//       bit-exact under the same feed (modulo the scipy quaternion's possible
//       sign flip, see below).
//
//   tinynav_bridge <relay.yaml>   (or --relay <yaml>, or TINYNAV_BRIDGE_CONFIG)
//       RELAY MODE for the gz split-site deployment. RELAY semantics: the
//       original bridge consumes the camera box's per-frame products
//       (/camera/camera/{depth,vio_image,infra1 image} synced, vio_100hz)
//       and does the keyframing itself; there the camera runs VIO and the
//       bridge produces the final keyframe products. In the gz sim split the
//       "camera" is our own perception component, which already emits the
//       final products — so this mode runs as a QoS-aware relay over the
//       SAME publication contract: every entry re-publishes one crossing
//       topic under its canonical local name, stamp untouched. This was the
//       first attempt at replacing the fleet bridge; it answers a different
//       question (the x86 side already emits finished products there), which
//       is why the HIL mode above exists.
//
// Relay bandwidth argument (why a relay at all): DDS unicasts one copy per
// subscriber, so a crossing topic with N local consumers costs N× link
// bandwidth. The relay is the ONLY subscriber of its crossing topics; the
// canonical names exist only on this side (the x86 site remaps perception's
// /slam/* products into the camera-box namespace /camera/camera/slam/*), so
// nothing double-matches. imu, infra2 camera_info and /clock cross directly
// on purpose: small, single Orin-side consumer each, and no local republish
// — relaying them would only duplicate them (the rename-exists-to-prevent-
// collision rule, see docs).
//
// Separate process on purpose: it is the link endpoint — a stack crash must
// not take the link down with it (fleet lesson: anything owned by the
// backend dies with the crash it was supposed to record).
//
// QoS: relay keyframe topics RELIABLE (mapping's ExactTime sync must not drop
// one of the trio); /slam/depth best-effort both sides (planning consumes
// latest-only, drops are its semantics). HIL mode mirrors the Python
// original: inputs RELIABLE KEEP_LAST (depth 50; the sync trio's pose keeps
// the message_filters default, 10), outputs rclpy's create_publisher(topic,
// 10) = RELIABLE VOLATILE KEEP_LAST(10).
//
// HIL-mode parity notes vs the Python original:
//   * TimeSynchronizer(EXACT, queue 20) is reproduced by a three-way stamp
//     pool: every arrival (depth, image OR pose) completes any fully covered
//     triple — matching on the pose alone was tried and dropped ~half the
//     frames, because DDS delivers the three topics without cross-topic
//     ordering. Parts older than 2 s / beyond 20 are dropped, which is what
//     a full sync queue would have done to them. The emu stamps all three
//     identically per frame, so exact stamp match is the only policy that
//     can fire.
//   * np2msg's scipy quaternion may come out sign-flipped against the
//     hand-rolled extraction here: same rotation, different hemisphere of
//     numbers. Compare poses via |dot(q1, q2)| ~ 1, never element-wise.
//   * build_odom is called TWICE per frame in the original and the second
//     call sees dt == 0: /slam/odometry_visual carries the world-frame linear
//     velocity, the keyframe odom (same pose) carries none. Both calls touch
//     the same last-pose chain, which is a no-op the second time — here the
//     chain is advanced once (on pose arrival, matching publish_visual_odom)
//     and the two odometries are built with and without the velocity.
//   * The original subscribes /tf_static only to log that it arrived; no
//     output depends on it, so the HIL mode does not subscribe.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "tinynav_cpp/logging_setup.hpp"

// ───────────────────────────────────────────────────────────────── relay mode ──

namespace {

struct RelaySpec {
    std::string type;
    std::string from;
    std::string to;
    int depth = 50;
    bool reliable = true;
};

std::vector<RelaySpec> load_relay_table(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);
    std::vector<RelaySpec> table;
    for (const auto& entry : config["relay"]) {
        RelaySpec spec;
        spec.type = entry["type"].as<std::string>();
        spec.from = entry["from"].as<std::string>();
        spec.to = entry["to"].as<std::string>();
        if (entry["depth"]) spec.depth = entry["depth"].as<int>();
        if (entry["reliable"]) spec.reliable = entry["reliable"].as<bool>();
        table.push_back(spec);
    }
    return table;
}

class RelayBridgeNode : public rclcpp::Node {
  public:
    explicit RelayBridgeNode(const std::vector<RelaySpec>& table)
        : Node("looper_bridge_node") {
        for (const auto& spec : table) {
            rclcpp::QoS qos(rclcpp::KeepLast(spec.depth));
            if (!spec.reliable) qos.best_effort();
            if (spec.type == "Image") {
                image_pubs_.push_back(create_publisher<sensor_msgs::msg::Image>(
                    spec.to, qos));
                image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
                    spec.from, qos,
                    [pub = image_pubs_.back()](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        pub->publish(*msg);
                    }));
            } else if (spec.type == "Odometry") {
                odom_pubs_.push_back(create_publisher<nav_msgs::msg::Odometry>(
                    spec.to, qos));
                odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                    spec.from, qos,
                    [pub = odom_pubs_.back()](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                        pub->publish(*msg);
                    }));
            } else if (spec.type == "Path") {
                // Orin->x86 leg of the bench topology: planning runs beside
                // the bridge, the trajectory follower sits on the sim site,
                // so the trajectory must cross the link back (renamed to
                // avoid same-name self-match on this node).
                path_pubs_.push_back(create_publisher<nav_msgs::msg::Path>(
                    spec.to, qos));
                path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
                    spec.from, qos,
                    [pub = path_pubs_.back()](nav_msgs::msg::Path::ConstSharedPtr msg) {
                        pub->publish(*msg);
                    }));
            } else {
                RCLCPP_FATAL(get_logger(), "relay entry %s: unsupported type %s",
                             spec.to.c_str(), spec.type.c_str());
                throw std::runtime_error("unsupported relay type " + spec.type);
            }
            RCLCPP_INFO(get_logger(), "relay %s -> %s (%s, %s)", spec.from.c_str(),
                        spec.to.c_str(), spec.type.c_str(),
                        spec.reliable ? "reliable" : "best_effort");
        }
        RCLCPP_INFO(get_logger(),
                    "%zu relay entries up; imu / infra2 camera_info / clock cross directly",
                    table.size());
    }

  private:
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;
};

// ──────────────────────────────────────────────────────────── full HIL mode ──

constexpr const char* kDepthIn = "/camera/camera/depth/image_rect_raw";
constexpr const char* kImageIn = "/camera/camera/infra1/image_rect_raw";
constexpr const char* kPoseIn = "/camera/camera/vio_image";
constexpr const char* kFastPoseIn = "/camera/camera/vio_100hz";
constexpr const char* kCameraInfoIn = "/camera/camera/infra1/camera_info";

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Odometry = nav_msgs::msg::Odometry;

/// pose_msg2np: PoseStamped -> rotation (row-major 3x3) + translation.
void pose_to_iso(const PoseStamped& msg, double R[9], double t[3]) {
    const auto& q = msg.pose.orientation;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double x2 = x + x, y2 = y + y, z2 = z + z;
    const double xx = x * x2, xy = x * y2, xz = x * z2;
    const double yy = y * y2, yz = y * z2, zz = z * z2;
    const double wx = w * x2, wy = w * y2, wz = w * z2;
    R[0] = 1 - (yy + zz); R[1] = xy - wz;       R[2] = xz + wy;
    R[3] = xy + wz;       R[4] = 1 - (xx + zz); R[5] = yz - wx;
    R[6] = xz - wy;       R[7] = yz + wx;       R[8] = 1 - (xx + yy);
    t[0] = msg.pose.position.x;
    t[1] = msg.pose.position.y;
    t[2] = msg.pose.position.z;
}

/// The inverse, for np2msg: rotation matrix (row-major) -> quaternion xyzw
/// (Shepperd).
void rotation_to_quat(const double R[9], double q[4]) {
    const double trace = R[0] + R[4] + R[8];
    if (trace > 0.0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        q[3] = 0.25 / s;
        q[0] = (R[7] - R[5]) * s;
        q[1] = (R[2] - R[6]) * s;
        q[2] = (R[3] - R[1]) * s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
        q[3] = (R[7] - R[5]) / s;
        q[0] = 0.25 * s;
        q[1] = (R[1] + R[3]) / s;
        q[2] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        const double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
        q[3] = (R[2] - R[6]) / s;
        q[0] = (R[1] + R[3]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[5] + R[7]) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
        q[3] = (R[3] - R[1]) / s;
        q[0] = (R[2] + R[6]) / s;
        q[1] = (R[5] + R[7]) / s;
        q[2] = 0.25 * s;
    }
}

class HilBridgeNode : public rclcpp::Node {
  public:
    HilBridgeNode(double kf_translation, double kf_rotation_deg)
        : Node("looper_bridge_node"), kf_translation_(kf_translation),
          kf_rotation_rad_(kf_rotation_deg * M_PI / 180.0) {
        auto input_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
        auto sync_pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));

        ci_sub_ = create_subscription<CameraInfo>(
            kCameraInfoIn, input_qos,
            [this](CameraInfo::ConstSharedPtr msg) {
                if (!cached_ci_) cached_ci_ = msg;
            });
        depth_sub_ = create_subscription<Image>(
            kDepthIn, input_qos, [this](Image::ConstSharedPtr msg) {
                push(depth_pool_, msg->header.stamp, msg);
                try_complete();
            });
        image_sub_ = create_subscription<Image>(
            kImageIn, input_qos, [this](Image::ConstSharedPtr msg) {
                push(image_pool_, msg->header.stamp, msg);
                try_complete();
            });
        pose_sub_ = create_subscription<PoseStamped>(
            kPoseIn, sync_pose_qos,
            [this](PoseStamped::ConstSharedPtr msg) { on_sync_pose(msg); });
        fast_pose_sub_ = create_subscription<PoseStamped>(
            kFastPoseIn, input_qos,
            [this](PoseStamped::ConstSharedPtr msg) { on_fast_pose(msg); });

        odom_pub_ = create_publisher<Odometry>("/slam/odometry", output_qos);
        odom_visual_pub_ =
            create_publisher<Odometry>("/slam/odometry_visual", output_qos);
        depth_pub_ = create_publisher<Image>("/slam/depth", output_qos);
        slam_ci_pub_ =
            create_publisher<CameraInfo>("/slam/camera_info", output_qos);
        ci_alias_pub_ = create_publisher<CameraInfo>(
            "/camera/camera/infra2/camera_info", output_qos);
        kf_odom_pub_ =
            create_publisher<Odometry>("/slam/keyframe_odom", output_qos);
        kf_image_pub_ =
            create_publisher<Image>("/slam/keyframe_image", output_qos);
        kf_depth_pub_ =
            create_publisher<Image>("/slam/keyframe_depth", output_qos);

        RCLCPP_INFO(get_logger(),
                    "bridging %s + %s + %s (exact-stamp sync, keyframe "
                    "%.3f m / %.1f deg / 3 s) into the /slam topics",
                    kPoseIn, kDepthIn, kImageIn, kf_translation_, kf_rotation_deg);
        RCLCPP_INFO(get_logger(), "bridging %s into /slam/odometry", kFastPoseIn);
    }

  private:
    /// One slot per sync stream: an Image or a PoseStamped, stamped in
    /// seconds for the exact-match scan.
    struct Part {
        double stamp;
        Image::ConstSharedPtr img;
        PoseStamped::ConstSharedPtr pose;
    };
    using Pool = std::deque<Part>;
    static constexpr size_t kNone = static_cast<size_t>(-1);

    /// The sync queue's bounded memory: keep at most 20, drop parts older
    /// than 2 s behind the newest arrival. Stamps arrive in order per
    /// publisher, so the pool is sorted.
    template <typename M>
    static void push(Pool& pool, const builtin_interfaces::msg::Time& stamp_msg,
                     std::shared_ptr<const M> msg) {
        Part part;
        part.stamp = rclcpp::Time(stamp_msg).seconds();
        if constexpr (std::is_same_v<M, Image>) {
            part.img = std::move(msg);
        } else {
            part.pose = std::move(msg);
        }
        pool.push_back(std::move(part));
        while (pool.size() > 20 ||
               pool.back().stamp - pool.front().stamp > 2.0) {
            pool.pop_front();
        }
    }

    static size_t find(const Pool& pool, double stamp) {
        for (size_t i = 0; i < pool.size(); ++i) {
            if (pool[i].stamp == stamp) return i;
        }
        return kNone;
    }

    /// A part arrived; complete every triple now fully covered — matching
    /// fires on any of the three arrivals, like TimeSynchronizer, not only on
    /// the pose. Matched parts are consumed (stamps are unique, so a part can
    /// only ever complete one triple). Erase highest index first, rescan from
    /// the front: completions can unlock further completions only behind the
    /// consumed slots.
    void try_complete() {
        for (size_t pi = 0; pi < pose_pool_.size();) {
            const size_t di = find(depth_pool_, pose_pool_[pi].stamp);
            const size_t ii = find(image_pool_, pose_pool_[pi].stamp);
            if (di == kNone || ii == kNone) {
                ++pi;
                continue;
            }
            const auto pose = pose_pool_[pi].pose;
            const auto depth = depth_pool_[di].img;
            const auto image = image_pool_[ii].img;
            image_pool_.erase(image_pool_.begin() + static_cast<long>(ii));
            depth_pool_.erase(depth_pool_.begin() + static_cast<long>(di));
            pose_pool_.erase(pose_pool_.begin() + static_cast<long>(pi));
            sync_callback(*pose, *depth, *image);
            pi = 0;
        }
    }

    /// vio_100hz_callback: np2msg without velocity, last-pose chain untouched.
    void on_fast_pose(PoseStamped::ConstSharedPtr msg) {
        double R[9], t[3];
        pose_to_iso(*msg, R, t);
        odom_pub_->publish(odom_from(R, t, msg->header.stamp, nullptr));
    }

    /// publish_visual_odom (every pose, sync or not) + pose into the sync
    /// pool, in the original's order.
    void on_sync_pose(PoseStamped::ConstSharedPtr msg) {
        double R[9], t[3];
        pose_to_iso(*msg, R, t);
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();

        // build_odom #1: velocity from the previous pose, chain advanced.
        double velocity[3] = {0, 0, 0};
        const bool have_v = advance_chain(t, stamp, velocity);
        odom_visual_pub_->publish(
            odom_from(R, t, msg->header.stamp, have_v ? velocity : nullptr));

        push(pose_pool_, msg->header.stamp, msg);
        try_complete();
    }

    void sync_callback(const PoseStamped& pose, const Image& depth,
                       const Image& image) {
        if (!cached_ci_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "waiting for %s", kCameraInfoIn);
            return;
        }
        double R[9], t[3];
        pose_to_iso(pose, R, t);
        // build_odom #2, the original's second call on the same stamp: its
        // dt is 0, so the keyframe odom carries no velocity.
        const Odometry kf_odom = odom_from(R, t, pose.header.stamp, nullptr);

        Image depth_out = depth_to_meters(depth);
        depth_out.header.stamp = pose.header.stamp;
        depth_out.header.frame_id = "camera";

        Image image_out = image;  // the original's deepcopy, header restamped
        image_out.header.stamp = pose.header.stamp;
        image_out.header.frame_id = "camera";

        CameraInfo ci_out = *cached_ci_;
        ci_out.header.stamp = pose.header.stamp;
        ci_out.header.frame_id = "camera";

        depth_pub_->publish(depth_out);
        slam_ci_pub_->publish(ci_out);
        ci_alias_pub_->publish(ci_out);

        if (should_add_keyframe(R, t, pose.header.stamp)) {
            kf_odom_pub_->publish(kf_odom);
            kf_image_pub_->publish(image_out);
            kf_depth_pub_->publish(depth_out);
            std::copy(R, R + 9, last_kf_R_);
            std::copy(t, t + 3, last_kf_t_);
            last_kf_time_ = rclcpp::Time(pose.header.stamp).seconds();
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000, "sync t=%.3f depth=%ux%u image=%ux%u",
            rclcpp::Time(pose.header.stamp).seconds(), depth.width, depth.height,
            image.width, image.height);
    }

    /// decode_depth_meters + build_depth_msg: mono16 millimeters -> 32FC1
    /// meters; anything already float passes through (the original's else
    /// branch reinterprets the bytes as float32).
    static Image depth_to_meters(const Image& depth) {
        Image out;
        out.header = depth.header;
        out.height = depth.height;
        out.width = depth.width;
        out.is_bigendian = 0;
        const size_t n = static_cast<size_t>(depth.width) * depth.height;
        out.encoding = "32FC1";
        out.step = depth.width * 4;
        out.data.resize(n * 4);
        auto* out_f = reinterpret_cast<float*>(out.data.data());
        if (depth.encoding == "mono16" || depth.encoding == "16UC1") {
            const auto* in = reinterpret_cast<const uint16_t*>(depth.data.data());
            for (size_t i = 0; i < n; ++i) {
                out_f[i] = static_cast<float>(in[i]) / 1000.0f;
            }
        } else {
            std::memcpy(out_f, depth.data.data(), n * 4);
        }
        return out;
    }

    /// should_add_keyframe: first frame always; then translation, rotation,
    /// or the 3 s timer.
    bool should_add_keyframe(const double R[9], const double t[3],
                             const builtin_interfaces::msg::Time& stamp) const {
        if (last_kf_time_ <= 0.0) return true;
        const double d[3] = {t[0] - last_kf_t_[0], t[1] - last_kf_t_[1],
                             t[2] - last_kf_t_[2]};
        const double translation = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        // R_rel = last_R^T * R, row-major: rel[i][j] = sum_k last[k][i] R[k][j].
        double rel[9];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rel[i * 3 + j] = last_kf_R_[0 * 3 + i] * R[0 * 3 + j] +
                                 last_kf_R_[1 * 3 + i] * R[1 * 3 + j] +
                                 last_kf_R_[2 * 3 + i] * R[2 * 3 + j];
            }
        }
        const double cos_angle =
            std::clamp((rel[0] + rel[4] + rel[8] - 1.0) * 0.5, -1.0, 1.0);
        return translation >= kf_translation_ || std::acos(cos_angle) >= kf_rotation_rad_ ||
               rclcpp::Time(stamp).seconds() - last_kf_time_ > 3.0;
    }

    /// build_odom's velocity slot: world-frame linear velocity from the
    /// previous pose on this chain; dt <= 1 ms leaves it unset (the original's
    /// second, same-stamp call). The chain advances on every call either way.
    bool advance_chain(const double t[3], double sec, double velocity[3]) {
        bool have_v = false;
        if (last_pose_time_ > 0.0) {
            const double dt = sec - last_pose_time_;
            if (dt > 1e-3) {
                for (int i = 0; i < 3; ++i) velocity[i] = (t[i] - last_t_[i]) / dt;
                have_v = true;
            }
        }
        std::copy(t, t + 3, last_t_);
        last_pose_time_ = sec;
        return have_v;
    }

    static Odometry odom_from(const double R[9], const double t[3],
                              const builtin_interfaces::msg::Time& stamp,
                              const double velocity[3]) {
        Odometry m;
        m.header.stamp = stamp;
        m.header.frame_id = "world";
        m.child_frame_id = "camera";
        m.pose.pose.position.x = t[0];
        m.pose.pose.position.y = t[1];
        m.pose.pose.position.z = t[2];
        double q[4];
        rotation_to_quat(R, q);
        m.pose.pose.orientation.x = q[0];
        m.pose.pose.orientation.y = q[1];
        m.pose.pose.orientation.z = q[2];
        m.pose.pose.orientation.w = q[3];
        if (velocity != nullptr) {
            m.twist.twist.linear.x = velocity[0];
            m.twist.twist.linear.y = velocity[1];
            m.twist.twist.linear.z = velocity[2];
        }
        return m;
    }

    double kf_translation_;
    double kf_rotation_rad_;

    CameraInfo::ConstSharedPtr cached_ci_;
    Pool depth_pool_;
    Pool image_pool_;
    Pool pose_pool_;

    double last_t_[3] = {0, 0, 0};
    double last_pose_time_ = 0.0;

    double last_kf_R_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double last_kf_t_[3] = {0, 0, 0};
    double last_kf_time_ = 0.0;

    rclcpp::Subscription<CameraInfo>::SharedPtr ci_sub_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<Image>::SharedPtr image_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr fast_pose_sub_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<Odometry>::SharedPtr odom_visual_pub_;
    rclcpp::Publisher<Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<CameraInfo>::SharedPtr slam_ci_pub_;
    rclcpp::Publisher<CameraInfo>::SharedPtr ci_alias_pub_;
    rclcpp::Publisher<Odometry>::SharedPtr kf_odom_pub_;
    rclcpp::Publisher<Image>::SharedPtr kf_image_pub_;
    rclcpp::Publisher<Image>::SharedPtr kf_depth_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    tinynav::logging_setup::install();  // tag: looper_bridge (fleet-identical)

    const std::string first = argc > 1 ? argv[1] : "";
    try {
        if (first == "--hil") {
            double kf_translation = 0.03;
            double kf_rotation_deg = 1.0;
            for (int i = 2; i < argc - 1; ++i) {
                const std::string arg = argv[i];
                if (arg == "--keyframe-translation") {
                    kf_translation = std::atof(argv[i + 1]);
                }
                if (arg == "--keyframe-rotation-deg") {
                    kf_rotation_deg = std::atof(argv[i + 1]);
                }
            }
            rclcpp::spin(std::make_shared<HilBridgeNode>(kf_translation, kf_rotation_deg));
        } else {
            // Relay mode: a yaml path positionally (what orin_stack.launch.py
            // passes), after --relay, or via TINYNAV_BRIDGE_CONFIG.
            const char* config = nullptr;
            if (!first.empty() && first != "--relay") {
                config = argv[1];
            } else if (first == "--relay" && argc > 2) {
                config = argv[2];
            } else {
                config = std::getenv("TINYNAV_BRIDGE_CONFIG");
            }
            if (config == nullptr || *config == '\0') {
                std::fprintf(stderr,
                             "usage: tinynav_bridge --hil "
                             "[--keyframe-translation M] [--keyframe-rotation-deg D]\n"
                             "       tinynav_bridge <relay.yaml>  "
                             "(or --relay <yaml>, or TINYNAV_BRIDGE_CONFIG)\n");
                rclcpp::shutdown();
                return 1;
            }
            rclcpp::spin(std::make_shared<RelayBridgeNode>(load_relay_table(config)));
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("looper_bridge"), "bridge failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

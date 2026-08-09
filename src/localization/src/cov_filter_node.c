#include <math.h>

#include <apriltag_msgs/msg/april_tag_detection_array.h>
#include <nav_msgs/msg/odometry.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>

#include "utils.h"

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : ((val) < (min)) ? (min) : (val))

typedef enum cameras_e
{
    cameras_left_e = 0,
    cameras_back_e = 1,
    cameras_right_e = 2,
    max_cameras_e = 3,
} cameras_t;
const char* const camera_names[max_cameras_e] = {"left", "back", "right"};
const char* const camera_tag_topics[max_cameras_e] = {
    "left/detector/tags",
    "back/detector/tags",
    "right/detector/tags"};

#define STALE_DATA_TIMEOUT (500LL * 1000LL * 1000LL)
#define REFERENCE_AREA (10000.0)
#define MIN_COV_MULT (0.1)
#define MAX_COV_MULT (500.0)
typedef struct camera_state_s
{
    uint32_t detection_num;
    double total_area;
    int64_t last_seen;
} camera_state_t;

typedef struct cov_filter_node_s
{
    rcl_node_t handle;
    rcl_subscription_t odom_sub;
    nav_msgs__msg__Odometry odom_msg;
    rcl_subscription_t cameras[max_cameras_e];
    apriltag_msgs__msg__AprilTagDetectionArray camera_msgs[max_cameras_e];
    camera_state_t states[max_cameras_e];
    rcl_publisher_t filtered_odom_pub;
    rcl_clock_t* clock;
} cov_filter_node_t;

void odom_callback(const void* msg_in, void* ctx)
{
    const nav_msgs__msg__Odometry* msg = (const nav_msgs__msg__Odometry*)msg_in;
    nav_msgs__msg__Odometry filtered_msg = {0};
    nav_msgs__msg__Odometry__init(&filtered_msg);
    nav_msgs__msg__Odometry__copy(msg, &filtered_msg);

    cov_filter_node_t* node = (cov_filter_node_t*)ctx;

    int64_t now = 0;
    const rcl_ret_t ret = rcl_clock_get_now(node->clock, &now);
    if (ret != RCL_RET_OK)
    {
        RCUTILS_LOG_INFO("Failed to get clock->now, dropping message");  // NOLINT
        return;
    }

    uint32_t total_tags = 0;
    double total_area = 0.0;

    for (size_t i = 0; i < max_cameras_e; i++)
    {
        if (now - node->states[i].last_seen < STALE_DATA_TIMEOUT)
        {
            total_tags += node->states[i].detection_num;
            total_area += node->states[i].total_area;
        }
    }

    const double avg_area = (total_tags > 0) ? (total_area / total_tags) : 1.0;
    const double area_factor = (REFERENCE_AREA / fmax(avg_area, 1.0));
    const double count_factor = (total_tags > 1) ? total_tags : 1.0;

    double multiplier = count_factor > 1 ? 0.1 : pow(area_factor, 2);
    multiplier = CLAMP(multiplier, MIN_COV_MULT, MAX_COV_MULT);  // NOLINT

    const size_t matrix_indices[6] = {0, 7, 14, 21, 28, 35};
    for (size_t i = 0; i < 6; i++)
    {
        filtered_msg.pose.covariance[matrix_indices[i]] = multiplier;
    }

    (void)rcl_publish(&node->filtered_odom_pub, &filtered_msg, NULL);
}

void tag_callback(const void* msg_in, void* ctx, cameras_t cam)
{
    const apriltag_msgs__msg__AprilTagDetectionArray* msg =
        (const apriltag_msgs__msg__AprilTagDetectionArray*)(msg_in);
    cov_filter_node_t* node = (cov_filter_node_t*)(ctx);
    int64_t temp_now = 0;
    const rcl_ret_t ret = rcl_clock_get_now(node->clock, &temp_now);
    if (ret != RCL_RET_OK)
    {
        RCUTILS_LOG_INFO("Failed to get clock->now, dropping message");  // NOLINT
        return;
    }
    camera_state_t state = {0};
    state.detection_num = msg->detections.size;
    state.last_seen = temp_now;
    if (state.detection_num > 0)
    {
        for (size_t i = 0; i < state.detection_num; i++)
        {
            apriltag_msgs__msg__AprilTagDetection* detection = &msg->detections.data[i];
            const double width = fabs(detection->corners[1].x - detection->corners[0].x);
            const double height = fabs(detection->corners[3].y - detection->corners[0].y);
            state.total_area += width * height;
        }
    }
    node->states[cam] = state;
}

#define TAG_CALLBACK(name)                                  \
    void tag_callback_##name(const void* msg_in, void* ctx) \
    {                                                       \
        tag_callback(msg_in, ctx, cameras_##name##_e);      \
    }
TAG_CALLBACK(left)
TAG_CALLBACK(back)
TAG_CALLBACK(right)

const rclc_subscription_callback_with_context_t callbacks[max_cameras_e] = {
    tag_callback_left,
    tag_callback_back,
    tag_callback_right,
};

int main(const int argc, const char* const* argv)
{
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_support_t sup = {0};
    RCL_CHECK(rclc_support_init(&sup, argc, argv, &alloc));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCL_CHECK(rclc_executor_init(&executor, &sup.context, 4, sup.allocator));

    cov_filter_node_t node = {0};
    node.clock = &sup.clock;
    RCL_CHECK(rclc_node_init_default(&node.handle, "cov_filter", "", &sup));

    RCL_CHECK(rclc_subscription_init_default(
        &node.odom_sub,
        &node.handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/body_rig"))

    nav_msgs__msg__Odometry__init(&node.odom_msg);
    RCL_CHECK(rclc_executor_add_subscription_with_context(
        &executor,
        &node.odom_sub,
        &node.odom_msg,
        odom_callback,
        &node,
        ON_NEW_DATA));

    int64_t now = 0;
    RCL_CHECK(rcl_clock_get_now(&sup.clock, &now));
    for (size_t i = 0; i < max_cameras_e; i++)
    {
        apriltag_msgs__msg__AprilTagDetectionArray__init(&node.camera_msgs[i]);
        RCL_CHECK(rclc_subscription_init_default(
            &node.cameras[i],
            &node.handle,
            ROSIDL_GET_MSG_TYPE_SUPPORT(apriltag_msgs, msg, AprilTagDetectionArray),
            camera_tag_topics[i]));
        RCL_CHECK(rclc_executor_add_subscription_with_context(
            &executor,
            &node.cameras[i],
            &node.camera_msgs[i],
            callbacks[i],
            &node,
            ON_NEW_DATA));

        node.states[i].total_area = 0.0;
        node.states[i].detection_num = 0;
        node.states[i].last_seen = now;
    }

    rclc_executor_spin(&executor);

    for (size_t i = 0; i < max_cameras_e; i++)
    {
        RCL_CHECK(rcl_subscription_fini(&node.cameras[i], &node.handle));
        apriltag_msgs__msg__AprilTagDetectionArray__fini(&node.camera_msgs[i]);
    }
    nav_msgs__msg__Odometry__fini(&node.odom_msg);
    RCL_CHECK(rcl_subscription_fini(&node.odom_sub, &node.handle));
    RCL_CHECK(rcl_node_fini(&node.handle));
    RCL_CHECK(rclc_executor_fini(&executor))
    RCL_CHECK(rclc_support_fini(&sup));
}

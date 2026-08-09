#include <apriltag_msgs/msg/april_tag_detection.h>
#include <apriltag_msgs/msg/april_tag_detection_array.h>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection.h>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include "utils.h"

typedef struct tag_translator_s
{
    rcl_node_t handle;
    rcl_subscription_t sub;
    rcl_publisher_t pub;
    isaac_ros_apriltag_interfaces__msg__AprilTagDetectionArray msg;
} tag_translator_t;

void tag_callback(const void *msg_in, void *ctx)
{
    const isaac_ros_apriltag_interfaces__msg__AprilTagDetectionArray *msg =
        (const isaac_ros_apriltag_interfaces__msg__AprilTagDetectionArray *)msg_in;
    tag_translator_t *node = (tag_translator_t *)ctx;

    apriltag_msgs__msg__AprilTagDetectionArray out;
    apriltag_msgs__msg__AprilTagDetectionArray__init(&out);

    out.header.stamp = msg->header.stamp;
    rosidl_runtime_c__String__copy(&msg->header.frame_id, &out.header.frame_id);

    if (msg->detections.size > 0)
    {
        apriltag_msgs__msg__AprilTagDetection__Sequence__init(
            &out.detections,
            msg->detections.size);

        for (size_t i = 0; i < msg->detections.size; i++)
        {
            isaac_ros_apriltag_interfaces__msg__AprilTagDetection detection_in =
                msg->detections.data[i];
            apriltag_msgs__msg__AprilTagDetection *d_out = &out.detections.data[i];

            rosidl_runtime_c__String__copy(&detection_in.family, &d_out->family);
            d_out->id = detection_in.id;
            d_out->centre.x = detection_in.center.x;
            d_out->centre.y = detection_in.center.y;

            for (size_t j = 0; j < 4; j++)
            {
                d_out->corners[j].x = detection_in.corners[j].x;
                d_out->corners[j].y = detection_in.corners[j].y;
            }
        }
    }

    (void)rcl_publish(&node->pub, &out, NULL);
    apriltag_msgs__msg__AprilTagDetectionArray__fini(&out);
}

rcl_ret_t tag_translator_init(
    tag_translator_t *node,
    rclc_support_t *sup,
    rclc_executor_t *executor,
    const char *node_name,
    const char *subscription_name,
    const char *publisher_name)
{
    node->handle = rcl_get_zero_initialized_node();
    RCL_TRY(rclc_node_init_default(&node->handle, node_name, "", sup));

    node->sub = rcl_get_zero_initialized_subscription();
    RCL_TRY(rclc_subscription_init_best_effort(
        &node->sub,
        &node->handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(isaac_ros_apriltag_interfaces, msg, AprilTagDetectionArray),
        subscription_name));

    node->pub = rcl_get_zero_initialized_publisher();
    RCL_TRY(rclc_publisher_init_default(
        &node->pub,
        &node->handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(apriltag_msgs, msg, AprilTagDetectionArray),
        publisher_name));

    isaac_ros_apriltag_interfaces__msg__AprilTagDetectionArray__init(&node->msg);

    RCL_TRY(rclc_executor_add_subscription_with_context(
        executor,
        &node->sub,
        &node->msg,
        tag_callback,
        node,
        ON_NEW_DATA));

    return RCL_RET_OK;
}

rcl_ret_t tag_translator_fini(tag_translator_t *node)
{
    isaac_ros_apriltag_interfaces__msg__AprilTagDetectionArray__fini(&node->msg);
    RCL_TRY(rcl_publisher_fini(&node->pub, &node->handle));
    RCL_TRY(rcl_subscription_fini(&node->sub, &node->handle));
    RCL_TRY(rcl_node_fini(&node->handle));
    return RCL_RET_OK;
}

int main(const int argc, const char *const *argv)
{
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_support_t sup = {0};
    RCL_CHECK(rclc_support_init(&sup, argc, argv, &alloc));

    const size_t nodes_size = 3;
    const char *node_names[nodes_size] = {"trans_left", "trans_back", "trans_right"};
    const char *sub_topics[nodes_size] = {
        "/left/detector/tags_nvidia",
        "/back/detector/tags_nvidia",
        "/right/detector/tags_nvidia"};
    const char *pub_topics[nodes_size] = {
        "/left/detector/tags",
        "/back/detector/tags",
        "/right/detector/tags"};
    tag_translator_t nodes[nodes_size];

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCL_CHECK(rclc_executor_init(&executor, &sup.context, nodes_size, sup.allocator));

    for (size_t i = 0; i < nodes_size; i++)
    {
        RCL_CHECK(tag_translator_init(
            &nodes[i],
            &sup,
            &executor,
            node_names[i],
            sub_topics[i],
            pub_topics[i]));
    }

    rclc_executor_spin(&executor);

    for (size_t i = 0; i < nodes_size; i++)
    {
        RCL_CHECK(tag_translator_fini(&nodes[i]));
    }

    RCL_CHECK(rclc_executor_fini(&executor));
    RCL_CHECK(rclc_support_fini(&sup));
}

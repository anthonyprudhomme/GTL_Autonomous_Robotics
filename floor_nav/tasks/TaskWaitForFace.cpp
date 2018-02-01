#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{
    const face_detect_base::ROIArray faces = env->getFaces();
    if (faces.ROIArray.size() > 0) {
        ROS_INFO("Detected Face ");
        return TaskStatus::TASK_COMPLETED;
    }
    return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);

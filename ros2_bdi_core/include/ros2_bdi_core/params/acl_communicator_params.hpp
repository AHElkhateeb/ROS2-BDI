#ifndef ACL_COMMUNICATOR_PARAMS_H_
#define ACL_COMMUNICATOR_PARAMS_H_

/* Parameters affecting internal logic (recompiling required) */

#define IS_ACCEPTED_OP_SRV "is_accepted_operation"

#define CK_BELIEF_SRV "check_belief_srv"
#define ADD_BELIEF_SRV "add_belief_srv"
#define DEL_BELIEF_SRV "del_belief_srv"

#define CK_DESIRE_SRV "check_desire_srv"
#define ADD_DESIRE_SRV "add_desire_srv"
#define DEL_DESIRE_SRV "del_desire_srv"

#define ADD_I 1
#define DEL_I 0
#define MAX_WAIT_UPD 4 // indicates number of belief/desire set notification to wait before considering a submitted upd request failed 

/* ROS2 Parameter names for PlanSys2Monitor node */
#define PARAM_BELIEF_CHECK "belief_ck"
#define PARAM_BELIEF_WRITE "belief_w"
#define PARAM_DESIRE_CHECK "desire_ck"
#define PARAM_DESIRE_WRITE "desire_w"
#define PARAM_DESIRE_MAX_PRIORITIES "desire_pr"

#define ACL_SRV "acl_srv"
#define ACL_MSG_TOPIC "acl_msg"
#define DEL_CONV_TOPIC "del_conv"

#define CURRENT_TIME_MILLIS std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

#endif // ACL_COMMUNICATOR_PARAMS_H_
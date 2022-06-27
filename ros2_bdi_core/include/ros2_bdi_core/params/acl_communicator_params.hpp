#ifndef ACL_COMMUNICATOR_PARAMS_H_
#define ACL_COMMUNICATOR_PARAMS_H_

/* Parameters affecting internal logic (recompiling required) */
#define ACL_COMMUNICATOR_NODE_NAME "acl_communicator"

#define ACL_SRV "acl_srv"

#define ADD_I 1
#define DEL_I 0
#define MAX_WAIT_UPD 4 // indicates number of belief/desire set notification to wait before considering a submitted upd request failed 

/* ROS2 Parameter names for PlanSys2Monitor node */
#define PARAM_BELIEF_CHECK "belief_ck"
#define PARAM_BELIEF_WRITE "belief_w"
#define PARAM_DESIRE_CHECK "desire_ck"
#define PARAM_DESIRE_WRITE "desire_w"
#define PARAM_DESIRE_MAX_PRIORITIES "desire_pr"


#endif // ACL_COMMUNICATOR_PARAMS_H_

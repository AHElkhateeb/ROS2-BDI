#ifndef ACL_COMMUNICATOR_PARAMS_H_
#define ACL_COMMUNICATOR_PARAMS_H_

/* Parameters affecting internal logic (recompiling required) */
#define ACL_COMMUNICATOR_NODE_NAME "acl_communicator"

#define ACL_SRV "acl_srv"
#define ACL_MSG_TOPIC "acl_msg"
#define DEL_CONV_TOPIC "del_conv"

#define ADD_I 1
#define DEL_I 0
#define MAX_WAIT_UPD 4 // indicates number of belief/desire set notification to wait before considering a submitted upd request failed

#define CURRENT_TIME_MILLIS std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

#endif // ACL_COMMUNICATOR_PARAMS_H_

#ifndef CONV_CLIENT_PARAMS_H_
#define CONV_CLIENT_PARAMS_H_

#define ACL_SRV "acl_srv"
#define ACL_MSG_TOPIC "acl_msg"
#define DEL_CONV_TOPIC "del_conv"
#define ADD_DESIRE_TOPIC "add_desire"
#define DEL_DESIRE_TOPIC "del_desire"
#define ADD_BELIEF_TOPIC "add_belief"
#define DEL_BELIEF_TOPIC "del_belief"

#define CURRENT_TIME_MILLIS std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

#endif // CONV_CLIENT_PARAMS_H_
